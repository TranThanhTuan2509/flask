#pragma once
// FLASK-RRTConnect with the search semantics of the authors' implementation
// (KavrakiLab/ompl-flask, branch `flat`: og::RRTConnect on ob::FlatStateSpace).
//
// Differences from the legacy reconstruction in rrt_connect.hh, each taken from the author source:
//  * metric      FlatStateSpace FLAT_STATE_METRIC = CompoundStateSpace weighted SUM of per-level
//                Euclidean distances, weights 1 / (maximum extent of the level)
//                (FlatStateSpace::setDefaultWeights, CompoundStateSpace::distance); exact nearest neighbour.
//  * range       og::RRTConnect default maxDistance = 0.2 x space extent (= 0.2 x number of levels).
//  * extend      FlatStateSpace::interpolate: steer nearest -> sample, evaluate the SAME polynomial at
//                t * T for every derivative level; the stored edge is then re-steered between the node and
//                that state (FlatVampMotionValidator steers each edge); start tree edges run node -> new,
//                goal tree edges run new -> node (RRTConnect::growTree).
//  * failure     an invalid edge returns TRAPPED (no retries); no dynamic domain, no tree balancing,
//                trees alternate every iteration, connect repeats growTree while ADVANCED.
//  * goals       goal states are added lazily like RRTConnect with GoalStates.
// Steering (lqmt.hh) and edge validation (flask_cc.hh) are shared with the legacy planner.

#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <vamp/utils.hh>

#include "rrt_connect.hh"

namespace flask
{
template <typename Robot> struct OmplRRTConnect
{
    static constexpr std::size_t n = Robot::dimension;
    enum Grow
    {
        TRAPPED,
        ADVANCED,
        REACHED
    };

    struct Tree
    {
        bool is_start;
        std::vector<Vector<n>> Q, V;
        std::vector<std::size_t> parent;
        std::vector<double> T;  // duration of the edge to the parent (directed in physical time)

        auto size() const noexcept -> std::size_t
        {
            return Q.size();
        }
        auto add(const Vector<n> &q, const Vector<n> &v, std::size_t p, double t) -> std::size_t
        {
            Q.push_back(q);
            V.push_back(v);
            parent.push_back(p);
            T.push_back(t);
            return Q.size() - 1;
        }
    };

    struct Metric
    {
        double wq, wv;
        auto operator()(const Vector<n> &qa, const Vector<n> &va, const Vector<n> &qb,
                        const Vector<n> &vb) const noexcept -> double
        {
            double sq = 0., sv = 0.;
            for (std::size_t j = 0; j < n; ++j)
            {
                const double dq = qa[j] - qb[j], dv = va[j] - vb[j];
                sq += dq * dq;
                sv += dv * dv;
            }
            return wq * std::sqrt(sq) + wv * std::sqrt(sv);
        }
    };

    static auto nearest(const Tree &t, const Metric &d, const Vector<n> &q, const Vector<n> &v)
        -> std::pair<std::size_t, double>
    {
        std::size_t best = 0;
        double bd = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < t.size(); ++i)
        {
            const double x = d(t.Q[i], t.V[i], q, v);
            if (x < bd)
            {
                bd = x;
                best = i;
            }
        }
        return {best, bd};
    }

    static auto solve(const Vector<n> &start, const std::vector<Vector<n>> &goals,
                      const JointLimits<n> &limits, const VectorEnvironment<Robot> &env,
                      const PlannerSettings &settings, std::uint64_t seed,
                      const Vector<n> &start_velocity, const Vector<n> &goal_velocity)
        -> PlanningResult<n>
    {
        PlanningResult<n> result;
        const auto t0 = std::chrono::steady_clock::now();
        ValidationCounters &counter = result.counter;
        if (goals.empty())
        {
            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
            return result;
        }
        for (const auto &goal : goals)
        {
            if (start == goal && start_velocity == goal_velocity &&
                flask_cc<Robot>(make_edge<n>(start, start_velocity, goal, goal_velocity, 1e-6),
                                limits, env, true, &counter))
            {
                result.solved = true;
                result.trajectory.Q = {start};
                result.trajectory.V = {start_velocity};
                result.start_nodes = result.goal_nodes = 1;
                result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
                return result;
            }
        }
        double eq = 0., ev = 0.;
        for (std::size_t j = 0; j < n; ++j)
        {
            const double dq = limits.q_hi[j] - limits.q_lo[j];
            const double dv = 2. * limits.v_max[j] * settings.velocity_sample_scale;
            eq += dq * dq;
            ev += dv * dv;
        }
        eq = std::sqrt(eq);
        ev = std::sqrt(ev);
        const Metric metric{1. / eq, ev > 0. ? 1. / ev : 1.};
        // Extent of the compound space with these weights is 2 (one per level).
        const double range = settings.range_fraction * 2.;

        Tree ts{true, {}, {}, {}, {}}, tg{false, {}, {}, {}, {}};
        ts.add(start, start_velocity, 0, 0.);
        std::size_t goals_added = 0;
        std::mt19937_64 rng(seed);
        std::uniform_real_distribution<double> U01(0., 1.);
        auto exhausted = [&]()
        {
            return ts.size() + tg.size() >= settings.max_samples ||
                   (settings.max_ns && vamp::utils::get_elapsed_nanoseconds(t0) >= settings.max_ns);
        };

        // growTree of og::RRTConnect.  `added` receives the new node index.
        auto grow = [&](Tree &tree, const Vector<n> &tq, const Vector<n> &tv, std::size_t &added) -> Grow
        {
            const auto [i, d] = nearest(tree, metric, tq, tv);
            if (tree.Q[i] == tq && tree.V[i] == tv)
            {
                added = i;
                return REACHED;
            }
            if (exhausted())
                return TRAPPED;
            Vector<n> dq = tq, dv = tv;
            bool reach = true;
            if (d > range)
            {
                // interpolate(nmotion, rmotion, range / d): same polynomial evaluated at a time fraction.
                const auto m = bvp<n>(tree.Q[i], tree.V[i], tq, tv, settings.rho);
                if (!m)
                    return TRAPPED;
                const double t = m->T * (range / d);
                for (std::size_t j = 0; j < n; ++j)
                {
                    dq[j] = m->q(j, t);
                    dv[j] = m->velocity(j, t);
                }
                if (dq == tree.Q[i] && dv == tree.V[i])
                    return TRAPPED;
                reach = false;
            }
            const auto e = tree.is_start ? bvp<n>(tree.Q[i], tree.V[i], dq, dv, settings.rho)
                                         : bvp<n>(dq, dv, tree.Q[i], tree.V[i], settings.rho);
            if (!e || !flask_cc<Robot>(*e, limits, env, true, &counter))
            {
                ++result.steering_failures;
                return TRAPPED;
            }
            added = tree.add(dq, dv, i, e->T);
            ++result.accepted_extensions;
            return reach ? REACHED : ADVANCED;
        };

        bool start_turn = true;
        std::size_t iterations = 0;
        Vector<n> sq, sv;
        while (iterations < settings.max_iterations && ts.size() + tg.size() < settings.max_samples)
        {
            if (exhausted())
                break;
            ++iterations;
            Tree &tree = start_turn ? ts : tg;
            Tree &other = start_turn ? tg : ts;
            start_turn = !start_turn;
            if (goals_added < goals.size() && (tg.size() == 0 || goals_added < tg.size() / 2))
            {
                tg.add(goals[goals_added++], goal_velocity, std::numeric_limits<std::size_t>::max(), 0.);
                tg.parent.back() = tg.size() - 1;  // root
            }
            for (std::size_t j = 0; j < n; ++j)
                sq[j] = limits.q_lo[j] + (limits.q_hi[j] - limits.q_lo[j]) * U01(rng);
            for (std::size_t j = 0; j < n; ++j)
                sv[j] = (2. * U01(rng) - 1.) * limits.v_max[j] * settings.velocity_sample_scale;
            if (tree.size() == 0)
                continue;
            std::size_t a = 0;
            if (grow(tree, sq, sv, a) == TRAPPED)
                continue;
            const Vector<n> target_q = tree.Q[a], target_v = tree.V[a];
            ++result.connection_attempts;
            std::size_t b = 0;
            Grow g = grow(other, target_q, target_v, b);
            while (g == ADVANCED && !exhausted())
                g = grow(other, target_q, target_v, b);
            if (g != REACHED)
                continue;
            // Trees joined: node b of `other` coincides with node a of `tree`.
            const Tree &s = tree.is_start ? tree : other;
            const Tree &gt = tree.is_start ? other : tree;
            std::size_t si = tree.is_start ? a : b, gi = tree.is_start ? b : a;
            // Step back once on the start side to avoid the duplicate state (RRTConnect::solve).
            if (s.parent[si] != si)
                si = s.parent[si];
            else
                gi = gt.parent[gi];
            std::vector<std::size_t> chain;
            for (std::size_t k = si;; k = s.parent[k])
            {
                chain.push_back(k);
                if (s.parent[k] == k)
                    break;
            }
            Trajectory<n> path;
            for (auto it = chain.rbegin(); it != chain.rend(); ++it)
            {
                path.Q.push_back(s.Q[*it]);
                path.V.push_back(s.V[*it]);
                if (it != chain.rbegin())
                    path.T.push_back(s.T[*it]);
            }
            // Edge from the last start-side state to the first goal-side state.
            {
                const auto e = bvp<n>(path.Q.back(), path.V.back(), gt.Q[gi], gt.V[gi], settings.rho);
                path.T.push_back(e ? e->T : 0.);
            }
            for (std::size_t k = gi;; k = gt.parent[k])
            {
                path.Q.push_back(gt.Q[k]);
                path.V.push_back(gt.V[k]);
                if (gt.parent[k] == k)
                    break;
                path.T.push_back(gt.T[k]);
            }
            result.trajectory = path;
            result.solved = true;
            break;
        }
        result.iterations = iterations;
        result.start_nodes = ts.size();
        result.goal_nodes = tg.size();
        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
        return result;
    }
};
}  // namespace flask
