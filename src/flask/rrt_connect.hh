

#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <vamp/planning/nn.hh>
#include <vamp/utils.hh>

#include "flask_cc.hh"

namespace flask
{
struct PlannerSettings
{
    double range = 2.0;
    double w = 1.0;
    bool cost_neighbors = false;
    std::size_t k = 10;
    double rho = 1.0;
    double velocity_sample_scale = 1.0;
    // Reconstructed RRT range policy. Legacy mode is retained explicitly
    // for auditing archived runs; prefix mode truncates the actual BVP.
    bool prefix_steering = true;
    bool dynamic_domain = true;
    double radius = 4.0;
    double alpha = 0.0001;
    double min_radius = 1.0;
    bool balance = true;
    double tree_ratio = 1.0;
    std::size_t max_iterations = 100000;
    std::size_t max_samples = 100000;
    std::size_t max_ns = 1000000000;
    // 0: legacy reconstruction (KinodynamicRRTConnect); 1: author/OMPL search semantics
    // (OmplRRTConnect in rrt_connect_ompl.hh).  range_fraction is used by mode 1 only.
    int extend_mode = 0;
    double range_fraction = 0.2;
};

template <std::size_t n> struct Trajectory
{
    std::vector<Vector<n>> Q, V;
    std::vector<double> T;

    inline auto edge(std::size_t i) const noexcept -> Edge<n>
    {
        return make_edge<n>(Q[i], V[i], Q[i + 1], V[i + 1], T[i]);
    }
};

template <std::size_t n> struct PlanningResult
{
    bool solved = false;
    Trajectory<n> trajectory;
    std::size_t iterations = 0;
    std::size_t start_nodes = 0, goal_nodes = 0;
    std::size_t nanoseconds = 0;
    std::size_t dynamic_domain_rejections = 0, steering_failures = 0;
    std::size_t accepted_extensions = 0, connection_attempts = 0;
    ValidationCounters counter;
};

template <typename Robot> struct KinodynamicRRTConnect
{
    static constexpr std::size_t n = Robot::dimension;
    static constexpr std::size_t d2 = 2 * n;
    using Key = vamp::FloatVector<d2>;
    static constexpr std::size_t stride = Key::num_scalars_rounded;
    using NN = vamp::planning::NN<d2>;
    using Node = vamp::planning::NNNode<d2>;
    using Arr = vamp::planning::NNFloatArray<d2>;

    struct Tree
    {
        bool is_start;
        std::vector<Vector<n>> Q, V;
        std::vector<std::size_t> parent;
        std::vector<double> T;
        std::vector<double> r;
        std::unique_ptr<float, decltype(&free)> buf;
        NN nn;

        Tree(bool s, std::size_t cap)
            : is_start(s), buf(vamp::utils::vector_alloc<float, vamp::FloatVectorAlignment,
                                                         vamp::FloatVectorWidth>(cap * stride),
                               &free)
        {
            Q.reserve(cap);
            V.reserve(cap);
            parent.reserve(cap);
            T.reserve(cap);
            r.reserve(cap);
        }

        inline auto size() const noexcept -> std::size_t
        {
            return Q.size();
        }

        inline auto insert(const Vector<n> &q, const Vector<n> &v, std::size_t parent,
                           bool original, double Tc, double w) -> std::size_t
        {
            const std::size_t i = Q.size();
            Q.push_back(q);
            V.push_back(v);
            this->parent.push_back(original ? i : parent);
            T.push_back(Tc);
            r.push_back(std::numeric_limits<double>::infinity());
            float *p = buf.get() + i * stride;
            for (std::size_t j = 0; j < stride; ++j)
            {
                p[j] = 0.F;
            }
            for (std::size_t j = 0; j < n; ++j)
            {
                p[j] = static_cast<float>(q[j]);
                p[n + j] = static_cast<float>(w * v[j]);
            }
            nn.insert(Node{i, Arr{p}});
            return i;
        }

        inline auto connection(std::size_t i, const Vector<n> &q, const Vector<n> &v,
                               double rho) const noexcept -> std::optional<Edge<n>>
        {
            return is_start ? bvp<n>(Q[i], V[i], q, v, rho) : bvp<n>(q, v, Q[i], V[i], rho);
        }

        inline auto distance(std::size_t i, const Vector<n> &q, const Vector<n> &v,
                             double w) const noexcept -> double
        {
            double s = 0.0;
            for (std::size_t j = 0; j < n; ++j)
            {
                const double dq = Q[i][j] - q[j], dv = V[i][j] - v[j];
                s += dq * dq + w * w * dv * dv;
            }
            return std::sqrt(s);
        }

        inline auto nearest(const Vector<n> &q, const Vector<n> &v,
                            const PlannerSettings &settings) const -> std::pair<std::size_t, double>
        {
            alignas(32) std::array<float, stride> key{};
            for (std::size_t j = 0; j < n; ++j)
            {
                key[j] = static_cast<float>(q[j]);
                key[n + j] = static_cast<float>(settings.w * v[j]);
            }
            if (!settings.cost_neighbors)
            {
                const auto result = nn.nearest(Arr{key.data()});
                const std::size_t i = result->first.index;
                return {i, distance(i, q, v, settings.w)};
            }
            std::vector<std::pair<Node, float>> nb;
            nn.nearest(nb, Arr{key.data()}, settings.k);
            std::size_t best = nb.front().first.index;
            double best_c = std::numeric_limits<double>::infinity();
            for (const auto &[node, d] : nb)
            {
                const std::size_t i = node.index;
                const auto &Q0 = is_start ? Q[i] : q;
                const auto &V0 = is_start ? V[i] : v;
                const auto &Q1 = is_start ? q : Q[i];
                const auto &V1 = is_start ? v : V[i];
                const auto T = optimal_duration<n>(Q0, V0, Q1, V1, settings.rho);
                if (!T)
                {
                    best = i;
                    break;
                }
                const double c = boundary_cost<n>(Q0, V0, Q1, V1, *T, settings.rho);
                if (c < best_c)
                {
                    best_c = c;
                    best = i;
                }
            }
            return {best, distance(best, q, v, settings.w)};
        }

        inline void root_chain(std::size_t i, std::vector<std::size_t> &out) const
        {
            out.clear();
            out.push_back(i);
            while (parent[out.back()] != out.back())
            {
                out.push_back(parent[out.back()]);
            }
        }
    };

    static auto join_trees(const Tree &s, std::size_t start_index, const Tree &g,
                           std::size_t goal_index) -> Trajectory<n>
    {
        Trajectory<n> trajectory;
        std::vector<std::size_t> start_chain, goal_chain;
        s.root_chain(start_index, start_chain);
        g.root_chain(goal_index, goal_chain);
        std::reverse(start_chain.begin(), start_chain.end());
        for (auto i : start_chain)
        {
            trajectory.Q.push_back(s.Q[i]);
            trajectory.V.push_back(s.V[i]);
        }
        for (std::size_t k = 1; k < start_chain.size(); ++k)
        {
            trajectory.T.push_back(s.T[start_chain[k]]);
        }
        for (std::size_t k = 1; k < goal_chain.size(); ++k)
        {
            trajectory.Q.push_back(g.Q[goal_chain[k]]);
            trajectory.V.push_back(g.V[goal_chain[k]]);
        }
        for (std::size_t k = 0; k + 1 < goal_chain.size(); ++k)
        {
            trajectory.T.push_back(g.T[goal_chain[k]]);
        }
        return trajectory;
    }

    static auto solve(const Vector<n> &start, const std::vector<Vector<n>> &goals,
                      const JointLimits<n> &limits, const VectorEnvironment<Robot> &env,
                      const PlannerSettings &settings, std::uint64_t seed,
                      const Vector<n> &start_velocity,
                      const Vector<n> &goal_velocity) -> PlanningResult<n>
    {
        PlanningResult<n> result;
        const auto t0 = std::chrono::steady_clock::now();
        ValidationCounters &counter = result.counter;
        auto timed_out = [&]()
        {
            return settings.max_ns && vamp::utils::get_elapsed_nanoseconds(t0) >= settings.max_ns;
        };

        if (goals.empty())
        {
            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
            return result;
        }
        for (const auto &g : goals)
        {
            if (start == g && start_velocity == goal_velocity)
            {
                const auto hold = make_edge<n>(start, start_velocity, start, start_velocity, 1e-6);
                if (flask_cc<Robot>(hold, limits, env, true, &counter))
                {
                    result.solved = true;
                    result.trajectory.Q = {start};
                    result.trajectory.V = {start_velocity};
                    result.start_nodes = result.goal_nodes = 1;
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
                    return result;
                }
            }
        }

        for (const auto &g : goals)
        {
            if (timed_out())
                break;
            const auto c = bvp<n>(start, start_velocity, g, goal_velocity, settings.rho);
            if (c && flask_cc<Robot>(*c, limits, env, true, &counter))
            {
                result.solved = true;
                result.trajectory.Q = {start, g};
                result.trajectory.V = {start_velocity, goal_velocity};
                result.trajectory.T = {c->T};
                result.start_nodes = result.goal_nodes = 1;
                result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
                return result;
            }
        }

        Tree start_tree(true, settings.max_samples), goal_tree(false, settings.max_samples);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        start_tree.insert(start, start_velocity, 0, true, nan, settings.w);
        for (const auto &g : goals)
        {
            goal_tree.insert(g, goal_velocity, 0, true, nan, settings.w);
        }

        std::mt19937_64 rng(seed);
        std::uniform_real_distribution<double> U01(0.0, 1.0);

        Tree *a = &goal_tree, *b = &start_tree;
        std::size_t iterations = 0;
        while (iterations < settings.max_iterations &&
               start_tree.size() + goal_tree.size() < settings.max_samples)
        {
            ++iterations;
            if (timed_out())
            {
                break;
            }
            const double sa = static_cast<double>(a->size()), sb = static_cast<double>(b->size());
            if (!settings.balance || std::abs(sa - sb) / sa < settings.tree_ratio)
            {
                std::swap(a, b);
            }

            Vector<n> sample_q, sample_v;
            for (std::size_t j = 0; j < n; ++j)
            {
                sample_q[j] = limits.q_lo[j] + (limits.q_hi[j] - limits.q_lo[j]) * U01(rng);
            }
            for (std::size_t j = 0; j < n; ++j)
            {
                sample_v[j] =
                    (2.0 * U01(rng) - 1.0) * limits.v_max[j] * settings.velocity_sample_scale;
            }

            const auto [nearest_index, nearest_distance] = a->nearest(sample_q, sample_v, settings);
            if (settings.dynamic_domain && a->r[nearest_index] < nearest_distance)
            {
                ++result.dynamic_domain_rejections;
                continue;
            }
            double dist = 0.0;
            for (std::size_t j = 0; j < n; ++j)
            {
                const double d = sample_q[j] - a->Q[nearest_index][j];
                dist += d * d;
            }
            dist = std::sqrt(dist);
            Vector<n> target_q = sample_q;
            if (!(dist < settings.range))
            {
                for (std::size_t j = 0; j < n; ++j)
                {
                    target_q[j] = a->Q[nearest_index][j] +
                                  (sample_q[j] - a->Q[nearest_index][j]) * (settings.range / dist);
                }
            }
            auto c = a->connection(nearest_index, settings.prefix_steering ? sample_q : target_q,
                                   sample_v, settings.rho);
            bool accepted = false;
            if (settings.prefix_steering && c)
            {
                const auto whole = *c;
                const double length = whole.length();
                double fraction = length > settings.range ? settings.range / length : 1.;
                for (unsigned int attempt = 0; attempt < 7; ++attempt)
                {
                    c = a->is_start ? whole.slice(0., whole.T * fraction)
                                    : whole.slice(whole.T * (1. - fraction), whole.T);
                    if (flask_cc<Robot>(*c, limits, env, true, &counter))
                    {
                        target_q = a->is_start ? c->q1 : c->q0;
                        sample_v = a->is_start ? c->v1 : c->v0;
                        accepted =
                            target_q != a->Q[nearest_index] || sample_v != a->V[nearest_index];
                        break;
                    }
                    fraction *= .5;
                }
            }
            else if (c)
            {
                accepted = flask_cc<Robot>(*c, limits, env, true, &counter);
            }
            if (!accepted)
            {
                ++result.steering_failures;
                if (settings.dynamic_domain)
                {
                    a->r[nearest_index] =
                        std::isinf(a->r[nearest_index])
                            ? settings.radius
                            : std::max(a->r[nearest_index] * (1.0 - settings.alpha),
                                       settings.min_radius);
                }
                continue;
            }
            std::size_t new_index =
                a->insert(target_q, sample_v, nearest_index, false, c->T, settings.w);
            ++result.accepted_extensions;
            if (settings.dynamic_domain && !std::isinf(a->r[nearest_index]))
            {
                a->r[nearest_index] *= (1.0 + settings.alpha);
            }

            const std::size_t j = b->nearest(a->Q[new_index], a->V[new_index], settings).first;
            ++result.connection_attempts;
            const auto cn =
                a->is_start
                    ? bvp<n>(a->Q[new_index], a->V[new_index], b->Q[j], b->V[j], settings.rho)
                    : bvp<n>(b->Q[j], b->V[j], a->Q[new_index], a->V[new_index], settings.rho);

            bool connected =
                cn.has_value() || (a->Q[new_index] == b->Q[j] && a->V[new_index] == b->V[j]);
            if (cn)
            {
                double dq = 0.0;
                for (std::size_t jj = 0; jj < n; ++jj)
                {
                    const double d = b->Q[j][jj] - a->Q[new_index][jj];
                    dq += d * d;
                }
                const std::size_t m = std::max<std::size_t>(
                    static_cast<std::size_t>(std::ceil(std::sqrt(dq) / settings.range)), 1);
                std::size_t previous = new_index;
                for (std::size_t s = 0; s < m; ++s)
                {
                    if (timed_out())
                    {
                        connected = false;
                        break;
                    }
                    const std::size_t k = a->is_start ? s : m - 1 - s;
                    const double ta = cn->T * static_cast<double>(k) / static_cast<double>(m);
                    const double tb = cn->T * static_cast<double>(k + 1) / static_cast<double>(m);
                    const auto segment = cn->slice(ta, tb);
                    if (!flask_cc<Robot>(segment, limits, env, true, &counter) ||
                        start_tree.size() + goal_tree.size() >= settings.max_samples)
                    {
                        connected = false;
                        break;
                    }
                    previous = a->is_start ? a->insert(segment.q1, segment.v1, previous, false,
                                                       segment.T, settings.w)
                                           : a->insert(segment.q0, segment.v0, previous, false,
                                                       segment.T, settings.w);
                }
                new_index = previous;
            }
            if (connected)
            {
                result.trajectory = a->is_start ? join_trees(*a, new_index, *b, j)
                                                : join_trees(*b, j, *a, new_index);
                result.solved = true;
                break;
            }
        }
        result.iterations = iterations;
        result.start_nodes = start_tree.size();
        result.goal_nodes = goal_tree.size();
        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
        return result;
    }
};

template <typename Robot>
inline auto shortcut(const Trajectory<Robot::dimension> &input,
                     const JointLimits<Robot::dimension> &limits,
                     const VectorEnvironment<Robot> &env, double rho,
                     std::size_t *nanoseconds = nullptr) -> Trajectory<Robot::dimension>
{
    constexpr std::size_t n = Robot::dimension;
    const auto t0 = std::chrono::steady_clock::now();
    Trajectory<n> q = input;
    std::size_t a = 0;
    while (a + 2 < q.Q.size())
    {
        for (std::size_t b = q.Q.size() - 1; b > a + 1; --b)
        {
            const auto c = bvp<n>(q.Q[a], q.V[a], q.Q[b], q.V[b], rho);
            if (c && flask_cc<Robot>(*c, limits, env))
            {
                q.Q.erase(q.Q.begin() + a + 1, q.Q.begin() + b);
                q.V.erase(q.V.begin() + a + 1, q.V.begin() + b);
                q.T.erase(q.T.begin() + a, q.T.begin() + b);
                q.T.insert(q.T.begin() + a, c->T);
                break;
            }
        }
        ++a;
    }
    if (nanoseconds)
    {
        *nanoseconds = vamp::utils::get_elapsed_nanoseconds(t0);
    }
    return q;
}
} // namespace flask
