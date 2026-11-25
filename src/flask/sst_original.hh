#pragma once
// First-party original-coordinate SST* reconstruction; not the upstream Table I baseline.
//
// Propagation-based kinodynamic planning in the ORIGINAL state space with ORIGINAL controls, explicit
// Euler integration (dt = 0.01 s, as DynoBench), scalar (non-SIMD) per-state collision checking, the
// same sphere geometry, workspace, state/control limits and DynoBench goal regions as the FLASK
// variants, SST best-near selection, witness pruning and the SST* shrinking schedule (Li, Littlefield,
// Bekris 2016). This is a supplementary ablation, not an original Dynoplan/OMPL measurement.
//
// Models (paper Sec. VII-A):
//   unicycle  x=(x,y,theta)               u=(v,omega), |v|<=1, |omega|<=1.5
//   quad2d    x=(x,z,theta,vx,vz,omega)   u=(f1,f2) in [0, 0.65 m g], m=0.034, J=1e-4, l=0.1
//   quad3d    x=(p, q_xyzw, v, omega)     u=(f,tau), 0<=f<=1.5 m g, |tau|<=2, m=1, J=diag(.1,.1,.2),
//                                         |v|<=4, |omega|<=8

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <vamp/collision/environment.hh>

#include "flat_scalar.hh"  // scalar_environment_collision: same geometry equations as the SIMD checker

namespace flask::orig
{
struct Settings
{
    double dt = .01;
    int min_steps = 2, max_steps = 20;
    double goal_epsilon = .2, goal_bias = .1;
    double selection_radius = 1., pruning_radius = .1, shrink = .8;
    std::size_t phase_iterations = 1000;
    std::size_t max_iterations = 1000000000;
    std::uint64_t budget_ns = 1000000000;
    std::size_t max_nodes = 1000000;
    bool stop_first = true;
    bool trace = false;
    double velocity_sample = 2., rate_sample = 2.;  // state-sampling bounds for selection targets
};

struct World
{
    const vamp::collision::Environment<float> *environment;
    std::array<double, 3> lower{}, upper{};
    int dimensions = 2;
    double radius = .1;
    bool position_valid(double x, double y, double z) const
    {
        const double p[3] = {x, y, z};
        for (int j = 0; j < dimensions; ++j)
            if (!(p[j] >= lower[j] && p[j] <= upper[j]))
                return false;
        return !flask::flat::scalar_environment_collision(*environment, float(x), float(y), float(z),
                                                           float(radius));
    }
};

inline double wrap(double a)
{
    return std::remainder(a, 2 * M_PI);
}

struct Unicycle
{
    static constexpr int nx = 3, nu = 2;
    static constexpr int state_dimension = 3;
    using X = std::array<double, nx>;
    using U = std::array<double, nu>;
    template <class G> static X sample_state(G &g, const World &w, const Settings &)
    {
        std::uniform_real_distribution<double> u(0., 1.);
        return {w.lower[0] + u(g) * (w.upper[0] - w.lower[0]), w.lower[1] + u(g) * (w.upper[1] - w.lower[1]),
                (2 * u(g) - 1) * M_PI};
    }
    template <class G> static U sample_control(G &g)
    {
        std::uniform_real_distribution<double> u(-1., 1.);
        return {u(g) * 1., u(g) * 1.5};
    }
    static X step(const X &x, const U &u, double dt)
    {
        return {x[0] + dt * u[0] * std::cos(x[2]), x[1] + dt * u[0] * std::sin(x[2]), wrap(x[2] + dt * u[1])};
    }
    static bool valid(const X &x, const World &w)
    {
        return w.position_valid(x[0], x[1], 0.);
    }
    static double distance(const X &a, const X &b)  // DynoBench unicycle1 weights [1, .5]
    {
        return std::hypot(a[0] - b[0], a[1] - b[1]) + .5 * std::abs(wrap(a[2] - b[2]));
    }
};

struct Quad2D
{
    static constexpr int nx = 6, nu = 2;
    static constexpr int state_dimension = 6;
    static constexpr double m = .034, J = 1e-4, l = .1, g = 9.81, fmax = .65 * m * g;
    using X = std::array<double, nx>;
    using U = std::array<double, nu>;
    template <class G> static X sample_state(G &gen, const World &w, const Settings &s)
    {
        std::uniform_real_distribution<double> u(0., 1.);
        return {w.lower[0] + u(gen) * (w.upper[0] - w.lower[0]), w.lower[1] + u(gen) * (w.upper[1] - w.lower[1]),
                (2 * u(gen) - 1) * M_PI, (2 * u(gen) - 1) * s.velocity_sample, (2 * u(gen) - 1) * s.velocity_sample,
                (2 * u(gen) - 1) * s.rate_sample};
    }
    template <class G> static U sample_control(G &gen)
    {
        std::uniform_real_distribution<double> u(0., fmax);
        return {u(gen), u(gen)};
    }
    static X step(const X &x, const U &u, double dt)
    {
        const double f = u[0] + u[1];
        return {x[0] + dt * x[3], x[1] + dt * x[4], wrap(x[2] + dt * x[5]),
                x[3] + dt * (-f * std::sin(x[2]) / m), x[4] + dt * (f * std::cos(x[2]) / m - g),
                x[5] + dt * (l * (u[0] - u[1]) / J)};
    }
    static bool valid(const X &x, const World &w)
    {
        return w.position_valid(x[0], x[1], 0.);
    }
    static double distance(const X &a, const X &b)  // DynoBench quad2d weights [1, .5, .2, .2]
    {
        return std::hypot(a[0] - b[0], a[1] - b[1]) + .5 * std::abs(wrap(a[2] - b[2])) +
               .2 * std::hypot(a[3] - b[3], a[4] - b[4]) + .2 * std::abs(a[5] - b[5]);
    }
};

struct Quad3D
{
    static constexpr int nx = 13, nu = 4;
    static constexpr int state_dimension = 12;  // quaternion storage has one unit-norm constraint
    static constexpr double m = 1., g = 9.81, fmax = 1.5 * m * g, tmax = 2., vmax = 4., wmax = 8.;
    static constexpr double Jx = .1, Jy = .1, Jz = .2;
    using X = std::array<double, nx>;  // p(3) q_xyzw(4) v(3) omega_body(3)
    using U = std::array<double, nu>;
    template <class G> static X sample_state(G &gen, const World &w, const Settings &s)
    {
        std::uniform_real_distribution<double> u(0., 1.);
        std::normal_distribution<double> n(0., 1.);
        X x{};
        for (int j = 0; j < 3; ++j)
            x[j] = w.lower[j] + u(gen) * (w.upper[j] - w.lower[j]);
        double qn = 0.;
        for (int j = 3; j < 7; ++j)
        {
            x[j] = n(gen);
            qn += x[j] * x[j];
        }
        for (int j = 3; j < 7; ++j)
            x[j] /= std::sqrt(qn);
        for (int j = 7; j < 10; ++j)
            x[j] = (2 * u(gen) - 1) * s.velocity_sample;
        for (int j = 10; j < 13; ++j)
            x[j] = (2 * u(gen) - 1) * s.rate_sample;
        return x;
    }
    template <class G> static U sample_control(G &gen)
    {
        std::uniform_real_distribution<double> u(0., 1.);
        std::normal_distribution<double> n(0., 1.);
        double t[3] = {n(gen), n(gen), n(gen)}, tn = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
        const double r = tmax * std::cbrt(u(gen));  // uniform in the torque ball
        return {u(gen) * fmax, r * t[0] / tn, r * t[1] / tn, r * t[2] / tn};
    }
    static X step(const X &x, const U &u, double dt)
    {
        const double qx = x[3], qy = x[4], qz = x[5], qw = x[6];
        // body z axis in world frame (third column of R(q))
        const double zx = 2 * (qx * qz + qw * qy), zy = 2 * (qy * qz - qw * qx), zz = 1 - 2 * (qx * qx + qy * qy);
        const double wx = x[10], wy = x[11], wz = x[12];
        X y = x;
        for (int j = 0; j < 3; ++j)
            y[j] = x[j] + dt * x[7 + j];
        // q_dot = 0.5 q (x) (omega, 0), xyzw convention
        y[3] = qx + dt * .5 * (qw * wx + qy * wz - qz * wy);
        y[4] = qy + dt * .5 * (qw * wy + qz * wx - qx * wz);
        y[5] = qz + dt * .5 * (qw * wz + qx * wy - qy * wx);
        y[6] = qw + dt * .5 * (-qx * wx - qy * wy - qz * wz);
        const double qn = std::sqrt(y[3] * y[3] + y[4] * y[4] + y[5] * y[5] + y[6] * y[6]);
        for (int j = 3; j < 7; ++j)
            y[j] /= qn;
        y[7] = x[7] + dt * (u[0] * zx / m);
        y[8] = x[8] + dt * (u[0] * zy / m);
        y[9] = x[9] + dt * (u[0] * zz / m - g);
        y[10] = wx + dt * ((u[1] - (wy * Jz * wz - wz * Jy * wy)) / Jx);
        y[11] = wy + dt * ((u[2] - (wz * Jx * wx - wx * Jz * wz)) / Jy);
        y[12] = wz + dt * ((u[3] - (wx * Jy * wy - wy * Jx * wx)) / Jz);
        return y;
    }
    static bool valid(const X &x, const World &w)
    {
        if (x[7] * x[7] + x[8] * x[8] + x[9] * x[9] > vmax * vmax ||
            x[10] * x[10] + x[11] * x[11] + x[12] * x[12] > wmax * wmax)
            return false;
        return w.position_valid(x[0], x[1], x[2]);
    }
    static double distance(const X &a, const X &b)  // DynoBench quad3d weights [1, .5, .1, .05]
    {
        double dq = std::abs(a[3] * b[3] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6]);
        const double angle = dq > 1. - 1e-5 ? 0. : std::acos(dq);
        return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2)) +
               .5 * angle +
               .1 * std::sqrt(std::pow(a[7] - b[7], 2) + std::pow(a[8] - b[8], 2) + std::pow(a[9] - b[9], 2)) +
               .05 * std::sqrt(std::pow(a[10] - b[10], 2) + std::pow(a[11] - b[11], 2) +
                               std::pow(a[12] - b[12], 2));
    }
};

template <class M> struct Result
{
    bool solved = false;
    std::uint64_t first_ns = 0, plan_ns = 0;
    std::size_t iterations = 0, nodes = 0, propagation_steps = 0, invalid_propagations = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<typename M::X> states;   // node states from the root
    std::vector<typename M::U> controls; // control held on the edge into states[i + 1]
    std::vector<int> steps;              // Euler steps of that edge
    std::vector<std::pair<std::uint64_t, double>> improvements;  // (ns, cost) anytime trace
    std::vector<typename M::X> node_states, witnesses;
    std::vector<std::size_t> parents, children, representatives, phase_restarts;
    std::vector<double> costs;
    std::vector<bool> active, alive;
    double final_selection_radius = 0., final_pruning_radius = 0.;
};

template <class M>
Result<M> solve(const typename M::X &start, const typename M::X &goal, const World &world,
                const Settings &cfg, std::uint64_t seed)
{
    using X = typename M::X;
    using U = typename M::U;
    struct Node
    {
        X x;
        std::size_t parent;
        U u;
        int steps;
        double cost;
        bool active = true, alive = true;
        std::size_t children = 0;
    };
    const auto t0 = std::chrono::steady_clock::now();
    auto now = [&] { return std::uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count()); };
    Result<M> out;
    if (!M::valid(start, world))
    {
        out.plan_ns = now();
        return out;
    }
    if (M::distance(start, goal) <= cfg.goal_epsilon)
    {
        out.solved = true;
        out.best_cost = 0.;
        out.states = {start};
        out.nodes = 1;
        out.first_ns = out.plan_ns = now();
        if (cfg.trace)
        {
            out.node_states = out.witnesses = {start};
            out.parents = out.children = out.representatives = {0};
            out.costs = {0.};
            out.active = out.alive = {true};
            out.final_selection_radius = cfg.selection_radius;
            out.final_pruning_radius = cfg.pruning_radius;
        }
        return out;
    }
    std::mt19937_64 gen(seed);
    std::uniform_real_distribution<double> u01(0., 1.);
    std::uniform_int_distribution<int> steps_dist(cfg.min_steps, cfg.max_steps);
    std::vector<Node> tree{{start, 0, {}, 0, 0.}};
    std::vector<X> witnesses{start};
    std::vector<std::size_t> representative{0};
    // The incumbent owns its state/control sequence independently of search
    // nodes, which may be replaced, pruned or cleared at the next SST* phase.
    auto save_incumbent = [&](std::size_t goal_index)
    {
        std::vector<std::size_t> chain;
        for (std::size_t k = goal_index;; k = tree[k].parent)
        {
            chain.push_back(k);
            if (k == 0)
                break;
        }
        out.states.clear();
        out.controls.clear();
        out.steps.clear();
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        {
            out.states.push_back(tree[*it].x);
            if (it != chain.rbegin())
            {
                out.controls.push_back(tree[*it].u);
                out.steps.push_back(tree[*it].steps);
            }
        }
    };
    double selection = cfg.selection_radius, pruning = cfg.pruning_radius;
    std::size_t phase = 0, phase_end = cfg.phase_iterations;
    constexpr double dimension_term = M::state_dimension + M::nu + 1;
    for (std::size_t it = 0;; ++it)
    {
        if ((cfg.stop_first && out.solved) || it >= cfg.max_iterations || now() >= cfg.budget_ns || tree.size() >= cfg.max_nodes)
            break;
        out.iterations = it + 1;
        if (it == phase_end)  // SST* phase: shrink radii, restart witnesses/representatives
        {
            ++phase;
            selection *= cfg.shrink;
            pruning *= cfg.shrink;
            const double count = std::ceil(double(cfg.phase_iterations) *
                                           std::pow(std::pow(cfg.shrink, -dimension_term), phase) *
                                           (1 + std::log(double(phase))));
            const auto remaining = std::numeric_limits<std::size_t>::max() - phase_end;
            phase_end += !std::isfinite(count) || count >= double(remaining) ? remaining : std::size_t(count);
            // Algorithm 9 invokes a fresh SST phase. Active representatives and
            // witnesses must be initialized together; retaining the old active
            // tree while deleting its witnesses violates the graph invariant.
            tree.assign(1, Node{start, 0, {}, 0, 0.});
            witnesses.assign(1, start);
            representative.assign(1, 0);
            out.phase_restarts.push_back(it);
        }
        const X target = u01(gen) < cfg.goal_bias ? goal : M::sample_state(gen, world, cfg);
        // best-near: lowest cost active node within the selection radius, else nearest active node
        std::size_t nearest = 0, best = std::size_t(-1);
        double nd = std::numeric_limits<double>::infinity(), bc = nd;
        for (std::size_t i = 0; i < tree.size(); ++i)
        {
            if (!tree[i].alive || !tree[i].active)
                continue;
            const double d = M::distance(tree[i].x, target);
            if (d < nd)
            {
                nd = d;
                nearest = i;
            }
            if (d <= selection && tree[i].cost < bc)
            {
                bc = tree[i].cost;
                best = i;
            }
        }
        const std::size_t parent = best == std::size_t(-1) ? nearest : best;
        const U control = M::sample_control(gen);
        const int steps = steps_dist(gen);
        X x = tree[parent].x;
        bool ok = true;
        for (int k = 0; k < steps && ok; ++k)
        {
            x = M::step(x, control, cfg.dt);
            ++out.propagation_steps;
            ok = M::valid(x, world);
        }
        if (!ok)
        {
            ++out.invalid_propagations;
            continue;
        }
        const double cost = tree[parent].cost + steps * cfg.dt;  // DynoBench SST cost: time
        std::size_t wi = 0;
        double wd = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < witnesses.size(); ++i)
        {
            const double d = M::distance(x, witnesses[i]);
            if (d < wd)
            {
                wd = d;
                wi = i;
            }
        }
        if (wd > pruning)
        {
            witnesses.push_back(x);
            representative.push_back(std::size_t(-1));
            wi = witnesses.size() - 1;
        }
        std::size_t old = representative[wi];
        if (old != std::size_t(-1) && tree[old].alive && tree[old].cost <= cost)
            continue;
        tree[parent].children++;
        tree.push_back({x, parent, control, steps, cost});
        const std::size_t j = tree.size() - 1;
        if (old != std::size_t(-1) && tree[old].alive)
        {
            tree[old].active = false;
            while (old != 0 && !tree[old].active && tree[old].children == 0)
            {
                tree[old].alive = false;
                const std::size_t p = tree[old].parent;
                --tree[p].children;
                old = p;
            }
        }
        representative[wi] = j;
        if (M::distance(x, goal) <= cfg.goal_epsilon && cost < out.best_cost)
        {
            if (!out.solved)
                out.first_ns = now();
            out.solved = true;
            out.best_cost = cost;
            save_incumbent(j);
            out.improvements.emplace_back(now(), cost);
        }
    }
    out.plan_ns = now();
    out.nodes = tree.size();
    if (cfg.trace)
    {
        for (const auto &node : tree)
        {
            out.node_states.push_back(node.x);
            out.parents.push_back(node.parent);
            out.children.push_back(node.children);
            out.costs.push_back(node.cost);
            out.active.push_back(node.active);
            out.alive.push_back(node.alive);
        }
        out.witnesses = witnesses;
        out.representatives = representative;
        out.final_selection_radius = selection;
        out.final_pruning_radius = pruning;
    }
    return out;
}
}  // namespace flask::orig
