

#pragma once
#include "precise_vamp.hh"

#include <cmath>
#include <cstddef>

#include <vamp/collision/environment.hh>
#include <vamp/vector.hh>

#include "lqmt.hh"

namespace flask
{
inline constexpr std::size_t K = vamp::FloatVectorWidth;

template <typename Robot>
using VectorEnvironment = vamp::collision::Environment<vamp::FloatVector<K>>;

template <typename Robot>
inline auto sample_count(const Edge<Robot::dimension> &c) noexcept -> std::size_t
{
    const double batch_count = std::ceil(c.length() / static_cast<double>(K) * Robot::resolution);
    return static_cast<std::size_t>(std::max(batch_count, 1.0)) * K;
}

struct ValidationCounters
{
    std::size_t edge = 0, lower = 0, limit_rejections = 0, collision_rejections = 0,
                task_rejections = 0;
};

template <typename Robot>
inline auto flask_cc(const Edge<Robot::dimension> &c, const JointLimits<Robot::dimension> &g,
                     const VectorEnvironment<Robot> &env, bool check_limits = true,
                     ValidationCounters *counter = nullptr) noexcept -> bool
{
    constexpr std::size_t n = Robot::dimension;
    using V = vamp::FloatVector<K>;
    if (!std::isfinite(c.T) || c.T <= 0.)
    {
        return false;
    }
    if (counter)
    {
        ++counter->edge;
    }
    if (check_limits && !within_limits<n>(c, g))
    {
        if (counter)
        {
            ++counter->limit_rejections;
        }
        return false;
    }
    if (!std::isfinite(g.collision_resolution_scale) || g.collision_resolution_scale <= 0. ||
        !(g.max_validation_dt > 0.))
    {
        return false;
    }
    double requested = c.length() * Robot::resolution * g.collision_resolution_scale;
    if (g.bound_validation_speed)
    {
        double speed_squared = 0.;
        for (std::size_t j = 0; j < n; ++j)
        {
            double vmax = std::max(std::abs(c.v0[j]), std::abs(c.velocity(j, c.T)));
            const double h2 = (c.c2[j] * c.T) * c.T;
            const double h3 = ((c.c3[j] * c.T) * c.T) * c.T;
            if (h3 != 0.)
            {
                const double s = -h2 / (3. * h3);
                if (s > 0. && s < 1.)
                    vmax = std::max(vmax, std::abs(c.velocity(j, s * c.T)));
            }
            speed_squared += vmax * vmax;
        }
        requested = std::max(requested, c.T * std::sqrt(speed_squared) * Robot::resolution *
                                            g.collision_resolution_scale);
    }
    if (std::isfinite(g.max_validation_dt))
        requested = std::max(requested, std::ceil(c.T / g.max_validation_dt));
    if (g.certify_delta > 0.)
    {
        // Exact per-joint peak speed of the cubic (endpoints and interior vertex).
        double sweep = 0.;
        for (std::size_t j = 0; j < n; ++j)
        {
            double vmax = std::max(std::abs(c.v0[j]), std::abs(c.velocity(j, c.T)));
            const double h2 = (c.c2[j] * c.T) * c.T;
            const double h3 = ((c.c3[j] * c.T) * c.T) * c.T;
            if (h3 != 0.)
            {
                const double s = -h2 / (3. * h3);
                if (s > 0. && s < 1.)
                    vmax = std::max(vmax, std::abs(c.velocity(j, s * c.T)));
            }
            sweep += g.certify_reach[j] * vmax;
        }
        const double usable = g.certify_delta - g.certify_float_margin;
        if (!(usable > 0.))
            return false;
        // Samples at spacing h = T/N cover [t - h/2, t + h/2]; displacement <= sweep * h / 2 <= usable.
        requested = std::max(requested, std::ceil(c.T * sweep / (2. * usable)) + 1.);
    }
    const std::size_t N = std::max<std::size_t>(K, std::ceil(requested / K) * K);
    const std::size_t m = N / K;
    const double dt = c.T / static_cast<double>(N);
    const bool task_enabled =
        std::isfinite(g.max_tool_tilt_rad) || std::isfinite(g.max_tool_angular_speed);
    auto tool_valid = [&](double t)
    {
        std::array<float, n> q{}, plus{}, minus{};
        for (std::size_t j = 0; j < n; ++j)
        {
            q[j] = float(c.q(j, t));
            plus[j] = float(c.q(j, t) + .001 * c.velocity(j, t));
            minus[j] = float(c.q(j, t) - .001 * c.velocity(j, t));
        }
        const auto R = Robot::eefk(q).linear().eval();
        if (std::isfinite(g.max_tool_tilt_rad) && -R(2, 2) < std::cos(g.max_tool_tilt_rad) - 1e-6)
            return false;
        if (std::isfinite(g.max_tool_angular_speed))
        {
            const auto derivative =
                ((Robot::eefk(plus).linear() - Robot::eefk(minus).linear()) * 500.f).eval();
            const auto skew = (R.transpose() * derivative).eval();
            const Eigen::Vector3f omega((skew(2, 1) - skew(1, 2)) * .5f,
                                        (skew(0, 2) - skew(2, 0)) * .5f,
                                        (skew(1, 0) - skew(0, 1)) * .5f);
            if (omega.squaredNorm() > g.max_tool_angular_speed * g.max_tool_angular_speed + 1e-4)
                return false;
        }
        return true;
    };

    std::array<V, n> C3, C2, C1, C0;
    for (std::size_t j = 0; j < n; ++j)
    {
        C3[j] = V(static_cast<float>(c.c3[j]));
        C2[j] = V(static_cast<float>(c.c2[j]));
        C1[j] = V(static_cast<float>(c.v0[j]));
        C0[j] = V(static_cast<float>(c.q0[j]));
    }

    typename Robot::template ConfigurationBlock<K> block;

    for (std::size_t j = 0; j < n; ++j)
    {
        block[j] = V(static_cast<float>(c.q0[j]));
    }
    if (!(env.attachments ? Robot::template fkcc_attach<K>(env, block)
                          : Robot::template fkcc<K>(env, block)))
    {
        if (counter)
        {
            ++counter->collision_rejections;
        }
        return false;
    }
    if (task_enabled && !tool_valid(0.))
    {
        if (counter)
            ++counter->task_rejections;
        return false;
    }
    alignas(32) std::array<float, K> tt;
    for (std::size_t i = 0; i < m; ++i)
    {
        for (std::size_t k = 0; k < K; ++k)
        {
            tt[k] = static_cast<float>(dt * static_cast<double>(i + 1 + k * m));
        }
        const V t(tt);
        for (std::size_t j = 0; j < n; ++j)
        {
            block[j] = ((C3[j] * t + C2[j]) * t + C1[j]) * t + C0[j];
        }
        if (counter)
        {
            ++counter->lower;
        }
        const bool ok = env.attachments ? Robot::template fkcc_attach<K>(env, block)
                                        : Robot::template fkcc<K>(env, block);
        if (!ok)
        {
            if (counter)
            {
                ++counter->collision_rejections;
            }
            return false;
        }
        if (task_enabled)
            for (std::size_t k = 0; k < K; ++k)
                if (!tool_valid(tt[k]))
                {
                    if (counter)
                        ++counter->task_rejections;
                    return false;
                }
    }
    return true;
}
} // namespace flask
