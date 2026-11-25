

#pragma once
#include "polynomial.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace flask
{
template <std::size_t n> using Vector = std::array<double, n>;

inline constexpr std::array<double, 16> GL_X = {
    -0.9894009349916499, -0.9445750230732326,  -0.8656312023878318, -0.755404408355003,
    -0.6178762444026438, -0.45801677765722737, -0.2816035507792589, -0.09501250983763745,
    0.09501250983763745, 0.2816035507792589,   0.45801677765722737, 0.6178762444026438,
    0.755404408355003,   0.8656312023878318,   0.9445750230732326,  0.9894009349916499};
inline constexpr std::array<double, 16> GL_W = {
    0.027152459411754037, 0.062253523938647706, 0.09515851168249259,  0.12462897125553403,
    0.14959598881657676,  0.16915651939500262,  0.1826034150449236,   0.18945061045506859,
    0.18945061045506859,  0.1826034150449236,   0.16915651939500262,  0.14959598881657676,
    0.12462897125553403,  0.09515851168249259,  0.062253523938647706, 0.027152459411754037};

struct CostCoefficients
{
    double a, b, c;
};

template <std::size_t n>
inline auto cost_coefficients(const Vector<n> &q0, const Vector<n> &v0, const Vector<n> &q1,
                              const Vector<n> &v1) noexcept -> CostCoefficients
{
    CostCoefficients r{0.0, 0.0, 0.0};
    for (std::size_t j = 0; j < n; ++j)
    {
        const double dq = q1[j] - q0[j];
        r.a += dq * dq;
        r.b += (v1[j] + v0[j]) * dq;
        r.c += v0[j] * v0[j] + v0[j] * v1[j] + v1[j] * v1[j];
    }
    return r;
}

inline auto cost(double T, const CostCoefficients &h, double rho) noexcept -> double
{
    return 12.0 * h.a / (T * T * T) - 12.0 * h.b / (T * T) + 4.0 * h.c / T + rho * T;
}

// Positive decomposition of the acceleration integral. The monomial
// expression loses all significant digits for a short near-drift edge.
template <std::size_t n>
inline double boundary_cost(const Vector<n> &q0, const Vector<n> &v0, const Vector<n> &q1,
                            const Vector<n> &v1, double T, double rho) noexcept
{
    if (!(T > 0.) || !std::isfinite(T))
        return std::numeric_limits<double>::infinity();
    double energy = 0.;
    for (std::size_t j = 0; j < n; ++j)
    {
        double residual = ((q1[j] - q0[j]) - .5 * (v0[j] + v1[j]) * T) / T;
        double difference = v1[j] - v0[j];
        energy += (12. * residual * residual + difference * difference) / T;
    }
    return energy + rho * T;
}

template <std::size_t n>
inline auto optimal_duration(const Vector<n> &q0, const Vector<n> &v0, const Vector<n> &q1,
                             const Vector<n> &v1, double rho) noexcept -> std::optional<double>
{
    if (!std::isfinite(rho) || rho <= 0.)
        return std::nullopt;
    double displacement = 0., velocity = 0.;
    for (std::size_t j = 0; j < n; ++j)
    {
        if (!std::isfinite(q0[j]) || !std::isfinite(q1[j]) || !std::isfinite(v0[j]) ||
            !std::isfinite(v1[j]))
            return std::nullopt;
        displacement = std::hypot(displacement, q1[j] - q0[j]);
        velocity = std::hypot(velocity, std::sqrt(.75) * (v0[j] + v1[j]));
        velocity = std::hypot(velocity, .5 * (v1[j] - v0[j]));
    }
    const double root_rho = std::sqrt(rho);
    if (displacement == 0.)
        return velocity == 0. ? std::nullopt : std::optional<double>{2. * velocity / root_rho};
    const double scale =
        std::max(std::sqrt(6. * (displacement / root_rho)), 2. * velocity / root_rho);
    if (!(scale > 0.) || !std::isfinite(scale))
        return std::nullopt;
    Vector<n> zero{}, position{}, initial{}, terminal{};
    for (std::size_t j = 0; j < n; ++j)
    {
        position[j] = ((q1[j] - q0[j]) / root_rho) / scale / scale;
        initial[j] = (v0[j] / root_rho) / scale;
        terminal[j] = (v1[j] / root_rho) / scale;
    }
    const auto h = cost_coefficients<n>(zero, initial, position, terminal);
    std::vector<double> coefficients{-36. * h.a, 24. * h.b, -4. * h.c, 0., 1.};
    double upper = 1. + std::max({36. * h.a, 24. * std::abs(h.b), 4. * h.c});
    auto roots = polynomial::roots_between(coefficients, 0., upper);
    std::vector<double> candidates;
    for (double root : roots)
        candidates.push_back(root * scale);
    // A drift-aligned feasible duration provides a rounding-level upper
    // bound when its stationary root is closer than floating precision.
    for (std::size_t j = 0; j < n; ++j)
    {
        double mean = .5 * (v0[j] + v1[j]);
        if (mean != 0.)
        {
            double T = (q1[j] - q0[j]) / mean;
            if (T > 0. && std::isfinite(T))
                candidates.push_back(T);
        }
    }
    double best = std::numeric_limits<double>::infinity(), duration = 0.;
    for (double T : candidates)
    {
        double value = boundary_cost<n>(q0, v0, q1, v1, T, rho);
        if (value < best)
        {
            best = value;
            duration = T;
        }
    }
    return duration > 0. ? std::optional<double>{duration} : std::nullopt;
}

template <std::size_t n> struct Edge
{
    Vector<n> q0, v0, q1, v1;
    double T;
    Vector<n> c3, c2;

    inline void compute_coefficients() noexcept
    {
        const double T2 = T * T, T3 = T2 * T;
        for (std::size_t j = 0; j < n; ++j)
        {
            const double dp = q1[j] - q0[j] - T * v0[j];
            const double dv = v1[j] - v0[j];
            c3[j] = -2.0 * dp / T3 + dv / T2;
            c2[j] = 3.0 * dp / T2 - dv / T;
        }
    }

    inline auto q(std::size_t j, double t) const noexcept -> double
    {
        return ((c3[j] * t + c2[j]) * t + v0[j]) * t + q0[j];
    }

    inline auto velocity(std::size_t j, double t) const noexcept -> double
    {
        return (3.0 * c3[j] * t + 2.0 * c2[j]) * t + v0[j];
    }

    inline auto length() const noexcept -> double
    {
        double s = 0.0;
        for (std::size_t k = 0; k < 16; ++k)
        {
            const double t = 0.5 * T * (GL_X[k] + 1.0);
            double v2 = 0.0;
            for (std::size_t j = 0; j < n; ++j)
            {
                const double v = velocity(j, t);
                v2 += v * v;
            }
            s += GL_W[k] * std::sqrt(v2);
        }
        return 0.5 * T * s;
    }

    inline auto slice(double ta, double tb) const noexcept -> Edge
    {
        Edge r;
        for (std::size_t j = 0; j < n; ++j)
        {
            r.q0[j] = q(j, ta);
            r.v0[j] = velocity(j, ta);
            r.q1[j] = q(j, tb);
            r.v1[j] = velocity(j, tb);
        }
        r.T = tb - ta;
        r.compute_coefficients();
        return r;
    }
};

template <std::size_t n>
inline auto make_edge(const Vector<n> &q0, const Vector<n> &v0, const Vector<n> &q1,
                      const Vector<n> &v1, double T) noexcept -> Edge<n>
{
    Edge<n> c{q0, v0, q1, v1, T, {}, {}};
    c.compute_coefficients();
    return c;
}

template <std::size_t n>
inline auto bvp(const Vector<n> &q0, const Vector<n> &v0, const Vector<n> &q1, const Vector<n> &v1,
                double rho) noexcept -> std::optional<Edge<n>>
{
    const auto T = optimal_duration<n>(q0, v0, q1, v1, rho);
    if (!T)
    {
        return std::nullopt;
    }
    return make_edge<n>(q0, v0, q1, v1, *T);
}

template <std::size_t n> struct JointLimits
{
    Vector<n> q_lo, q_hi, v_max, a_max;
    double collision_resolution_scale = 1.;
    double max_validation_dt = std::numeric_limits<double>::infinity();
    bool bound_validation_speed = false;
    double max_tool_tilt_rad = std::numeric_limits<double>::infinity();
    double max_tool_angular_speed = std::numeric_limits<double>::infinity();
    // Certified environment checking (certify_delta > 0). certify_reach[j] bounds the distance from
    // joint j's axis to every collision-sphere centre downstream of joint j, so a centre moves at most
    // sum_j reach_j |qdot_j| per unit time. Samples are spaced so that each one certifies the time
    // interval around it when the ENVIRONMENT passed to flask_cc is inflated by certify_delta
    // (minus certify_float_margin for single-precision geometry). Self-collision remains sampled.
    Vector<n> certify_reach{};
    double certify_delta = 0.;
    double certify_float_margin = 1e-4;
};

template <std::size_t n>
inline auto within_limits(const Edge<n> &c, const JointLimits<n> &g) noexcept -> bool
{
    const double T = c.T;
    if (!std::isfinite(T) || T <= 0.)
    {
        return false;
    }
    for (std::size_t j = 0; j < n; ++j)
    {
        if (!std::isfinite(c.q0[j]) || !std::isfinite(c.q1[j]) || !std::isfinite(c.v0[j]) ||
            !std::isfinite(c.v1[j]) || !std::isfinite(c.c3[j]) || !std::isfinite(c.c2[j]) ||
            !std::isfinite(g.q_lo[j]) || !std::isfinite(g.q_hi[j]) || !std::isfinite(g.v_max[j]) ||
            !std::isfinite(g.a_max[j]) || g.q_lo[j] > g.q_hi[j] || g.v_max[j] < 0. ||
            g.a_max[j] < 0.)
        {
            return false;
        }
        const double c3 = c.c3[j], c2 = c.c2[j], c1 = c.v0[j];

        const double a0 = 2.0 * c2, aT = 6.0 * c3 * T + 2.0 * c2;
        if (std::abs(a0) > g.a_max[j] || std::abs(aT) > g.a_max[j])
        {
            return false;
        }

        // Find extrema in normalized time s=t/T. An absolute coefficient
        // threshold in physical time can discard a genuine cubic when T is
        // large, even if the position excursion violates joint limits.
        const double h1 = c1 * T, h2 = (c2 * T) * T, h3 = ((c3 * T) * T) * T;
        if (!std::isfinite(h1) || !std::isfinite(h2) || !std::isfinite(h3))
            return false;
        double vmax = std::max(std::abs(c1), std::abs(c.velocity(j, T)));
        if (h3 != 0.)
        {
            const double s = -h2 / (3.0 * h3);
            if (s > 0.0 && s < 1.)
            {
                vmax = std::max(vmax, std::abs(c.velocity(j, s * T)));
            }
        }
        if (vmax > g.v_max[j])
        {
            return false;
        }

        double qmin = std::min(c.q0[j], c.q(j, T)), qmax = std::max(c.q0[j], c.q(j, T));
        const double scale = std::max({std::abs(h1), std::abs(h2), std::abs(h3)});
        const double A = scale > 0. ? 3. * (h3 / scale) : 0.;
        const double B = scale > 0. ? 2. * (h2 / scale) : 0.;
        const double C = scale > 0. ? h1 / scale : 0.;
        auto check_position_extremum = [&](double r)
        {
            if (r > 0.0 && r < 1.)
            {
                const double x = c.q(j, r * T);
                qmin = std::min(qmin, x);
                qmax = std::max(qmax, x);
            }
        };
        if (A != 0.)
        {
            const double D = B * B - 4.0 * A * C;
            if (D >= 0.0)
            {
                // Stable quadratic roots avoid cancellation of one root.
                const double q = -.5 * (B + std::copysign(std::sqrt(D), B));
                check_position_extremum(q / A);
                if (q != 0.)
                    check_position_extremum(C / q);
            }
        }
        else if (B != 0.)
        {
            check_position_extremum(-C / B);
        }
        if (qmin < g.q_lo[j] || qmax > g.q_hi[j])
        {
            return false;
        }
    }
    return true;
}
} // namespace flask
