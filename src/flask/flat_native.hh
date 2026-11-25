// FLASK integrator-chain steering, SIMD recovery, and low-DOF planners.
#pragma once
#include "precise_vamp.hh"

#include "flat_scalar.hh"
#include "lqmt.hh"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vamp/collision/validity.hh>
#include <vamp/planning/nn.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vector>
namespace flask::flat
{
inline double factorial(int n)
{
    double r = 1;
    for (int i = 2; i <= n; ++i)
        r *= i;
    return r;
}
inline double peval(const std::vector<double> &coefficients, double x)
{
    return polynomial::evaluate(coefficients, x);
}
inline std::vector<double> roots_between(std::vector<double> coefficients, double lower,
                                         double upper)
{
    return polynomial::roots_between(std::move(coefficients), lower, upper);
}
template <int R, int N> using State = std::array<double, R * N>;
// Stable aligned storage is necessary because Nigh retains pointers to keys.
// Padded SIMD lanes are initialized to zero. The search uses float keys, as in
// VAMP; the trajectory itself retains double-precision boundary states.
template <int R, int N> class NearestIndex
{
    static constexpr size_t dimensions = R * N;
    using Key = vamp::planning::NNFloatArray<dimensions>;
    static constexpr size_t stride = vamp::FloatVector<dimensions>::num_scalars_rounded;
    std::unique_ptr<float, decltype(&free)> storage;
    vamp::planning::NN<dimensions> index;
    size_t capacity;
    double derivative_weight;
    void encode(float *destination, const State<R, N> &state) const
    {
        for (size_t j = 0; j < stride; ++j)
            destination[j] = 0.f;
        for (size_t j = 0; j < dimensions; ++j)
        {
            double weight = j < N ? 1. : derivative_weight / std::pow(3., j / N - 1);
            destination[j] = float(std::sqrt(weight) * state[j]);
        }
    }

  public:
    NearestIndex(size_t maximum_nodes, double weight)
        : storage(
              vamp::utils::vector_alloc<float, vamp::FloatVectorAlignment, vamp::FloatVectorWidth>(
                  maximum_nodes * stride),
              &free),
          capacity(maximum_nodes), derivative_weight(weight)
    {
        if (!storage)
            throw std::bad_alloc();
    }
    void insert(size_t node, const State<R, N> &state)
    {
        if (node >= capacity)
            throw std::out_of_range("Nearest-neighbor index capacity exceeded");
        float *values = storage.get() + node * stride;
        encode(values, state);
        index.insert(vamp::planning::NNNode<dimensions>{node, Key{values}});
    }
    size_t nearest(const State<R, N> &state) const
    {
        alignas(vamp::FloatVectorAlignment) std::array<float, stride> values{};
        encode(values.data(), state);
        auto result = index.nearest(Key{values.data()});
        if (!result)
            throw std::logic_error("Nearest-neighbor query on an empty index");
        return result->first.index;
    }
};
template <int R, int N> struct Edge
{
    std::array<double, 2 * R * N> c{};
    double T = 0;
    template <class V> V derivative(int d, int j, V s) const
    {
        V y(0.f);
        for (int k = 2 * R - 1; k >= d; --k)
            y = y * s + V(float(c[k * N + j] * factorial(k) / factorial(k - d) / std::pow(T, d)));
        return y;
    }
    double value(int d, int j, double t) const
    {
        double y = 0, s = t / T;
        for (int k = 2 * R - 1; k >= d; --k)
            y = y * s + c[k * N + j] * factorial(k) / factorial(k - d) / std::pow(T, d);
        return y;
    }
    State<R, N> state(double t) const
    {
        State<R, N> z{};
        for (int d = 0; d < R; ++d)
            for (int j = 0; j < N; ++j)
                z[d * N + j] = value(d, j, t);
        return z;
    }
    double effort() const
    {
        // The pseudo-control has degree R-1, so R-point Gauss quadrature is exact
        // for its squared norm. Positive weights avoid a cancellation-prone sum
        // of signed monomial cross terms. The Python oracle uses a Gramian.
        static_assert(R == 2 || R == 4);
        double sum = 0.;
        for (int k = 0; k < R; ++k)
        {
            double x, w;
            if constexpr (R == 2)
            {
                x = k == 0 ? -.57735026918962576451 : .57735026918962576451;
                w = 1.;
            }
            else
            {
                constexpr double xs[]{-.86113631159405257522, -.33998104358485626480,
                                      .33998104358485626480, .86113631159405257522};
                constexpr double ws[]{.34785484513745385737, .65214515486254614263,
                                      .65214515486254614263, .34785484513745385737};
                x = xs[k];
                w = ws[k];
            }
            double norm2 = 0.;
            for (int j = 0; j < N; ++j)
            {
                double control = value(R, j, .5 * T * (x + 1.));
                norm2 += control * control;
            }
            sum += w * norm2;
        }
        return .5 * T * sum;
    }
    double cost(double rho) const
    {
        return effort() + rho * T;
    }
    double length(int dimensions = N) const
    {
        double sum = 0;
        for (int k = 0; k < 16; ++k)
        {
            double vv = 0, t = .5 * T * (GL_X[k] + 1);
            for (int j = 0; j < dimensions; ++j)
            {
                double v = value(1, j, t);
                vv += v * v;
            }
            sum += GL_W[k] * std::sqrt(vv);
        }
        return .5 * T * sum;
    }
};
template <int R, int N> Edge<R, N> fixed(const State<R, N> &a, const State<R, N> &b, double T)
{
    static const std::array<double, R * R> inverse = []()
    {
        if constexpr (R == 2)
            return std::array<double, 4>{3, -1, -2, 1};
        else
            return std::array<double, 16>{35, -15, 2.5, -1. / 6, -84, 39, -7, .5,
                                          70, -34, 6.5, -.5,     -20, 10, -2, 1. / 6};
    }();
    Edge<R, N> e;
    e.T = T;
    for (int k = 0; k < R; ++k)
        for (int j = 0; j < N; ++j)
            e.c[k * N + j] = a[k * N + j] * std::pow(T, k) / factorial(k);
    for (int j = 0; j < N; ++j)
    {
        Eigen::Matrix<double, R, 1> rhs;
        for (int d = 0; d < R; ++d)
        {
            rhs[d] = b[d * N + j] * std::pow(T, d);
            for (int k = d; k < R; ++k)
                rhs[d] -= e.c[k * N + j] * factorial(k) / factorial(k - d);
        }
        for (int k = R; k < 2 * R; ++k)
        {
            double high = 0;
            for (int d = 0; d < R; ++d)
                high += inverse[(k - R) * R + d] * rhs[d];
            e.c[k * N + j] = high;
        }
    }
    return e;
}
template <int R, int N> Edge<R, N> bvp(const State<R, N> &a, const State<R, N> &b, double rho = 1.)
{
    if constexpr (R == 2)
    {
        Vector<N> q0{}, v0{}, q1{}, v1{};
        for (int j = 0; j < N; ++j)
        {
            q0[j] = a[j];
            v0[j] = a[N + j];
            q1[j] = b[j];
            v1[j] = b[N + j];
        }
        auto duration = flask::optimal_duration<N>(q0, v0, q1, v1, rho);
        return duration ? fixed<R, N>(a, b, *duration) : Edge<R, N>{};
    }
    else
    {
        static_assert(R == 4);
        if (!(rho > 0.) || !std::isfinite(rho))
            return {};
        const double root_rho = std::sqrt(rho);
        double time_scale = 0.;
        for (int degree = 0; degree < R; ++degree)
            for (int j = 0; j < N; ++j)
            {
                double magnitude = degree == 0 ? std::abs(b[j] - a[j])
                                               : std::max(std::abs(a[degree * N + j]),
                                                          std::abs(b[degree * N + j]));
                time_scale =
                    std::max(time_scale, std::pow(magnitude / root_rho, 1. / (R - degree)));
            }
        if (!(time_scale > 0.) || !std::isfinite(time_scale))
            return {};
        State<R, N> x{}, y{};
        for (int degree = 0; degree < R; ++degree)
            for (int j = 0; j < N; ++j)
            {
                x[degree * N + j] = degree == 0 ? 0. : a[degree * N + j] / root_rho;
                y[degree * N + j] = (degree == 0 ? b[j] - a[j] : b[degree * N + j]) / root_rho;
                for (int power = 0; power < R - degree; ++power)
                {
                    x[degree * N + j] /= time_scale;
                    y[degree * N + j] /= time_scale;
                }
            }
        constexpr std::array<double, 16> inverse_gramian{
            100800, -50400, 10080, -840, -50400, 25920, -5400, 480,
            10080,  -5400,  1200,  -120, -840,   480,   -120,  16};
        std::array<double, 2 * R> energy{};
        double difference[R][R][N]{};
        for (int i = 0; i < R; ++i)
            for (int k = 0; k < R - i; ++k)
                for (int j = 0; j < N; ++j)
                    difference[i][k][j] =
                        -x[(i + k) * N + j] / factorial(k) + (k == 0 ? y[i * N + j] : 0.);
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < R; ++j)
                for (int k = 0; k < R - i; ++k)
                    for (int l = 0; l < R - j; ++l)
                        for (int q = 0; q < N; ++q)
                            energy[2 * R - 1 - i - j - k - l] += inverse_gramian[i * R + j] *
                                                                 difference[i][k][q] *
                                                                 difference[j][l][q];
        double scale = 0.;
        for (int k = 1; k < 2 * R; ++k)
            scale = std::max(scale, std::pow(std::abs(k * energy[k]), 1. / (k + 1)));
        if (!(scale > 0.))
            return {};
        std::vector<double> coefficients(2 * R + 1, 0.);
        coefficients.back() = 1.;
        for (int k = 1; k < 2 * R; ++k)
            coefficients[2 * R - k - 1] = -k * energy[k] / std::pow(scale, k + 1);
        double upper = 2.;
        for (int k = 0; k < 2 * R; ++k)
            upper = std::max(upper, 1. + std::abs(coefficients[k]));
        auto roots = roots_between(coefficients, 0., upper);
        std::vector<double> candidates;
        for (double root : roots)
            candidates.push_back(root * scale * time_scale);
        for (int degree = 0; degree < R - 1; ++degree)
            for (int j = 0; j < N; ++j)
            {
                double mean = .5 * (a[(degree + 1) * N + j] + b[(degree + 1) * N + j]);
                if (mean != 0.)
                {
                    double T = (b[degree * N + j] - a[degree * N + j]) / mean;
                    if (T > 0. && std::isfinite(T))
                        candidates.push_back(T);
                }
            }
        Edge<R, N> best;
        double best_cost = std::numeric_limits<double>::infinity();
        for (double T : candidates)
        {
            auto edge = fixed<R, N>(a, b, T);
            double cost = edge.cost(rho);
            if (cost < best_cost)
            {
                best_cost = cost;
                best = edge;
            }
        }
        return best;
    }
}
// Cheapest BVP over yaw windings (output 3 of the spatial quadrotor); see Settings::yaw_winding.
template <int R, int N>
Edge<R, N> winding_bvp(const State<R, N> &a, const State<R, N> &b, double rho, size_t &extra_calls)
{
    static_assert(N == 4);
    constexpr double turn = 2. * M_PI;
    const double nearest = std::round((a[3] - b[3]) / turn);
    Edge<R, N> best;
    double best_cost = std::numeric_limits<double>::infinity();
    for (double k : {nearest, nearest - 1., nearest + 1.})
    {
        State<R, N> c = b;
        c[3] += k * turn;
        auto e = bvp<R, N>(a, c, rho);
        if (k != nearest)
            ++extra_calls;
        if (!(e.T > 0.) || !std::isfinite(e.T))
            continue;
        const double cost = e.cost(rho);
        if (cost < best_cost)
        {
            best_cost = cost;
            best = e;
        }
    }
    return best;
}
// Shift each edge's yaw by whole turns so that consecutive edges join continuously (Settings::yaw_winding).
template <int R, int N> void unwrap_yaw(std::vector<Edge<R, N>> &edges)
{
    static_assert(N == 4);
    constexpr double turn = 2. * M_PI;
    for (size_t i = 1; i < edges.size(); ++i)
    {
        const double end = edges[i - 1].state(edges[i - 1].T)[3], begin = edges[i].c[3];
        const double k = std::round((end - begin) / turn);
        if (k != 0. && std::abs(end - begin - k * turn) < 1e-6)
            edges[i].c[3] += k * turn;
    }
}
template <int R, int N>
Edge<R, N> propagate(const State<R, N> &a, const std::array<double, N> &w, double T)
{
    Edge<R, N> e;
    e.T = T;
    for (int d = 0; d < R; ++d)
        for (int j = 0; j < N; ++j)
            e.c[d * N + j] = a[d * N + j] * std::pow(T, d) / factorial(d);
    for (int j = 0; j < N; ++j)
        e.c[R * N + j] = w[j] * std::pow(T, R) / factorial(R);
    return e;
}
template <class V> using Vec = std::array<V, 3>;
template <class V> Vec<V> add(const Vec<V> &a, const Vec<V> &b)
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
template <class V> Vec<V> sub(const Vec<V> &a, const Vec<V> &b)
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
template <class V> Vec<V> mul(const Vec<V> &a, V b)
{
    return {a[0] * b, a[1] * b, a[2] * b};
}
template <class V> V dot(const Vec<V> &a, const Vec<V> &b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
template <class V> Vec<V> cross(const Vec<V> &a, const Vec<V> &b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
template <class V> bool any(V mask)
{
    return mask.any();
}
// VAMP's generic sqrt is x*rsqrt(x), an approximate geometric primitive.
// Flatness recovery differentiates normalized vectors twice; its errors also
// affect physical limits. Use rounded hardware sqrt without modifying VAMP.
template <class V> V precise_sqrt(const V &value)
{
    V result;
#if defined(__x86_64__)
    for (size_t i = 0; i < V::num_vectors; ++i)
        result.data[i] = _mm256_sqrt_ps(value.data[i]);
#else
    std::array<float, V::num_scalars> values;
    for (size_t i = 0; i < values.size(); ++i)
        values[i] = std::sqrt(value.element(i));
    result = V(values);
#endif
    return result;
}
template <class V>
bool normalize(const Vec<V> &a, const Vec<V> &da, const Vec<V> &dda, Vec<V> &v, Vec<V> &dv,
               Vec<V> &ddv)
{
    V n2 = dot(a, a);
    if (any(n2 < V(1e-12f)))
        return false;
    V n = precise_sqrt(n2);
    v = mul(a, V(1.f) / n);
    V dn = dot(v, da);
    dv = mul(sub(da, mul(v, dn)), V(1.f) / n);
    V ddn = (dot(da, da) + dot(a, dda) - dn * dn) / n;
    ddv = mul(sub(sub(dda, mul(v, ddn)), mul(dv, V(2.f) * dn)), V(1.f) / n);
    return true;
}
struct Settings
{
    double rho = 1, range = .35, goal_bias = .1, validation_dt = .02, spacing = .025,
           derivative_bound = 1.;
    double propagation_min = 0., propagation_max = .3, control_bound = 2., selection_radius = 1.,
           witness_radius = .15, shrink = .8, goal_tolerance = .1;
    size_t max_iterations = 1000000, max_nodes = 100000, phase_iterations = 1000;
    uint64_t budget_ns = 1000000000;
    bool stop_first = true;
    bool scalar_validation = false;
    double boundary_speed = .3;
    double max_speed = 1., max_yaw_rate = 1.5, max_torque = 2.;
    double derivative_metric_weight = .2;
    bool trace = false;
    // v2 options; defaults reproduce the v1 reconstruction exactly.
    // metric_mode 1: authors' FlatStateSpace metric (weighted sum of per-level Euclidean
    //   distances, weight 1/extent of the level).
    // metric_mode 2: reconstructed unicycle position/velocity/heading embedding;
    //   heading scale is max_speed/max_yaw_rate. Uniform proposal support is unchanged.
    // rrt_mode 1: og::RRTConnect growTree semantics (range = range_fraction x space extent,
    //   time-fraction truncation, re-steered edge, TRAPPED on failure, repeated connect).
    // rrt_mode 2: VAMP-style extension: the target POSITION is clipped to `range` along the straight
    //   line from the nearest node (sampled derivative levels kept), one BVP to it, TRAPPED on failure,
    //   repeated connect (the policy matching the Table II reconstruction).
    // goal_mode 1: DynoBench goal region (robot distance_weights, goal_tolerance = goal_epsilon).
    // bound1..3: sampling bounds of flat derivative levels 1..3 (<= 0 uses derivative_bound).
    int metric_mode = 0, rrt_mode = 0, goal_mode = 0;
    double range_fraction = .2, bound1 = 0., bound2 = 0., bound3 = 0.;
    // speed_min > 0: the first derivative level of position outputs is sampled uniformly in the
    // annulus speed_min <= |v| <= level-1 bound (isotropic direction) instead of the box; this keeps
    // samples away from the unicycle chart singularity at zero speed. 0 keeps box sampling.
    double speed_min = 0.;
    // Aerial rest/uniform mixture. Values below one retain full proposal support.
    // Zero preserves the historical RNG stream and proposal distribution.
    double rest_sample_probability = 0.;
    // Optional per-output pseudo-control bounds, in each output's derivative units.
    // Empty uses the historical scalar control_bound.
    std::vector<double> control_bounds;
    // Optional exact-metric indices for SST selection/witnesses and heading search.
    // False preserves the historical scan and float-index behavior.
    bool indexed_search = false;
    // Necessary endpoint checks precede the unchanged analytic extrema tests.
    bool early_constraint_screening = true;
    // SST* phases (Li, Littlefield, Bekris 2016, Alg. 9). true (historical): discard the tree and
    // witnesses at every phase. false: SST Sec. 4.3 ("SST is assumed to be operating on the same
    // graph data structure G over repeated calls") -- only the radii shrink; tree and witnesses persist.
    bool phase_restart = true;
    // true: edge sample count from the spatial bound T * peak_speed / spacing only (as the authors'
    // FlatStateSpace::validSegmentCount); false (historical) also enforces samples every validation_dt.
    // Limits between samples are then established by the independent interval validator.
    bool spatial_only_validation = false;
    // Unicycle stop-turn-go protocol (v8). DynoBench unicycle1 is first order (controls v, omega), so the
    // robot may stop at any node, turn in place with |omega| <= max_yaw_rate and continue; its disk
    // geometry does not change while turning. true: straight edges (collinear coefficients; constant
    // heading, zero yaw rate, zero speed admissible) are accepted with exact speed extrema; curved edges
    // keep the strict analytic speed/yaw-rate test (nonzero speed throughout). With all nodes at rest
    // (rest_sample_probability = 1) every order-two BVP edge is straight: y0 + (yf - y0)(3s^2 - 2s^3).
    // Turns are reconstructed and validated in Python (flask_repro/unicycle_turns.py).
    // false keeps the strict single-chart v7 behaviour.
    bool unicycle_rest_turns = false;
    // v10: the spatial quadrotor's yaw output lives on S^1. true: as the authors' FlatStateSpace::optimalEdge
    // (KavrakiLab/ompl-flask flat, FlatStateSpace.cpp), the terminal yaw of every BVP may be shifted by whole
    // turns and the cheapest motion is kept (bounded candidate set: nearest winding and its two neighbours;
    // the authors walk every winding change up to the cost horizon). Returned edges are then shifted by whole
    // turns so that the yaw is continuous; recovery uses only sin/cos of yaw, so validity is unchanged.
    // false (historical): yaw is treated as a real output.
    bool yaw_winding = false;
};
struct Counters
{
    size_t edges = 0, batches = 0, collision = 0, speed = 0, angular = 0, singular = 0, control = 0,
           bounds = 0, bvp = 0, nonfinite = 0;
};
using Simd = vamp::FloatVector<vamp::FloatVectorWidth>;
using Environment = vamp::collision::Environment<Simd>;
template <class V> bool finite_vector(V value)
{
    const V largest(std::numeric_limits<float>::max());
    // VAMP's ordered NEQ comparison is false for NaN. Ordered finite-range
    // comparisons reject NaN and both infinities without relying on NEQ.
    return ((value <= largest) & (value >= -largest)).all();
}
template <class V> struct PlanarRecovery
{
    V thrust, torque, omega, force_left, force_right;
};
template <class V> struct UnicycleRecovery
{
    V speed, yaw_rate, squared_speed;
};
template <class V>
bool recover_unicycle(const std::array<std::array<V, 2>, 5> &p, UnicycleRecovery<V> &out)
{
    out.squared_speed = p[1][0] * p[1][0] + p[1][1] * p[1][1];
    if (!finite_vector(out.squared_speed) || any(out.squared_speed < V(1e-20f)))
        return false;
    out.speed = precise_sqrt(out.squared_speed);
    out.yaw_rate = (p[1][0] * p[2][1] - p[1][1] * p[2][0]) / out.squared_speed;
    return finite_vector(out.yaw_rate);
}
template <class V>
bool recover_planar(const std::array<std::array<V, 2>, 5> &p, PlanarRecovery<V> &out)
{
    V ax = p[2][0], az = p[2][1] + V(9.81f), jx = p[3][0], jz = p[3][1], sx = p[4][0], sz = p[4][1];
    V squared = ax * ax + az * az;
    if (!finite_vector(squared) || any(squared < V(1e-10f)))
        return false;
    V numerator = ax * jz - az * jx;
    out.omega = numerator / squared;
    out.torque = V(.0001f) * ((ax * sz - az * sx) / squared -
                              V(2.f) * numerator * (ax * jx + az * jz) / (squared * squared));
    out.thrust = V(.034f) * precise_sqrt(squared);
    out.force_left = (out.thrust + out.torque / V(.1f)) * V(.5f);
    out.force_right = (out.thrust - out.torque / V(.1f)) * V(.5f);
    return finite_vector(out.omega) && finite_vector(out.torque) && finite_vector(out.thrust);
}
template <class V> struct SpatialRecovery
{
    std::array<Vec<V>, 3> rotation;
    Vec<V> omega, torque;
    V thrust;
};
template <class V>
bool recover_spatial(const std::array<std::array<V, 4>, 5> &p, SpatialRecovery<V> &out)
{
    Vec<V> a{p[2][0], p[2][1], p[2][2] + V(9.81f)}, da{p[3][0], p[3][1], p[3][2]},
        dda{p[4][0], p[4][1], p[4][2]};
    Vec<V> rz, drz, ddrz;
    if (!normalize(a, da, dda, rz, drz, ddrz))
        return false;
    out.thrust = precise_sqrt(dot(a, a));
    V psi = p[0][3], pd = p[1][3], pdd = p[2][3], sn = psi.sin(), cs = psi.cos();
    Vec<V> h{-sn, cs, V(0.f)}, dh{-cs * pd, -sn * pd, V(0.f)},
        ddh{sn * pd * pd - cs * pdd, -cs * pd * pd - sn * pdd, V(0.f)};
    Vec<V> rx, drx, ddrx;
    if (!normalize(cross(h, rz), add(cross(dh, rz), cross(h, drz)),
                   add(add(cross(ddh, rz), mul(cross(dh, drz), V(2.f))), cross(h, ddrz)), rx, drx,
                   ddrx))
        return false;
    auto ry = cross(rz, rx), dry = add(cross(drz, rx), cross(rz, drx)),
         ddry = add(add(cross(ddrz, rx), mul(cross(drz, drx), V(2.f))), cross(rz, ddrx));
    out.rotation = {rx, ry, rz};
    out.omega = {dot(rz, dry), dot(rx, drz), dot(ry, drx)};
    Vec<V> wd{dot(drz, dry) + dot(rz, ddry), dot(drx, drz) + dot(rx, ddrz),
              dot(dry, drx) + dot(ry, ddrx)};
    Vec<V> Jw{V(.1f) * out.omega[0], V(.1f) * out.omega[1], V(.2f) * out.omega[2]},
        Jwd{V(.1f) * wd[0], V(.1f) * wd[1], V(.2f) * wd[2]};
    out.torque = add(Jwd, cross(out.omega, Jw));
    if (!finite_vector(out.thrust))
        return false;
    for (int j = 0; j < 3; ++j)
    {
        if (!finite_vector(out.omega[j]) || !finite_vector(out.torque[j]))
            return false;
        for (int k = 0; k < 3; ++k)
            if (!finite_vector(out.rotation[j][k]))
                return false;
    }
    return true;
}
struct World
{
    std::array<double, 3> lower{}, upper{};
    double radius = .1;
    int dimensions = 2;
    const Environment *environment = nullptr;
    const vamp::collision::Environment<float> *scalar_environment = nullptr;
};
inline std::vector<double> polynomial_product(const std::vector<double> &a,
                                              const std::vector<double> &b)
{
    std::vector<double> c(a.size() + b.size() - 1, 0.);
    for (size_t i = 0; i < a.size(); ++i)
        for (size_t j = 0; j < b.size(); ++j)
            c[i + j] += a[i] * b[j];
    return c;
}
inline std::vector<double> polynomial_derivative(const std::vector<double> &a)
{
    std::vector<double> d(std::max<size_t>(1, a.size() - 1), 0.);
    for (size_t i = 1; i < a.size(); ++i)
        d[i - 1] = i * a[i];
    return d;
}
// Exact candidate sets for extrema of squared speed and rational yaw rate.
// A time grid can miss arbitrarily narrow peaks close to the flatness singularity.
// Straight unicycle edge (stop-turn-go protocol): all polynomial coefficient vectors c1..c3 are
// collinear, so the heading is constant along the edge (kappa chosen per edge, Eq. 10), the yaw rate is
// identically zero and zero speed (rest endpoints, or a stop and reversal along the line) is admissible
// for the first-order unicycle. Returns 0 (not straight), 1 (valid straight edge), -1 (speed violated).
inline int unicycle_rest_edge(const Edge<2, 2> &e, Counters &counter, double max_speed)
{
    // Normalized cubic y(s) = c0 + c1 s + c2 s^2 + c3 s^3; dy/dt = (c1 + 2 c2 s + 3 c3 s^2) / T.
    const double c1x = e.c[2], c1y = e.c[3], c2x = e.c[4], c2y = e.c[5], c3x = e.c[6], c3y = e.c[7];
    if (!(e.T > 0) || !std::isfinite(e.T))
        return 0;
    const double n1 = std::hypot(c1x, c1y), n2 = std::hypot(c2x, c2y), n3 = std::hypot(c3x, c3y);
    double ux = c1x, uy = c1y, nu = n1;
    if (n2 > nu) { ux = c2x; uy = c2y; nu = n2; }
    if (n3 > nu) { ux = c3x; uy = c3y; nu = n3; }
    if (!(nu > 0) || !std::isfinite(nu))
        return 0;
    ux /= nu; uy /= nu;
    const double tol = 1e-9 * nu;
    if (std::abs(c1x * uy - c1y * ux) > tol || std::abs(c2x * uy - c2y * ux) > tol ||
        std::abs(c3x * uy - c3y * ux) > tol)
        return 0;
    // Exact speed maximum: stationary points of the squared speed and the endpoints.
    std::vector<double> vx{c1x / e.T, 2 * c2x / e.T, 3 * c3x / e.T}, vy{c1y / e.T, 2 * c2y / e.T, 3 * c3y / e.T};
    auto D = polynomial_product(vx, vx), yy = polynomial_product(vy, vy);
    for (size_t i = 0; i < D.size(); ++i)
        D[i] += yy[i];
    auto points = roots_between(polynomial_derivative(D), 0., 1.);
    points.insert(points.end(), {0., .5, 1.});
    for (double t : points)
    {
        double x = peval(vx, t), y = peval(vy, t), speed2 = x * x + y * y;
        if (!std::isfinite(speed2)) { ++counter.nonfinite; return -1; }
        if (speed2 > max_speed * max_speed + 1e-10) { ++counter.speed; return -1; }
    }
    return 1;
}
inline bool unicycle_extrema_valid(const Edge<2, 2> &e, Counters &counter, double max_speed = 1.,
                                   double max_yaw_rate = 1.5, bool early_screening = true)
{
    std::vector<double> vx(3), vy(3), ax(2), ay(2);
    for (int k = 1; k < 4; ++k)
    {
        vx[k - 1] = k * e.c[k * 2] / e.T;
        vy[k - 1] = k * e.c[k * 2 + 1] / e.T;
    }
    for (int k = 2; k < 4; ++k)
    {
        ax[k - 2] = k * (k - 1) * e.c[k * 2] / (e.T * e.T);
        ay[k - 2] = k * (k - 1) * e.c[k * 2 + 1] / (e.T * e.T);
    }
    auto finite = [](const std::vector<double> &p)
    { return std::all_of(p.begin(), p.end(), [](double x) { return std::isfinite(x); }); };
    if (!finite(vx) || !finite(vy) || !finite(ax) || !finite(ay))
    { ++counter.nonfinite; return false; }
    if (early_screening)
        for (double t : {0., 1.})
        {
            double x = peval(vx, t), y = peval(vy, t), speed2 = x * x + y * y;
            if (!std::isfinite(speed2)) { ++counter.nonfinite; return false; }
            if (speed2 <= 1e-20) { ++counter.singular; return false; }
            if (speed2 > max_speed * max_speed + 1e-10) { ++counter.speed; return false; }
        }
    auto D = polynomial_product(vx, vx), yy = polynomial_product(vy, vy);
    for (size_t i = 0; i < D.size(); ++i)
        D[i] += yy[i];
    if (!finite(D)) { ++counter.nonfinite; return false; }
    auto speeds = roots_between(polynomial_derivative(D), 0., 1.);
    speeds.insert(speeds.end(), {0., 1.});
    for (double t : speeds)
    {
        double x = peval(vx, t), y = peval(vy, t), speed2 = x * x + y * y;
        if (!std::isfinite(speed2)) { ++counter.nonfinite; return false; }
        if (speed2 <= 1e-20)
        {
            ++counter.singular;
            return false;
        }
        if (speed2 > max_speed * max_speed + 1e-10)
        {
            ++counter.speed;
            return false;
        }
    }
    if (early_screening)
        for (double t : {0., 1.})
        {
            double x = peval(vx, t), y = peval(vy, t), speed2 = x * x + y * y;
            double yaw_rate = (x * peval(ay, t) - y * peval(ax, t)) / speed2;
            if (!std::isfinite(yaw_rate)) { ++counter.nonfinite; return false; }
            if (std::abs(yaw_rate) > max_yaw_rate + 1e-10) { ++counter.angular; return false; }
        }
    auto numerator = polynomial_product(vx, ay), cross2 = polynomial_product(vy, ax);
    for (size_t i = 0; i < numerator.size(); ++i)
        numerator[i] -= cross2[i];
    if (!finite(numerator)) { ++counter.nonfinite; return false; }
    auto stationary = polynomial_product(polynomial_derivative(numerator), D),
         rhs = polynomial_product(numerator, polynomial_derivative(D));
    stationary.resize(std::max(stationary.size(), rhs.size()), 0.);
    for (size_t i = 0; i < rhs.size(); ++i)
        stationary[i] -= rhs[i];
    if (!finite(stationary)) { ++counter.nonfinite; return false; }
    auto angles = roots_between(stationary, 0., 1.);
    angles.insert(angles.end(), {0., 1.});
    for (double t : angles)
    {
        double x = peval(vx, t), y = peval(vy, t), denominator = x * x + y * y;
        double yaw_rate = (x * peval(ay, t) - y * peval(ax, t)) / denominator;
        if (!std::isfinite(yaw_rate)) { ++counter.nonfinite; return false; }
        if (std::abs(yaw_rate) > max_yaw_rate + 1e-10)
        {
            ++counter.angular;
            return false;
        }
    }
    return true;
}
template <class V, int R, int N>
bool validation_kernel(const Edge<R, N> &e, int robot, const World &world, const Settings &cfg,
                       Counters &counter)
{
    ++counter.edges;
    if (!(e.T > 0) || !std::isfinite(e.T))
    {
        ++counter.bvp;
        return false;
    }
    bool rest_straight = false;
    if constexpr (R == 2 && N == 2)
        if (robot == 0)
        {
            const int rest = cfg.unicycle_rest_turns ? unicycle_rest_edge(e, counter, cfg.max_speed) : 0;
            if (rest < 0)
                return false;
            rest_straight = rest == 1;
            if (!rest_straight && !unicycle_extrema_valid(e, counter, cfg.max_speed, cfg.max_yaw_rate,
                                                          cfg.early_constraint_screening))
                return false;
        }
    constexpr int K = V::num_scalars;
    constexpr int sampling_width = vamp::FloatVectorWidth;
    double speed_bound2 = 0;
    for (int j = 0; j < world.dimensions; ++j)
    {
        std::vector<double> acc(2 * R - 2);
        for (int k = 2; k < 2 * R; ++k)
            acc[k - 2] = k * (k - 1) * e.c[k * N + j];
        auto rs = roots_between(acc, 0, 1);
        rs.push_back(0);
        rs.push_back(1);
        double m = 0;
        for (double s : rs)
            m = std::max(m, std::abs(e.value(1, j, s * e.T)));
        speed_bound2 += m * m;
    }
    double requested =
        std::max(2., std::max(cfg.spatial_only_validation ? 0. : std::ceil(e.T / cfg.validation_dt) + 1.,
                              std::ceil(e.T * std::sqrt(speed_bound2) / cfg.spacing) + 1.));
    if (!std::isfinite(requested) ||
        requested >= double(std::numeric_limits<size_t>::max() - sampling_width))
    {
        ++counter.nonfinite;
        return false;
    }
    size_t samples = size_t(requested);
    samples = ((samples + sampling_width - 1) / sampling_width) * sampling_width;
    size_t batches = samples / K;
    for (size_t batch = 0; batch < batches; ++batch)
    {
        ++counter.batches;
        std::array<float, K> time{};
        for (int k = 0; k < K; ++k)
        {
            // Scalar traversal uses the same distributed point order and
            // identical total sample grid as the eight-wide SIMD traversal.
            size_t index = K == 1 ? (batch / sampling_width +
                                     (batch % sampling_width) * (samples / sampling_width))
                                  : (batch + k * batches);
            time[k] = float(double(index) / (samples - 1));
        }
        V s(time);
        std::array<std::array<V, N>, 5> p;
        for (int d = 0; d < 5; ++d)
            for (int j = 0; j < N; ++j)
            {
                p[d][j] = e.template derivative<V>(d, j, s);
                if (!finite_vector(p[d][j]))
                {
                    ++counter.nonfinite;
                    return false;
                }
            }
        for (int j = 0; j < world.dimensions; ++j)
            if (any((p[0][j] < V(float(world.lower[j] - 1e-5))) |
                    (p[0][j] > V(float(world.upper[j] + 1e-5)))))
            {
                ++counter.bounds;
                return false;
            }
        if constexpr (N == 2)
        {
            // A rest-to-rest straight edge has zero yaw rate inside and exactly checked speed; its
            // endpoint samples lie on the zero-speed chart boundary, so only geometry is sampled.
            if (robot == 0 && !rest_straight)
            {
                UnicycleRecovery<V> physical;
                if (!recover_unicycle(p, physical))
                {
                    ++counter.singular;
                    return false;
                }
                if (any(physical.squared_speed > V(float(cfg.max_speed * cfg.max_speed + 1e-5))))
                {
                    ++counter.speed;
                    return false;
                }
                if (any(physical.yaw_rate * physical.yaw_rate >
                        V(float(cfg.max_yaw_rate * cfg.max_yaw_rate + 1e-5))))
                {
                    ++counter.angular;
                    return false;
                }
            }
        }
        if constexpr (N == 2)
        {
            if (robot == 1)
            {
                PlanarRecovery<V> physical;
                if (!recover_planar(p, physical))
                {
                    ++counter.singular;
                    return false;
                }
                V limit(.65f * .034f * 9.81f + 1e-5f);
                if (any((physical.force_left < V(-1e-5f)) | (physical.force_right < V(-1e-5f)) |
                        (physical.force_left > limit) | (physical.force_right > limit)))
                {
                    ++counter.control;
                    return false;
                }
                if (any(p[1][0] * p[1][0] + p[1][1] * p[1][1] >
                        V(float(cfg.max_speed * cfg.max_speed + 1e-5))))
                {
                    ++counter.speed;
                    return false;
                }
                if (any(physical.omega * physical.omega >
                        V(float(cfg.max_yaw_rate * cfg.max_yaw_rate + 1e-5))))
                {
                    ++counter.angular;
                    return false;
                }
            }
        }
        if constexpr (N == 4)
        {
            SpatialRecovery<V> physical;
            if (!recover_spatial(p, physical))
            {
                ++counter.singular;
                return false;
            }
            if (any(physical.thrust > V(float(1.5 * 9.81))))
            {
                ++counter.control;
                return false;
            }
            if (any(dot(physical.omega, physical.omega) >
                    V(float(cfg.max_yaw_rate * cfg.max_yaw_rate + 1e-5))))
            {
                ++counter.angular;
                return false;
            }
            if (any(dot(physical.torque, physical.torque) >
                    V(float(cfg.max_torque * cfg.max_torque + 1e-5))))
            {
                ++counter.control;
                return false;
            }
            V squared_speed = p[1][0] * p[1][0] + p[1][1] * p[1][1] + p[1][2] * p[1][2];
            if (any(squared_speed > V(float(cfg.max_speed * cfg.max_speed + 1e-5))))
            {
                ++counter.speed;
                return false;
            }
        }
        V z = world.dimensions == 3 ? p[0][2] : V(0.f);
        bool collision;
        if constexpr (std::is_same_v<V, ScalarFloat>)
            collision = scalar_environment_collision(*world.scalar_environment, p[0][0].value,
                                                     p[0][1].value, z.value, float(world.radius));
        else
            collision = vamp::sphere_environment_in_collision(*world.environment, p[0][0], p[0][1],
                                                              z, V(float(world.radius)));
        if (collision)
        {
            ++counter.collision;
            return false;
        }
    }
    return true;
}
bool scalar_valid_2_2(const Edge<2, 2> &, int, const World &, const Settings &, Counters &);
bool scalar_valid_4_2(const Edge<4, 2> &, int, const World &, const Settings &, Counters &);
bool scalar_valid_4_4(const Edge<4, 4> &, int, const World &, const Settings &, Counters &);
bool scalar_valid_2_4(const Edge<2, 4> &, int, const World &, const Settings &, Counters &);
template <int R, int N>
bool valid(const Edge<R, N> &edge, int robot, const World &world, const Settings &cfg,
           Counters &counter)
{
    if (cfg.scalar_validation)
    {
        if (!world.scalar_environment)
            throw std::invalid_argument(
                "Scalar validation requires the original float environment");
        if constexpr (R == 2 && N == 2)
            return scalar_valid_2_2(edge, robot, world, cfg, counter);
        if constexpr (R == 4 && N == 2)
            return scalar_valid_4_2(edge, robot, world, cfg, counter);
        if constexpr (R == 4 && N == 4)
            return scalar_valid_4_4(edge, robot, world, cfg, counter);
        if constexpr (R == 2 && N == 4)
            return scalar_valid_2_4(edge, robot, world, cfg, counter);
    }
    return validation_kernel<Simd, R, N>(edge, robot, world, cfg, counter);
}
template <int R, int N> struct Result
{
    bool solved = false;
    std::vector<Edge<R, N>> edges;
    size_t iterations = 0, nodes = 0;
    uint64_t plan_ns = 0, post_ns = 0, first_ns = 0;
    double terminal_error = 0;
    Counters counters;
    std::vector<State<R, N>> start_trace, goal_trace;
    std::vector<size_t> start_parents, goal_parents;
    size_t reached_connections = 0;
    std::vector<size_t> phase_restart_iterations;
    std::vector<bool> start_active, start_alive;
    std::vector<double> start_costs;
    std::vector<size_t> start_children, witness_representatives;
    std::vector<State<R, N>> witness_states;
    double final_selection_radius = 0., final_pruning_radius = 0.;
    std::string reason = "search_exhausted";
    size_t bvp_calls = 0, planning_bvp_calls = 0, samples = 0, rest_samples = 0;
    size_t accepted_propagations = 0, witness_rejections = 0;
    size_t metric_evaluations = 0;
    // acceptance V3d: cost of the search's best path before Alg. 4 post-processing (NaN when unsolved)
    double search_cost = std::numeric_limits<double>::quiet_NaN();
};
enum class ExtendStatus
{
    Trapped,
    Advanced,
    Reached
};
inline uint64_t now()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
template <int R, int N>
double distance(const State<R, N> &a, const State<R, N> &b, double derivative_weight = .2)
{
    double d = 0;
    for (int i = 0; i < R * N; ++i)
    {
        double w = i < N ? 1. : derivative_weight / std::pow(3., i / N - 1);
        d += w * (a[i] - b[i]) * (a[i] - b[i]);
    }
    return std::sqrt(d);
}
inline double unicycle_search_distance(const State<2, 2> &a, const State<2, 2> &b,
                                        double derivative_weight, double turn_radius)
{
    double base = distance<2, 2>(a, b, derivative_weight);
    const double an = std::hypot(a[2], a[3]), bn = std::hypot(b[2], b[3]);
    // A zero-speed proposal has measure zero under continuous box sampling.
    // Its direction is canonically zero; accepted tree nodes remain nonsingular.
    double ax = an > 0. ? a[2] / an : 1., ay = an > 0. ? a[3] / an : 0.;
    double bx = bn > 0. ? b[2] / bn : 1., by = bn > 0. ? b[3] / bn : 0.;
    return std::hypot(base, turn_radius * std::hypot(ax - bx, ay - by));
}
// Double-precision Euclidean features supply candidate sets. Every final choice
// is refined with the exact planner metric and original insertion-order ties.
// Inactive/deleted vertices stay indexed but are excluded by the eligibility predicate.
template <int R, int N> class ExactSearchIndex
{
    static constexpr int dimensions = R * N + 2;
    using Key = Eigen::Matrix<double, dimensions, 1>;
    struct Entry
    {
        Key key;
        State<R, N> state;
    };
    struct Value
    {
        size_t index;
        const Entry *entry;
    };
    struct KeyOf
    {
        const Key &operator()(const Value &value) const noexcept { return value.entry->key; }
    };
    using Index = vamp::planning::nigh::Nigh<Value,
        vamp::planning::nigh::metric::L2Space<double, dimensions>, KeyOf,
        vamp::planning::nigh::NoThreadSafety, vamp::planning::nigh::KDTreeBatch<128>>;
    // Stable blocks retain keys while Nigh moves/sorts only sixteen-byte handles.
    // Declared before the index so keys also outlive index destruction.
    static constexpr size_t block_capacity = 128;
    std::vector<std::unique_ptr<Entry[]>> storage_blocks;
    size_t stored = 0;
    Index index;
    mutable std::vector<std::pair<Value, double>> buffer;
    double derivative_weight, heading_scale, maximum_coordinate = 1.;
    bool heading;

    Key encode(const State<R, N> &state) const
    {
        Key key = Key::Zero();
        for (int i = 0; i < R * N; ++i)
            key[i] = state[i] * std::sqrt(i < N ? 1. :
                derivative_weight / std::pow(3., i / N - 1));
        if constexpr (R == 2 && N == 2)
            if (heading)
            {
                double norm = std::hypot(state[2], state[3]);
                key[R * N] = heading_scale * (norm > 0. ? state[2] / norm : 1.);
                key[R * N + 1] = heading_scale * (norm > 0. ? state[3] / norm : 0.);
            }
        if (!key.allFinite())
            throw std::invalid_argument("Non-finite neighbor-index features");
        return key;
    }
    double margin(const Key &key) const
    {
        // Cover rounding in the six/eighteen-coordinate embedding and its
        // distance evaluation. Exact metric refinement resolves this small band.
        return 128. * std::numeric_limits<double>::epsilon() * dimensions *
               std::max(maximum_coordinate, key.cwiseAbs().maxCoeff());
    }

  public:
    ExactSearchIndex(double weight, bool include_heading = false, double scale = 1.)
        : derivative_weight(weight), heading_scale(scale), heading(include_heading) {}
    void clear()
    {
        index.clear();
        buffer.clear();
        storage_blocks.clear();
        stored = 0;
        maximum_coordinate = 1.;
    }
    void insert(size_t id, const State<R, N> &state)
    {
        Key key = encode(state);
        maximum_coordinate = std::max(maximum_coordinate, key.cwiseAbs().maxCoeff());
        if (stored % block_capacity == 0)
            storage_blocks.emplace_back(new Entry[block_capacity]);
        Entry &entry = storage_blocks.back()[stored % block_capacity];
        entry.key = key;
        entry.state = state;
        ++stored;
        index.insert(Value{id, &entry});
    }
    template <class Eligible, class Metric>
    size_t nearest(const State<R, N> &target, const Eligible &eligible, const Metric &metric) const
    {
        Key key = encode(target);
        size_t best = size_t(-1), count = std::min<size_t>(1, index.size());
        double best_distance = std::numeric_limits<double>::infinity();
        auto consider = [&](const Value &value)
        {
            if (!eligible(value.index)) return;
            double d = metric(value.entry->state, target);
            if (d < best_distance || (d == best_distance && value.index < best))
            {
                best_distance = d;
                best = value.index;
            }
        };
        while (count)
        {
            index.nearest(buffer, key, count);
            for (const auto &candidate : buffer) consider(candidate.first);
            if (best != size_t(-1)) break;
            if (count == index.size()) break;
            count = std::min(index.size(), std::max<size_t>(8, 2 * count));
        }
        if (best == size_t(-1))
            throw std::logic_error("SST has no active representative");
        // Include all possible near-ties, then restore the scan's exact choice.
        index.nearest(buffer, key, index.size(), best_distance + margin(key));
        for (const auto &candidate : buffer) consider(candidate.first);
        return best;
    }
    template <class Eligible, class Metric, class Cost>
    size_t best_near(const State<R, N> &target, double radius, const Eligible &eligible,
                     const Metric &metric, const Cost &cost) const
    {
        Key key = encode(target);
        index.nearest(buffer, key, index.size(), radius + margin(key));
        size_t best = size_t(-1);
        double best_cost = std::numeric_limits<double>::infinity();
        for (const auto &candidate : buffer)
        {
            const auto &value = candidate.first;
            if (!eligible(value.index) || metric(value.entry->state, target) > radius) continue;
            double c = cost(value.index);
            if (c < best_cost || (c == best_cost && best != size_t(-1) && value.index < best))
            {
                best_cost = c;
                best = value.index;
            }
        }
        return best != size_t(-1) ? best : nearest(target, eligible, metric);
    }
};
// Original SST Algorithm 6: minimum cost among active nodes in the ball;
// nearest active node if the ball is empty. Inactive/deleted nodes never expand.
template <int R, int N, class Node, class Metric>
size_t best_near_by(const std::vector<Node> &nodes, const State<R, N> &target, double radius,
                    const Metric &metric)
{
    size_t nearest = size_t(-1), best = size_t(-1);
    double nearest_distance = std::numeric_limits<double>::infinity(),
           best_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        if (!nodes[i].alive || !nodes[i].active)
            continue;
        double d = metric(nodes[i].z, target);
        if (d < nearest_distance)
        {
            nearest_distance = d;
            nearest = i;
        }
        if (d <= radius && nodes[i].cost < best_cost)
        {
            best_cost = nodes[i].cost;
            best = i;
        }
    }
    if (nearest == size_t(-1))
        throw std::logic_error("SST has no active representative");
    return best == size_t(-1) ? nearest : best;
}
// Legacy signature (weighted Euclidean flat metric), kept for the Python binding and tests.
template <int R, int N, class Node>
size_t best_near(const std::vector<Node> &nodes, const State<R, N> &target, double radius,
                 double derivative_weight)
{
    return best_near_by<R, N>(nodes, target, radius,
                              [derivative_weight](const State<R, N> &a, const State<R, N> &b)
                           { return distance<R, N>(a, b, derivative_weight); });
}
// Goal membership belongs to the original state space, independently of the
// search metric. Flat velocity proximity does not bound unicycle heading.
template <int R, int N>
double physical_distance(const State<R, N> &a, const State<R, N> &b, int robot = -1)
{
    double squared = 0.;
    constexpr int dimensions = N == 4 ? 3 : 2;
    for (int j = 0; j < dimensions; ++j)
        squared += std::pow(a[j] - b[j], 2);
    if constexpr (R == 2)
        if (robot == 1 || N == 4)
        {
            // Reduced-order (r = 2) aerial model: the flat state holds position and velocity
            // only; attitude is not part of the state and is not compared here.
            for (int j = 0; j < dimensions; ++j)
                squared += std::pow(a[N + j] - b[N + j], 2);
            return std::sqrt(squared);
        }
    if constexpr (R == 2 && N == 2)
    {
        if (std::hypot(a[2], a[3]) <= 1e-10 || std::hypot(b[2], b[3]) <= 1e-10)
            return std::numeric_limits<double>::infinity();
        double theta = std::remainder(std::atan2(a[3], a[2]) - std::atan2(b[3], b[2]), 2 * M_PI);
        return std::sqrt(squared + theta * theta);
    }
    else if constexpr (R == 4)
    {
        for (int j = 0; j < dimensions; ++j)
            squared += std::pow(a[N + j] - b[N + j], 2);
        if constexpr (N == 2)
        {
            auto attitude = [](const State<R, N> &z)
            {
                double ax = z[4], az = z[5] + 9.81, d = ax * ax + az * az;
                if (d <= 1e-20)
                    return std::array<double, 2>{NAN, NAN};
                return std::array<double, 2>{std::atan2(-ax, az), (ax * z[7] - az * z[6]) / d};
            };
            auto x = attitude(a), y = attitude(b);
            double theta = std::remainder(x[0] - y[0], 2 * M_PI);
            return std::sqrt(squared + theta * theta + std::pow(x[1] - y[1], 2));
        }
        else
        {
            auto attitude = [](const State<R, N> &z)
            {
                Eigen::Vector3d thrust(z[8], z[9], z[10] + 9.81), jerk(z[12], z[13], z[14]);
                double norm = thrust.norm();
                Eigen::Vector3d rz = thrust / norm, drz = (jerk - rz * rz.dot(jerk)) / norm;
                double psi = z[3], pd = z[7];
                Eigen::Vector3d h(-std::sin(psi), std::cos(psi), 0),
                    dh(-std::cos(psi) * pd, -std::sin(psi) * pd, 0);
                Eigen::Vector3d raw = h.cross(rz), draw = dh.cross(rz) + h.cross(drz),
                                rx = raw.normalized(),
                                drx = (draw - rx * rx.dot(draw)) / raw.norm(), ry = rz.cross(rx),
                                dry = drz.cross(rx) + rz.cross(drx);
                Eigen::Matrix3d rotation;
                rotation.col(0) = rx;
                rotation.col(1) = ry;
                rotation.col(2) = rz;
                return std::make_pair(rotation,
                                      Eigen::Vector3d(rz.dot(dry), rx.dot(drz), ry.dot(drx)));
            };
            auto x = attitude(a), y = attitude(b);
            double angle = Eigen::AngleAxisd(x.first.transpose() * y.first).angle();
            return std::sqrt(squared + (x.second - y.second).squaredNorm() + angle * angle);
        }
    }
    return std::numeric_limits<double>::infinity();
}
// DynoBench goal metric (dynobench/src/{unicycle1,planar_rotor,quadrotor}.cpp `distance` with the
// models' distance_weights): unicycle |dp| + .5|dtheta|; quad2d |dp| + .5|dtheta| + .2|dv| + .2|domega|;
// quad3d |dp| + .5 acos|q1.q2| + .1|dv| + .05|domega| (body rates).
template <int R, int N>
double dynobench_distance(const State<R, N> &a, const State<R, N> &b, int robot = -1)
{
    constexpr int dimensions = N == 4 ? 3 : 2;
    double dp = 0.;
    for (int j = 0; j < dimensions; ++j)
        dp += std::pow(a[j] - b[j], 2);
    dp = std::sqrt(dp);
    if constexpr (R == 2)
        if (robot == 1 || N == 4)
        {
            // Reduced-order aerial model: position and velocity terms only (attitude/rate terms
            // are undefined at order two and omitted; disclosed approximation).
            double dv = 0.;
            for (int j = 0; j < dimensions; ++j)
                dv += std::pow(a[N + j] - b[N + j], 2);
            return dp + (N == 4 ? .1 : .2) * std::sqrt(dv);
        }
    if constexpr (R == 2 && N == 2)
    {
        if (std::hypot(a[2], a[3]) <= 1e-10 || std::hypot(b[2], b[3]) <= 1e-10)
            return std::numeric_limits<double>::infinity();
        double theta = std::remainder(std::atan2(a[3], a[2]) - std::atan2(b[3], b[2]), 2 * M_PI);
        return dp + .5 * std::abs(theta);
    }
    else if constexpr (R == 4)
    {
        double dv = 0.;
        for (int j = 0; j < dimensions; ++j)
            dv += std::pow(a[N + j] - b[N + j], 2);
        dv = std::sqrt(dv);
        // Attitude and body rate are recovered exactly as in physical_distance.
        if constexpr (N == 2)
        {
            auto attitude = [](const State<R, N> &z)
            {
                double ax = z[4], az = z[5] + 9.81, d = ax * ax + az * az;
                return std::array<double, 2>{std::atan2(-ax, az), (ax * z[7] - az * z[6]) / d};
            };
            auto p = attitude(a), q = attitude(b);
            return dp + .5 * std::abs(std::remainder(p[0] - q[0], 2 * M_PI)) + .2 * dv +
                   .2 * std::abs(p[1] - q[1]);
        }
        else
        {
            auto attitude = [](const State<R, N> &z)
            {
                Eigen::Vector3d thrust(z[8], z[9], z[10] + 9.81), jerk(z[12], z[13], z[14]);
                double norm = thrust.norm();
                Eigen::Vector3d rz = thrust / norm, drz = (jerk - rz * rz.dot(jerk)) / norm;
                double psi = z[3], pd = z[7];
                Eigen::Vector3d h(-std::sin(psi), std::cos(psi), 0),
                    dh(-std::cos(psi) * pd, -std::sin(psi) * pd, 0);
                Eigen::Vector3d raw = h.cross(rz), draw = dh.cross(rz) + h.cross(drz),
                                rx = raw.normalized(),
                                drx = (draw - rx * rx.dot(draw)) / raw.norm(), ry = rz.cross(rx),
                                dry = drz.cross(rx) + rz.cross(drx);
                Eigen::Matrix3d rotation;
                rotation.col(0) = rx;
                rotation.col(1) = ry;
                rotation.col(2) = rz;
                return std::make_pair(rotation,
                                      Eigen::Vector3d(rz.dot(dry), rx.dot(drz), ry.dot(drx)));
            };
            auto p = attitude(a), q = attitude(b);
            double angle = Eigen::AngleAxisd(p.first.transpose() * q.first).angle();
            return dp + .5 * (angle / 2.) + .1 * dv + .05 * (p.second - q.second).norm();
        }
    }
    return std::numeric_limits<double>::infinity();
}
template <int R, int N>
Result<R, N> solve(const State<R, N> &start, const State<R, N> &goal, int robot, const World &world,
                   const Settings &cfg, uint64_t seed, int variant)
{
    Result<R, N> out;
    auto t0 = now();
    std::mt19937_64 gen(seed);
    std::uniform_real_distribution<double> u(0., 1.);
    struct Node
    {
        State<R, N> z;
        size_t parent;
        Edge<R, N> edge;
        double cost = 0;
        bool active = true, alive = true;
        size_t children = 0;
    };
    auto edge_ok = [&](const Edge<R, N> &e) { return valid(e, robot, world, cfg, out.counters); };
    auto exact = [&](const State<R, N> &a, const State<R, N> &b)
    {
        ++out.bvp_calls;
        if constexpr (N == 4)
            if (cfg.yaw_winding)
                return winding_bvp<R, N>(a, b, cfg.rho, out.bvp_calls);
        return bvp<R, N>(a, b, cfg.rho);
    };
    auto continuous = [&](std::vector<Edge<R, N>> &edges)
    {
        if constexpr (N == 4)
            if (cfg.yaw_winding)
                unwrap_yaw<R, N>(edges);
    };
    auto level_bound = [&](int level)
    {
        double b = level == 1 ? cfg.bound1 : level == 2 ? cfg.bound2 : level == 3 ? cfg.bound3 : 0.;
        return b > 0. ? b : cfg.derivative_bound;
    };
    std::array<double, R> level_weight{};
    {
        double e0 = 0.;
        for (int j = 0; j < world.dimensions; ++j)
            e0 += std::pow(world.upper[j] - world.lower[j], 2);
        if constexpr (N == 4)
            e0 += std::pow(2 * M_PI, 2);  // yaw output sampled in [-pi, pi]
        level_weight[0] = 1. / std::sqrt(e0);
        for (int l = 1; l < R; ++l)
            level_weight[l] = 1. / (2. * level_bound(l) * std::sqrt(double(N)));
    }
    auto metric = [&](const State<R, N> &a, const State<R, N> &b)
    {
        ++out.metric_evaluations;
        if constexpr (R == 2 && N == 2)
            if (cfg.metric_mode == 2)
                return unicycle_search_distance(a, b, cfg.derivative_metric_weight,
                                                  cfg.max_speed / cfg.max_yaw_rate);
        if (cfg.metric_mode != 1)
            return distance<R, N>(a, b, cfg.derivative_metric_weight);
        double d = 0.;
        for (int l = 0; l < R; ++l)
        {
            double s2 = 0.;
            for (int j = 0; j < N; ++j)
                s2 += (a[l * N + j] - b[l * N + j]) * (a[l * N + j] - b[l * N + j]);
            d += level_weight[l] * std::sqrt(s2);
        }
        return d;
    };
    auto goal_distance = [&](const State<R, N> &a, const State<R, N> &b)
    {
        // Stop-turn-go: the robot stops and turns in place to the goal heading, so the DynoBench
        // distance |dp| + .5|dtheta| reduces to |dp| (checked independently after the final turn).
        if (cfg.unicycle_rest_turns && robot == 0)
            return std::hypot(a[0] - b[0], a[1] - b[1]);
        return cfg.goal_mode == 1 ? dynobench_distance<R, N>(a, b, robot)
                                  : physical_distance<R, N>(a, b, robot);
    };
    if (start == goal)
    {
        std::array<double, N> w{};
        out.solved = edge_ok(propagate<R, N>(start, w, 1e-9));
        out.nodes = 2;
        out.plan_ns = now() - t0;
        out.first_ns = out.solved ? out.plan_ns : 0;
        out.reason = out.solved ? "solved" : "invalid_identity_state";
        return out;
    }
    if (variant == 0 && cfg.rrt_mode != 1 && now() - t0 < cfg.budget_ns)
    {
        auto direct = exact(start, goal);
        if (edge_ok(direct))
        {
            out.solved = true;
            out.reason = "solved";
            out.edges = {direct};
            out.search_cost = direct.cost(cfg.rho);
            out.first_ns = now() - t0;
            out.nodes = 2;
            out.plan_ns = now() - t0;
            out.planning_bvp_calls = out.bvp_calls;
            out.terminal_error = physical_distance<R, N>(direct.state(direct.T), goal, robot);
            if (cfg.trace)
            {
                out.start_trace = {start};
                out.goal_trace = {goal};
                out.start_parents = {0};
                out.goal_parents = {0};
            }
            return out;
        }
    }
    std::vector<Node> st{{start, 0, {}}}, gt{{goal, 0, {}}};
    st.reserve(std::min<size_t>(cfg.max_nodes, 10000));
    gt.reserve(std::min<size_t>(cfg.max_nodes, 10000));
    std::unique_ptr<NearestIndex<R, N>> start_index, goal_index;
    std::unique_ptr<ExactSearchIndex<R, N>> start_exact, goal_exact, witness_exact;
    const bool use_exact_index = cfg.indexed_search && cfg.metric_mode != 1 &&
                                (variant == 1 || variant == 2 || cfg.metric_mode == 2);
    auto make_exact_index = [&]()
    { return std::make_unique<ExactSearchIndex<R, N>>(cfg.derivative_metric_weight,
                cfg.metric_mode == 2, cfg.metric_mode == 2 ? cfg.max_speed / cfg.max_yaw_rate : 1.); };
    if (use_exact_index)
    {
        start_exact = make_exact_index();
        start_exact->insert(0, start);
        if (variant == 0)
        {
            goal_exact = make_exact_index();
            goal_exact->insert(0, goal);
        }
        if (variant == 1 || variant == 2)
        {
            witness_exact = make_exact_index();
            witness_exact->insert(0, start);
        }
    }
    if ((variant == 0 || variant == 3) && cfg.metric_mode == 0)
    {
        start_index =
            std::make_unique<NearestIndex<R, N>>(cfg.max_nodes, cfg.derivative_metric_weight);
        goal_index =
            std::make_unique<NearestIndex<R, N>>(cfg.max_nodes, cfg.derivative_metric_weight);
        start_index->insert(0, start);
        goal_index->insert(0, goal);
    }
    auto nearest =
        [&](const std::vector<Node> &tree, const State<R, N> &z, bool active_only = false)
    {
        auto &exact_index = &tree == &st ? start_exact : goal_exact;
        if (exact_index)
            return exact_index->nearest(z, [&](size_t id)
                { return tree[id].alive && (!active_only || tree[id].active); }, metric);
        if (!active_only && start_index)
            return (&tree == &st ? start_index : goal_index)->nearest(z);
        size_t best = 0;
        double ds = 1e300;
        for (size_t i = 0; i < tree.size(); ++i)
            if (tree[i].alive && (!active_only || tree[i].active))
            {
                double d = metric(tree[i].z, z);
                if (d < ds)
                {
                    best = i;
                    ds = d;
                }
            }
        return best;
    };
    auto update_index = [&](const std::vector<Node> &tree, size_t id, const State<R, N> &z)
    {
        auto &precise = &tree == &st ? start_exact : goal_exact;
        if (precise) precise->insert(id, z);
        if (start_index) (&tree == &st ? start_index : goal_index)->insert(id, z);
    };
    auto chain = [&](const std::vector<Node> &tree, size_t i)
    {
        std::vector<Edge<R, N>> path;
        while (i != tree[i].parent)
        {
            path.push_back(tree[i].edge);
            i = tree[i].parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    };
    auto sample = [&]()
    {
        ++out.samples;
        State<R, N> z{};
        for (int i = 0; i < R * N; ++i)
            z[i] = (2 * u(gen) - 1) * level_bound(i / N);
        if (cfg.speed_min > 0. && R > 1)
        {
            const double hi = level_bound(1), lo = std::min(cfg.speed_min, hi);
            std::normal_distribution<double> gauss(0., 1.);
            double norm = 0.;
            std::array<double, 3> d{};
            for (int j = 0; j < world.dimensions; ++j)
            {
                d[j] = gauss(gen);
                norm += d[j] * d[j];
            }
            norm = std::sqrt(norm);
            const double k = double(world.dimensions);
            const double radius =
                std::pow(std::pow(lo, k) + u(gen) * (std::pow(hi, k) - std::pow(lo, k)), 1. / k);
            for (int j = 0; j < world.dimensions; ++j)
                z[N + j] = norm > 0. ? radius * d[j] / norm : radius;
        }
        for (int j = 0; j < world.dimensions; ++j)
            z[j] = world.lower[j] + u(gen) * (world.upper[j] - world.lower[j]);
        if constexpr (N == 4)
            z[3] = (2 * u(gen) - 1) * M_PI;
        if (cfg.rest_sample_probability > 0. && u(gen) < cfg.rest_sample_probability)
        {
            for (int i = N; i < R * N; ++i)
                z[i] = 0.;
            ++out.rest_samples;
        }
        return z;
    };
    auto may_expand = [&]()
    { return st.size() + gt.size() < cfg.max_nodes && now() - t0 < cfg.budget_ns; };
    // Extend the time-parameterized BVP, not an interpolation of unrelated derivatives.
    auto extend = [&](std::vector<Node> &tree, const State<R, N> &target, bool forward,
                      size_t &index, bool continue_from_index = false)
    {
        if (!continue_from_index)
            index = nearest(tree, target);
        if (tree[index].z == target)
            return ExtendStatus::Reached;
        if (!may_expand())
            return ExtendStatus::Trapped;
        auto e = forward ? exact(tree[index].z, target) : exact(target, tree[index].z);
        if (!(e.T > 0))
            return ExtendStatus::Trapped;
        double L = e.length(world.dimensions);
        double frac = L > cfg.range ? std::max(.005, cfg.range / L) : 1.;
        for (int attempt = 0; attempt < 7; ++attempt)
        {
            if (!may_expand())
                return ExtendStatus::Trapped;
            auto sub = e;
            const bool reached = frac >= .999999;
            if (!reached)
            {
                auto a = forward ? e.state(0) : e.state(e.T * (1 - frac));
                auto b = forward ? e.state(e.T * frac) : e.state(e.T);
                sub = fixed<R, N>(a, b, e.T * frac);
            }
            if (edge_ok(sub))
            {
                auto z = reached ? target : (forward ? sub.state(sub.T) : sub.state(0));
                if (z == tree[index].z)
                    return ExtendStatus::Trapped;
                tree[index].children++;
                tree.push_back({z, index, sub, tree[index].cost + sub.cost(cfg.rho)});
                index = tree.size() - 1;
                update_index(tree, index, z);
                return reached ? ExtendStatus::Reached : ExtendStatus::Advanced;
            }
            frac *= .5;
        }
        return ExtendStatus::Trapped;
    };
    // og::RRTConnect::growTree on the flat state space (rrt_mode 1).
    const double ompl_range = cfg.range_fraction * double(R);  // compound extent = number of levels
    auto grow = [&](std::vector<Node> &tree, const State<R, N> &target, bool forward, size_t &index)
    {
        size_t i = nearest(tree, target);
        index = i;
        if (tree[i].z == target)
            return ExtendStatus::Reached;
        if (!may_expand())
            return ExtendStatus::Trapped;
        double d = metric(tree[i].z, target);
        State<R, N> dz = target;
        bool reach = true;
        if (cfg.rrt_mode == 2)
        {
            double dp = 0.;
            for (int j = 0; j < world.dimensions; ++j)
                dp += std::pow(target[j] - tree[i].z[j], 2);
            dp = std::sqrt(dp);
            if (dp > cfg.range)
            {
                for (int j = 0; j < world.dimensions; ++j)
                    dz[j] = tree[i].z[j] + (target[j] - tree[i].z[j]) * (cfg.range / dp);
                reach = false;
            }
        }
        else if (d > ompl_range)
        {
            auto m = exact(tree[i].z, target);
            if (!(m.T > 0))
                return ExtendStatus::Trapped;
            dz = m.state(m.T * (ompl_range / d));
            if (dz == tree[i].z)
                return ExtendStatus::Trapped;
            reach = false;
        }
        auto e = forward ? exact(tree[i].z, dz) : exact(dz, tree[i].z);
        if (!(e.T > 0) || !edge_ok(e))
            return ExtendStatus::Trapped;
        tree[i].children++;
        tree.push_back({dz, i, e, tree[i].cost + e.cost(cfg.rho)});
        index = tree.size() - 1;
        update_index(tree, index, dz);
        return reach ? ExtendStatus::Reached : ExtendStatus::Advanced;
    };
    std::vector<State<R, N>> witnesses{start};
    std::vector<size_t> representatives{0};
    double best_cost = 1e300;
    size_t phase_end = cfg.phase_iterations, phase = 0;
    double selection = cfg.selection_radius, pruning = cfg.witness_radius;
    for (size_t it = 0; !out.solved || (!cfg.stop_first && variant != 0); ++it)
    {
        if (it >= cfg.max_iterations)
        {
            out.reason = "iteration_limit";
            break;
        }
        if (now() - t0 >= cfg.budget_ns)
        {
            out.reason = "time_limit";
            break;
        }
        if (st.size() + gt.size() >= cfg.max_nodes)
        {
            out.reason = "node_limit";
            break;
        }
        out.iterations = it + 1;
        if (variant == 0 && (cfg.rrt_mode == 1 || cfg.rrt_mode == 2))
        {
            bool forward = (it % 2 == 0);
            auto &a = forward ? st : gt;
            auto &b = forward ? gt : st;
            size_t ai = 0, bi = 0;
            if (grow(a, sample(), forward, ai) == ExtendStatus::Trapped)
                continue;
            const State<R, N> target = a[ai].z;
            auto status = grow(b, target, !forward, bi);
            while (status == ExtendStatus::Advanced && now() - t0 < cfg.budget_ns &&
                   st.size() + gt.size() < cfg.max_nodes)
                status = grow(b, target, !forward, bi);
            if (status == ExtendStatus::Reached)
            {
                // b[bi] coincides with a[ai]; edges already join without a bridge.
                auto left = forward ? chain(st, ai) : chain(st, bi);
                auto right = forward ? chain(gt, bi) : chain(gt, ai);
                std::reverse(right.begin(), right.end());
                out.edges = left;
                out.edges.insert(out.edges.end(), right.begin(), right.end());
                out.solved = true;
                out.first_ns = now() - t0;
                ++out.reached_connections;
            }
        }
        else if (variant == 0)
        {
            bool forward = (it % 2 == 0);
            auto &a = forward ? st : gt;
            auto &b = forward ? gt : st;
            auto target = u(gen) < cfg.goal_bias ? (forward ? goal : start) : sample();
            size_t ai = 0;
            if (extend(a, target, forward, ai) == ExtendStatus::Trapped)
                continue;
            size_t bi = nearest(b, a[ai].z);
            Edge<R, N> bridge;
            bool connected = a[ai].z == b[bi].z, have_bridge = false;
            if (!connected)
            {
                bridge = forward ? exact(a[ai].z, b[bi].z) : exact(b[bi].z, a[ai].z);
                connected = have_bridge = edge_ok(bridge);
            }
            while (!connected && now() - t0 < cfg.budget_ns &&
                   st.size() + gt.size() < cfg.max_nodes)
            {
                auto status = extend(b, a[ai].z, !forward, bi, true);
                if (status == ExtendStatus::Trapped)
                    break;
                if (status == ExtendStatus::Reached)
                {
                    connected = true;
                    have_bridge = false;
                    ++out.reached_connections;
                    break;
                }
                bridge = forward ? exact(a[ai].z, b[bi].z) : exact(b[bi].z, a[ai].z);
                connected = have_bridge = edge_ok(bridge);
            }
            if (connected)
            {
                auto left = forward ? chain(st, ai) : chain(st, bi);
                auto right = forward ? chain(gt, bi) : chain(gt, ai);
                std::reverse(right.begin(), right.end());
                out.edges = left;
                if (have_bridge)
                    out.edges.push_back(bridge);
                out.edges.insert(out.edges.end(), right.begin(), right.end());
                out.solved = true;
                out.first_ns = now() - t0;
            }
        }
        else
        {
            bool sst = variant != 3;
            if (sst && it == phase_end)
            {
                ++phase;
                selection *= cfg.shrink;
                pruning *= cfg.shrink;
                double factor = std::pow(cfg.shrink, -(R * N + N + 1));
                double count = std::ceil(double(cfg.phase_iterations) * std::pow(factor, phase) *
                                         (1 + std::log(phase)));
                size_t remaining = std::numeric_limits<size_t>::max() - phase_end;
                phase_end +=
                    !std::isfinite(count) || count >= double(remaining) ? remaining : size_t(count);
                // phase_restart: Algorithm 9 read as re-initializing Algorithm 5 (historical).
                // Otherwise keep G across phases (SST Sec. 4.3); only the radii shrink.
                if (!cfg.phase_restart)
                {
                    out.phase_restart_iterations.push_back(it);
                    goto phase_done;
                }
                st.clear();
                st.push_back({start, 0, {}});
                witnesses = {start};
                representatives = {0};
                if (start_exact)
                {
                    start_exact->clear();
                    start_exact->insert(0, start);
                    witness_exact->clear();
                    witness_exact->insert(0, start);
                }
                out.phase_restart_iterations.push_back(it);
            }
        phase_done:
            auto target = u(gen) < cfg.goal_bias ? goal : sample();
            size_t ni = sst ? (start_exact ? start_exact->best_near(target, selection,
                [&](size_t id) { return st[id].active && st[id].alive; }, metric,
                [&](size_t id) { return st[id].cost; })
                : best_near_by<R, N>(st, target, selection, metric)) : nearest(st, target);
            std::array<double, N> w{};
            for (int j = 0; j < N; ++j)
                w[j] = (2 * u(gen) - 1) *
                       (cfg.control_bounds.empty() ? cfg.control_bound : cfg.control_bounds[j]);
            auto e = propagate<R, N>(st[ni].z, w,
                                     cfg.propagation_min +
                                         u(gen) * (cfg.propagation_max - cfg.propagation_min));
            if (!edge_ok(e))
                continue;
            ++out.accepted_propagations;
            auto z = e.state(e.T);
            // Stop-turn-go: the first-order unicycle stops at every node (its speed is a control), so the
            // node is stored at rest; the next edge leaves it along a straight line after an in-place turn.
            if constexpr (R == 2 && N == 2)
                if (cfg.unicycle_rest_turns && robot == 0)
                    for (int i = N; i < R * N; ++i)
                        z[i] = 0.;
            double cost = st[ni].cost + e.cost(cfg.rho);
            size_t wi = 0;
            double wd = 1e300;
            if (sst)
            {
                if (witness_exact)
                {
                    wi = witness_exact->nearest(z, [](size_t) { return true; }, metric);
                    wd = metric(z, witnesses[wi]);
                }
                else
                    for (size_t i = 0; i < witnesses.size(); ++i)
                    {
                        double ds = metric(z, witnesses[i]);
                        if (ds < wd)
                        {
                            wd = ds;
                            wi = i;
                        }
                    }
                if (wd > pruning)
                {
                    witnesses.push_back(z);
                    representatives.push_back(size_t(-1));
                    wi = witnesses.size() - 1;
                    if (witness_exact) witness_exact->insert(wi, z);
                }
                size_t old = representatives[wi];
                if (old != size_t(-1) && st[old].cost <= cost)
                {
                    ++out.witness_rejections;
                    continue;
                }
                if (old != size_t(-1))
                {
                    st[old].active = false;
                    while (!st[old].active && st[old].children == 0 && old != 0)
                    {
                        st[old].alive = false;
                        size_t parent = st[old].parent;
                        --st[parent].children;
                        old = parent;
                    }
                }
            }
            st[ni].children++;
            st.push_back({z, ni, e, cost});
            size_t j = st.size() - 1;
            update_index(st, j, z);
            if (sst)
                representatives[wi] = j;
            bool reached = false;
            Edge<R, N> tail;
            if (variant == 1)
            {
                tail = exact(z, goal);
                reached = edge_ok(tail);
            }
            else
                reached = goal_distance(z, goal) < cfg.goal_tolerance;
            if (reached)
            {
                auto path = chain(st, j);
                if (tail.T > 0)
                    path.push_back(tail);
                double c = 0;
                for (auto &e : path)
                    c += e.cost(cfg.rho);
                if (out.first_ns == 0)
                    out.first_ns = now() - t0;
                if (c < best_cost)
                {
                    best_cost = c;
                    out.edges = path;
                    out.solved = true;
                }
            }
        }
    }
    out.nodes = st.size() + gt.size();
    out.plan_ns = now() - t0;
    out.planning_bvp_calls = out.bvp_calls;
    if (out.solved)
    {
        out.search_cost = 0.;
        for (auto &e : out.edges)
            out.search_cost += e.cost(cfg.rho);
        out.reason = "solved";
        auto begin = now();
        continuous(out.edges);
        for (size_t a = 0; a + 1 < out.edges.size(); ++a)
        {
            for (size_t b = out.edges.size() - 1; b > a; --b)
            {
                auto from = out.edges[a].state(0), to = out.edges[b].state(out.edges[b].T);
                // Stop-turn-go: the first-order unicycle stops at every node (nodes are stored at rest),
                // so Alg. 4 bypasses between the stopped states: a straight rest-to-rest shortcut. The
                // velocity jump at either end is an instantaneous stop (validated in unicycle_turns.py).
                if constexpr (R == 2 && N == 2)
                    if (cfg.unicycle_rest_turns && robot == 0)
                        for (int i = N; i < R * N; ++i)
                            from[i] = to[i] = 0.;
                auto e = exact(from, to);
                if (edge_ok(e))
                {
                    out.edges.erase(out.edges.begin() + a, out.edges.begin() + b + 1);
                    out.edges.insert(out.edges.begin() + a, e);
                    break;
                }
            }
        }
        continuous(out.edges);
        out.post_ns = now() - begin;
        out.terminal_error =
            physical_distance<R, N>(out.edges.back().state(out.edges.back().T), goal, robot);
    }
    if (cfg.trace)
    {
        for (auto &node : st)
        {
            out.start_trace.push_back(node.z);
            out.start_parents.push_back(node.parent);
            out.start_active.push_back(node.active);
            out.start_alive.push_back(node.alive);
            out.start_costs.push_back(node.cost);
            out.start_children.push_back(node.children);
        }
        out.witness_states = witnesses;
        out.witness_representatives = representatives;
        out.final_selection_radius = selection;
        out.final_pruning_radius = pruning;
        for (auto &node : gt)
        {
            out.goal_trace.push_back(node.z);
            out.goal_parents.push_back(node.parent);
        }
    }
    return out;
}
} // namespace flask::flat
