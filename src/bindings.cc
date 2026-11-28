// The precision policy must precede all VAMP geometry and robot headers.
#include "flask/precise_vamp.hh"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <stdexcept>

#include <vamp/collision/factory.hh>
#include <vamp/collision/filter.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>
#include <vamp/robots/sphere.hh>

#include "flask/flat_native.hh"
#include "flask/rrt_connect.hh"
#include "flask/rrt_connect_ompl.hh"
#include "flask/sst_original.hh"

namespace nb = nanobind;
namespace vc = vamp::collision;
namespace vf = vamp::collision::factory;

struct Environment
{
    vc::Environment<float> e;
    std::optional<vc::Environment<vamp::FloatVector<flask::K>>> ev;

    auto vec() -> const vc::Environment<vamp::FloatVector<flask::K>> &
    {
        if (!ev)
        {
            ev.emplace(e);
        }
        return *ev;
    }

    void add_sphere(const std::array<float, 3> &c, float r)
    {
        e.spheres.emplace_back(vf::sphere::array(c, r));
        e.sort();
        ev.reset();
    }

    void add_cuboid(const std::array<float, 3> &c, const std::array<float, 3> &eul,
                    const std::array<float, 3> &h)
    {
        auto s = vf::cuboid::array(c, eul, h);
        if (s.axis_3_z == 1.)
        {
            e.z_aligned_cuboids.emplace_back(s);
        }
        else
        {
            e.cuboids.emplace_back(s);
        }
        e.sort();
        ev.reset();
    }

    auto add_pointcloud(const nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu> &pc,
                        float r_min, float r_max, float r_point, float min_dist, float cull_r,
                        const std::array<float, 3> &original)
        -> std::tuple<std::size_t, std::size_t, std::size_t>
    {
        std::size_t filter_ns = 0;
        std::vector<vc::Point> points;
        auto t0 = std::chrono::steady_clock::now();
        if (min_dist > 0.F)
        {
            const vc::Point lower{original[0] - cull_r, original[1] - cull_r, original[2] - cull_r};
            const vc::Point upper{original[0] + cull_r, original[1] + cull_r, original[2] + cull_r};
            points = vc::filter_pointcloud(pc, min_dist, cull_r, original, lower, upper, true);
            filter_ns = vamp::utils::get_elapsed_nanoseconds(t0);
        }
        else
        {
            points.reserve(pc.shape(0));
            for (std::size_t i = 0; i < pc.shape(0); ++i)
            {
                points.push_back(vc::Point{pc(i, 0), pc(i, 1), pc(i, 2)});
            }
        }
        if (points.empty())
        {
            return {filter_ns, 0, 0};
        }
        if (points.size() == 1)
        {

            points.push_back(points.front());
        }
        t0 = std::chrono::steady_clock::now();
        e.pointclouds.emplace_back(points, r_min, r_max, r_point);
        const std::size_t build_ns = vamp::utils::get_elapsed_nanoseconds(t0);
        ev.reset();
        return {filter_ns, build_ns, points.size()};
    }

    // A held object: spheres given in the end-effector frame (VAMP Attachment). Every manipulator check
    // (FLASKCC, count_collisions, validate) then also checks these spheres, posed by the end-effector FK.
    void attach(const std::vector<std::array<float, 4>> &spheres,
                const std::array<std::array<float, 4>, 4> &tf)
    {
        if (spheres.empty())
        {
            throw std::invalid_argument("attach needs at least one sphere");
        }
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                T.matrix()(i, j) = tf[i][j];
        vc::Attachment<float> a(T);
        for (const auto &s : spheres)
        {
            if (!(s[3] > 0.F) || !std::isfinite(s[0]) || !std::isfinite(s[1]) || !std::isfinite(s[2]))
            {
                throw std::invalid_argument("attachment spheres need finite centres and positive radii");
            }
            a.spheres.emplace_back(s[0], s[1], s[2], s[3]);
        }
        e.attachments.emplace(a);
        ev.reset();
    }

    void detach()
    {
        e.attachments.reset();
        ev.reset();
    }

    auto attached() const -> bool { return e.attachments.has_value(); }

    auto prepare() -> std::size_t
    {
        const auto t0 = std::chrono::steady_clock::now();
        ev.reset();
        vec();
        return vamp::utils::get_elapsed_nanoseconds(t0);
    }

    void add_capsule(const std::array<float, 3> &c, const std::array<float, 3> &eul, float r,
                     float len)
    {
        auto s = vf::cylinder::center::array(c, eul, r, len);
        if (s.xv == 0. and s.yv == 0.)
        {
            e.z_aligned_capsules.emplace_back(s);
        }
        else
        {
            e.capsules.emplace_back(s);
        }
        e.sort();
        ev.reset();
    }
};

template <std::size_t n> using V = std::array<double, n>;
template <typename T> struct RobotTag
{
    using type = T;
};

template <typename Robot> void register_robot(nb::module_ &m)
{
    constexpr std::size_t n = Robot::dimension;
    using GH = flask::JointLimits<n>;

    auto joint_limits = [](const V<n> &lower, const V<n> &upper, const V<n> &vm, const V<n> &am)
    { return GH{lower, upper, vm, am}; };

    nb::class_<GH>(m, "JointLimits")
        .def("__init__", [=](GH *p, const V<n> &lower, const V<n> &upper, const V<n> &vm,
                             const V<n> &am) { new (p) GH(joint_limits(lower, upper, vm, am)); })
        .def_rw("q_lo", &GH::q_lo)
        .def_rw("q_hi", &GH::q_hi)
        .def_rw("v_max", &GH::v_max)
        .def_rw("a_max", &GH::a_max)
        .def_rw("collision_resolution_scale", &GH::collision_resolution_scale)
        .def_rw("max_validation_dt", &GH::max_validation_dt)
        .def_rw("bound_validation_speed", &GH::bound_validation_speed)
        .def_rw("max_tool_tilt_rad", &GH::max_tool_tilt_rad)
        .def_rw("max_tool_angular_speed", &GH::max_tool_angular_speed)
        .def_rw("certify_reach", &GH::certify_reach)
        .def_rw("certify_delta", &GH::certify_delta)
        .def_rw("certify_float_margin", &GH::certify_float_margin);

    m.attr("dimension") = n;
    m.attr("resolution") = Robot::resolution;

    m.def(
        "lqmt",
        [](const V<n> &q0, const V<n> &v0, const V<n> &q1, const V<n> &v1,
           double rho) -> std::optional<std::tuple<double, V<n>, V<n>, double>>
        {
            if (!std::isfinite(rho) || rho <= 0.)
            {
                throw std::invalid_argument("rho must be finite and positive");
            }
            const auto c = flask::bvp<n>(q0, v0, q1, v1, rho);
            if (!c)
            {
                return std::nullopt;
            }
            const double C = flask::boundary_cost<n>(q0, v0, q1, v1, c->T, rho);
            return std::make_tuple(c->T, c->c3, c->c2, C);
        },
        "LQMT boundary-value solution");

    m.def("sample_count",
          [](const V<n> &q0, const V<n> &v0, const V<n> &q1, const V<n> &v1, double T)
          { return flask::sample_count<Robot>(flask::make_edge<n>(q0, v0, q1, v1, T)); });

    m.def("length", [](const V<n> &q0, const V<n> &v0, const V<n> &q1, const V<n> &v1, double T)
          { return flask::make_edge<n>(q0, v0, q1, v1, T).length(); });

    m.def(
        "flask_cc",
        [](Environment &env, const V<n> &q0, const V<n> &v0, const V<n> &q1, const V<n> &v1,
           double T, const GH &limits, bool check_limits)
        {
            return flask::flask_cc<Robot>(flask::make_edge<n>(q0, v0, q1, v1, T), limits, env.vec(),
                                          check_limits);
        },
        nb::arg("env"), nb::arg("q0"), nb::arg("v0"), nb::arg("q1"), nb::arg("v1"), nb::arg("T"),
        nb::arg("limits"), nb::arg("check_limits") = true);

    m.def("within_limits", [](const V<n> &q0, const V<n> &v0, const V<n> &q1, const V<n> &v1,
                              double T, const GH &limits)
          { return flask::within_limits<n>(flask::make_edge<n>(q0, v0, q1, v1, T), limits); });

    m.def(
        "plan",
        [](Environment &env, const V<n> &start, const std::vector<V<n>> &goals, const GH &limits,
           const flask::PlannerSettings &settings, std::uint64_t seed, bool shortcut,
           const std::optional<V<n>> &v_start) -> nb::dict
        {
            if (!std::isfinite(settings.rho) || settings.rho <= 0. ||
                !std::isfinite(settings.range) || settings.range <= 0. ||
                !std::isfinite(settings.w) || settings.w <= 0. ||
                !std::isfinite(settings.velocity_sample_scale) ||
                settings.velocity_sample_scale < 0. || settings.velocity_sample_scale > 1. ||
                settings.k == 0 || settings.max_samples < goals.size() + 2 ||
                (settings.extend_mode != 0 && settings.extend_mode != 1) ||
                !(settings.range_fraction > 0.) || !std::isfinite(settings.range_fraction))
            {
                throw std::invalid_argument("invalid planner configuration");
            }
            auto finite_state = [](const V<n> &x)
            { return std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); }); };
            if (!finite_state(start) || (v_start && !finite_state(*v_start)) ||
                !std::all_of(goals.begin(), goals.end(), finite_state))
            {
                throw std::invalid_argument("nonfinite state");
            }
            const auto &ev = env.vec();
            V<n> zero{};
            const V<n> vs = v_start ? *v_start : zero;
            flask::PlanningResult<n> result;
            std::size_t shortcut_ns = 0;
            flask::Trajectory<n> trajectory;
            {
                nb::gil_scoped_release nogil;
                result = settings.extend_mode == 1
                             ? flask::OmplRRTConnect<Robot>::solve(start, goals, limits, ev, settings,
                                                                   seed, vs, zero)
                             : flask::KinodynamicRRTConnect<Robot>::solve(start, goals, limits, ev,
                                                                          settings, seed, vs, zero);
                if (result.solved)
                {
                    trajectory = shortcut ? flask::shortcut<Robot>(result.trajectory, limits, ev,
                                                                   settings.rho, &shortcut_ns)
                                          : result.trajectory;
                }
            }
            nb::dict d;
            d["solved"] = result.solved;
            d["plan_ns"] = result.nanoseconds;
            d["shortcut_ns"] = shortcut_ns;
            d["iterations"] = result.iterations;
            d["start_nodes"] = result.start_nodes;
            d["goal_nodes"] = result.goal_nodes;
            d["edge_checks"] = result.counter.edge;
            d["batch_checks"] = result.counter.lower;
            d["limit_rejections"] = result.counter.limit_rejections;
            d["collision_rejections"] = result.counter.collision_rejections;
            d["task_rejections"] = result.counter.task_rejections;
            d["dynamic_domain_rejections"] = result.dynamic_domain_rejections;
            d["steering_failures"] = result.steering_failures;
            d["accepted_extensions"] = result.accepted_extensions;
            d["connection_attempts"] = result.connection_attempts;
            d["raw_positions"] = result.trajectory.Q;
            d["raw_velocities"] = result.trajectory.V;
            d["raw_durations"] = result.trajectory.T;
            d["Q"] = trajectory.Q;
            d["V"] = trajectory.V;
            d["T"] = trajectory.T;
            return d;
        },
        nb::arg("env"), nb::arg("start"), nb::arg("goals"), nb::arg("limits"), nb::arg("settings"),
        nb::arg("seed") = 0, nb::arg("shortcut") = true, nb::arg("v_start") = nb::none());

    // Lazy certification (v9): plan and shortcut at the planner resolution against the nominal scene, then
    // certify only the returned path (sweep-bounded sampling against the delta-inflated scene,
    // flask_repro/certify.py). A failed certificate discards the attempt and replans with the next RNG
    // stream (seed + k * 1000003); every attempt's search, shortcut and certification time is charged.
    m.def(
        "plan_lazy_certified",
        [](Environment &env, Environment &certified_env, const V<n> &start, const std::vector<V<n>> &goals,
           const GH &limits, const GH &certified_limits, const flask::PlannerSettings &settings,
           std::uint64_t seed, std::size_t max_attempts) -> nb::dict
        {
            if (max_attempts == 0 || !(certified_limits.certify_delta > 0.))
                throw std::invalid_argument("lazy certification needs attempts and a certified limit profile");
            const auto &ev = env.vec();
            const auto &cv = certified_env.vec();
            V<n> zero{};
            flask::PlanningResult<n> result;
            flask::Trajectory<n> trajectory;
            std::size_t plan_ns = 0, shortcut_ns = 0, certify_ns = 0, attempts = 0, rejected_paths = 0;
            bool certified = false;
            {
                nb::gil_scoped_release nogil;
                for (; attempts < max_attempts && !certified; ++attempts)
                {
                    const std::uint64_t stream = seed + attempts * 1000003ULL;
                    result = settings.extend_mode == 1
                                 ? flask::OmplRRTConnect<Robot>::solve(start, goals, limits, ev, settings,
                                                                       stream, zero, zero)
                                 : flask::KinodynamicRRTConnect<Robot>::solve(start, goals, limits, ev,
                                                                              settings, stream, zero, zero);
                    plan_ns += result.nanoseconds;
                    if (!result.solved)
                        break;  // search budget exhausted: report unsolved, as the nominal planner does
                    std::size_t ns = 0;
                    trajectory = flask::shortcut<Robot>(result.trajectory, limits, ev, settings.rho, &ns);
                    shortcut_ns += ns;
                    const auto t0 = std::chrono::steady_clock::now();
                    certified = true;
                    for (std::size_t i = 0; i + 1 < trajectory.Q.size() && certified; ++i)
                        certified = flask::flask_cc<Robot>(
                            flask::make_edge<n>(trajectory.Q[i], trajectory.V[i], trajectory.Q[i + 1],
                                                trajectory.V[i + 1], trajectory.T[i]),
                            certified_limits, cv, true);
                    certify_ns += vamp::utils::get_elapsed_nanoseconds(t0);
                    rejected_paths += !certified;
                }
            }
            nb::dict d;
            d["solved"] = result.solved && certified;
            d["plan_ns"] = plan_ns;
            d["shortcut_ns"] = shortcut_ns + certify_ns;  // post-processing = shortcut + certification
            d["simplify_only_ns"] = shortcut_ns;
            d["certification_ns"] = certify_ns;
            d["certification_attempts"] = attempts;
            d["rejected_paths"] = rejected_paths;
            d["iterations"] = result.iterations;
            d["start_nodes"] = result.start_nodes;
            d["goal_nodes"] = result.goal_nodes;
            d["edge_checks"] = result.counter.edge;
            d["batch_checks"] = result.counter.lower;
            d["limit_rejections"] = result.counter.limit_rejections;
            d["collision_rejections"] = result.counter.collision_rejections;
            d["task_rejections"] = result.counter.task_rejections;
            d["dynamic_domain_rejections"] = result.dynamic_domain_rejections;
            d["steering_failures"] = result.steering_failures;
            d["accepted_extensions"] = result.accepted_extensions;
            d["connection_attempts"] = result.connection_attempts;
            d["raw_positions"] = result.trajectory.Q;
            d["raw_velocities"] = result.trajectory.V;
            d["raw_durations"] = result.trajectory.T;
            d["Q"] = trajectory.Q;
            d["V"] = trajectory.V;
            d["T"] = trajectory.T;
            return d;
        },
        nb::arg("env"), nb::arg("certified_env"), nb::arg("start"), nb::arg("goals"), nb::arg("limits"),
        nb::arg("certified_limits"), nb::arg("settings"), nb::arg("seed") = 0, nb::arg("max_attempts") = 20);

    m.def("validate_interval",
          [](Environment &env, const std::vector<V<n>> &Q, const std::vector<V<n>> &Vv,
             const std::vector<double> &T, double ta, double tb,
             const GH &limits) -> std::pair<bool, std::size_t>
          {
              const auto &ev = env.vec();
              const auto t0 = std::chrono::steady_clock::now();
              bool ok = true;
              double critical_points = 0.0;
              for (std::size_t i = 0; i < T.size() && ok; ++i)
              {
                  const double a = std::max(ta - critical_points, 0.0),
                               b = std::min(tb - critical_points, T[i]);
                  if (b - a > 1e-9)
                  {
                      const auto c = flask::make_edge<n>(Q[i], Vv[i], Q[i + 1], Vv[i + 1], T[i]);
                      ok = flask::flask_cc<Robot>((a > 0.0 || b < T[i]) ? c.slice(a, b) : c, limits,
                                                  ev, false);
                  }
                  critical_points += T[i];
              }
              return {ok, vamp::utils::get_elapsed_nanoseconds(t0)};
          });

    m.def("shortcut",
          [](Environment &env, const std::vector<V<n>> &Q, const std::vector<V<n>> &Vv,
             const std::vector<double> &T, const GH &limits, double rho)
          {
              flask::Trajectory<n> q{Q, Vv, T};
              std::size_t nanoseconds = 0;
              auto r = flask::shortcut<Robot>(q, limits, env.vec(), rho, &nanoseconds);
              return std::make_tuple(r.Q, r.V, r.T, nanoseconds);
          });

    m.def("count_collisions",
          [](Environment &env, const std::vector<V<n>> &Q) -> std::pair<std::size_t, long>
          {
              for (const auto &q : Q)
                  for (double value : q)
                      if (!std::isfinite(value) || !std::isfinite(static_cast<float>(value)))
                          throw std::invalid_argument("nonfinite or unrepresentable collision configuration");
              const auto &ev = env.vec();
              std::size_t counter = 0;
              long first = -1;
              typename Robot::template ConfigurationBlock<flask::K> block;
              for (std::size_t i = 0; i < Q.size(); i += flask::K)
              {
                  alignas(32) std::array<float, flask::K> col;
                  for (std::size_t j = 0; j < n; ++j)
                  {
                      for (std::size_t k = 0; k < flask::K; ++k)
                      {
                          col[k] = static_cast<float>(Q[std::min(i + k, Q.size() - 1)][j]);
                      }
                      block[j] = vamp::FloatVector<flask::K>(col);
                  }
                  if (!(ev.attachments ? Robot::template fkcc_attach<flask::K>(ev, block)
                                       : Robot::template fkcc<flask::K>(ev, block)))
                  {
                      for (std::size_t k = 0; k < flask::K && i + k < Q.size(); ++k)
                      {
                          typename Robot::template ConfigurationBlock<flask::K> b1;
                          for (std::size_t j = 0; j < n; ++j)
                          {
                              b1[j] = vamp::FloatVector<flask::K>(static_cast<float>(Q[i + k][j]));
                          }
                          if (!(ev.attachments ? Robot::template fkcc_attach<flask::K>(ev, b1)
                                           : Robot::template fkcc<flask::K>(ev, b1)))
                          {
                              ++counter;
                              if (first < 0)
                              {
                                  first = static_cast<long>(i + k);
                              }
                          }
                      }
                  }
              }
              return {counter, first};
          });

    m.def("validate",
          [](Environment &env, const V<n> &q)
          {
              typename Robot::template ConfigurationBlock<flask::K> block;
              for (std::size_t j = 0; j < n; ++j)
              {
                  if (!std::isfinite(q[j]) || !std::isfinite(static_cast<float>(q[j])))
                      return false;
                  block[j] = vamp::FloatVector<flask::K>(static_cast<float>(q[j]));
              }
              const auto &ev = env.vec();
              return ev.attachments ? Robot::template fkcc_attach<flask::K>(ev, block)
                                    : Robot::template fkcc<flask::K>(ev, block);
          });
}

template <int R, int N>
flask::flat::State<R, N> flat_state(const std::vector<std::vector<double>> &x)
{
    if (x.size() != R)
        throw std::invalid_argument("incorrect flat-state derivative order");
    flask::flat::State<R, N> z{};
    for (int i = 0; i < R; ++i)
    {
        if (x[i].size() != N)
            throw std::invalid_argument("incorrect flat-state output count");
        for (int j = 0; j < N; ++j)
        {
            if (!std::isfinite(x[i][j]))
                throw std::invalid_argument("nonfinite flat state");
            z[i * N + j] = x[i][j];
        }
    }
    return z;
}
template <int R, int N> nb::dict flat_edge_dict(const flask::flat::Edge<R, N> &e)
{
    nb::dict d;
    std::vector<std::vector<double>> coef(2 * R, std::vector<double>(N));
    for (int i = 0; i < 2 * R; ++i)
        for (int j = 0; j < N; ++j)
            coef[i][j] = e.c[i * N + j];
    d["coefficients"] = coef;
    d["duration"] = e.T;
    d["order"] = R;
    d["effort"] = e.T > 0 ? e.effort() : 0.;
    return d;
}

template <int R, int N>
nb::dict run_flat(const std::vector<std::vector<double>> &a,
                  const std::vector<std::vector<double>> &b, int robot, Environment &environment,
                  const std::vector<double> &lo, const std::vector<double> &hi, double radius,
                  const flask::flat::Settings &cfg, uint64_t seed, int variant)
{
    auto start = flat_state<R, N>(a), goal = flat_state<R, N>(b);
    flask::flat::World world;
    world.dimensions = robot == 2 ? 3 : 2;
    world.radius = radius;
    world.environment = &environment.vec();
    world.scalar_environment = &environment.e;
    if (cfg.scalar_validation && (!environment.e.pointclouds.empty() ||
                                  !environment.e.heightfields.empty() || environment.e.attachments))
        throw std::invalid_argument(
            "Scalar low-DOF validation supports sphere, capsule, and cuboid environments");
    if (lo.size() != size_t(world.dimensions) || hi.size() != lo.size())
        throw std::invalid_argument("incorrect workspace bounds");
    for (int i = 0; i < world.dimensions; ++i)
    {
        if (!std::isfinite(lo[i]) || !std::isfinite(hi[i]) || lo[i] >= hi[i])
            throw std::invalid_argument("invalid workspace bounds");
        world.lower[i] = lo[i];
        world.upper[i] = hi[i];
    }
    if (!(cfg.rho > 0) || !(cfg.range > 0) || !(cfg.spacing > 0) || !(cfg.validation_dt > 0) ||
        !std::isfinite(radius) || radius < 0 || cfg.max_nodes < 4 ||
        !(cfg.derivative_metric_weight > 0) || !std::isfinite(cfg.derivative_metric_weight) ||
        cfg.phase_iterations == 0 || cfg.propagation_min < 0 || !(cfg.propagation_max > 0) ||
        cfg.propagation_max < cfg.propagation_min)
        throw std::invalid_argument("invalid planner settings");
    for (double value : {cfg.rho, cfg.range, cfg.spacing, cfg.validation_dt,
                         cfg.derivative_bound, cfg.control_bound, cfg.propagation_max,
                         cfg.selection_radius, cfg.witness_radius, cfg.goal_tolerance,
                         cfg.range_fraction})
        if (!std::isfinite(value) || value <= 0.)
            throw std::invalid_argument("planner scales must be finite and positive");
    for (double value : {cfg.bound1, cfg.bound2, cfg.bound3, cfg.speed_min,
                         cfg.propagation_min})
        if (!std::isfinite(value) || value < 0.)
            throw std::invalid_argument("proposal bounds must be finite and nonnegative");
    if (!(cfg.goal_bias >= 0. && cfg.goal_bias <= 1.) ||
        !(cfg.shrink > 0. && cfg.shrink < 1.) ||
        !(cfg.rest_sample_probability >= 0. && (cfg.rest_sample_probability < 1. ||
          (cfg.unicycle_rest_turns && cfg.rest_sample_probability == 1.))) ||
        cfg.metric_mode < 0 || cfg.metric_mode > 2 ||
        cfg.rrt_mode < 0 || cfg.rrt_mode > 2 || cfg.goal_mode < 0 || cfg.goal_mode > 1 ||
        cfg.speed_min > (cfg.bound1 > 0. ? cfg.bound1 : cfg.derivative_bound))
        throw std::invalid_argument("invalid planner probability, mode or speed proposal");
    if (cfg.metric_mode == 2 && (robot != 0 || !std::isfinite(cfg.max_speed) ||
                                !std::isfinite(cfg.max_yaw_rate) || cfg.max_speed <= 0. ||
                                cfg.max_yaw_rate <= 0.))
        throw std::invalid_argument("heading metric requires a unicycle with finite positive speed/yaw-rate limits");
    if (robot == 0 && cfg.rest_sample_probability > 0. && !cfg.unicycle_rest_turns)
        throw std::invalid_argument("rest proposals are singular for unicycle flatness");
    if (cfg.unicycle_rest_turns && (robot != 0 || cfg.rest_sample_probability != 1. || cfg.metric_mode == 2))
        throw std::invalid_argument("unicycle_rest_turns requires the unicycle, rest_sample_probability = 1 and a position metric");
    if (!cfg.control_bounds.empty() && cfg.control_bounds.size() != size_t(N))
        throw std::invalid_argument("control_bounds must have one entry per flat output");
    for (double value : cfg.control_bounds)
        if (!std::isfinite(value) || value <= 0.)
            throw std::invalid_argument("control_bounds must be finite and positive");
    if (cfg.parent_candidates == 0)
        throw std::invalid_argument("parent_candidates must be at least 1");
    for (double value : cfg.duration_multipliers)
        if (!std::isfinite(value) || value <= 1.)
            throw std::invalid_argument("duration_multipliers must be finite and greater than one");
    if (cfg.shortcut_subdivisions == 0 || cfg.shortcut_rounds == 0)
        throw std::invalid_argument("shortcut_subdivisions and shortcut_rounds must be at least 1");
    flask::flat::Result<R, N> r;
    {
        nb::gil_scoped_release release;
        r = flask::flat::solve<R, N>(start, goal, robot, world, cfg, seed, variant);
    }
    nb::dict out;
    out["solved"] = r.solved;
    out["plan_ns"] = r.plan_ns;
    out["postprocess_ns"] = r.post_ns;
    out["first_solution_ns"] = r.first_ns;
    out["iterations"] = r.iterations;
    out["nodes"] = r.nodes;
    out["terminal_error"] = r.terminal_error;
    out["reason"] = r.reason;
    nb::list edges;
    for (auto &e : r.edges)
        edges.append(flat_edge_dict(e));
    out["edges"] = edges;
    nb::dict c;
    c["edges"] = r.counters.edges;
    c["batches"] = r.counters.batches;
    c["collision"] = r.counters.collision;
    c["bounds"] = r.counters.bounds;
    c["speed"] = r.counters.speed;
    c["angular"] = r.counters.angular;
    c["control"] = r.counters.control;
    c["singular"] = r.counters.singular;
    c["nonfinite"] = r.counters.nonfinite;
    c["moving"] = r.counters.moving;
    out["rejections"] = c;
    nb::dict diagnostics;
    diagnostics["phase_restart_iterations"] = r.phase_restart_iterations;
    diagnostics["planning_bvp_calls"] = r.planning_bvp_calls;
    diagnostics["postprocessing_bvp_calls"] = r.bvp_calls - r.planning_bvp_calls;
    diagnostics["samples"] = r.samples;
    diagnostics["parent_reranks"] = r.parent_reranks;
    diagnostics["duration_fallbacks"] = r.duration_fallbacks;
    diagnostics["duration_fallback_accepts"] = r.duration_fallback_accepts;
    diagnostics["shortcut_extra_accepts"] = r.shortcut_extra_accepts;
    diagnostics["rest_samples"] = r.rest_samples;
    diagnostics["accepted_propagations"] = r.accepted_propagations;
    diagnostics["witness_rejections"] = r.witness_rejections;
    diagnostics["exact_metric_evaluations"] = r.metric_evaluations;
    diagnostics["search_cost"] = r.search_cost;
    if (cfg.trace)
    {
        diagnostics["start_states"] = r.start_trace;
        diagnostics["goal_states"] = r.goal_trace;
        diagnostics["start_parents"] = r.start_parents;
        diagnostics["goal_parents"] = r.goal_parents;
        diagnostics["reached_connections"] = r.reached_connections;
        diagnostics["start_active"] = r.start_active;
        diagnostics["start_alive"] = r.start_alive;
        diagnostics["start_costs"] = r.start_costs;
        diagnostics["start_children"] = r.start_children;
        diagnostics["witness_states"] = r.witness_states;
        diagnostics["witness_representatives"] = r.witness_representatives;
        diagnostics["selection_radius"] = r.final_selection_radius;
        diagnostics["pruning_radius"] = r.final_pruning_radius;
    }
    out["diagnostics"] = diagnostics;
    return out;
}

template <int N>
nb::dict recover_flat_jets(const std::vector<std::vector<std::vector<double>>> &jets, bool unicycle,
                           bool reverse)
{
    using V = flask::flat::Simd;
    constexpr size_t K = vamp::FloatVectorWidth;
    nb::list positions, velocities, states, controls, rotations, rates, torques, thrusts;
    for (size_t offset = 0; offset < jets.size(); offset += K)
    {
        std::array<std::array<V, N>, 5> p;
        for (int degree = 0; degree < 5; ++degree)
            for (int j = 0; j < N; ++j)
            {
                std::array<float, K> values;
                for (size_t lane = 0; lane < K; ++lane)
                {
                    const auto &jet = jets[std::min(offset + lane, jets.size() - 1)];
                    if (jet.size() != 5 || jet[degree].size() != N)
                        throw std::invalid_argument(
                            "Recovery requires batches of (5, outputs) jets through snap");
                    values[lane] = float(jet[degree][j]);
                }
                p[degree][j] = V(values);
                if (!flask::flat::finite_vector(p[degree][j]))
                    throw std::invalid_argument(
                        "Recovery jets must be finite and representable as floats");
            }
        flask::flat::PlanarRecovery<V> planar;
        flask::flat::SpatialRecovery<V> spatial;
        flask::flat::UnicycleRecovery<V> mobile;
        if constexpr (N == 2)
        {
            if (unicycle && !flask::flat::recover_unicycle(p, mobile))
                throw std::invalid_argument(
                    "Unicycle recovery is outside the numerical flatness chart");
            if (!unicycle && !flask::flat::recover_planar(p, planar))
                throw std::invalid_argument(
                    "Planar recovery is outside the numerical flatness chart");
        }
        else if (!flask::flat::recover_spatial(p, spatial))
            throw std::invalid_argument("Spatial recovery is outside the numerical flatness chart");
        for (size_t lane = 0; lane < std::min(K, jets.size() - offset); ++lane)
        {
            std::vector<double> position, velocity, control, state;
            for (int j = 0; j < (N == 4 ? 3 : 2); ++j)
            {
                position.push_back(p[0][j].element(lane));
                velocity.push_back(p[1][j].element(lane));
            }
            positions.append(position);
            velocities.append(velocity);
            if constexpr (N == 2)
            {
                if (unicycle)
                {
                    double vx = velocity[0], vy = velocity[1];
                    state = {position[0], position[1], std::atan2(vy, vx) + (reverse ? M_PI : 0.)};
                    control = {(reverse ? -1. : 1.) * mobile.speed.element(lane),
                               mobile.yaw_rate.element(lane)};
                }
                else
                {
                    state = {position[0],
                             position[1],
                             std::atan2(-p[2][0].element(lane), p[2][1].element(lane) + 9.81f),
                             velocity[0],
                             velocity[1],
                             planar.omega.element(lane)};
                    control = {planar.force_left.element(lane), planar.force_right.element(lane)};
                    torques.append(planar.torque.element(lane));
                    thrusts.append(planar.thrust.element(lane));
                }
                states.append(state);
            }
            else
            {
                std::array<std::array<double, 3>, 3> rotation;
                std::array<double, 3> omega, torque;
                for (int j = 0; j < 3; ++j)
                {
                    omega[j] = spatial.omega[j].element(lane);
                    torque[j] = spatial.torque[j].element(lane);
                    for (int k = 0; k < 3; ++k)
                        rotation[j][k] = spatial.rotation[k][j].element(lane);
                }
                rotations.append(rotation);
                rates.append(omega);
                torques.append(torque);
                thrusts.append(spatial.thrust.element(lane));
                control = {spatial.thrust.element(lane), torque[0], torque[1], torque[2]};
            }
            controls.append(control);
        }
    }
    nb::dict out;
    out["position"] = positions;
    out["velocity"] = velocities;
    out["state"] = states;
    out["control"] = controls;
    out["rotation"] = rotations;
    out["omega"] = rates;
    out["torque"] = torques;
    out["thrust"] = thrusts;
    return out;
}

template <int R, int N>
nb::dict check_flat_edges(Environment &environment, int robot,
                          const std::vector<std::vector<std::vector<double>>> &coefficients,
                          const std::vector<double> &durations, const std::vector<double> &lower,
                          const std::vector<double> &upper, double radius,
                          const flask::flat::Settings &cfg, size_t repetitions)
{
    if (coefficients.size() != durations.size() || repetitions == 0 ||
        lower.size() != size_t(N == 4 ? 3 : 2) || upper.size() != lower.size() ||
        !std::isfinite(radius) || radius < 0 || !std::isfinite(cfg.validation_dt) ||
        !(cfg.validation_dt > 0) || !std::isfinite(cfg.spacing) || !(cfg.spacing > 0))
        throw std::invalid_argument("Invalid trajectory-validation inputs");
    flask::flat::World world;
    world.dimensions = N == 4 ? 3 : 2;
    world.radius = radius;
    world.environment = &environment.vec();
    world.scalar_environment = &environment.e;
    if (cfg.scalar_validation && (!environment.e.pointclouds.empty() ||
                                  !environment.e.heightfields.empty() || environment.e.attachments))
        throw std::invalid_argument(
            "Scalar validation supports sphere, capsule, and cuboid environments");
    for (size_t j = 0; j < lower.size(); ++j)
    {
        if (!std::isfinite(lower[j]) || !std::isfinite(upper[j]) || lower[j] >= upper[j])
            throw std::invalid_argument("Invalid workspace bounds");
        world.lower[j] = lower[j];
        world.upper[j] = upper[j];
    }
    std::vector<flask::flat::Edge<R, N>> edges;
    for (size_t i = 0; i < coefficients.size(); ++i)
    {
        if (!std::isfinite(durations[i]) || !(durations[i] > 0))
            throw std::invalid_argument("Edge durations must be finite and positive");
        flask::flat::Edge<R, N> edge;
        edge.c = flat_state<2 * R, N>(coefficients[i]);
        edge.T = durations[i];
        edges.push_back(edge);
    }
    std::vector<bool> valid(edges.size());
    flask::flat::Counters counter;
    uint64_t elapsed;
    {
        nb::gil_scoped_release release;
        auto begin = flask::flat::now();
        for (size_t repeat = 0; repeat < repetitions; ++repeat)
            for (size_t i = 0; i < edges.size(); ++i)
                valid[i] = flask::flat::valid(edges[i], robot, world, cfg, counter);
        elapsed = flask::flat::now() - begin;
    }
    nb::dict out;
    out["valid"] = valid;
    out["validation_ns"] = elapsed;
    out["edge_checks"] = counter.edges;
    out["batches"] = counter.batches;
    out["nonfinite_rejections"] = counter.nonfinite;
    out["singularity_rejections"] = counter.singular;
    out["backend"] = cfg.scalar_validation ? "scalar" : "simd";
    return out;
}

NB_MODULE(_core, m)
{
    m.def(
        "flat_check_edges",
        [](const std::string &robot, Environment &environment,
           const std::vector<std::vector<std::vector<double>>> &coefficients,
           const std::vector<double> &durations, const std::vector<double> &lower,
           const std::vector<double> &upper, double radius, const flask::flat::Settings &settings,
           size_t repetitions)
        {
            if (robot == "unicycle")
                return check_flat_edges<2, 2>(environment, 0, coefficients, durations, lower, upper,
                                              radius, settings, repetitions);
            if (robot == "quad2d")
                return check_flat_edges<4, 2>(environment, 1, coefficients, durations, lower, upper,
                                              radius, settings, repetitions);
            if (robot == "quad3d")
                return check_flat_edges<4, 4>(environment, 2, coefficients, durations, lower, upper,
                                              radius, settings, repetitions);
            throw std::invalid_argument("Unknown flat robot model");
        },
        nb::arg("robot"), nb::arg("environment"), nb::arg("coefficients"), nb::arg("durations"),
        nb::arg("lower"), nb::arg("upper"), nb::arg("radius"), nb::arg("settings"),
        nb::arg("repetitions") = 1);
    m.def(
        "flat_best_near",
        [](const std::vector<std::vector<std::vector<double>>> &states,
           const std::vector<double> &costs, const std::vector<bool> &active,
           const std::vector<bool> &alive, const std::vector<std::vector<double>> &target,
           double radius, double weight, bool indexed, bool heading)
        {
            if (states.empty() || states[0].empty() || states.size() != costs.size() ||
                states.size() != active.size() || states.size() != alive.size() || !(radius > 0) ||
                !std::isfinite(radius) || !(weight > 0) || !std::isfinite(weight))
                throw std::invalid_argument("Invalid best-near inputs");
            for (double cost : costs)
                if (!std::isfinite(cost))
                    throw std::invalid_argument("Best-near costs must be finite");
            auto select = [&](auto order, auto outputs)
            {
                constexpr int R = decltype(order)::value, N = decltype(outputs)::value;
                struct Node
                {
                    flask::flat::State<R, N> z;
                    double cost;
                    bool active, alive;
                };
                std::vector<Node> nodes;
                for (size_t i = 0; i < states.size(); ++i)
                    nodes.push_back({flat_state<R, N>(states[i]), costs[i], active[i], alive[i]});
                if (heading && !(R == 2 && N == 2))
                    throw std::invalid_argument("Heading features require unicycle flat-state shape");
                auto metric = [&](const auto &a, const auto &b)
                {
                    if constexpr (R == 2 && N == 2)
                        if (heading) return flask::flat::unicycle_search_distance(a, b, weight, 2./3.);
                    return flask::flat::distance<R, N>(a, b, weight);
                };
                auto query = flat_state<R, N>(target);
                if (indexed)
                {
                    flask::flat::ExactSearchIndex<R, N> index(weight, heading, 2./3.);
                    for (size_t i = 0; i < nodes.size(); ++i) index.insert(i, nodes[i].z);
                    return index.best_near(query, radius,
                        [&](size_t i) { return nodes[i].active && nodes[i].alive; }, metric,
                        [&](size_t i) { return nodes[i].cost; });
                }
                return flask::flat::best_near_by<R, N>(nodes, query, radius, metric);
            };
            if (states[0].size() == 2 && states[0][0].size() == 2)
                return select(std::integral_constant<int, 2>{}, std::integral_constant<int, 2>{});
            if (states[0].size() == 4 && states[0][0].size() == 2)
                return select(std::integral_constant<int, 4>{}, std::integral_constant<int, 2>{});
            if (states[0].size() == 4 && states[0][0].size() == 4)
                return select(std::integral_constant<int, 4>{}, std::integral_constant<int, 4>{});
            throw std::invalid_argument("Unsupported best-near state shape");
        },
        nb::arg("states"), nb::arg("costs"), nb::arg("active"), nb::arg("alive"), nb::arg("target"),
        nb::arg("radius"), nb::arg("derivative_metric_weight") = .2,
        nb::arg("indexed") = false, nb::arg("heading") = false);
    m.def(
        "flat_recover",
        [](const std::string &robot, const std::vector<std::vector<std::vector<double>>> &jets,
           bool reverse)
        {
            if (robot == "quad3d")
                return recover_flat_jets<4>(jets, false, false);
            if (robot == "quad2d")
                return recover_flat_jets<2>(jets, false, false);
            if (robot == "unicycle")
                return recover_flat_jets<2>(jets, true, reverse);
            throw std::invalid_argument("Unknown flat robot model");
        },
        nb::arg("robot"), nb::arg("jets"), nb::arg("reverse") = false);
    m.def(
        "flat_nearest_indices",
        [](const std::vector<std::vector<std::vector<double>>> &states,
           const std::vector<std::vector<std::vector<double>>> &queries, double weight)
        {
            if (states.empty() || states[0].empty() || !(weight > 0) || !std::isfinite(weight))
                throw std::invalid_argument(
                    "nonempty states and a finite positive metric weight are required");
            auto query = [&](auto order, auto outputs)
            {
                constexpr int R = decltype(order)::value, N = decltype(outputs)::value;
                flask::flat::NearestIndex<R, N> index(states.size(), weight);
                for (size_t i = 0; i < states.size(); ++i)
                    index.insert(i, flat_state<R, N>(states[i]));
                std::vector<size_t> result;
                for (auto &q : queries)
                    result.push_back(index.nearest(flat_state<R, N>(q)));
                return result;
            };
            if (states[0].size() == 2 && states[0][0].size() == 2)
                return query(std::integral_constant<int, 2>{}, std::integral_constant<int, 2>{});
            if (states[0].size() == 4 && states[0][0].size() == 2)
                return query(std::integral_constant<int, 4>{}, std::integral_constant<int, 2>{});
            if (states[0].size() == 4 && states[0][0].size() == 4)
                return query(std::integral_constant<int, 4>{}, std::integral_constant<int, 4>{});
            throw std::invalid_argument("unsupported nearest-neighbor state shape");
        },
        nb::arg("states"), nb::arg("queries"), nb::arg("derivative_metric_weight") = .2);
    m.def("flat_state_distance",
          [](const std::vector<std::vector<double>> &a, const std::vector<std::vector<double>> &b)
          {
              if (a.size() == 2 && a[0].size() == 2)
                  return flask::flat::physical_distance<2, 2>(flat_state<2, 2>(a),
                                                              flat_state<2, 2>(b));
              if (a.size() == 4 && a[0].size() == 2)
                  return flask::flat::physical_distance<4, 2>(flat_state<4, 2>(a),
                                                              flat_state<4, 2>(b));
              if (a.size() == 4 && a[0].size() == 4)
                  return flask::flat::physical_distance<4, 4>(flat_state<4, 4>(a),
                                                              flat_state<4, 4>(b));
              throw std::invalid_argument("unsupported original-state distance shape");
          });
    m.def(
        "unicycle_edge_valid",
        [](const std::vector<std::vector<double>> &coefficients, double duration, bool early_screening)
        {
            auto state = flat_state<4, 2>(coefficients);
            if (!(duration > 0) || !std::isfinite(duration))
                throw std::invalid_argument("duration must be finite and positive");
            flask::flat::Edge<2, 2> edge;
            edge.T = duration;
            edge.c = state;
            flask::flat::Counters counters;
            return flask::flat::unicycle_extrema_valid(edge, counters, 1., 1.5, early_screening);
        },
        nb::arg("coefficients"), nb::arg("duration"), nb::arg("early_constraint_screening") = true);
    m.def(
        "unicycle_rest_edge",
        [](const std::vector<std::vector<double>> &coefficients, double duration, double max_speed)
        {
            auto state = flat_state<4, 2>(coefficients);
            if (!(duration > 0) || !std::isfinite(duration))
                throw std::invalid_argument("duration must be finite and positive");
            flask::flat::Edge<2, 2> edge;
            edge.T = duration;
            edge.c = state;
            flask::flat::Counters counters;
            return flask::flat::unicycle_rest_edge(edge, counters, max_speed);
        },
        nb::arg("coefficients"), nb::arg("duration"), nb::arg("max_speed") = 1.);
    m.def("unicycle_search_distance",
          [](const std::vector<std::vector<double>> &a,
             const std::vector<std::vector<double>> &b, double derivative_weight,
             double turn_radius)
          {
              if (!std::isfinite(derivative_weight) || derivative_weight <= 0. ||
                  !std::isfinite(turn_radius) || turn_radius <= 0.)
                  throw std::invalid_argument("metric scales must be finite and positive");
              return flask::flat::unicycle_search_distance(flat_state<2, 2>(a),
                  flat_state<2, 2>(b), derivative_weight, turn_radius);
          }, nb::arg("start"), nb::arg("goal"), nb::arg("derivative_weight") = .2,
             nb::arg("turn_radius") = 2. / 3.);
    nb::class_<flask::flat::MovingPiece>(m, "MovingPiece")
        .def(nb::init<>())
        .def_rw("t0", &flask::flat::MovingPiece::t0)
        .def_rw("T", &flask::flat::MovingPiece::T)
        .def_rw("radius", &flask::flat::MovingPiece::radius)
        .def_rw("c", &flask::flat::MovingPiece::c);

    nb::class_<flask::flat::Settings>(m, "LowDOFSettings")
        .def(nb::init<>())
        .def_rw("rho", &flask::flat::Settings::rho)
        .def_rw("range", &flask::flat::Settings::range)
        .def_rw("goal_bias", &flask::flat::Settings::goal_bias)
        .def_rw("validation_dt", &flask::flat::Settings::validation_dt)
        .def_rw("spacing", &flask::flat::Settings::spacing)
        .def_rw("derivative_bound", &flask::flat::Settings::derivative_bound)
        .def_rw("propagation_min", &flask::flat::Settings::propagation_min)
        .def_rw("propagation_max", &flask::flat::Settings::propagation_max)
        .def_rw("control_bound", &flask::flat::Settings::control_bound)
        .def_rw("control_bounds", &flask::flat::Settings::control_bounds)
        .def_rw("indexed_search", &flask::flat::Settings::indexed_search)
        .def_rw("early_constraint_screening", &flask::flat::Settings::early_constraint_screening)
        .def_rw("exact_bounds", &flask::flat::Settings::exact_bounds)
        .def_rw("rest_sample_probability", &flask::flat::Settings::rest_sample_probability)
        .def_rw("selection_radius", &flask::flat::Settings::selection_radius)
        .def_rw("witness_radius", &flask::flat::Settings::witness_radius)
        .def_rw("shrink", &flask::flat::Settings::shrink)
        .def_rw("goal_tolerance", &flask::flat::Settings::goal_tolerance)
        .def_rw("max_iterations", &flask::flat::Settings::max_iterations)
        .def_rw("max_nodes", &flask::flat::Settings::max_nodes)
        .def_rw("phase_iterations", &flask::flat::Settings::phase_iterations)
        .def_rw("budget_ns", &flask::flat::Settings::budget_ns)
        .def_rw("stop_first", &flask::flat::Settings::stop_first)
        .def_rw("scalar_validation", &flask::flat::Settings::scalar_validation)
        .def_rw("max_speed", &flask::flat::Settings::max_speed)
        .def_rw("max_yaw_rate", &flask::flat::Settings::max_yaw_rate)
        .def_rw("max_torque", &flask::flat::Settings::max_torque)
        .def_rw("derivative_metric_weight", &flask::flat::Settings::derivative_metric_weight)
        .def_rw("trace", &flask::flat::Settings::trace)
        .def_rw("metric_mode", &flask::flat::Settings::metric_mode)
        .def_rw("rrt_mode", &flask::flat::Settings::rrt_mode)
        .def_rw("goal_mode", &flask::flat::Settings::goal_mode)
        .def_rw("range_fraction", &flask::flat::Settings::range_fraction)
        .def_rw("bound1", &flask::flat::Settings::bound1)
        .def_rw("bound2", &flask::flat::Settings::bound2)
        .def_rw("bound3", &flask::flat::Settings::bound3)
        .def_rw("speed_min", &flask::flat::Settings::speed_min)
        .def_rw("phase_restart", &flask::flat::Settings::phase_restart)
        .def_rw("unicycle_rest_turns", &flask::flat::Settings::unicycle_rest_turns)
        .def_rw("spatial_only_validation", &flask::flat::Settings::spatial_only_validation)
        .def_rw("yaw_winding", &flask::flat::Settings::yaw_winding)
        .def_rw("parent_candidates", &flask::flat::Settings::parent_candidates)
        .def_rw("duration_multipliers", &flask::flat::Settings::duration_multipliers)
        .def_rw("shortcut_subdivisions", &flask::flat::Settings::shortcut_subdivisions)
        .def_rw("shortcut_rounds", &flask::flat::Settings::shortcut_rounds)
        .def_rw("moving_obstacles", &flask::flat::Settings::moving_obstacles)
        .def_rw("moving_margin", &flask::flat::Settings::moving_margin);
    m.def(
        "flat_bvp",
        [](const std::vector<std::vector<double>> &a, const std::vector<std::vector<double>> &b,
           double rho, double duration)
        {
            if (a.empty() || b.empty() || !(rho > 0) || !std::isfinite(rho) ||
                !std::isfinite(duration))
                throw std::invalid_argument("invalid BVP input");
            int r = a.size(), n = a[0].size();
#define BVP_CASE(R, N)                                                                             \
    if (r == R && n == N)                                                                          \
    {                                                                                              \
        auto x = flat_state<R, N>(a), y = flat_state<R, N>(b);                                     \
        return flat_edge_dict(duration > 0 ? flask::flat::fixed<R, N>(x, y, duration)              \
                                           : flask::flat::bvp<R, N>(x, y, rho));                   \
    }
            BVP_CASE(2, 2);
            BVP_CASE(4, 2);
            BVP_CASE(4, 4);
            BVP_CASE(2, 4);
#undef BVP_CASE
            throw std::invalid_argument("supported BVP shapes: (2,2), (4,2), (4,4), (2,4)");
        },
        nb::arg("start"), nb::arg("goal"), nb::arg("rho") = 1., nb::arg("duration") = -1.);
    m.def(
        "flat_plan",
        [](const std::string &robot, Environment &env, const std::vector<std::vector<double>> &a,
           const std::vector<std::vector<double>> &b, const std::vector<double> &lo,
           const std::vector<double> &hi, double radius, const flask::flat::Settings &cfg,
           uint64_t seed, int variant)
        {
            if (variant < 0 || variant > 4)
                throw std::invalid_argument("unknown planner variant");
            if (robot == "unicycle")
                return run_flat<2, 2>(a, b, 0, env, lo, hi, radius, cfg, seed, variant);
            // Aerial models: order (rows of the flat state) 4 is the full model of Example 3;
            // order 2 is the reduced pseudo-control order the paper permits in practice (Sec. V-C1).
            if (robot == "quad2d")
                return a.size() == 2 ? run_flat<2, 2>(a, b, 1, env, lo, hi, radius, cfg, seed, variant)
                                     : run_flat<4, 2>(a, b, 1, env, lo, hi, radius, cfg, seed, variant);
            if (robot == "quad3d")
                return a.size() == 2 ? run_flat<2, 4>(a, b, 2, env, lo, hi, radius, cfg, seed, variant)
                                     : run_flat<4, 4>(a, b, 2, env, lo, hi, radius, cfg, seed, variant);
            throw std::invalid_argument("unknown robot model");
        },
        nb::arg("robot"), nb::arg("environment"), nb::arg("start"), nb::arg("goal"),
        nb::arg("lower"), nb::arg("upper"), nb::arg("radius"), nb::arg("settings"),
        nb::arg("seed") = 0, nb::arg("variant") = 0);

    m.def(
        "sphere_positions_valid",
        [](Environment &env, const nb::ndarray<double, nb::shape<-1, 3>, nb::device::cpu> &p,
           double radius)
        {
            if (!std::isfinite(radius) || radius < 0.)
            {
                throw std::invalid_argument("invalid robot sphere radius");
            }
            using F = vamp::FloatVector<flask::K>;
            const auto &ev = env.vec();
            nb::gil_scoped_release nogil;
            for (std::size_t i = 0; i < p.shape(0); i += flask::K)
            {
                std::array<float, flask::K> x{}, y{}, z{};
                for (std::size_t k = 0; k < flask::K; ++k)
                {
                    const auto j = std::min(i + k, p.shape(0) - 1);
                    for (std::size_t d = 0; d < 3; ++d)
                    {
                        if (!std::isfinite(p(j, d)))
                        {
                            return false;
                        }
                    }
                    x[k] = static_cast<float>(p(j, 0));
                    y[k] = static_cast<float>(p(j, 1));
                    z[k] = static_cast<float>(p(j, 2));
                }
                if (vamp::sphere_environment_in_collision(ev, F(x), F(y), F(z),
                                                          F(static_cast<float>(radius))))
                {
                    return false;
                }
            }
            return true;
        },
        nb::arg("env"), nb::arg("positions"), nb::arg("radius"));
    nb::class_<Environment>(m, "Environment")
        .def(nb::init<>())
        .def("add_sphere", &Environment::add_sphere)
        .def("add_cuboid", &Environment::add_cuboid)
        .def("add_capsule", &Environment::add_capsule)
        .def("add_pointcloud", &Environment::add_pointcloud, nb::arg("pc"), nb::arg("r_min"),
             nb::arg("r_max"), nb::arg("r_point"), nb::arg("min_dist") = 0.F,
             nb::arg("cull_r") = 1.2F, nb::arg("original") = std::array<float, 3>{0.F, 0.F, 0.91F})
        .def("attach", &Environment::attach, nb::arg("spheres"),
             nb::arg("tf") = std::array<std::array<float, 4>, 4>{{{1.F, 0.F, 0.F, 0.F}, {0.F, 1.F, 0.F, 0.F},
                                                                   {0.F, 0.F, 1.F, 0.F}, {0.F, 0.F, 0.F, 1.F}}})
        .def("detach", &Environment::detach)
        .def_prop_ro("attached", &Environment::attached)
        .def("prepare", &Environment::prepare);

    nb::class_<flask::PlannerSettings>(m, "PlannerSettings")
        .def(nb::init<>())
        .def_rw("range", &flask::PlannerSettings::range)
        .def_rw("w", &flask::PlannerSettings::w)
        .def_rw("cost_neighbors", &flask::PlannerSettings::cost_neighbors)
        .def_rw("k", &flask::PlannerSettings::k)
        .def_rw("rho", &flask::PlannerSettings::rho)
        .def_rw("velocity_sample_scale", &flask::PlannerSettings::velocity_sample_scale)
        .def_rw("prefix_steering", &flask::PlannerSettings::prefix_steering)
        .def_rw("dynamic_domain", &flask::PlannerSettings::dynamic_domain)
        .def_rw("radius", &flask::PlannerSettings::radius)
        .def_rw("alpha", &flask::PlannerSettings::alpha)
        .def_rw("min_radius", &flask::PlannerSettings::min_radius)
        .def_rw("balance", &flask::PlannerSettings::balance)
        .def_rw("tree_ratio", &flask::PlannerSettings::tree_ratio)
        .def_rw("max_iterations", &flask::PlannerSettings::max_iterations)
        .def_rw("max_samples", &flask::PlannerSettings::max_samples)
        .def_rw("max_ns", &flask::PlannerSettings::max_ns)
        .def_rw("extend_mode", &flask::PlannerSettings::extend_mode)
        .def_rw("range_fraction", &flask::PlannerSettings::range_fraction);

    nb::class_<flask::orig::Settings>(m, "OrigSettings")
        .def(nb::init<>())
        .def_rw("dt", &flask::orig::Settings::dt)
        .def_rw("min_steps", &flask::orig::Settings::min_steps)
        .def_rw("max_steps", &flask::orig::Settings::max_steps)
        .def_rw("goal_epsilon", &flask::orig::Settings::goal_epsilon)
        .def_rw("goal_bias", &flask::orig::Settings::goal_bias)
        .def_rw("selection_radius", &flask::orig::Settings::selection_radius)
        .def_rw("pruning_radius", &flask::orig::Settings::pruning_radius)
        .def_rw("shrink", &flask::orig::Settings::shrink)
        .def_rw("phase_iterations", &flask::orig::Settings::phase_iterations)
        .def_rw("max_iterations", &flask::orig::Settings::max_iterations)
        .def_rw("trace", &flask::orig::Settings::trace)
        .def_rw("budget_ns", &flask::orig::Settings::budget_ns)
        .def_rw("max_nodes", &flask::orig::Settings::max_nodes)
        .def_rw("stop_first", &flask::orig::Settings::stop_first)
        .def_rw("velocity_sample", &flask::orig::Settings::velocity_sample)
        .def_rw("rate_sample", &flask::orig::Settings::rate_sample);
    m.def(
        "orig_sst_plan",
        [](const std::string &robot, Environment &env, const std::vector<double> &start,
           const std::vector<double> &goal, const std::vector<double> &lo, const std::vector<double> &hi,
           double radius, const flask::orig::Settings &cfg, std::uint64_t seed)
        {
            const int dimensions = robot == "quad3d" ? 3 : 2;
            auto finite = [](const std::vector<double> &values)
            { return std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); }); };
            if ((robot != "unicycle" && robot != "quad2d" && robot != "quad3d") ||
                lo.size() != std::size_t(dimensions) || hi.size() != lo.size() ||
                !finite(lo) || !finite(hi) || !finite(start) || !finite(goal) ||
                !std::isfinite(radius) || radius < 0. || !std::isfinite(cfg.dt) || !(cfg.dt > 0.) ||
                cfg.min_steps < 1 || cfg.max_steps < cfg.min_steps || cfg.phase_iterations == 0 ||
                cfg.max_iterations == 0 || cfg.max_nodes < 2 || cfg.budget_ns == 0 ||
                !std::isfinite(cfg.goal_epsilon) || !(cfg.goal_epsilon > 0.) ||
                !(cfg.goal_bias >= 0. && cfg.goal_bias <= 1.) ||
                !std::isfinite(cfg.selection_radius) || !(cfg.selection_radius > 0.) ||
                !std::isfinite(cfg.pruning_radius) || !(cfg.pruning_radius > 0.) ||
                !(cfg.shrink > 0. && cfg.shrink < 1.) ||
                !std::isfinite(cfg.velocity_sample) || !(cfg.velocity_sample > 0.) ||
                !std::isfinite(cfg.rate_sample) || !(cfg.rate_sample > 0.))
                throw std::invalid_argument("invalid supplementary SST configuration or state");
            for (int j = 0; j < dimensions; ++j)
                if (!(lo[j] < hi[j]))
                    throw std::invalid_argument("invalid supplementary SST workspace bounds");
            flask::orig::World world;
            world.environment = &env.e;
            world.dimensions = int(lo.size());
            world.radius = radius;
            for (std::size_t j = 0; j < lo.size() && j < 3; ++j)
            {
                world.lower[j] = lo[j];
                world.upper[j] = hi[j];
            }
            auto run = [&](auto model) -> nb::dict
            {
                using M = decltype(model);
                typename M::X a{}, b{};
                if (start.size() != a.size() || goal.size() != b.size())
                    throw std::invalid_argument("state dimension mismatch");
                if constexpr (M::nx == 13)
                {
                    for (const auto *state : {&start, &goal})
                    {
                        double squared_norm = 0.;
                        for (int j = 3; j < 7; ++j)
                            squared_norm += (*state)[j] * (*state)[j];
                        if (std::abs(squared_norm - 1.) > 1e-8)
                            throw std::invalid_argument("supplementary SST requires a unit quaternion");
                    }
                }
                std::copy(start.begin(), start.end(), a.begin());
                std::copy(goal.begin(), goal.end(), b.begin());
                flask::orig::Result<M> r;
                {
                    nb::gil_scoped_release nogil;
                    r = flask::orig::solve<M>(a, b, world, cfg, seed);
                }
                nb::dict d;
                d["solved"] = r.solved;
                d["first_ns"] = r.first_ns;
                d["plan_ns"] = r.plan_ns;
                d["iterations"] = r.iterations;
                d["nodes"] = r.nodes;
                d["propagation_steps"] = r.propagation_steps;
                d["invalid_propagations"] = r.invalid_propagations;
                d["best_cost"] = r.best_cost;
                std::vector<std::vector<double>> X, U;
                for (auto &x : r.states)
                    X.emplace_back(x.begin(), x.end());
                for (auto &u : r.controls)
                    U.emplace_back(u.begin(), u.end());
                d["states"] = X;
                d["controls"] = U;
                d["steps"] = r.steps;
                d["improvements"] = r.improvements;
                if (cfg.trace)
                {
                    nb::dict trace;
                    trace["node_states"] = r.node_states;
                    trace["parents"] = r.parents;
                    trace["children"] = r.children;
                    trace["costs"] = r.costs;
                    trace["active"] = r.active;
                    trace["alive"] = r.alive;
                    trace["witnesses"] = r.witnesses;
                    trace["representatives"] = r.representatives;
                    trace["phase_restarts"] = r.phase_restarts;
                    trace["selection_radius"] = r.final_selection_radius;
                    trace["pruning_radius"] = r.final_pruning_radius;
                    d["diagnostics"] = trace;
                }
                return d;
            };
            if (robot == "unicycle")
                return run(flask::orig::Unicycle{});
            if (robot == "quad2d")
                return run(flask::orig::Quad2D{});
            if (robot == "quad3d")
                return run(flask::orig::Quad3D{});
            throw std::invalid_argument("unknown robot model");
        },
        nb::arg("robot"), nb::arg("environment"), nb::arg("start"), nb::arg("goal"), nb::arg("lower"),
        nb::arg("upper"), nb::arg("radius"), nb::arg("settings"), nb::arg("seed") = 0);

    m.attr("K") = flask::K;

    // End-effector pose and collision spheres of the VAMP model (showcases: grasp frames, swept volumes).
    auto kinematics = [](nb::module_ &r, auto robot_tag)
    {
        using Robot = typename decltype(robot_tag)::type;
        constexpr std::size_t n = Robot::dimension;
        r.def("eefk",
              [](const V<n> &q)
              {
                  std::array<float, n> a;
                  for (std::size_t i = 0; i < n; ++i)
                      a[i] = static_cast<float>(q[i]);
                  const Eigen::Matrix4f M = Robot::eefk(a).matrix();
                  std::array<std::array<double, 4>, 4> out;
                  for (int i = 0; i < 4; ++i)
                      for (int j = 0; j < 4; ++j)
                          out[i][j] = M(i, j);
                  return out;
              });
        r.def("sphere_fk",
              [](const V<n> &q)
              {
                  typename Robot::template ConfigurationBlock<1> b;
                  for (std::size_t i = 0; i < n; ++i)
                      b[i] = static_cast<float>(q[i]);
                  typename Robot::template Spheres<1> o;
                  Robot::template sphere_fk<1>(b, o);
                  std::vector<std::array<double, 4>> out(Robot::n_spheres);
                  for (std::size_t i = 0; i < Robot::n_spheres; ++i)
                      out[i] = {o.x[{i, 0}], o.y[{i, 0}], o.z[{i, 0}], o.r[{i, 0}]};
                  return out;
              });
        r.attr("end_effector") = Robot::end_effector;
    };

    auto ur5 = m.def_submodule("ur5");
    register_robot<vamp::robots::UR5>(ur5);
    auto panda = m.def_submodule("panda");
    register_robot<vamp::robots::Panda>(panda);
    auto fetch = m.def_submodule("fetch");   // showcase: torso + 7-DoF arm, VAMP sphere model (111 spheres)
    register_robot<vamp::robots::Fetch>(fetch);
    kinematics(ur5, RobotTag<vamp::robots::UR5>{});
    kinematics(panda, RobotTag<vamp::robots::Panda>{});
    kinematics(fetch, RobotTag<vamp::robots::Fetch>{});
    // showcase: fully actuated point robot (flat output = position, r = 2) with a settable collision radius,
    // used for the humanoid base (omnidirectional walking) in simulation/showcase/g1_courtyard.py
    auto sphere = m.def_submodule("sphere");
    register_robot<vamp::robots::Sphere>(sphere);
    sphere.def("set_radius", [](float r) { vamp::robots::Sphere::set_radius(r); });
}
