// Reference FLASK measurement driver built on the authors' OMPL fork (KavrakiLab/ompl-flask, branch
// `flat`).  It does NOT reimplement any FLASK component: steering (FlatStateSpace / MinimumEffortSteering),
// edge validation (FlatVampMotionValidator), the planner (og::RRTConnect) and simplification
// (og::PathSimplifier::reduceVertices) are the authors' code.  This file only loads MotionBenchMaker
// problems, applies the paper's settings (rho = 1, Panda limits), times the calls and writes JSONL.
//
// Problem file format (written by benchmarks/author_reference.py, one problem after another):
//   env index n_goals
//   start (7 numbers)
//   n_goals lines of 7 numbers
//   n_spheres     then lines: x y z r
//   n_capsules    then lines: x y z roll pitch yaw r length
//   n_cuboids     then lines: x y z roll pitch yaw hx hy hz
//
// Usage:
//   mbm_flat plan  <problems.txt> <seed> <budget_s> <accel_limits:0|1> <out.jsonl>
//   mbm_flat bvp   <pairs.txt> <rho> <out.jsonl>     (pairs: q0 v0 q1 v1, 28 numbers per line)

#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/flat/FlatStateSpace.h>
#include <ompl/base/spaces/flat/FlatTrajectory.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#ifdef FLASK_REFERENCE_BUDGETED
#include "BudgetedRRTConnect.h"
#else
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#endif
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include <ompl/vamp/FlatVampMotionValidator.h>
#include <ompl/vamp/VampStateSpace.h>
#include <ompl/vamp/VampStateValidityChecker.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;
constexpr unsigned int ORDER = 2;
constexpr std::array<double, 7> SPEED{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
// Franka Panda published joint acceleration limits (same values as flask_repro/robot.py).
constexpr std::array<double, 7> ACCEL{15., 7.5, 10., 12.5, 15., 20., 20.};

struct Problem
{
    std::string env;
    int index;
    std::array<double, 7> start;
    std::vector<std::array<double, 7>> goals;
    vamp::collision::Environment<float> world;
};

static bool readProblem(std::istream &in, Problem &p)
{
    int goals;
    if (!(in >> p.env >> p.index >> goals))
        return false;
    for (auto &x : p.start)
        in >> x;
    p.goals.resize(goals);
    for (auto &g : p.goals)
        for (auto &x : g)
            in >> x;
    p.world = {};
    int n;
    in >> n;
    for (int i = 0; i < n; ++i)
    {
        std::array<float, 3> c;
        float r;
        in >> c[0] >> c[1] >> c[2] >> r;
        p.world.spheres.emplace_back(vamp::collision::factory::sphere::array(c, r));
    }
    in >> n;
    for (int i = 0; i < n; ++i)
    {
        std::array<float, 3> c, e;
        float r, l;
        in >> c[0] >> c[1] >> c[2] >> e[0] >> e[1] >> e[2] >> r >> l;
        p.world.capsules.emplace_back(vamp::collision::factory::capsule::center::array(c, e, r, l));
    }
    in >> n;
    for (int i = 0; i < n; ++i)
    {
        std::array<float, 3> c, e, h;
        in >> c[0] >> c[1] >> c[2] >> e[0] >> e[1] >> e[2] >> h[0] >> h[1] >> h[2];
        p.world.cuboids.emplace_back(vamp::collision::factory::cuboid::array(c, e, h));
    }
    p.world.sort();
    return static_cast<bool>(in);
}

// Optional acceleration-limit check placed in front of the authors' validator.  For order 2 the
// steering polynomial is cubic, so acceleration is affine in time and its extrema are the endpoints.
class AccelLimitedValidator : public ompl::vamp::FlatVampMotionValidator<Robot>
{
public:
    AccelLimitedValidator(const ob::SpaceInformationPtr &si, const Environment &env)
      : FlatVampMotionValidator<Robot>(si, env), space_(si->getStateSpace()->as<ob::FlatStateSpace>())
    {
    }
    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        return accelerationOk(s1, s2) && FlatVampMotionValidator<Robot>::checkMotion(s1, s2);
    }
    bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &last) const override
    {
        if (!accelerationOk(s1, s2))
        {
            last.second = 0.;
            si_->copyState(last.first, s1);
            return false;
        }
        return FlatVampMotionValidator<Robot>::checkMotion(s1, s2, last);
    }
    mutable unsigned long rejected = 0;

private:
    bool accelerationOk(const ob::State *s1, const ob::State *s2) const
    {
        const auto motion = space_->steer(s1, s2);
        if (!motion)
            return true;  // the base validator handles the no-motion case
        Eigen::VectorXd a(7);
        for (double t : {0., motion->duration()})
        {
            motion->evaluate(t, 2u, a);
            for (int j = 0; j < 7; ++j)
                if (std::abs(a[j]) > ACCEL[j] + 1e-9)
                {
                    ++rejected;
                    return false;
                }
        }
        return true;
    }
    const ob::FlatStateSpace *space_;
};

static void writeArray(std::ostream &o, const std::vector<double> &v)
{
    o << '[';
    for (std::size_t i = 0; i < v.size(); ++i)
        o << (i ? "," : "") << std::setprecision(17) << v[i];
    o << ']';
}

static void writePath(std::ostream &o, const ob::FlatStateSpace *space, const og::PathGeometric &path)
{
    // Waypoints (q, v) and segment durations recovered from the authors' trajectory class.
    std::vector<double> Q, V, T;
    Eigen::MatrixXd flat(ORDER, 7);
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        space->toFlatState(path.getState(i), flat);
        for (int j = 0; j < 7; ++j)
        {
            Q.push_back(flat(0, j));
            V.push_back(flat(1, j));
        }
    }
    const ob::FlatTrajectory trajectory(path);
    for (const auto &m : trajectory.motions())
        T.push_back(m.duration());
    o << "\"Q\":";
    writeArray(o, Q);
    o << ",\"V\":";
    writeArray(o, V);
    o << ",\"T\":";
    writeArray(o, T);
}

// Rest proposals (reconstruction, Table II): every derivative level is sampled at zero. Installed through
// the authors' documented extension point (a derivative component's state sampler; FlatStateSpace.h);
// steering, metric, motion validation and RRTConnect stay the authors' code.
class RestDerivativeSampler : public ob::StateSampler
{
public:
    explicit RestDerivativeSampler(const ob::StateSpace *space) : ob::StateSampler(space) {}
    void sampleUniform(ob::State *state) override
    {
        auto *v = state->as<ob::RealVectorStateSpace::StateType>();
        for (unsigned int i = 0; i < space_->getDimension(); ++i)
            v->values[i] = 0.;
    }
    void sampleUniformNear(ob::State *state, const ob::State *, double) override { sampleUniform(state); }
    void sampleGaussian(ob::State *state, const ob::State *, double) override { sampleUniform(state); }
};

static int plan(int argc, char **argv)
{
    if (argc != 7 && argc != 8)
        return 2;
    const bool rest = argc == 8 && std::string(argv[7]) == "rest";
    const std::string problems = argv[2];
    const unsigned long seed = std::stoul(argv[3]);
    const double budget = std::stod(argv[4]);
    const bool accel = std::string(argv[5]) == "1";
    std::ofstream out(argv[6]);
    ompl::RNG::setSeed(seed);
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    std::ifstream in(problems);
    Problem p;
    while (readProblem(in, p))
    {
        const Environment environment(p.world);
        auto output = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
        auto space = std::make_shared<ob::FlatStateSpace>(output, ORDER);
        space->setRho(1.0);  // paper Sec. VII-A: rho = 1
        ob::RealVectorBounds speeds(7);
        for (unsigned int i = 0; i < 7; ++i)
        {
            speeds.setLow(i, -SPEED[i]);
            speeds.setHigh(i, SPEED[i]);
        }
        space->getDerivativeSpace(1u)->setBounds(speeds);
        if (rest)
            space->getDerivativeSpace(1u)->setStateSamplerAllocator(
                [](const ob::StateSpace *s) { return std::make_shared<RestDerivativeSampler>(s); });
        og::SimpleSetup setup(space);
        auto si = setup.getSpaceInformation();
        si->setStateValidityCheckingResolution(1. / (Robot::resolution * output->getMaximumExtent()));
        auto outputInfo = std::make_shared<ob::SpaceInformation>(space->getOutputSpace());
        auto collision = std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(outputInfo, environment);
        setup.setStateValidityChecker([space, collision](const ob::State *s)
                                      { return space->satisfiesBounds(s) &&
                                               collision->isValid(s->as<ob::FlatStateSpace::StateType>()->output()); });
        std::shared_ptr<AccelLimitedValidator> accelValidator;
        std::shared_ptr<ompl::vamp::FlatVampMotionValidator<Robot>> validator;
        if (accel)
        {
            accelValidator = std::make_shared<AccelLimitedValidator>(si, environment);
            validator = accelValidator;
        }
        else
            validator = std::make_shared<ompl::vamp::FlatVampMotionValidator<Robot>>(si, environment);
        si->setMotionValidator(validator);

        Eigen::MatrixXd flat = Eigen::MatrixXd::Zero(ORDER, 7);
        ob::ScopedState<> start(space);
        flat.row(0) = Eigen::Map<const Eigen::RowVectorXd>(p.start.data(), 7);
        space->fromFlatState(flat, start.get());
        setup.addStartState(start);
        auto goal = std::make_shared<ob::GoalStates>(si);
        for (const auto &g : p.goals)
        {
            ob::ScopedState<> gs(space);
            flat.row(0) = Eigen::Map<const Eigen::RowVectorXd>(g.data(), 7);
            space->fromFlatState(flat, gs.get());
            goal->addState(gs);
        }
        setup.setGoal(goal);
#ifdef FLASK_REFERENCE_BUDGETED
        auto planner = std::make_shared<og::BudgetedRRTConnect>(si);
#else
        auto planner = std::make_shared<og::RRTConnect>(si);
#endif
        setup.setPlanner(planner);
        setup.setup();  // outside the timed region, like VAMP's environment construction

        const auto t0 = std::chrono::steady_clock::now();
        const ob::PlannerStatus status = planner->ob::Planner::solve(budget);
        const auto t1 = std::chrono::steady_clock::now();
        const bool solved = status == ob::PlannerStatus::EXACT_SOLUTION;
        double range = planner->getRange();
        ob::PlannerData data(si);
        planner->getPlannerData(data);

        out << "{\"environment\":\"" << p.env << "\",\"problem\":" << p.index << ",\"seed\":" << seed
#if defined(FLASK_REFERENCE_AUTHOR_FIX)
            << ",\"method\":\"author_flask_rrt_connect_author_fix\",\"termination_patch\":\"author_branch_fix-rrt-connect-loop_3026754e\""
#elif defined(FLASK_REFERENCE_BUDGETED)
            << ",\"method\":\"author_flask_rrt_connect_budgeted\",\"termination_patch\":true"
#else
            << ",\"method\":\"author_flask_rrt_connect\",\"termination_patch\":false"
#endif
            << ",\"rest_samples\":" << (rest ? "true" : "false")
            << ",\"accel_limits\":" << (accel ? "true" : "false")
            << ",\"solved\":" << (solved ? "true" : "false")
            << ",\"plan_ns\":" << std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()
            << ",\"range\":" << range << ",\"vertices\":" << data.numVertices()
            << ",\"valid_motions\":" << validator->getValidMotionCount()
            << ",\"invalid_motions\":" << validator->getInvalidMotionCount()
            << ",\"accel_rejections\":" << (accelValidator ? accelValidator->rejected : 0);
        if (solved)
        {
            auto &path = *setup.getProblemDefinition()->getSolutionPath()->as<og::PathGeometric>();
            out << ",\"raw_states\":" << path.getStateCount() << ",\"raw\":{";
            writePath(out, space.get(), path);
            out << "}";
            og::PathSimplifier simplifier(si);
            const auto s0 = std::chrono::steady_clock::now();
            simplifier.reduceVertices(path);
            const auto s1 = std::chrono::steady_clock::now();
            out << ",\"simplify_ns\":" << std::chrono::duration_cast<std::chrono::nanoseconds>(s1 - s0).count()
                << ",\"states\":" << path.getStateCount() << ",\"path_check\":" << (path.check() ? "true" : "false")
                << ",";
            writePath(out, space.get(), path);
        }
        out << "}\n";
        out.flush();
    }
    return 0;
}

static int bvp(int argc, char **argv)
{
    if (argc != 5)
        return 2;
    std::ifstream in(argv[2]);
    const double rho = std::stod(argv[3]);
    std::ofstream out(argv[4]);
    ob::MinimumEffortSteering steering(ORDER);
    steering.setRho(rho);
    Eigen::MatrixXd a(ORDER, 7), b(ORDER, 7);
    while (true)
    {
        for (int l = 0; l < ORDER; ++l)
            for (int j = 0; j < 7; ++j)
                in >> a(l, j);
        for (int l = 0; l < ORDER; ++l)
            for (int j = 0; j < 7; ++j)
                in >> b(l, j);
        if (!in)
            break;
        const auto m = steering.steer(a, b);
        out << "{\"found\":" << (m ? "true" : "false");
        if (m)
        {
            out << ",\"T\":" << std::setprecision(17) << m->duration()
                << ",\"cost\":" << steering.cost(*m);
            Eigen::VectorXd y(7);
            m->evaluate(0.5 * m->duration(), 0u, y);
            out << ",\"mid\":[";
            for (int j = 0; j < 7; ++j)
                out << (j ? "," : "") << y[j];
            out << "]";
        }
        out << "}\n";
    }
    return 0;
}

// v10 acceptance V2: the authors' steering + FlatVampMotionValidator on given edges in one problem's scene.
//   mbm_flat edges <problem.txt> <out.jsonl>   (problem block, then N, then N lines q0 v0 q1 v1)
static int edges(int argc, char **argv)
{
    if (argc != 4)
        return 2;
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
    std::ifstream in(argv[2]);
    std::ofstream out(argv[3]);
    Problem p;
    if (!readProblem(in, p))
        return 3;
    const Environment environment(p.world);
    auto output = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
    auto space = std::make_shared<ob::FlatStateSpace>(output, ORDER);
    space->setRho(1.0);
    ob::RealVectorBounds speeds(7);
    for (unsigned int i = 0; i < 7; ++i)
    {
        speeds.setLow(i, -SPEED[i]);
        speeds.setHigh(i, SPEED[i]);
    }
    space->getDerivativeSpace(1u)->setBounds(speeds);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityCheckingResolution(1. / (Robot::resolution * output->getMaximumExtent()));
    auto outputInfo = std::make_shared<ob::SpaceInformation>(space->getOutputSpace());
    auto collision = std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(outputInfo, environment);
    si->setStateValidityChecker([space, collision](const ob::State *s)
                                { return space->satisfiesBounds(s) &&
                                         collision->isValid(s->as<ob::FlatStateSpace::StateType>()->output()); });
    auto validator = std::make_shared<ompl::vamp::FlatVampMotionValidator<Robot>>(si, environment);
    si->setMotionValidator(validator);
    si->setup();
    int count;
    in >> count;
    Eigen::MatrixXd a(ORDER, 7), b(ORDER, 7);
    ob::ScopedState<> s1(space), s2(space);
    for (int k = 0; k < count; ++k)
    {
        for (int l = 0; l < ORDER; ++l)
            for (int j = 0; j < 7; ++j)
                in >> a(l, j);
        for (int l = 0; l < ORDER; ++l)
            for (int j = 0; j < 7; ++j)
                in >> b(l, j);
        if (!in)
            return 4;
        space->fromFlatState(a, s1.get());
        space->fromFlatState(b, s2.get());
        const auto m = space->steer(s1.get(), s2.get());
        const bool valid = validator->checkMotion(s1.get(), s2.get());
        out << "{\"found\":" << (m ? "true" : "false") << ",\"T\":" << std::setprecision(17)
            << (m ? m->duration() : -1.) << ",\"valid\":" << (valid ? "true" : "false") << "}\n";
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::string(argv[1]) == "plan")
        return plan(argc, argv);
    if (argc > 1 && std::string(argv[1]) == "bvp")
        return bvp(argc, argv);
    if (argc > 1 && std::string(argv[1]) == "edges")
        return edges(argc, argv);
    std::cerr << "usage: mbm_flat plan|bvp ...\n";
    return 2;
}
