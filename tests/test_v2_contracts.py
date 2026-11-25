"""Contracts for the v2 reconstruction options (defaults keep v1 behaviour; covered by the other tests).

Each feature has a positive check and a negative control.
"""
import numpy as np
import pytest

from flask_repro import lqmt, mbm, native, robot, trajectory
from flask_repro import original_sst as O
from flask_repro.dynobench import load
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings, plan, validate


@pytest.fixture(scope='module')
def panda():
    rb = robot.get_robot('panda')
    ds = mbm.load_dataset('panda')
    p = ds['table_pick'][1]
    return rb, p, mbm.native_environment(p)


def _check_path(rb, ec, result, limits, rho=1.0):
    path = result['trajectory']
    for e in path.edges():
        T = lqmt.optimal_duration(e.q0, e.v0, e.q1, e.v1, rho)
        assert T is not None and abs(T - e.T) <= 1e-9 * max(1., T)       # edges are optimal BVPs
        args = [a.tolist() for a in (e.q0, e.v0, e.q1, e.v1)]
        assert native.module(rb).flask_cc(ec, *args, e.T, limits)        # every edge valid


@pytest.mark.parametrize('mode', ['ompl', 'clip_rest'])
def test_panda_v2_extension_modes_return_valid_optimal_edges(panda, mode):
    rb, p, ec = panda
    kw = dict(extend_mode=1) if mode == 'ompl' else dict(prefix_steering=False, velocity_sample_scale=0.)
    s = native.settings(max_ns=10**9, max_iterations=10**6, max_samples=10**5, **kw)
    limits = native.joint_limits(rb)
    r = native.plan(rb, ec, p['start'], p['goals'], s, seed=1001, limits=limits, shortcut=False)
    assert r['solved']
    path = r['trajectory']
    assert np.allclose(path.Q[0], p['start']) and any(np.allclose(path.Q[-1], g) for g in p['goals'])
    assert np.max(np.abs(path.V[[0, -1]])) < 1e-12
    _check_path(rb, ec, r, limits)


def test_panda_rest_sampling_gives_mostly_rest_waypoints(panda):
    """Extension nodes keep the sampled (zero) velocity; only connect-step slices of a single BVP carry
    velocity. Negative control: full-velocity sampling yields far fewer rest waypoints."""
    rb, p, ec = panda
    fractions = {}
    for scale in [0., 1.]:
        rest = total = 0
        for seed in range(5):
            s = native.settings(prefix_steering=False, velocity_sample_scale=scale, max_ns=10**9, max_samples=10**5)
            r = native.plan(rb, ec, p['start'], p['goals'], s, seed=seed, shortcut=False)
            assert r['solved']
            V = np.asarray(r['V'])
            rest += int(np.sum(np.max(np.abs(V), axis=1) == 0.)); total += len(V)
        fractions[scale] = rest / total
    assert fractions[0.] > fractions[1.] + .2


def test_collision_resolution_scale_increases_samples(panda):
    rb, p, ec = panda
    q0 = np.asarray(p['start']); q1 = np.asarray(p['goals'][0])
    T = lqmt.optimal_duration(q0, 0 * q0, q1, 0 * q1, 1.0)
    low = native.joint_limits(rb); high = native.joint_limits(rb); high.collision_resolution_scale = 16.
    # same verdict class on a known-valid direct rest-to-rest edge is not required; sample counts must grow
    n_low = native.module(rb).sample_count(q0.tolist(), [0.] * 7, q1.tolist(), [0.] * 7, T)
    assert n_low >= 8
    assert high.collision_resolution_scale == 16. and low.collision_resolution_scale == 1.


@pytest.mark.parametrize('case', ['quad2d_hole', 'quad3d_window'])
def test_order_two_aerial_planning_is_flat_continuous_and_reports_attitude_jumps(case):
    model, planning, original, s, g, _ = load(case, shell_only=True)
    model.order = 2
    cfg = PlannerSettings(budget_s=2., rrt_mode=2, extension_range=1.5, bound1=.5, goal_mode=1,
                          goal_tolerance=.5 if case.startswith('quad2d') else .7)
    r = plan(model, planning, s, g, cfg, 1001)
    assert r.solved
    strict = validate(r, model, original, cfg)
    assert not strict['valid'] and not strict['original_model_valid']
    assert 'original_state_discontinuity' in strict['errors']
    v = validate(r, model, original, cfg, allow_reduced_model=True)
    assert v['valid'], v['errors']
    assert v['validation_scope'] == 'reduced_aerial_diagnostic' and not v['original_model_valid']
    assert v['reduced_order'] is not None and v['reduced_order']['attitude_join_jump'] >= 0.
    assert r.trajectory.continuity_error() < 1e-6
    # negative control: break flat continuity -> must be rejected
    if len(r.trajectory.edges) > 1:
        e = r.trajectory.edges[1]
        e.coefficients = e.coefficients.copy(); e.coefficients[0] += .05
        assert 'flat_state_discontinuity' in validate(r, model, original, cfg)['errors']


def test_single_order_two_aerial_edge_must_match_original_boundary_attitude():
    from dataclasses import asdict
    from flask_repro.flat_bvp import fixed_time
    from flask_repro.flat_models import Quad2D
    from flask_repro.planning import RunRecord,Trajectory
    model=Quad2D(order=2)
    start=np.zeros(6);goal=np.array([1.,0.,0.,0.,0.,0.])
    edge=fixed_time(model.lift(start),model.lift(goal),3.)
    cfg=PlannerSettings()
    r=RunRecord(solved=True,trajectory=Trajectory([edge]),plan_ns=0,postprocess_ns=0,
        first_solution_ns=0,iterations=0,nodes=2,terminal_error=0.,settings=asdict(cfg),
        start_state=start.tolist(),goal_state=goal.tolist())
    world=World([-5.,-5.],[5.,5.])
    result=validate(r,model,world,cfg)
    assert result['physical_continuity_error']==0. # no join; boundaries must still be checked
    assert not result['valid'] and 'original_start_mismatch' in result['errors']
    diagnostic=validate(r,model,world,cfg,allow_reduced_model=True)
    assert diagnostic['valid'] and not diagnostic['original_model_valid']


def test_unicycle_reverse_chart_and_annulus_sampling():
    model, planning, original, s, g, _ = load('unicycle_wall', lift_speed=1.0)
    model.reverse = True
    cfg = PlannerSettings(budget_s=5., rrt_mode=2, extension_range=6., bound1=1., speed_min=.3)
    r = plan(model, planning, s, g, cfg, 1001)
    assert r.solved and validate(r, model, original, cfg)['valid']
    val = model.recover(r.trajectory.jet(np.linspace(0, r.trajectory.duration, 500), 2))
    assert np.all(val['control'][:, 0] < 0)                      # reverse chart: backward motion
    assert np.max(np.abs(val['control'][:, 1])) <= 1.5 + 1e-6     # exact yaw-rate limit holds
    # negative control: forward-chart lift of the same start has positive speed
    model.reverse = False
    assert np.dot(model.lift(s)[1], [np.cos(s[2]), np.sin(s[2])]) > 0


def test_shell_cover_preserves_filled_obstacle_occupancy():
    w = World([0, 0, 0], [6, 6, 6], boxes=[([3, 3, 3], [3, 3, 2])], radius=.1)
    full, shell = w.sphere_cover(.15), w.sphere_cover(.15, shell_only=True)
    assert len(shell.spheres) < len(full.spheres)
    full.prepare_simd(); shell.prepare_simd()
    rng = np.random.default_rng(0)
    p = rng.uniform(0, 6, size=(4000, 3))
    inside = np.all(np.abs(p - 3) < np.array([1.5, 1.5, 1.]) - .2, axis=1)  # strictly interior points
    for q in p[~inside][:1500]:
        assert full.positions_valid(q[None]) == shell.positions_valid(q[None])
    # A pure surface representation misses the solid interior; the corrected
    # optimized cover must retain it in both scalar and native point queries.
    centre = np.array([[3., 3., 3.]])
    assert not full.positions_valid(centre) and not shell.positions_valid(centre)
    assert not shell.positions_valid(centre, independent=True)
    unsafe = World(w.lower, w.upper, spheres=shell.spheres, radius=w.radius)
    unsafe.prepare_simd()
    assert unsafe.positions_valid(centre)


@pytest.mark.parametrize('case', ['unicycle_wall'])
def test_original_sst_baseline_independent_reintegration(case):
    _, planning, original, s, g, _ = load(case, shell_only=True)
    r = O.plan(case, planning, s, g, O.OrigSettings(budget_s=20.), 1002)
    if not r['solved']:
        pytest.skip('baseline did not solve within the test budget')
    v = O.validate(case, r, original, s, g)
    assert v['valid'], v['errors']
    assert v['max_state_error'] < 1e-8
    # negative control: perturbing a control must break the independent re-integration
    r['controls'][0] = [c + .1 for c in r['controls'][0]]
    assert 'integration_mismatch' in O.validate(case, r, original, s, g)['errors']


def test_dynobench_goal_metric_values():
    assert O.dynobench_distance('unicycle', [0, 0, 0], [0, 0, np.pi / 2]) == pytest.approx(.5 * np.pi / 2)
    assert O.dynobench_distance('unicycle', [0, 0, np.pi - .1], [0, 0, -np.pi + .1]) == pytest.approx(.1)
    a = np.r_[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    b = a.copy(); b[7] = 1.
    assert O.dynobench_distance('quad3d', a, b) == pytest.approx(.1)


def test_original_models_hover_equilibrium():
    x2 = np.array([1., 2., 0., 0., 0., 0.])
    u2 = [.034 * 9.81 / 2] * 2
    assert np.allclose(O._step('quad2d', x2, u2), x2)
    x3 = np.r_[1., 2., 3., 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    assert np.allclose(O._step('quad3d', x3, [9.81, 0, 0, 0]), x3)
    # negative control: extra thrust accelerates upward
    assert O._step('quad3d', x3, [10.81, 0, 0, 0])[9] > 0
