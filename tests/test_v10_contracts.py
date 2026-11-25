"""v10 contracts. Every option defaults to the earlier behaviour; each test carries a negative control."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from flask_repro.dynobench import load
from flask_repro.planning import PlannerSettings, plan, validate


def _quad3d_free(yaw0, yaw1, order=4):
    model, planning, original, start, goal, _ = load('quad3d_window')
    model.order = order
    # obstacle-free copy of the same bounds: the direct start->goal BVP is then accepted when feasible
    from flask_repro.flat_planner import World
    free = World(original.lower, original.upper, [], [], original.radius)
    free.prepare_simd()
    s, g = start.copy(), goal.copy()
    s[3:7] = Rotation.from_euler('z', yaw0).as_quat()
    g[3:7] = Rotation.from_euler('z', yaw1).as_quat()
    g[:3] = s[:3] + np.array([.5, 0., 0.])
    s[7:], g[7:] = 0., 0.
    return model, free, free, s, g


def _yaw_travel(record):
    jet = record.trajectory.jet(np.linspace(0, record.trajectory.duration, 400), through=1)
    return float(np.sum(np.abs(np.diff(jet[:, 0, 3])))), jet[:, 0, 3]


@pytest.mark.parametrize('order', [2, 4])
def test_yaw_winding_takes_the_short_way_and_stays_continuous(order):
    model, world, original, s, g = _quad3d_free(3.0, -3.0, order)
    base = dict(algorithm='rrt_connect', rrt_mode=2, budget_s=2., max_nodes=100000)
    plain = plan(model, world, s, g, PlannerSettings(**base), seed=1)
    wound = plan(model, world, s, g, PlannerSettings(**base, yaw_winding=True), seed=1)
    assert plain.solved and wound.solved
    travel_plain, _ = _yaw_travel(plain)
    travel_wound, yaw = _yaw_travel(wound)
    assert travel_plain > 5.5                     # negative control: the real-line yaw turns ~6 rad
    assert travel_wound < 2 * np.pi - 5.5         # short way: 2*pi - 6 = 0.28 rad
    assert np.max(np.abs(np.diff(yaw))) < .05     # no 2*pi jump between edges
    v = validate(wound, model, original, PlannerSettings(**base, yaw_winding=True), allow_reduced_model=order < 4)
    assert v['valid'], v['errors']


def test_yaw_winding_off_by_default():
    assert PlannerSettings().yaw_winding is False


def test_preflight_rejects_efficiency_cores_and_ignores_usb_power():
    from benchmarks.measurement_env import check
    snap = dict(measurement_cpu=20, per_cpu={20: dict(energy_performance_preference='performance')}, power_profile='performance',
                no_turbo='0', smt_siblings=[], cpu_load={20: 0.},
                power_supply={'ucsi-source-psy-USBC000:001/online': '0', 'ucsi-source-psy-USBC000:001/type': 'USB'},
                cpuinfo_max_khz={'10': '5800000', '20': '4300000'})
    problems = check(snap)
    assert any('not a performance core' in p for p in problems)
    assert not any('AC power' in p for p in problems)            # USB-C device sources are not mains
    snap.update(measurement_cpu=10, per_cpu={10: dict(energy_performance_preference='performance')}, cpu_load={10: 0.})
    assert check(snap) == []
    snap['power_supply'] = {'ADP1/online': '0', 'ADP1/type': 'Mains'}           # negative control: laptop on battery
    assert any('AC power' in p for p in check(snap))


def test_reactive_online_policies_are_explicit():
    import inspect
    import sys
    sys.path.insert(0, 'experiments'); sys.path.insert(0, 'simulation')
    from experiments import ur5_reactive_mujoco as E2
    sig = inspect.signature(E2.run_online)
    assert sig.parameters['obstacle_policy'].default == 'yield'          # v7 behaviour kept as the default
    assert sig.parameters['late_plan_policy'].default == 'insert'
    with pytest.raises(ValueError):
        E2.run_online(0, None, None, None, None, None, None, None, obstacle_policy='cooperative')


def test_native_kernel_rejects_quad3d_torque_above_the_limit():
    """Yaw-only rest-to-rest order-4 edge: torque_z = J_z * psi_ddot. Limit just below / above the exact peak."""
    from flask_repro import _core
    from flask_repro.flat_models import Quad3D
    from flask_repro.flat_planner import World
    world = World([0., 0., 0.], [4., 4., 4.], [], [], .1); world.prepare_simd()
    a = [[2., 2., 2., 0.]] + [[0.] * 4] * 3
    b = [[2., 2., 2., 2.5]] + [[0.] * 4] * 3
    e = _core.flat_bvp(a, b, 1., 2.)                        # fixed duration 2 s
    c = np.asarray(e['coefficients']); T = e['duration']
    s = np.linspace(0, 1, 4001)
    jet = np.zeros((len(s), 5, 4))
    for level in range(5):
        for k in range(level, c.shape[0]):
            fall = np.prod(np.arange(k - level + 1, k + 1)) if level else 1
            jet[:, level] += np.outer(fall * s ** (k - level), c[k]) / T ** level
    peak = float(np.max(np.linalg.norm(Quad3D().recover(jet)['torque'], axis=-1)))
    assert peak > .05
    def check(limit):
        cfg = PlannerSettings(validation_dt=.002).native(); cfg.max_torque = limit; cfg.max_yaw_rate = 100.; cfg.max_speed = 100.
        return _core.flat_check_edges('quad3d', world.cpp_environment, [e['coefficients']], [T],
                                      world.lower.tolist(), world.upper.tolist(), world.radius, cfg, 1)['valid'][0]
    assert check(1.05 * peak)                                 # negative control: within the limit
    assert not check(.9 * peak)                               # violated torque norm is rejected


def test_lqmt_cost_triangle_inequality_behind_alg4():
    """Alg. 4 replaces a chain by one optimal BVP; with the global T* the cost can only decrease."""
    from flask_repro import _core
    rng = np.random.default_rng(10)
    for _ in range(300):
        z = [rng.normal(size=(2, 2)) for _ in range(3)]
        cost = lambda p, q: _core.flat_bvp(p.tolist(), q.tolist(), 1.)['effort'] + _core.flat_bvp(p.tolist(), q.tolist(), 1.)['duration']
        assert cost(z[0], z[2]) <= cost(z[0], z[1]) + cost(z[1], z[2]) + 1e-9


def test_interval_oracle_quad2d_optional_speed_limit():
    """v10 Quad2D model (|v|<=4): constant 5 m/s cruise is clear without limits, rejected with them."""
    import numpy as np
    from flask_repro.interval_validation import audit_segment
    coefficients = np.zeros((8, 2)); coefficients[1, 0] = 5.
    assert audit_segment(coefficients, 1., 'quad2d')['status'] == 'clear'
    rejected = audit_segment(coefficients, 1., 'quad2d', limits=dict(velocity_max=4.))
    assert rejected['status'] == 'constraint_violation' and rejected['constraint'] == 'speed_squared'
    assert audit_segment(coefficients, 1., 'quad2d', limits=dict(velocity_max=6.))['status'] == 'clear'
    with pytest.raises(ValueError):
        audit_segment(np.zeros((8, 4)), 1., 'quad3d', limits=dict(velocity_max=1.))


def test_default_validation_profile_is_peak_speed():
    """Acceptance V2: the default edge sample count is the authors' peak-speed count, not arc length."""
    from flask_repro import native, robot as R
    rb = R.get_robot('panda')
    assert native.joint_limits(rb).bound_validation_speed
    assert not native.joint_limits(rb, 'published_resolution').bound_validation_speed


def test_alg4_never_increases_search_cost():
    """Acceptance V3d: returned cost <= cost of the search's best path (Alg. 4 relies on the triangle inequality)."""
    import json
    from pathlib import Path
    from flask_repro.dynobench import load
    from flask_repro.planning import PlannerSettings, plan
    p = json.loads((Path(__file__).resolve().parents[1] / 'config/low_dof_v10_order2.json').read_text())
    o = p['case_overrides']['quad2d_hole']
    model, planning, _, start, goal, _ = load('quad2d_hole', **p['geometry'], lift_speed=p['boundary']['lift_speed'])
    for k, v in (p.get('model', {}) | o.get('model', {})).items():
        setattr(model, k, v)
    for seed in range(1001, 1006):
        cfg = PlannerSettings(**(p['settings'] | o['settings'] | dict(algorithm='sst_bvp', budget_s=.3, stop_first=False)))
        rec = plan(model, planning, start, goal, cfg, seed)
        assert rec.solved
        assert rec.trajectory.cost(cfg.rho) <= rec.diagnostics['search_cost'] * (1 + 1e-9)
