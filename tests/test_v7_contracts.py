"""v7 options: certified Panda environment checking, SST* graph persistence, spatial-only sampling,
fixed-grid TOPP-RA. Each has a negative control; defaults keep earlier behaviour."""
import json
from pathlib import Path

import numpy as np
import pytest
import vamp

from flask_repro import certify, mbm, native, robot
from flask_repro.dynobench import load
from flask_repro.planning import PlannerSettings, plan

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('name', ['panda', 'ur5'])
def test_sweep_bound_dominates_sphere_displacement(name):
    rb = robot.get_robot(name)
    R = np.asarray(certify.reach(name))
    mod = getattr(vamp, name)
    rng = np.random.default_rng(3)
    for _ in range(400):
        q = rng.uniform(rb.q_lo, rb.q_hi)
        dq = rng.normal(size=rb.n) * rng.uniform(1e-4, .05)
        a = np.array([[s.x, s.y, s.z] for s in mod.fk(list(q))])
        b = np.array([[s.x, s.y, s.z] for s in mod.fk(list(q + dq))])
        assert np.max(np.linalg.norm(a - b, axis=1)) <= np.sum(R * np.abs(dq)) + 1e-9
    # negative control: dropping the base joint's reach breaks the bound for a pure base rotation
    q = np.zeros(rb.n); dq = np.zeros(rb.n); dq[0] = .01
    a = np.array([[s.x, s.y, s.z] for s in mod.fk(list(q))]); b = np.array([[s.x, s.y, s.z] for s in mod.fk(list(q + dq))])
    assert np.max(np.linalg.norm(a - b, axis=1)) > np.sum(np.r_[0., R[1:]] * np.abs(dq))


def _known_colliding():
    """First colliding FLASK path of the stored v2 Panda campaign (copied to tests/fixtures)."""
    f = Path(__file__).resolve().parent / 'fixtures'
    row = json.loads((f / 'known_colliding_panda_v2.json').read_text())
    with np.load(f / 'known_colliding_panda_v2.npz') as z:
        return row, z['Q'].copy(), z['V'].copy(), z['T'].copy()


def test_certified_rejects_stored_collision_that_nominal_grid_accepts():
    row, Q, V, T = _known_colliding()
    rb = robot.get_robot('panda'); mod = native.module(rb)
    problem = mbm.load_dataset('panda')[row['environment']][row['problem']]
    nominal_env = mbm.native_environment(problem)
    nominal = native.joint_limits(rb, 'published_resolution')  # the grid that accepted it
    assert all(mod.flask_cc(nominal_env, Q[i].tolist(), V[i].tolist(), Q[i + 1].tolist(), V[i + 1].tolist(), float(T[i]), nominal, False)
               for i in range(len(T)))  # the planner's own grid accepted it
    certified = native.joint_limits(rb, 'certified')
    inflated = mbm.native_environment(certify.inflate_problem(problem, certified.certify_delta))
    assert not all(mod.flask_cc(inflated, Q[i].tolist(), V[i].tolist(), Q[i + 1].tolist(), V[i + 1].tolist(), float(T[i]), certified, False)
                   for i in range(len(T)))


def test_certified_planning_returns_paths_clear_of_original_scene():
    rb = robot.get_robot('panda')
    problem = mbm.load_dataset('panda')['table_pick'][1]
    limits = native.joint_limits(rb, 'certified')
    inflated = mbm.native_environment(certify.inflate_problem(problem, limits.certify_delta))
    s = native.settings(prefix_steering=False, velocity_sample_scale=0., max_ns=10**9, max_samples=10**5)
    r = native.plan(rb, inflated, problem['start'], problem['goals'], s, seed=1001, limits=limits)
    assert r['solved']
    path = r['trajectory']
    Q = path.evaluate(np.linspace(0, path.duration(), 4000))[0]
    assert native.module(rb).count_collisions(mbm.native_environment(problem), Q.tolist())[0] == 0


def test_sst_phase_persistence_keeps_graph():
    model, planning, original, s, g, _ = load('quad2d_hole')
    base = dict(algorithm='sst_simd', budget_s=30., max_iterations=3000, phase_iterations=200, stop_first=False,
                goal_tolerance=1e-9, witness_radius=.05)
    restart = plan(model, planning, s, g, PlannerSettings(**base, phase_restart=True), 7)
    keep = plan(model, planning, s, g, PlannerSettings(**base, phase_restart=False), 7)
    assert restart.diagnostics['phase_restart_iterations'] and keep.diagnostics['phase_restart_iterations']
    assert keep.nodes > restart.nodes  # restarting discards the tree; persistence accumulates it


def test_spatial_only_validation_needs_fewer_batches():
    from flask_repro import _core
    model, planning, original, s, g, _ = load('quad3d_window')
    planning.prepare_simd()
    a = [[4., 1., 2., 0.], [0.] * 4, [0.] * 4, [0.] * 4]   # hover at the start, slow move to a nearby hover
    b = [[4., 1.4, 2., 0.], [0.] * 4, [0.] * 4, [0.] * 4]
    e = _core.flat_bvp(a, b, 1.)
    counts = {}
    for spatial in (False, True):
        cfg = PlannerSettings(spatial_only_validation=spatial).native()
        out = _core.flat_check_edges('quad3d', planning.cpp_environment, [e['coefficients']], [e['duration']],
                                     planning.lower.tolist(), planning.upper.tolist(), planning.radius, cfg, 1)
        assert out['valid'] == [True]
        counts[spatial] = out['batches']
    assert counts[True] < counts[False]


def test_fixed_grid_toppra():
    from benchmarks.mbm_baseline import topp_ra
    rb = robot.get_robot('panda')
    wp = np.cumsum(np.random.default_rng(0).normal(size=(6, 7)) * .3, axis=0)
    auto, fixed = topp_ra(wp, rb), topp_ra(wp, rb, gridpoints=101)
    assert abs(fixed.duration / auto.duration - 1) < .05
    with pytest.raises(Exception):
        topp_ra(wp[:1], rb, gridpoints=101)  # negative control: a single waypoint is rejected
