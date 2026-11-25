"""Test e2: FLASK implementation and evaluation utilities."""
import numpy as np
import pytest
import vamp
from flask_repro import _core, mbm, native, robot as R, reference as TC
pytestmark = pytest.mark.skipif(_core is None, reason='Native extension unavailable; run ./build.sh')
UR5 = R.get_robot('ur5')
L_ = lambda x: [float(v) for v in x]

def _problem(environment_name, i):
    for (name, k, s, g, ev, ec) in mbm.paired_problems('ur5', [environment_name]):
        if k == i:
            return (s, g, ev, ec)
    raise KeyError(i)

@pytest.mark.parametrize('environment_name,i', [('table_pick', 0), ('bookshelf_thin', 0), ('box', 1)])
def test_plan_from_state_state_moving_moving(environment_name, i):
    """Test plan from state state moving moving."""
    (s, g, ev, ec) = _problem(environment_name, i)
    rng = np.random.default_rng(3)
    limits = native.joint_limits(UR5)
    attempted = 0
    for _ in range(50):
        v0 = rng.uniform(-1, 1, UR5.n) * 0.3 * UR5.a_max * 0.2
        d = native.plan(UR5, ec, s, g, native.settings(max_iterations=100000), seed=2, v_start=v0, limits=limits)
        if not d['solved']:
            continue
        attempted += 1
        for trajectory in (d['raw_trajectory'], d['trajectory']):
            assert np.allclose(trajectory.Q[0], s) and np.allclose(trajectory.V[0], v0)
            assert any((np.allclose(trajectory.Q[-1], x) for x in g)) and np.allclose(trajectory.V[-1], 0)
            assert TC.validate_trajectory(trajectory, UR5, ev)
        if attempted >= 3:
            break
    assert attempted >= 3

def test_v_start_none_equal_not():
    (s, g, ev, ec) = _problem('table_pick', 1)
    settings = native.settings()
    a = native.plan(UR5, ec, s, g, settings, seed=7)
    b = native.plan(UR5, ec, s, g, settings, seed=7, v_start=np.zeros(UR5.n))
    assert a['iterations'] == b['iterations'] and np.array_equal(a['Q'], b['Q'])

def test_validate_interval_by_reference_python():
    """Validate a trajectory interval with the independent Python implementation."""
    (s, g, ev, ec) = _problem('bookshelf_thin', 0)
    limits = native.joint_limits(UR5)
    m = native.module(UR5)
    rng = np.random.default_rng(5)
    accepted_count = 0
    for k in range(40):
        q0 = s + rng.normal(0, 0.3, UR5.n)
        q1 = g[0] + rng.normal(0, 0.3, UR5.n)
        mid = 0.5 * (q0 + q1) + rng.normal(0, 0.4, UR5.n)
        Q = np.array([q0, mid, q1])
        V = np.array([np.zeros(6), rng.uniform(-0.5, 0.5, 6), np.zeros(6)])
        T = np.array([TC.bvp(Q[0], V[0], Q[1], V[1]).T, TC.bvp(Q[1], V[1], Q[2], V[2]).T])
        trajectory = TC.Trajectory(Q, V, T)
        (ta, tb) = sorted(rng.uniform(0, T.sum(), 2))
        if k % 4 == 0:
            tb = np.inf
        py = True
        critical_points = 0.0
        for (i, c) in enumerate(trajectory.edges()):
            (a, b) = (max(ta - critical_points, 0.0), min(tb - critical_points, c.T))
            if b - a > 1e-09:
                cc = c.slice(a, b) if a > 0 or b < c.T else c
                py = py and TC.flask_cc(cc, UR5, ev, check_limits=False)
            critical_points += c.T
        (cpp, _) = m.validate_interval(ec, Q.tolist(), V.tolist(), T.tolist(), ta, tb, limits)
        accepted_count += cpp
        assert cpp == py, (k, ta, tb)
    assert 0 < accepted_count < 40

@pytest.mark.parametrize('point_count,half_extents,r', [(60, 0.6, 0.0025), (25, 0.6, 0.04), (300, 0.9, 0.0025), (300, 0.9, 0.03)])
def test_capt_equal_set_sphere(point_count, half_extents, r):
    """Test capt equal set sphere."""
    rng = np.random.default_rng(point_count)
    (r_min, r_max) = vamp.ur5.min_max_radii()
    P = np.column_stack([rng.uniform(-half_extents, half_extents, point_count), rng.uniform(-half_extents, half_extents, point_count), rng.uniform(0.9, 0.9 + half_extents, point_count)]).astype(np.float32)
    e_capt = _core.Environment()
    sphere_environment = _core.Environment()
    (l, x, n) = e_capt.add_pointcloud(P, r_min, r_max, r)
    assert n == point_count and x > 0 and (l == 0)
    for p in P:
        sphere_environment.add_sphere(L_(p), r)
    difference = 0
    accepted_count = 0
    for _ in range(4000):
        q = rng.uniform(UR5.q_lo, UR5.q_hi)
        a = _core.ur5.validate(e_capt, L_(q))
        b = _core.ur5.validate(sphere_environment, L_(q))
        accepted_count += a
        difference += a != b
    assert 100 < accepted_count < 3900, accepted_count
    assert difference == 0

def test_capt_filtered_and_empty():
    e = _core.Environment()
    (r_min, r_max) = vamp.ur5.min_max_radii()
    xa = np.array([[5.0, 5.0, 5.0], [5.0, 5.01, 5.0]], np.float32)
    (l, x, n) = e.add_pointcloud(xa, r_min, r_max, 0.03, 0.005)
    assert n in (0, 2)
    assert _core.ur5.validate(e, [0.0] * 6)
    nearest = np.array([[0.3, 0.3, 1.2], [0.3, 0.3, 1.201], [0.4, 0.3, 1.2]], np.float32)
    e2 = _core.Environment()
    (l, x, n) = e2.add_pointcloud(nearest, r_min, r_max, 0.03, 0.005)
    assert 2 <= n <= 3 and x > 0
    assert e2.prepare() > 0

def test_quadratic_braking_path():
    """Constant deceleration produces a quadratic braking trajectory."""
    v = np.array([0.8, -1.2, 0.5, 2.0, -0.3, 0.1])
    q = np.zeros(6)
    Tb = float(np.max(np.abs(v) / UR5.a_max))
    c = TC.Edge(q, v, q + v * Tb / 2, np.zeros(6), Tb)
    (c3, c2, _, _) = c.coef
    assert np.allclose(c3, 0, atol=1e-12) and np.allclose(2 * c2, -v / Tb)
    assert np.all(np.abs(2 * c2) <= UR5.a_max + 1e-09)
