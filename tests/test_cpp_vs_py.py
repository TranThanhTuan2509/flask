"""Test cpp vs py: FLASK implementation and evaluation utilities."""
import numpy as np
import pytest
import vamp
from flask_repro import _core, lqmt as L, mbm, native, robot as R, reference as TC
pytestmark = pytest.mark.skipif(_core is None, reason='Native extension unavailable; run ./build.sh')
PANDA = R.get_robot('panda')
UR5 = R.get_robot('ur5')
L_ = lambda x: [float(v) for v in x]

def _problem(robot, environment_name, i):
    for (name, k, s, g, ev, ec) in mbm.paired_problems(robot, [environment_name]):
        if k == i:
            return (s, g, ev, ec)
    raise KeyError(i)

def _random_edge(rng, rb, s, g, v=0.3, noise=0.3):
    q0 = s + rng.normal(0, noise, rb.n)
    q1 = g + rng.normal(0, noise, rb.n)
    v0 = rng.uniform(-1, 1, rb.n) * rb.v_max * v
    v1 = rng.uniform(-1, 1, rb.n) * rb.v_max * v
    return (q0, v0, q1, v1)

@pytest.mark.parametrize('rb', [PANDA, UR5], ids=['panda', 'ur5'])
def test_lqmt_10k(rb):
    rng = np.random.default_rng(10)
    m = native.module(rb)
    for k in range(10000):
        sc = [0.01, 1.0, 5.0][k % 3]
        (q0, v0, q1, v1) = (rng.uniform(-np.pi, np.pi, rb.n), rng.uniform(-sc, sc, rb.n), rng.uniform(-np.pi, np.pi, rb.n), rng.uniform(-sc, sc, rb.n))
        if k % 50 == 0:
            q1 = q0.copy()
        rho = [1.0, 0.3, 3.0][k % 3]
        (T, c3, c2, C) = m.lqmt(L_(q0), L_(v0), L_(q1), L_(v1), rho)
        Tp = L.optimal_duration(q0, v0, q1, v1, rho)
        assert T == pytest.approx(Tp, rel=1e-09)
        (p3, p2, _, _) = L.hermite_coefficients(q0, v0, q1, v1, Tp)
        assert np.allclose(c3, p3, rtol=1e-07, atol=1e-12) and np.allclose(c2, p2, rtol=1e-07, atol=1e-12)
        assert C == pytest.approx(L.cost(Tp, *L.cost_coefficients(q0, v0, q1, v1), rho), rel=1e-09)
    z = [0.0] * rb.n
    assert m.lqmt(z, z, z, z, 1.0) is None

def test_length_length_count_sample_limit_limit_match():
    rng = np.random.default_rng(11)
    limits = native.joint_limits(PANDA)
    s = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    limit_mismatches = 0
    for _ in range(10000):
        (q0, v0, q1, v1) = _random_edge(rng, PANDA, s, s + 0.5, v=0.6, noise=0.8)
        c = TC.bvp(q0, v0, q1, v1)
        a = (L_(q0), L_(v0), L_(q1), L_(v1), c.T)
        assert _core.panda.length(*a) == pytest.approx(c.length(), rel=1e-10)
        assert _core.panda.sample_count(*a) == TC.sample_count(c, PANDA)
        limit_mismatches += _core.panda.within_limits(*a, limits) != L.within_limits(c.coef, c.T, PANDA.q_lo, PANDA.q_hi, PANDA.v_max, PANDA.a_max)
    assert limit_mismatches == 0

@pytest.mark.parametrize('name,environment_name', [('panda', 'table_pick'), ('panda', 'cage'), ('ur5', 'bookshelf_thin')])
def test_validate_sphere_geometry_match_vamp(name, environment_name):
    rb = R.get_robot(name)
    (s, g, ev, ec) = _problem(name, environment_name, 0)
    rng = np.random.default_rng(12)
    m = native.module(rb)
    difference = 0
    for _ in range(5000):
        q = rng.uniform(rb.q_lo, rb.q_hi)
        difference += m.validate(ec, L_(q)) != rb.vamp.validate(q, ev)
    assert difference == 0

@pytest.mark.parametrize('name,environment_name', [('panda', 'table_pick'), ('panda', 'bookshelf_small'), ('ur5', 'box'), ('ur5', 'cage')])
def test_flask_cc_10k_edge(name, environment_name):
    """Test flask cc 10k edges."""
    rb = R.get_robot(name)
    (s, g, ev, ec) = _problem(name, environment_name, 0)
    limits = native.joint_limits(rb, 'published_resolution')  # the Python reference uses the arc-length count
    m = native.module(rb)
    rng = np.random.default_rng(13)
    difference = []
    accepted_count = 0
    for _ in range(10000):
        (q0, v0, q1, v1) = _random_edge(rng, rb, s, g[0])
        c = TC.bvp(q0, v0, q1, v1)
        py = TC.flask_cc(c, rb, ev, check_limits=True)
        cpp = m.flask_cc(ec, L_(q0), L_(v0), L_(q1), L_(v1), c.T, limits, True)
        accepted_count += py
        if py != cpp:
            difference.append((q0, v0, q1, v1))
    print(f'{name}/{environment_name}: accepted_count le {accepted_count}/10000, difference {len(difference)}')
    assert 50 < accepted_count < 9800, 'Edge sample lacks diversity'
    assert len(difference) <= 10
    for (q0, v0, q1, v1) in difference:
        changed = False
        for k in range(20):
            e = np.random.default_rng(k).normal(0, 0.0001, rb.n)
            c = TC.bvp(q0, v0, q1 + e, v1)
            if c is not None and TC.flask_cc(c, rb, ev) != TC.flask_cc(TC.bvp(q0, v0, q1, v1), rb, ev):
                changed = True
                break
        assert changed, 'The disagreement is not confined to an obstacle boundary'

def test_negative_control_negative_sphere_blocking():
    s = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    limits = native.joint_limits(PANDA)
    rng = np.random.default_rng(14)
    e0 = _core.Environment()
    for _ in range(100):
        (q0, v0, q1, v1) = _random_edge(rng, PANDA, s, s + 0.4)
        c = TC.bvp(q0, v0, q1, v1)
        if _core.panda.flask_cc(e0, L_(q0), L_(v0), L_(q1), L_(v1), c.T, limits, False):
            break
    midpoint_q = c.evaluate(np.array([0.5 * c.T]))[0][0]
    sphere = vamp.panda.fk(midpoint_q)[-1]
    e1 = _core.Environment()
    e1.add_sphere([sphere.x, sphere.y, sphere.z], 0.03)
    assert not _core.panda.flask_cc(e1, L_(q0), L_(v0), L_(q1), L_(v1), c.T, limits, False)
SCENARIOS = [('panda', 'table_pick', 0), ('panda', 'table_pick', 1), ('panda', 'box', 0), ('panda', 'cage', 2), ('ur5', 'table_pick', 0), ('ur5', 'bookshelf_thin', 0), ('ur5', 'box', 1)]

@pytest.mark.parametrize('name,environment_name,i', SCENARIOS)
@pytest.mark.parametrize('lg', ['A', 'B'])
def test_plan_cpp_valid_valid_by_reference_python(name, environment_name, i, lg):
    """Check returned trajectories with a fixed iteration budget, independent of CPU speed."""
    rb = R.get_robot(name)
    (s, g, ev, ec) = _problem(name, environment_name, i)
    # Performance deadlines belong to retained benchmark attempts. This
    # correctness fixture must not silently truncate its 100,000 iterations
    # because another test or the execution host changes CPU availability.
    settings = native.settings(neighbor_mode=lg, velocity_sample_scale=0.25, max_iterations=100000,max_ns=0)
    d = native.plan(rb, ec, s, g, settings, seed=1)
    assert d['solved'], f"No solution after {d['iterations']} iterations"
    for trajectory in (d['raw_trajectory'], d['trajectory']):
        assert np.allclose(trajectory.Q[0], s) and np.allclose(trajectory.V[0], 0)
        assert any((np.allclose(trajectory.Q[-1], x) for x in g)) and np.allclose(trajectory.V[-1], 0)
        assert np.all(trajectory.T > 0)
        assert TC.validate_trajectory(trajectory, rb, ev)
    assert len(d['Q']) <= len(d['raw_positions'])

def test_plan_cpp_deterministic_deterministic_by_seed():
    (s, g, ev, ec) = _problem('ur5', 'table_pick', 1)
    settings = native.settings(velocity_sample_scale=0.25)
    a = native.plan(UR5, ec, s, g, settings, seed=7)
    b = native.plan(UR5, ec, s, g, settings, seed=7)
    assert a['iterations'] == b['iterations'] and np.array_equal(a['Q'], b['Q'])
