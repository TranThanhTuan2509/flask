"""Test reference: FLASK implementation and evaluation utilities."""
import math
import numpy as np
import pytest
import vamp
from flask_repro import mbm, robot as R, reference as TC
PANDA = R.get_robot('panda')
READY = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
UR5 = R.get_robot('ur5')

def _problem(robot, environment_name, i):
    for (name, k, s, g, env) in mbm.problems(robot, [environment_name]):
        if k == i:
            return (s, g, env)
    raise KeyError(i)

@pytest.mark.parametrize('N', [8, 16, 64, 200, 1000])
def test_each_index_count_valid_one_time(N):
    lower = TC.batch_order(N, 8)
    plane = [j for b in lower for j in b]
    assert sorted(plane) == list(range(1, N + 1))
    assert all((len(b) <= 8 for b in lower))
    m = math.ceil(N / 8)
    assert lower[0] == [1 + k * m for k in range(8) if 1 + k * m <= N]

def test_count_sample_match_vamp_on_edge_straight():
    """Test count sample match vamp on straight edge."""
    rng = np.random.default_rng(1)
    for _ in range(50):
        q0 = rng.uniform(PANDA.q_lo, PANDA.q_hi)
        q1 = rng.uniform(PANDA.q_lo, PANDA.q_hi)
        T = rng.uniform(0.2, 3.0)
        v = (q1 - q0) / T
        c = TC.Edge(q0, v, q1, v, T)
        dist = np.linalg.norm(q1 - q0)
        assert c.length() == pytest.approx(dist, rel=1e-09)
        assert TC.sample_count(c, PANDA) == max(math.ceil(dist / 8 * 32), 1) * 8

def _random_edge(rng, robot, s, g):
    q0 = s + rng.normal(0, 0.3, robot.n)
    q1 = g + rng.normal(0, 0.3, robot.n)
    v0 = rng.uniform(-1, 1, robot.n) * robot.v_max * 0.3
    v1 = rng.uniform(-1, 1, robot.n) * robot.v_max * 0.3
    return TC.bvp(q0, v0, q1, v1)

def test_flask_cc_count_against_scan_dense_factor_4():
    """Test flask cc count against scan dense factor 4."""
    (s, g, env) = _problem('panda', 'table_pick', 0)
    rng = np.random.default_rng(2)
    difference = 0
    ca = 0
    for _ in range(150):
        c = _random_edge(rng, PANDA, s, g[0])
        cc = TC.flask_cc(c, PANDA, env, check_limits=False)
        N = TC.sample_count(c, PANDA) * 4
        (q, _, _, _) = c.evaluate(c.T * np.arange(1, N + 1) / N)
        wall_thickness = all((vamp.panda.validate(x, env) for x in q))
        assert not (wall_thickness and (not cc))
        ca += 1
        difference += int(cc and (not wall_thickness))
    assert difference <= 0.05 * ca

def test_negative_control_negative_sphere_blocking_middle_edge():
    """Test negative control negative sphere blocking middle edge."""
    rng = np.random.default_rng(3)
    env0 = vamp.Environment()
    c = None
    for _ in range(100):
        c = _random_edge(rng, PANDA, READY, READY + 0.4)
        if TC.flask_cc(c, PANDA, env0, check_limits=False):
            break
    assert TC.flask_cc(c, PANDA, env0, check_limits=False)
    midpoint_q = c.evaluate(np.array([0.5 * c.T]))[0][0]
    sphere = vamp.panda.fk(midpoint_q)[-1]
    env1 = vamp.Environment()
    env1.add_sphere(vamp.Sphere([sphere.x, sphere.y, sphere.z], 0.03))
    assert not vamp.panda.validate(midpoint_q, env1)
    assert not TC.flask_cc(c, PANDA, env1, check_limits=False)

def test_limit_limit_velocity_velocity_is_rejected():
    q0 = READY.copy()
    q1 = q0.copy()
    q1[0] = 0.3
    large_velocity = np.zeros(7)
    large_velocity[0] = PANDA.v_max[0] * 1.05
    c = TC.bvp(q0, large_velocity, q1, np.zeros(7))
    assert not TC.flask_cc(c, PANDA, vamp.Environment())
    assert TC.flask_cc(c, PANDA, vamp.Environment(), check_limits=False)

def _run_recorded(monkeypatch, *args, **kw):
    checked = []
    original = TC.flask_cc

    def write(edge, *a, **k):
        ok = original(edge, *a, **k)
        if ok:
            checked.append(edge)
        return ok
    monkeypatch.setattr(TC, 'flask_cc', write)
    result = TC.rrtc(*args, **kw)
    return (result, checked)

def _inside(edge, checked):
    return any((np.allclose(edge.q0, c.q0, atol=1e-09) and np.allclose(edge.v0, c.v0, atol=1e-09) and np.allclose(edge.q1, c.q1, atol=1e-09) and np.allclose(edge.v1, c.v1, atol=1e-09) and (abs(edge.T - c.T) < 1e-09) for c in checked))
VELOCITY_SAMPLE_SCALE = 0.25
EASY_SCENARIOS = [('panda', 'table_pick', 0), ('panda', 'table_pick', 1), ('panda', 'box', 0), ('ur5', 'table_pick', 0), ('ur5', 'bookshelf_thin', 0)]

@pytest.mark.parametrize('robot_name,environment_name,i', EASY_SCENARIOS)
@pytest.mark.parametrize('neighbor_mode', ['A', 'B'])
def test_rrtc_kino(monkeypatch, robot_name, environment_name, i, neighbor_mode):
    rb = R.get_robot(robot_name)
    (s, g, env) = _problem(robot_name, environment_name, i)
    settings = TC.PlannerSettings(neighbor_mode=neighbor_mode, velocity_sample_scale=VELOCITY_SAMPLE_SCALE, max_iterations=20000)
    (result, checked) = _run_recorded(monkeypatch, s, g, rb, env, settings, seed=0)
    assert result.solved, f'No solution after {result.iterations} iterations'
    trajectory = result.trajectory
    assert np.allclose(trajectory.Q[0], s) and np.allclose(trajectory.V[0], 0)
    assert any((np.allclose(trajectory.Q[-1], x) for x in g)) and np.allclose(trajectory.V[-1], 0)
    assert np.all(trajectory.T > 0)
    for c in trajectory.edges():
        assert _inside(c, checked)
    rg = TC.shortcut(trajectory, rb, env)
    assert len(rg.Q) <= len(trajectory.Q)
    assert np.allclose(rg.Q[0], trajectory.Q[0]) and np.allclose(rg.Q[-1], trajectory.Q[-1])
    assert TC.validate_trajectory(rg, rb, env)
    t = rg.t
    for k in range(1, len(t) - 1):
        a = rg.evaluate([t[k] - 1e-09])
        b = rg.evaluate([t[k] + 1e-09])
        assert np.allclose(a[0], b[0], atol=1e-06) and np.allclose(a[1], b[1], atol=1e-05)

def test_negative_control_negative_reverse_direction_bvp_tree_goal(monkeypatch):
    """Test negative control negative reverse direction bvp tree goal."""

    def errors(self, node_index, q, v, rho):
        return TC.bvp(self.Q[node_index], self.V[node_index], q, v, rho)
    monkeypatch.setattr(TC._Tree, 'connection', errors)
    detected = False
    for (robot_name, environment_name, i) in EASY_SCENARIOS:
        rb = R.get_robot(robot_name)
        (s, g, env) = _problem(robot_name, environment_name, i)
        (result, checked) = _run_recorded(monkeypatch, s, g, rb, env, TC.PlannerSettings(velocity_sample_scale=VELOCITY_SAMPLE_SCALE, max_iterations=20000), seed=0)
        monkeypatch.setattr(TC._Tree, 'connection', errors)
        if result.solved and (not all((_inside(c, checked) for c in result.trajectory.edges()))):
            detected = True
            break
    assert detected
