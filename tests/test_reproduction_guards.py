from types import SimpleNamespace
import numpy as np
import pytest
from flask_repro import _core, native, robot as R, mbm

def test_defaults_and_invalid_configuration():
    rb = R.get_robot('panda')
    env = _core.Environment()
    assert native.settings().velocity_sample_scale == 1.0
    with pytest.raises(ValueError):
        native.settings(neighbor_mode='typo')
    for (name, value) in [('rho', 0.0), ('range', 0.0), ('w', -1.0), ('velocity_sample_scale', 2.0)]:
        settings = native.settings(**{name: value})
        with pytest.raises(ValueError):
            native.plan(rb, env, np.zeros(7), [], settings)
    with pytest.raises(ValueError):
        native.plan(rb, env, np.zeros(6), [])
    with pytest.raises(ValueError):
        _core.panda.lqmt([0.0] * 7, [0.0] * 7, [1.0] * 7, [0.0] * 7, -1.0)

def test_identity_and_empty_goal_length_not_crash():
    rb = R.get_robot('panda')
    (_, _, s, _, _, env) = next(mbm.paired_problems('panda', ['table_pick']))
    result = native.plan(rb, env, s, [s])
    assert result['solved'] and result['trajectory'].duration() == 0
    np.testing.assert_allclose(result['trajectory'].evaluate([0])[0][0], s)
    assert not native.plan(rb, env, s, [])['solved']


def test_manipulator_bvp_prefix_preserves_directed_full_state_and_limits():
    """Nontrivial two-tree path: audit every raw edge before shortcutting."""
    rb=R.get_robot('panda')
    _,_,start,goals,_,env=next(x for x in mbm.paired_problems('panda',['cage']) if x[1]==20)
    cfg=native.settings(prefix_steering=True,velocity_sample_scale=1.,max_iterations=3000,max_ns=int(2e9))
    result=native.plan(rb,env,start,goals,cfg,seed=20)
    assert result['solved'] and result['accepted_extensions']>0
    path=result['raw_trajectory']
    np.testing.assert_allclose(path.Q[0],start,atol=1e-8)
    assert any(np.allclose(path.Q[-1],goal,atol=1e-8) for goal in goals)
    np.testing.assert_allclose(path.V[[0,-1]],0.,atol=1e-8)
    limits=native.joint_limits(rb,'strict_development')
    for edge in path.edges():
        np.testing.assert_allclose(edge.evaluate([0.,edge.T])[0],[edge.q0,edge.q1],atol=1e-7)
        np.testing.assert_allclose(edge.evaluate([0.,edge.T])[1],[edge.v0,edge.v1],atol=1e-7)
        assert native.module(rb).within_limits(*[x.tolist() for x in (edge.q0,edge.v0,edge.q1,edge.v1)],edge.T,limits)
        assert native.module(rb).flask_cc(env,*[x.tolist() for x in (edge.q0,edge.v0,edge.q1,edge.v1)],edge.T,limits)

@pytest.mark.parametrize('distance', [1e-20, 1e-12, 0.0001, 1.0, 100000000.0])
def test_small_distinct_states_have_nonzero_optimal_duration(distance):
    from flask_repro.lqmt import optimal_duration
    from flask_repro.flat_bvp import optimal_time
    zero = np.zeros(7)
    q = zero.copy()
    q[0] = distance
    expected = (36 * distance ** 2) ** 0.25
    assert optimal_duration(zero, zero, q, zero) == pytest.approx(expected, rel=1e-08)
    assert _core.panda.lqmt(zero.tolist(), zero.tolist(), q.tolist(), zero.tolist(), 1.0)[0] == pytest.approx(expected, rel=1e-08)
    assert optimal_time(np.stack([zero, zero]), np.stack([q, zero])) == pytest.approx(expected, rel=1e-08)

def test_spline_interpolation_can_change_a_collision_free_polyline():
    from toppra import SplineInterpolator
    w = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [2.0, 1.0]])
    path = SplineInterpolator([0.0, 1.0, 2.0, 3.0], w)
    points = path(np.linspace(0, 3, 501))

    def distances_to_polyline(points):
        d = []
        for (a, b) in zip(w, w[1:]):
            u = np.clip((points - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
            d.append(np.linalg.norm(points - (a + u[:, None] * (b - a)), axis=1))
        return np.min(d, axis=0)
    distances = distances_to_polyline(points)
    i = np.argmax(distances)
    center = points[i]
    radius = distances[i] / 2
    assert radius > 0.01
    assert distances_to_polyline(center[None])[0] > radius
    assert np.linalg.norm(path(np.linspace(0, 3, 501))[i] - center) < radius
    s = np.linspace(0, 3, 501)[i]
    np.testing.assert_allclose(path(3 * (s / 3) ** 1), center)
    np.testing.assert_allclose(path(3 * np.sqrt(s / 3) ** 2), center)

def test_duplicate_waypoints_are_removed_before_toppra():
    from benchmarks.mbm_baseline import topp_ra
    rb = SimpleNamespace(v_max=np.array([2.0, 2.0]), a_max=np.array([3.0, 3.0]))
    wp = [[0.0, 0.0], [0.0, 0.0], [0.5, 0.3], [1.0, 1.0]]
    traj = topp_ra(wp, rb)
    assert traj is not None and np.isfinite(traj.duration) and (traj.duration > 0)
    with pytest.raises(ValueError):
        topp_ra([[0.0, 0.0], [0.0, 0.0]], rb)
