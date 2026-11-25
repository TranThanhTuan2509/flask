"""v8 options: unicycle stop-turn-go protocol and pure-sphere shell covers. Defaults keep v7 behaviour;
each contract has a negative control."""
from dataclasses import replace

import numpy as np
import pytest

from flask_repro import _core, unicycle_turns
from flask_repro.dynobench import load
from flask_repro.flat_bvp import FlatEdge
from flask_repro.planning import PlannerSettings, plan, validate

REST = dict(unicycle_rest_turns=True, rest_sample_probability=1., extension_range=1.5, goal_mode=1,
            goal_tolerance=.2, budget_s=5., max_iterations=10**7)


def _rest_model(case='unicycle_wall'):
    model, planning, original, s, g, _ = load(case)
    model.rest_turns = True
    return model, planning, original, s, g


def _check(world, coefficients, duration, cfg):
    world.prepare_simd()
    out = _core.flat_check_edges('unicycle', world.cpp_environment, [coefficients], [duration],
                                 world.lower.tolist(), world.upper.tolist(), world.radius, cfg.native(), 1)
    return out['valid'][0]


def test_rest_to_rest_edge_is_straight_zero_yaw_rate_and_accepted():
    e = _core.flat_bvp([[1., 1.], [0., 0.]], [[2.2, 1.9], [0., 0.]], 1.)
    assert _core.unicycle_rest_edge(e['coefficients'], e['duration'], 1.) == 1
    assert not _core.unicycle_edge_valid(e['coefficients'], e['duration'])  # strict chart: singular ends
    edge = FlatEdge(np.asarray(e['coefficients']), e['duration'], 2)
    t = np.linspace(0, edge.duration, 1000)[1:-1]
    v, a = edge.derivative(t, 1), edge.derivative(t, 2)
    omega = (v[:, 0] * a[:, 1] - v[:, 1] * a[:, 0]) / np.sum(v * v, axis=1)
    assert np.max(np.abs(omega)) < 1e-9
    _, planning, *_ = _rest_model()
    assert _check(planning, e['coefficients'], e['duration'], PlannerSettings(**REST))
    assert not _check(planning, e['coefficients'], e['duration'], PlannerSettings())  # v7 strict rejects


def test_mixed_and_fast_rest_edges_are_rejected():
    mixed = _core.flat_bvp([[1., 1.], [0., 0.]], [[2., 1.5], [.5, 0.]], 1.)
    assert _core.unicycle_rest_edge(mixed['coefficients'], mixed['duration'], 1.) == 0
    _, planning, *_ = _rest_model()
    assert not _check(planning, mixed['coefficients'], mixed['duration'], PlannerSettings(**REST))
    # rest BVP peak speed 1.5|d|/T* = 0.612 sqrt|d| exceeds 1 m/s beyond |d| = 2.67 m
    fast = _core.flat_bvp([[.5, .5], [0., 0.]], [[3.8, .5], [0., 0.]], 1.)
    assert _core.unicycle_rest_edge(fast['coefficients'], fast['duration'], 1.) == -1


def test_rest_turn_settings_are_guarded():
    with pytest.raises(ValueError):
        PlannerSettings(**(REST | dict(rest_sample_probability=.5))).native()
    model, planning, original, s, g = _rest_model()
    model.rest_turns = False
    with pytest.raises(ValueError):
        plan(model, planning, s, g, PlannerSettings(**REST), 1)


def test_stop_turn_plan_validates_and_turn_time_is_exact():
    model, planning, original, s, g = _rest_model()
    cfg = PlannerSettings(**REST)
    r = plan(model, planning, s, g, cfg, 1001)
    v = validate(r, model, original, cfg)
    assert r.solved and v['valid'], v['errors']
    # independent heading schedule: analytic direction of motion at each edge end (velocity, or the
    # acceleration/jerk direction where the robot is at rest)
    wrap = lambda a: abs(np.angle(np.exp(1j * a)))

    def direction(e, t, sign):
        for k in (1, 2, 3):
            d = e.derivative(t, k) * (sign ** (k - 1))
            if np.linalg.norm(d) > 1e-9:
                return np.arctan2(d[1], d[0])

    theta, total = s[2], 0.
    for e in r.trajectory.edges:
        a0, a1 = direction(e, 0., 1), direction(e, e.duration, -1)
        start, end = min([(a0, a1), (a0 + np.pi, a1 + np.pi)], key=lambda o: wrap(o[0] - theta))
        total += wrap(start - theta)
        theta = end
    expected = (total + wrap(g[2] - theta)) / 1.5
    assert abs(v['stop_turn']['turn_time_s'] - expected) < 1e-9
    assert abs(v['stop_turn']['duration_s'] - r.trajectory.duration - expected) < 1e-9


def test_stop_turn_validator_negative_controls():
    model, planning, original, s, g = _rest_model()
    cfg = PlannerSettings(**REST)
    r = plan(model, planning, s, g, cfg, 1002)
    assert validate(r, model, original, cfg)['valid']
    # (1) a curved edge (moving endpoint) is not a stop-turn edge
    curved = replace(r, trajectory=replace(r.trajectory, edges=list(r.trajectory.edges)))
    e0 = curved.trajectory.edges[0]
    bent = _core.flat_bvp(e0.state(0.).tolist(), [e0.state(e0.duration)[0].tolist(), [.3, .3]], 1.)
    curved.trajectory.edges[0] = FlatEdge(np.asarray(bent['coefficients']), bent['duration'], 2)
    assert not validate(curved, model, original, cfg)['valid']
    # (2) goal moved away: goal region violated
    far = replace(r, goal_state=(np.asarray(r.goal_state) + [0., .5, 0.]).tolist())
    assert 'original_goal_mismatch' in validate(far, model, original, cfg)['errors']
    # (3) slower turn limit is respected by construction, a faster original limit is not needed:
    #     a world with an obstacle on the path must be caught
    blocked = load('unicycle_wall')[2]
    blocked.boxes = list(blocked.boxes) + [(list(r.trajectory.edges[0].state(r.trajectory.edges[0].duration / 2)[0]), [.3, .3])]
    errors = unicycle_turns.validate(r, model, blocked, cfg)['errors']
    assert any('collision' in e for e in errors)


def test_pure_shell_cover_preserves_search_and_rejects_crossing():
    full = load('quad3d_block')
    shell = load('quad3d_block', shell_only=True, shell_keep_boxes=False)
    assert len(shell[1].spheres) < len(full[1].spheres) and not shell[1].boxes
    assert .14 < full[2].shell_thickness(.15) < .15 and .015 < full[2].shell_thickness(.15)
    cfg = PlannerSettings(budget_s=5., extension_range=1.5, goal_tolerance=.7, goal_mode=1)
    for seed in (1, 2):
        a = plan(full[0], full[1], full[3], full[4], cfg, seed)
        b = plan(shell[0], shell[1], shell[3], shell[4], cfg, seed)
        assert (a.iterations, a.rejections['edges'], a.solved) == (b.iterations, b.rejections['edges'], b.solved)
    # an edge straight through the box centre must hit the shell
    world = shell[1]; world.prepare_simd()
    a = [[1., 3., 3., 0.]] + [[0.] * 4] * 3
    b = [[5., 3., 3., 0.]] + [[0.] * 4] * 3
    e = _core.flat_bvp(a, b, 1.)
    out = _core.flat_check_edges('quad3d', world.cpp_environment, [e['coefficients']], [e['duration']],
                                 world.lower.tolist(), world.upper.tolist(), world.radius, PlannerSettings().native(), 1)
    assert out['valid'] == [False]


@pytest.mark.parametrize('algorithm', ['sst_bvp', 'rrt_dp'])
def test_stop_turn_shortcut_bypasses_propagated_chain(algorithm):
    """Alg. 4 shortcuts between stopped nodes: the propagated chain collapses to a few straight edges."""
    model, planning, original, s, g = _rest_model()
    cfg = PlannerSettings(**(REST | dict(algorithm=algorithm, budget_s=10., selection_radius=.5, witness_radius=.1)))
    r = plan(model, planning, s, g, cfg, 1001)
    v = validate(r, model, original, cfg)
    assert r.solved and v['valid'], v['errors']
    assert all(unicycle_turns.straight_direction(e) is not None for e in r.trajectory.edges)
    assert len(r.trajectory.edges) < 15 < r.diagnostics['accepted_propagations']
    assert r.trajectory.length(2) < 6.  # Wall start-goal distance is 2.5 m; an unshortened chain wanders far longer
