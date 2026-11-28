"""Impossible queries fail fast with a reason (no budget spent, no path returned); malformed input raises."""
import time

import numpy as np
import pytest

from flask_repro.planning import plan
from flask_repro.profiles import load_profile


@pytest.mark.parametrize('case', ['unicycle_wall', 'quad2d_hole', 'quad3d_block'])
@pytest.mark.parametrize('algorithm', ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp'])
def test_start_or_goal_in_collision_or_outside_fails_fast(case, algorithm):
    s = load_profile(case, algorithm, budget_s=5.)
    dim = len(s.original.lower)
    inside = np.asarray(s.original.boxes[0][0], float)                  # centre of an obstacle box
    for which, state in (('start', s.start), ('goal', s.goal)):
        bad = state.copy(); bad[:dim] = inside
        t = time.perf_counter()
        r = plan(s.model, s.planning, bad if which == 'start' else s.start, bad if which == 'goal' else s.goal, s.settings, 1)
        assert not r.solved and r.reason == f'invalid_{which}' and not r.trajectory.edges
        assert time.perf_counter() - t < .5
        outside = state.copy(); outside[0] = s.original.upper[0] + 1.
        r = plan(s.model, s.planning, outside if which == 'start' else s.start, outside if which == 'goal' else s.goal, s.settings, 1)
        assert not r.solved and r.reason == f'invalid_{which}'


@pytest.mark.parametrize('case', ['unicycle_wall', 'quad3d_block'])
def test_nonfinite_input_raises_a_clear_error(case):
    s = load_profile(case)
    bad = s.start.copy(); bad[0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        plan(s.model, s.planning, bad, s.goal, s.settings, 1)
    bad = s.goal.copy(); bad[1] = np.inf
    with pytest.raises(ValueError, match='finite'):
        plan(s.model, s.planning, s.start, bad, s.settings, 1)


def test_panda_reports_why_a_query_failed():
    from flask_repro import mbm, native, robot
    panda = robot.get_robot('panda'); m = native.module(panda)
    p = mbm.load_dataset('panda')['bookshelf_small'][0]; env = mbm.native_environment(p)
    st = native.settings(range=1.0, velocity_sample_scale=0.0, dynamic_domain=True, prefix_steering=False)
    rng = np.random.default_rng(0)
    colliding = next(q for q in (np.asarray(p['start']) + rng.normal(0, 1., 7) for _ in range(5000))
                     if not m.validate(env, q.tolist()))
    assert native.plan(panda, env, p['start'], p['goals'], st, seed=1)['reason'] == 'solved'
    assert native.plan(panda, env, colliding.tolist(), p['goals'], st, seed=1)['reason'] == 'invalid_start'
    assert native.plan(panda, env, p['start'], [colliding.tolist()], st, seed=1)['reason'] == 'invalid_goal'
    with pytest.raises(ValueError):
        native.plan(panda, env, [np.nan] * 7, p['goals'], st, seed=1)
