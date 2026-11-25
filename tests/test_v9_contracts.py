"""v9 options: lazy path certification for the Panda (Table II). Defaults keep earlier behaviour; the
contracts include a negative control (a nominal path that collides is replaced, never returned)."""
import json
from pathlib import Path

import numpy as np
import pytest

from flask_repro import certify, mbm, native, robot

DELTA = .001
PROFILE = json.loads((Path(__file__).resolve().parents[1] / 'config/manipulator_final_v9.json').read_text())


def _problem(env, index):
    return mbm.load_dataset('panda')[env][index]


def _dense_collisions(rb, problem, Q, V, T):
    from flask_repro.reference import Trajectory
    path = Trajectory(np.asarray(Q), np.asarray(V), np.asarray(T))
    samples = path.evaluate(np.linspace(0, path.duration(), 6000))[0]
    return native.module(rb).count_collisions(mbm.native_environment(problem), samples.tolist())[0]


@pytest.mark.parametrize('env,index,seed', [('box', 25, 1001), ('cage', 1, 1001)])  # nominal paths that collide at range 1.0
def test_lazy_certification_replaces_colliding_nominal_path(env, index, seed):
    rb = robot.get_robot('panda')
    problem = _problem(env, index)
    s = native.settings(**PROFILE['settings'])  # the campaign's Panda settings
    nominal = native.plan(rb, mbm.native_environment(problem), problem['start'], problem['goals'], s, seed=seed,
                          limits=native.joint_limits(rb, 'published_resolution'))  # grid of the v9 campaign
    assert nominal['solved'] and _dense_collisions(rb, problem, nominal['Q'], nominal['V'], nominal['T']) > 0  # negative control
    lazy = native.plan_lazy_certified(rb, mbm.native_environment(problem),
                                      mbm.native_environment(certify.inflate_problem(problem, DELTA)),
                                      problem['start'], problem['goals'], s, seed=seed, certify_delta=DELTA,
                                      nominal_profile='published_resolution')
    assert lazy['solved'] and lazy['rejected_paths'] >= 1 and lazy['certification_attempts'] == lazy['rejected_paths'] + 1
    assert _dense_collisions(rb, problem, lazy['Q'], lazy['V'], lazy['T']) == 0
    assert lazy['shortcut_ns'] == lazy['simplify_only_ns'] + lazy['certification_ns']


def test_lazy_certification_first_attempt_matches_nominal_when_certified():
    rb = robot.get_robot('panda')
    problem = _problem('table_pick', 1)
    s = native.settings(**PROFILE['settings'])  # the campaign's Panda settings
    nominal = native.plan(rb, mbm.native_environment(problem), problem['start'], problem['goals'], s, seed=1001)
    lazy = native.plan_lazy_certified(rb, mbm.native_environment(problem),
                                      mbm.native_environment(certify.inflate_problem(problem, DELTA)),
                                      problem['start'], problem['goals'], s, seed=1001, certify_delta=DELTA)
    if lazy['rejected_paths'] == 0:  # same RNG stream on the first attempt: identical path
        assert np.allclose(lazy['Q'], nominal['Q']) and np.allclose(lazy['T'], nominal['T'])
    with pytest.raises(Exception):
        native.plan_lazy_certified(rb, mbm.native_environment(problem), mbm.native_environment(problem),
                                   problem['start'], problem['goals'], s, seed=1, max_attempts=0)
