"""Held objects (VAMP attachments) in the manipulator checks: a sphere posed by the end-effector FK is checked in
validate, count_collisions and FLASKCC (planning). Negative and positive controls."""
import numpy as np
import pytest

from flask_repro import mbm, native, robot as R
from flask_repro import _core

READY = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]


def _box(name, p, h):
    return dict(name=name, position=list(p), half_extents=list(h), orientation_euler_xyz=(0., 0., 0.))


def _env(boxes):
    return mbm.native_environment(dict(problem='t', sphere=[], cylinder=[], box=boxes))


def test_eefk_and_sphere_fk_are_consistent():
    T = np.array(_core.panda.eefk(READY))
    assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-5) and np.allclose(T[3], [0, 0, 0, 1])
    S = np.array(_core.panda.sphere_fk(READY))
    assert S.shape == (59, 4) and np.all(S[:, 3] > 0)
    assert np.min(np.linalg.norm(S[:, :3] - T[:3, 3], axis=1)) < 0.15     # hand spheres near the grasp frame


def test_held_object_collides_where_the_empty_hand_does_not():
    p = np.array(_core.panda.eefk(READY))[:3, 3]
    shelf = _box('shelf', p - [0, 0, 0.16], (0.12, 0.12, 0.01))           # 15 cm below the grasp frame
    env = _env([shelf])
    mod = _core.panda
    assert mod.validate(env, READY)                                       # empty hand: clear
    env.attach([[0., 0., 0., 0.18]])                                      # a 18 cm ball held at the grasp frame
    assert env.attached and not mod.validate(env, READY)                 # reaches the shelf
    assert mod.count_collisions(env, [READY] * 5)[0] == 5
    env.detach()
    assert not env.attached and mod.validate(env, READY)                  # negative control: detached again
    env.attach([[0., 0., 0., 0.02]])                                      # a small held object: still clear
    assert mod.validate(env, READY)


def test_planning_with_a_held_object_keeps_it_clear():
    rb = R.get_robot('panda'); mod = native.module(rb)
    a = np.array(READY); b = a.copy(); b[0] = 1.2
    pa = np.array(_core.panda.eefk(a.tolist()))[:3, 3]; pb = np.array(_core.panda.eefk(b.tolist()))[:3, 3]
    mid = (pa + pb) / 2
    bar = _box('bar', (mid[0], mid[1], pa[2] - 0.15), (0.05, 0.05, 0.05))   # sits under the hand's sweep
    env = _env([bar])
    env.attach([[0., 0., 0.08, 0.06], [0., 0., 0.16, 0.06]])            # a held stick along the gripper axis
    assert mod.validate(env, a.tolist()) and mod.validate(env, b.tolist())
    solved = 0
    for seed in range(1, 6):
        d = native.plan(rb, env, a, [b], native.settings(range=1.0, velocity_sample_scale=0.0), seed=seed)
        if not d['solved']:
            continue
        solved += 1
        tr = d['trajectory']
        Q = tr.evaluate(np.linspace(0, tr.duration(), 3000))[0]
        assert mod.count_collisions(env, Q.tolist())[0] == 0               # held stick clear along the motion
    assert solved >= 3


@pytest.mark.parametrize('bad', [[], [[0., 0., 0., 0.]], [[np.nan, 0., 0., .1]]])
def test_attach_rejects_bad_spheres(bad):
    env = _env([])
    with pytest.raises(ValueError):
        env.attach(bad)
