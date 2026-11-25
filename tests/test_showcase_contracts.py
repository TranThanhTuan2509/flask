"""Contracts for the robots added for the showcases (Fetch arm, point base) and the showcase scenes."""
import dataclasses
from pathlib import Path

import numpy as np
import pytest

from flask_repro import mbm, native, robot as R

ROOT = Path(__file__).resolve().parents[1]


def _box(name, p, h):
    return dict(name=name, position=list(p), half_extents=list(h), orientation_euler_xyz=(0., 0., 0.))


def _dense_collisions(rb, env, traj, n=2000):
    Q = traj.evaluate(np.linspace(0, traj.duration(), n))[0]
    return native.module(rb).count_collisions(env, Q.tolist())[0]


def test_fetch_binding_plans_valid_trajectories():
    rb = R.get_robot('fetch'); mod = native.module(rb)
    assert mod.dimension == 8 and rb.n == 8
    from simulation.showcase import fetch_warehouse as S          # the showcase's own scene and reach target
    env = S.arm_environment(S.STATION)
    tuck = S.TUCK.tolist(); goal = S.ik(S.REACH[0], S.TUCK)
    assert goal is not None
    goal = goal.tolist()
    assert mod.validate(env, tuck) and mod.validate(env, goal)
    for seed in range(1, 6):
        d = native.plan(rb, env, np.array(tuck), [np.array(goal)], native.settings(range=1.0, velocity_sample_scale=0.0), seed=seed)
        assert d['solved']
        tr = d['trajectory']
        assert _dense_collisions(rb, env, tr) == 0
        q, v, a = tr.evaluate(np.linspace(0, tr.duration(), 500))[:3]
        assert np.all(np.abs(v) <= rb.v_max + 1e-6) and np.all(np.abs(a) <= rb.a_max + 1e-6)
        assert np.allclose(q[0], tuck) and np.allclose(q[-1], goal, atol=1e-9)


def test_fetch_negative_control_blocked_goal_is_rejected():
    rb = R.get_robot('fetch'); mod = native.module(rb)
    wall = mbm.native_environment(dict(problem='t', sphere=[], cylinder=[], box=[_box('wall', (0.0, 0.0, 0.8), (2.0, 2.0, 2.0))]))
    assert not mod.validate(wall, [0.05, 1.32, 1.40, -0.20, 1.72, 0.0, 1.66, 0.0])


def test_point_base_radius_is_respected():
    """The point (sphere) robot keeps |p - obstacle| >= radius along the whole planned motion."""
    rb = dataclasses.replace(R.get_robot('sphere'), q_lo=np.array([0., 0., 0.79]), q_hi=np.array([6., 4., 0.81]),
                             v_max=np.array([0.7, 0.7, 0.05]), a_max=np.array([1.0, 1.0, 0.1]))
    mod = native.module(rb)
    pillar = _box('pillar', (3.0, 2.0, 1.0), (0.3, 1.2, 1.0))
    env = mbm.native_environment(dict(problem='t', sphere=[], cylinder=[], box=[pillar]))
    for radius in (0.2, 0.5):
        mod.set_radius(radius)
        d = native.plan(rb, env, np.array([0.8, 2.0, 0.8]), [np.array([5.2, 2.0, 0.8])],
                        native.settings(range=2.0, velocity_sample_scale=0.0), seed=1, limits=native.joint_limits(rb))
        assert d['solved']
        P = d['trajectory'].evaluate(np.linspace(0, d['trajectory'].duration(), 3000))[0]
        dx = np.maximum(np.abs(P[:, 0] - 3.0) - 0.3, 0); dy = np.maximum(np.abs(P[:, 1] - 2.0) - 1.2, 0)
        assert np.min(np.hypot(dx, dy)) >= radius - 1e-3
    mod.set_radius(0.2)                                      # restore the VAMP default for other tests


def test_point_base_blocked_start_fails_cleanly():
    rb = R.get_robot('sphere'); mod = native.module(rb); mod.set_radius(0.4)
    env = mbm.native_environment(dict(problem='t', sphere=[], cylinder=[], box=[_box('b', (1.0, 1.0, 1.0), (0.5, 0.5, 1.0))]))
    d = native.plan(rb, env, np.array([1.0, 1.0, 0.8]), [np.array([3.0, 3.0, 0.8])], native.settings(max_ns=10**8), seed=1)
    assert not d['solved']
    mod.set_radius(0.2)


@pytest.mark.parametrize('module', ['g1_courtyard', 'drone_hall', 'ur5_cell', 'fetch_warehouse'])
def test_showcase_scene_waypoints_are_collision_free(module):
    """Every showcase start/goal is valid in its planning model (guards against scene edits)."""
    import importlib
    if module == 'g1_courtyard':
        pytest.importorskip('torch')          # the G1 showcase needs the optional showcase extras
    S = importlib.import_module(f'simulation.showcase.{module}')
    if module == 'g1_courtyard':
        rb = S.point_robot(); env = S.base_environment(); mod = native.module(rb)
        assert all(mod.validate(env, [x, y, 0.8]) for x, y, _ in S.WAYPOINTS)
        mod.set_radius(0.2)
    elif module == 'drone_hall':
        from flask_repro.flat_planner import World
        world = World(S.BOUNDS[0], S.BOUNDS[1], S.obstacle_boxes(), [], S.RADIUS)
        assert np.all(world.positions_valid(np.array(S.WAYPOINTS)))
    elif module == 'ur5_cell':
        import vamp
        from experiments.ur5_scene import PADDING, inflate_scene
        assert len(S.goal_configurations(vamp.problem_dict_to_vamp(inflate_scene(S.scene(), PADDING + 0.001)))) == len(S.TARGETS)
    else:
        world, _ = S.planning_world()
        assert np.all(world.positions_valid(np.array([S.START[:2], S.STATION[:2], S.DOCK[:2]])))


@pytest.mark.parametrize('module', ['drone_hall', 'g1_courtyard'])
def test_planning_radius_covers_the_simulated_robot(module):
    """The planner's robot radius must exceed the simulated collision geometry by a tracking margin (>= 5 cm)."""
    import importlib
    import mujoco
    if module == 'g1_courtyard':
        pytest.importorskip('torch')
    S = importlib.import_module(f'simulation.showcase.{module}')
    m = S.build_model(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    body = m.body('airframe' if module == 'drone_hall' else 'pelvis').id
    if module == 'drone_hall':
        extent = max(np.linalg.norm(d.geom_xpos[g] - d.xpos[body]) + m.geom_rbound[g]
                     for g in range(m.ngeom) if m.geom_bodyid[g] == body and m.geom_contype[g])
    else:                                    # horizontal reach of the pelvis-attached upper body (fixed arms)
        extent = 0.
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == body and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                k = m.geom_dataid[g]; v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
                P = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
                extent = max(extent, np.linalg.norm(P[:, :2] - d.xpos[body][:2], axis=1).max())
    assert S.RADIUS >= extent + 0.05
