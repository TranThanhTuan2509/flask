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
    try:
        m = S.build_model()
    except FileNotFoundError as e:           # showcase assets are fetched only by ./install.sh --showcase
        pytest.skip(f'showcase assets not installed: {e.filename}')
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
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


# ---------------- showcase_2 ----------------

def _model_or_skip(build):
    try:
        return build()
    except (FileNotFoundError, ValueError) as e:     # Menagerie assets are fetched by ./install.sh --showcase
        pytest.skip(f'showcase assets not installed: {e}')


def test_robotiq_2f85_stays_inside_the_planners_ur5_spheres():
    """The actuated Menagerie 2F-85 (simulation) replaces the URDF Robotiq 85 that VAMP's spheres model: every vertex
    of its collision meshes (fingers open) must lie inside the UR5's VAMP spheres, within 1.5 cm."""
    import mujoco
    from simulation.showcase import arms as A
    arm = A.Arm('ur5', 'ur5/')
    m = _model_or_skip(lambda: A.build([arm], [], {}, lambda n: (.6, .6, .6, 1)))
    d = mujoco.MjData(m); io = A.ArmIO(arm, m)
    for q in ([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], [0.6, -1.2, 1.4, -1.8, -1.2, 0.7]):
        d.qpos[io.qadr] = q; mujoco.mj_forward(m, d)
        S = arm.world_spheres(np.asarray(q))
        worst = 0.
        for g in range(m.ngeom):
            if not m.geom(g).name.startswith('ur5/g_') and not m.body(m.geom_bodyid[g]).name.startswith('ur5/g_'):
                continue
            if m.geom_contype[g] == 0 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            k = m.geom_dataid[g]; v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
            P = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            out = np.min(np.linalg.norm(P[:, None, :] - S[None, :, :3], axis=2) - S[None, :, 3], axis=1)
            worst = max(worst, float(out.max()))
        assert worst <= 0.015, worst


@pytest.mark.parametrize('module', ['ur5_pickplace', 'multiarm_handover'])
def test_pickplace_poses_are_reachable_and_free(module):
    """Every grasp / place / hand-over pose has an IK solution (the action queues build), and the static poses that
    are not inside a support are collision free in the planner's padded scene."""
    import importlib
    from simulation.showcase import arms as A
    from simulation.showcase.manip import PADDING
    S = importlib.import_module(f'simulation.showcase.{module}')
    queues = [S.queue(None)] if module == 'ur5_pickplace' else list(S.queues())
    arms = [S.ARM] if module == 'ur5_pickplace' else S.ARMS
    for arm, Q in zip(arms, queues):
        env = A.environment(arm, S.scene(), PADDING)
        moves = [a for a in Q if a[0] == 'move']
        assert moves
        for a in moves:
            if len(a) > 3 and a[3]:                      # approach / lift / lower: may touch what they ignore
                continue
            assert arm.mod.validate(env, list(map(float, a[1]))), (module, a[2])


def test_multidrone_scene_starts_and_goals():
    """Starts and goals lie in the free space of the planning world, and no goal is within reach of any start (a
    lower-priority drone waits at its start; revised prioritized planning)."""
    from flask_repro.flat_planner import World
    from simulation.showcase import multidrone_swap as S
    world = World(S.BOUNDS[0], S.BOUNDS[1], S.obstacle_boxes(), [], S.RADIUS)
    pts = [np.array(s) for s, _, _ in S.DRONES.values()] + [np.array(g) for _, g, _ in S.DRONES.values()]
    assert np.all(world.positions_valid(np.array(pts)))
    starts = [np.array(s) for s, _, _ in S.DRONES.values()]; goals = [np.array(g) for _, g, _ in S.DRONES.values()]
    for g in goals:
        assert min(np.linalg.norm(g - s) for s in starts) >= 2 * S.RADIUS + S.MARGIN


def test_fetch_carry_pose_fits_the_carry_footprint():
    """While the Fetch drives with the carton, its arm, gripper and carton stay inside the base planner's disk."""
    import vamp
    from flask_repro import _core
    from simulation.showcase import fetch_pickplace as F, fetch_warehouse as W
    from simulation.showcase.arms import held_spheres, object_T
    pose = np.array(F.STATION_A)
    pick = F.to_base(pose, np.array(F.CARTON) + [-F.GRASP_DX, 0, F.GRASP_DZ])
    q_pick = F.ik(pick, F.ik(pick - [0.11, 0, 0], W.TUCK))
    G = np.linalg.inv(np.asarray(vamp.fetch.eefk(list(q_pick)))) @ object_T(F.to_base(pose, np.array(F.CARTON)), -pose[2])
    env = mbm.native_environment(dict(problem='f', sphere=[], cylinder=[], box=[])); env.attach(held_spheres(F.HALF, G))
    q = F.ik(F.CARRY_POINT, q_pick, env, restarts=30)
    assert q is not None
    S = np.array(_core.fetch.sphere_fk(list(q))); S = S[S[:, 2] > 0.45]
    T = np.asarray(vamp.fetch.eefk(list(q)))
    H = np.array([[*(T[:3, :3] @ s[:3] + T[:3, 3]), s[3]] for s in held_spheres(F.HALF, G)])
    reach = max(np.max(np.hypot(S[:, 0], S[:, 1]) + S[:, 3]), np.max(np.hypot(H[:, 0], H[:, 1]) + H[:, 3]))
    assert reach <= F.CARRY_RADIUS
