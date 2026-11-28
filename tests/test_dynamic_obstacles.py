"""Prioritized space-time planning (PlannerSettings.moving_obstacles): another robot's trajectory is a moving
sphere checked exactly in the forward-tree planners. Positive and negative controls, and no effect when empty."""
from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from flask_repro.flat_models import Quad3D
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings, hold_piece, moving_pieces, plan, space_time_clearance, validate
from flask_repro.profiles import load_profile

R = 0.3


def hover(p):
    return np.r_[p, Rotation.from_euler('z', 0.).as_quat(), np.zeros(6)]


def corridor_problem():
    s = load_profile('quad3d_block', 'sst_bvp')
    world = World(np.array([0., 0., 0.]), np.array([6., 3., 3.]), [], [], R)
    cfg = replace(s.settings, budget_s=10.0, goal_tolerance=0.25, stop_first=True)
    return Quad3D(), world, cfg


def test_second_robot_avoids_the_first_in_space_and_time():
    quad, world, cfg = corridor_problem()
    first = plan(quad, world, hover(np.array([1., 1.5, 1.5])), hover(np.array([5., 1.5, 1.5])), cfg, 1)
    assert first.solved
    pieces = moving_pieces(first.trajectory, R)
    b0, b1 = np.array([5., 2.6, 1.5]), np.array([1., 2.6, 1.5])          # opposite direction, adjacent lane
    second = None
    for seed in range(2, 8):
        rec = plan(quad, world, hover(b0), hover(b1), replace(cfg, moving_obstacles=pieces, moving_margin=0.05), seed)
        if rec.solved:
            second = rec
            break
    assert second is not None
    d, where = space_time_clearance([first.trajectory, second.trajectory], [R, R])   # independent, 1 kHz
    assert d >= 0.05 - 1e-3, (d, where)
    assert validate(second, quad, world, cfg)['valid']


def test_moving_obstacle_blocks_a_conflicting_hold():
    """A sphere parked on the goal for ever: the planner must not report a path ending there."""
    quad, world, cfg = corridor_problem()
    goal = np.array([5., 1.5, 1.5])
    blocked = replace(cfg, budget_s=1.0, moving_obstacles=(hold_piece(goal, 0.3),))
    rec = plan(quad, world, hover(np.array([1., 1.5, 1.5])), hover(goal), blocked, 1)
    assert not rec.solved
    assert rec.rejections['moving'] > 0


def test_moving_obstacle_sweeping_the_line_is_avoided():
    """A sphere moving head-on along the straight start-goal line (linear in time) forces a detour or a delay."""
    quad, world, cfg = corridor_problem()
    a0, a1 = np.array([1., 1.5, 1.5]), np.array([5., 1.5, 1.5])
    T = 6.
    sweep = dict(t0=0., T=T, radius=R, c=[[5., -4.], [1.5, 0.], [1.5, 0.]])     # x = 5 - 4 s
    def gap(traj):
        ts = np.arange(0., min(T, traj.duration), 1e-3)
        p = traj.jet(ts, through=0)[:, 0, :3]
        o = np.c_[5. - 4. * ts / T, np.full_like(ts, 1.5), np.full_like(ts, 1.5)]
        return np.min(np.linalg.norm(p - o, axis=1))
    free = plan(quad, world, hover(a0), hover(a1), cfg, 1)                 # negative control: meets the sweep
    assert free.solved and gap(free.trajectory) < 2 * R
    rec = None
    for seed in range(1, 8):
        r = plan(quad, world, hover(a0), hover(a1), replace(cfg, moving_obstacles=(sweep,), moving_margin=0.05), seed)
        if r.solved:
            rec = r
            break
    assert rec is not None and rec.rejections['moving'] > 0
    assert gap(rec.trajectory) >= 2 * R + 0.05 - 1e-3


def test_rrt_connect_is_refused_and_empty_has_no_effect():
    quad, world, cfg = corridor_problem()
    with pytest.raises(ValueError):
        replace(cfg, algorithm='rrt_connect', moving_obstacles=(hold_piece([0, 0, 0], .1),)).native()
    a = plan(quad, world, hover(np.array([1., 1., 1.])), hover(np.array([5., 2., 2.])), cfg, 3)
    b = plan(quad, world, hover(np.array([1., 1., 1.])), hover(np.array([5., 2., 2.])), replace(cfg, moving_obstacles=()), 3)
    assert a.iterations == b.iterations and a.trajectory.duration == b.trajectory.duration


def test_rrt_forward_plans_valid_paths_and_respects_moving_obstacles():
    """FLASK-RRT on a single forward tree (showcase planner for space-time planning)."""
    quad, world, cfg = corridor_problem()
    fwd = replace(cfg, algorithm='rrt_forward')
    a0, a1 = np.array([1., 1.5, 1.5]), np.array([5., 1.5, 1.5])
    for seed in (1, 2, 3):
        r = plan(quad, world, hover(a0), hover(a1), fwd, seed)
        assert r.solved and validate(r, quad, world, fwd)['valid'], seed
    T = 6.
    sweep = dict(t0=0., T=T, radius=R, c=[[5., -4.], [1.5, 0.], [1.5, 0.]])
    r = plan(quad, world, hover(a0), hover(a1), replace(fwd, moving_obstacles=(sweep,), moving_margin=0.05), 1)
    assert r.solved and r.rejections['moving'] > 0
    ts = np.arange(0., min(T, r.trajectory.duration), 1e-3)
    p = r.trajectory.jet(ts, through=0)[:, 0, :3]
    o = np.c_[5. - 4. * ts / T, np.full_like(ts, 1.5), np.full_like(ts, 1.5)]
    assert np.min(np.linalg.norm(p - o, axis=1)) >= 2 * R + 0.05 - 1e-3
