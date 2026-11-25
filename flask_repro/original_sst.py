"""First-party original-coordinate SST* reconstruction with independent validation.

This is not the original Dynoplan/OMPL baseline used for the paper's Table I.

Planning runs natively (src/flask/sst_original.hh): original states/controls, Euler dt = 0.01,
scalar collision checks on the same sphere cover as FLASK, DynoBench goal regions.
`validate` re-integrates the returned controls with an independent NumPy implementation of the same
equations and checks every Euler state against the ORIGINAL boxes, the limits and the goal region.
"""
from dataclasses import dataclass
import numpy as np
from . import _core

GOAL_EPSILON = dict(unicycle=.2, quad2d=.5, quad3d=.7)                     # dynoplan sst_v0.yaml
STEP_RANGE = dict(unicycle=(2, 20), quad2d=(10, 50), quad3d=(5, 50))        # min/max_control_duration


@dataclass
class OrigSettings:
    budget_s: float = 10.
    stop_first: bool = True
    goal_bias: float = .1
    selection_radius: float = 1.
    pruning_radius: float = .1
    shrink: float = .8
    phase_iterations: int = 1000
    max_iterations: int = 1000000000
    trace: bool = False
    max_nodes: int = 2000000
    velocity_sample: float = 2.
    rate_sample: float = 2.

    def native(self, robot):
        c = _core.OrigSettings()
        c.budget_ns = int(self.budget_s * 1e9)
        c.min_steps, c.max_steps = STEP_RANGE[robot]
        c.goal_epsilon = GOAL_EPSILON[robot]
        for name in ['stop_first', 'goal_bias', 'selection_radius', 'pruning_radius', 'shrink',
                     'phase_iterations', 'max_iterations', 'trace', 'max_nodes', 'velocity_sample', 'rate_sample']:
            setattr(c, name, getattr(self, name))
        return c


def robot_name(case):
    return case.split('_')[0]


def plan(case, planning_world, start, goal, settings, seed):
    robot = robot_name(case)
    if planning_world.cpp_environment is None:
        planning_world.prepare_simd()
    r = _core.orig_sst_plan(robot, planning_world.cpp_environment, list(map(float, start)), list(map(float, goal)),
                            planning_world.lower.tolist(), planning_world.upper.tolist(), planning_world.radius,
                            settings.native(robot), seed)
    return r


# ---------------------------------------------------------------- independent model (NumPy)
def _step(robot, x, u, dt=.01):
    x = np.asarray(x, float)
    if robot == 'unicycle':
        th = np.remainder(x[2] + dt * u[1] + np.pi, 2 * np.pi) - np.pi
        return np.array([x[0] + dt * u[0] * np.cos(x[2]), x[1] + dt * u[0] * np.sin(x[2]), th])
    if robot == 'quad2d':
        m, J, l, g = .034, 1e-4, .1, 9.81
        f = u[0] + u[1]
        th = np.remainder(x[2] + dt * x[5] + np.pi, 2 * np.pi) - np.pi
        return np.array([x[0] + dt * x[3], x[1] + dt * x[4], th, x[3] - dt * f * np.sin(x[2]) / m,
                         x[4] + dt * (f * np.cos(x[2]) / m - g), x[5] + dt * l * (u[0] - u[1]) / J])
    from scipy.spatial.transform import Rotation
    m, g, J = 1., 9.81, np.array([.1, .1, .2])
    p, q, v, w = x[:3], x[3:7], x[7:10], x[10:13]
    zb = Rotation.from_quat(q).as_matrix()[:, 2]
    qx, qy, qz, qw = q
    dq = .5 * np.array([qw * w[0] + qy * w[2] - qz * w[1], qw * w[1] + qz * w[0] - qx * w[2],
                        qw * w[2] + qx * w[1] - qy * w[0], -qx * w[0] - qy * w[1] - qz * w[2]])
    qn = q + dt * dq
    qn /= np.linalg.norm(qn)
    dw = (np.asarray(u[1:]) - np.cross(w, J * w)) / J
    return np.r_[p + dt * v, qn, v + dt * (u[0] * zb / m - np.array([0, 0, g])), w + dt * dw]


def dynobench_distance(robot, a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    wrap = lambda t: abs(np.remainder(t + np.pi, 2 * np.pi) - np.pi)
    if robot == 'unicycle':
        return np.linalg.norm(a[:2] - b[:2]) + .5 * wrap(a[2] - b[2])
    if robot == 'quad2d':
        return (np.linalg.norm(a[:2] - b[:2]) + .5 * wrap(a[2] - b[2]) + .2 * np.linalg.norm(a[3:5] - b[3:5])
                + .2 * abs(a[5] - b[5]))
    dq = min(1., abs(float(a[3:7] @ b[3:7])))
    return (np.linalg.norm(a[:3] - b[:3]) + .5 * (0. if dq > 1 - 1e-5 else np.arccos(dq))
            + .1 * np.linalg.norm(a[7:10] - b[7:10]) + .05 * np.linalg.norm(a[10:13] - b[10:13]))


def validate(case, result, original_world, start, goal):
    robot = robot_name(case)
    errors = []
    if not result['solved']:
        return dict(valid=False, errors=['no_solution'], length=None, duration=None)
    states = np.asarray(result['states'], float)
    x = np.asarray(start, float)
    trajectory = [x]
    max_state_error = 0.
    for i, (u, n) in enumerate(zip(result['controls'], result['steps'])):
        u = np.asarray(u, float)
        if robot == 'unicycle' and (abs(u[0]) > 1 + 1e-9 or abs(u[1]) > 1.5 + 1e-9):
            errors.append('control_limit')
        if robot == 'quad2d' and (np.any(u < -1e-12) or np.any(u > .65 * .034 * 9.81 + 1e-12)):
            errors.append('control_limit')
        if robot == 'quad3d' and (u[0] < -1e-12 or u[0] > 1.5 * 9.81 + 1e-9 or np.linalg.norm(u[1:]) > 2 + 1e-9):
            errors.append('control_limit')
        for _ in range(n):
            x = _step(robot, x, u)
            trajectory.append(x)
        max_state_error = max(max_state_error, float(np.max(np.abs(x - states[i + 1]))))
    trajectory = np.asarray(trajectory)
    if max_state_error > 1e-6:
        errors.append('integration_mismatch')
    dims = 3 if robot == 'quad3d' else 2
    if not original_world.positions_valid(trajectory[:, :dims], independent=True):
        errors.append('collision_or_bounds')
    if robot == 'quad3d' and (np.max(np.linalg.norm(trajectory[:, 7:10], axis=1)) > 4 + 1e-9
                              or np.max(np.linalg.norm(trajectory[:, 10:13], axis=1)) > 8 + 1e-9):
        errors.append('state_limit')
    goal_distance = dynobench_distance(robot, trajectory[-1], goal)
    if goal_distance > GOAL_EPSILON[robot] + 1e-9:
        errors.append('goal_region')
    length = float(np.sum(np.linalg.norm(np.diff(trajectory[:, :dims], axis=0), axis=1)))
    return dict(valid=not errors, errors=sorted(set(errors)), length=length, duration=.01 * (len(trajectory) - 1),
                goal_distance=float(goal_distance), max_state_error=max_state_error)
