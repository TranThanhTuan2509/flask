"""Reactive scene: FLASK implementation and evaluation utilities."""
import sys
from pathlib import Path
import numpy as np
import vamp
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ur5_scene
REPLAN_PERIOD = 0.05
START_DELAY = 0.05
TOTAL_TIME = 120.0
PAUSE = 0.5
PADDING = ur5_scene.PADDING
TUBE_SPEED = 0.3
STEP = 0.01
FILTER_SPACING = 0.02
GRID_HALF_DIAGONAL = 0.0361
STATIC_POINT_RADIUS = float(np.hypot(PADDING, GRID_HALF_DIAGONAL))
TUBE_POINT_RADIUS = float(np.hypot(PADDING + TUBE_SPEED * REPLAN_PERIOD, GRID_HALF_DIAGONAL))
REACTIVE_GOAL_CLEARANCE = 0.06
FILTER_ORIGIN = (0.0, 0.0, 0.91)
CULL_R = 1.2
GOAL_CLEARANCE = 0.1
TUBE_RADIUS = 0.0325
TUBE_LENGTH = 1.2
ARC_RADIUS = 1.0

def create_scene():
    return ur5_scene.create_scene('original')

def table_box(p):
    return next((b for b in p['box'] if b['name'] == 'table'))

def configurations(inflated_vamp_environment=None):
    """Configurations."""
    (start_q, _, _) = ur5_scene.configurations(inflated_vamp_environment)
    (kx, ky) = ur5_scene.BLOCK_STACK['center']
    (gx, gy) = ur5_scene.BASKET['center']
    vertex_state = ur5_scene.TABLE_HEIGHT + ur5_scene.BLOCK_STACK['edge'] * ur5_scene.BLOCK_STACK['block_count']
    output = [start_q]
    for p in (np.array([kx, ky, vertex_state + REACTIVE_GOAL_CLEARANCE + ur5_scene.FINGERTIP_OFFSET]), np.array([gx, gy, ur5_scene.TABLE_HEIGHT + ur5_scene.BASKET['wall_thickness'] + REACTIVE_GOAL_CLEARANCE + ur5_scene.FINGERTIP_OFFSET])):
        q0 = ur5_scene._HOME_CONFIGURATION.copy()
        q0[0] = np.arctan2(p[1], p[0]) - np.pi / 2
        q = ur5_scene.ik(p, q0, env=inflated_vamp_environment)
        if q is None:
            raise RuntimeError(f'Inverse kinematics failed at {p}')
        output.append(q)
    return tuple(output)

def _box_surface(center, half_extents, stride=STEP):
    """ box surface."""
    center = np.asarray(center, float)
    half_extents = np.asarray(half_extents, float)
    output = []
    for ax in range(3):
        (u, v) = [a for a in range(3) if a != ax]
        su = np.linspace(-half_extents[u], half_extents[u], int(np.ceil(2 * half_extents[u] / stride)) + 1)
        sv = np.linspace(-half_extents[v], half_extents[v], int(np.ceil(2 * half_extents[v] / stride)) + 1)
        (U, Vv) = np.meshgrid(su, sv, indexing='ij')
        for s in (-1, 1):
            P = np.zeros((U.size, 3))
            P[:, u] = U.ravel()
            P[:, v] = Vv.ravel()
            P[:, ax] = s * half_extents[ax]
            output.append(P + center)
    return np.concatenate(output)

def static_points(p):
    """Static points."""
    P = [_box_surface(b['position'], b['half_extents']) for b in p['box'] if b['name'] != 'table']
    return np.ascontiguousarray(np.concatenate(P), dtype=np.float32)
_SCENES = np.linspace(-TUBE_LENGTH / 2, TUBE_LENGTH / 2, int(round(TUBE_LENGTH / STEP)) + 1)
_ANGLES = _SCENES / ARC_RADIUS
_LOCAL_CENTERS = np.stack([ARC_RADIUS * np.sin(_ANGLES), np.zeros_like(_ANGLES), -ARC_RADIUS * (1 - np.cos(_ANGLES))], 1)
_LOCAL_NORMALS = np.stack([np.sin(_ANGLES), np.zeros_like(_ANGLES), np.cos(_ANGLES)], 1)
_LOCAL_BINORMALS = np.tile([0.0, 1.0, 0.0], (len(_SCENES), 1))
_RING_COUNT = int(np.ceil(2 * np.pi * TUBE_RADIUS / STEP))
_ALPHA = np.linspace(0, 2 * np.pi, _RING_COUNT, endpoint=False)
_LOCAL_SURFACE = (_LOCAL_CENTERS[:, None, :] + TUBE_RADIUS * (np.cos(_ALPHA)[None, :, None] * _LOCAL_NORMALS[:, None, :] + np.sin(_ALPHA)[None, :, None] * _LOCAL_BINORMALS[:, None, :])).reshape(-1, 3)
_load = []
for i in (0, -1):
    for rr in (TUBE_RADIUS / 3, 2 * TUBE_RADIUS / 3):
        a = np.linspace(0, 2 * np.pi, max(int(np.ceil(2 * np.pi * rr / STEP)), 3), endpoint=False)
        _load.append(_LOCAL_CENTERS[i] + rr * (np.cos(a)[:, None] * _LOCAL_NORMALS[i] + np.sin(a)[:, None] * _LOCAL_BINORMALS[i]))
    _load.append(_LOCAL_CENTERS[i][None])
_LOCAL_SURFACE = np.concatenate([_LOCAL_SURFACE] + _load)

def rotation_matrix(psi, theta):
    """Rotation matrix."""
    (cz, sz, cy, sy) = (np.cos(psi), np.sin(psi), np.cos(theta), np.sin(theta))
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    return Rz @ Ry
INSERTION_POSES = {'I1': (-0.39, 0.6, 1.46, 0.31, 0.54), 'I2': (-0.56, 0.72, 1.45, 2.97 - np.pi, 0.25), 'I3': (-0.15, 0.92, 1.33, 1.7 - np.pi, 0.48), 'I4': (-0.02, 0.89, 1.04, 1.27 - np.pi, -0.26)}
RETREAT = 0.6
RETREAT_OFFSETS = {'I4': (0.2, 0.55, 0.0)}
MOTION_SCHEDULE = [('C:I1', 2.0)]
for _k in ['I1', 'I2', 'I4', 'I3', 'I1', 'I2', 'I4', 'I3', 'I1']:
    MOTION_SCHEDULE += [(f'C:{_k}', 1.0), (_k, 4.0), (f'C:{_k}', 0.5)]

def _pose(name):
    if name.startswith('C:'):
        x = np.asarray(INSERTION_POSES[name[2:]], float).copy()
        x[:3] += RETREAT_OFFSETS.get(name[2:], (0.0, RETREAT, 0.0))
        return x
    return np.asarray(INSERTION_POSES[name], float)

def tube_centers(x):
    """Tube centers."""
    return _LOCAL_CENTERS @ rotation_matrix(x[3], x[4]).T + x[:3]

def tube_surface(x):
    return np.ascontiguousarray(_LOCAL_SURFACE @ rotation_matrix(x[3], x[4]).T + x[:3], dtype=np.float32)

def _minimum_jerk(u):
    return u ** 3 * (10 - 15 * u + 6 * u * u)

class TubeMotion:
    """TubeMotion state and operations."""

    def __init__(self, schedule=MOTION_SCHEDULE, v=TUBE_SPEED):
        self.segments = []
        t = 0.0
        previous_pose = _pose(schedule[0][0])
        for (name, keep_plan) in schedule:
            x = _pose(name)
            if np.any(x != previous_pose):
                u = np.linspace(0, 1, 201)
                C = np.stack([tube_centers(previous_pose + (x - previous_pose) * uu) for uu in u])
                dmax = np.max(np.linalg.norm(np.diff(C, axis=0), axis=2)) / (u[1] - u[0])
                T = 1.875 * dmax / v * 1.02
                self.segments.append((t, t + T, previous_pose, x))
                t += T
            self.segments.append((t, t + keep_plan, x, x))
            t += keep_plan
            previous_pose = x
        self.end_time = t

    def pose(self, t):
        for (t0, t1, x0, x1) in self.segments:
            if t <= t1:
                u = min(max((t - t0) / (t1 - t0), 0.0), 1.0)
                return x0 + (x1 - x0) * _minimum_jerk(u)
        return self.segments[-1][3]

    def insertion_active(self, t):
        """Insertion active."""
        th = [np.asarray(v, float) for v in INSERTION_POSES.values()]
        for (t0, t1, x0, x1) in self.segments:
            if t <= t1:
                return bool(any((np.abs(x0 - v).max() < 1e-09 or np.abs(x1 - v).max() < 1e-09 for v in th)))
        return False

def robot_spheres(q):
    """Robot spheres."""
    return np.array([[s.x, s.y, s.z, s.r] for s in vamp.ur5.fk(list(map(float, q)))])

def tube_clearance(q, x):
    """Tube clearance."""
    S = robot_spheres(q)
    C = tube_centers(x)
    d = np.linalg.norm(S[:, None, :3] - C[None], axis=2) - S[:, None, 3] - TUBE_RADIUS
    return float(d.min())
if __name__ == '__main__':
    p = edge()
    ev = vamp.problem_dict_to_vamp(ur5_scene.inflate_scene(p))
    qs = configurations(ev)
    for (name, q) in zip(('start', 'pick', 'place'), qs):
        print(f'{name:6s} ee = {np.round(np.asarray(vamp.ur5.eefk(list(q)))[:3, 3], 3)}  q = {np.round(q, 3)}')
    P = static_points(p)
    print(f'points compute: {len(P)}; points tube: {len(_LOCAL_SURFACE)}; path center {len(_SCENES)} points')
    trail = TubeMotion()
    print(f'trail tube: {trail.end_time:.1f} s, {len(trail.segments)} segments')
    t = np.arange(0, TOTAL_TIME, 0.01)
    X = np.array([trail.pose(tt) for tt in t])
    C = np.stack([tube_centers(x) for x in X])
    v = np.linalg.norm(np.diff(C, axis=0), axis=2).max(1) / 0.01
    print(f'toc do lon nhat knots points tren tube: {v.max():.3f} m/s (gioi han {TUBE_SPEED})')
    for (name, q) in zip(('start', 'pick', 'place'), qs):
        kh = min((tube_clearance(q, x) for x in X[::5]))
        print(f'minimum clearance tube -> robot at {name}: {kh * 100:.1f} cm (required >= {GOAL_CLEARANCE * 100:.0f})')
    for (k, x) in INSERTION_POSES.items():
        print(k, 'minimum clearance reached 3 goal (cm):', [round(tube_clearance(q, np.asarray(x)) * 100, 1) for q in qs])
    accepted_count = [(np.array(b['position']), np.array(b['half_extents'])) for b in p['box']]
    box_clearance = min((np.linalg.norm(np.maximum(np.abs(c[:, None] - hc) - hh, 0), axis=2).min() for c in C[::10] for (hc, hh) in [(np.array([h[0] for h in accepted_count]), np.array([h[1] for h in accepted_count]))])) - TUBE_RADIUS
    print(f'minimum clearance tube -> stationary objects (whole trajectory): {box_clearance * 100:.1f} cm')
    inside = [np.linalg.norm(c - np.array(FILTER_ORIGIN), axis=1).min() < CULL_R for c in C[::10]]
    print(f'tube inside vung loc {np.mean(inside) * 100:.0f}% of time; insertion active {np.mean([trail.insertion_active(tt) for tt in t[::10]]) * 100:.0f} %')
