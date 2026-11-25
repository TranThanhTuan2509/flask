"""Ur5 scene: FLASK implementation and evaluation utilities."""
import numpy as np
import vamp
TABLE_HEIGHT = 0.86
FINGERTIP_OFFSET = 0.155
GRANOLA_BOX = dict(center=(0.4, 0.55), half_extents=(0.03, 0.08, 0.11))
BASKET = dict(center=(-0.4, 0.55), inner_half_width=0.14, wall_thickness=0.03, height=0.22)
BLOCK_STACK = dict(center=(0.0, 0.55), edge=0.1, block_count=4)
ARM_HALF_EXTENT = 0.077
GRIPPER_BOX = dict(clearance=0.03, wall_thickness=0.06, half_width_y=0.1, extra_height=0.1)
PADDING = 0.03
PICK_CLEARANCE = 0.04

def _box(name, center, half_extents):
    return dict(name=name, position=[float(x) for x in center], half_extents=[float(x) for x in half_extents], orientation_euler_xyz=(0.0, 0.0, 0.0), orientation_quat_xyzw=[0.0, 0.0, 0.0, 1.0])

def create_scene(variant='original'):
    b = [_box('table', (0.0, 0.6, TABLE_HEIGHT - 0.03), (0.75, 0.3, 0.03))]
    (hx, hy) = GRANOLA_BOX['center']
    n = GRANOLA_BOX['half_extents']
    b.append(_box('granola', (hx, hy, TABLE_HEIGHT + n[2]), n))
    (gx, gy) = BASKET['center']
    (r, d, c) = (BASKET['inner_half_width'], BASKET['wall_thickness'], BASKET['height'])
    b.append(_box('basket_bottom', (gx, gy, TABLE_HEIGHT + d / 2), (r + d, r + d, d / 2)))
    for s in (-1, 1):
        b.append(_box(f'basket_wall_x{s:+d}', (gx + s * (r + d / 2), gy, TABLE_HEIGHT + c / 2), (d / 2, r + d, c / 2)))
        b.append(_box(f'basket_wall_y{s:+d}', (gx, gy + s * (r + d / 2), TABLE_HEIGHT + c / 2), (r + d, d / 2, c / 2)))
    (kx, ky) = BLOCK_STACK['center']
    a = BLOCK_STACK['edge']
    for k in range(BLOCK_STACK['block_count']):
        b.append(_box(f'block_{k}', (kx, ky, TABLE_HEIGHT + a * (k + 0.5)), (a / 2, a / 2, a / 2)))
    if variant == 're':
        k = GRIPPER_BOX
        height = 2 * n[2] + k['extra_height']
        for s in (-1, 1):
            b.append(_box(f'gripper_enclosure{s:+d}', (hx + s * (ARM_HALF_EXTENT + k['clearance'] + k['wall_thickness'] / 2), hy, TABLE_HEIGHT + height / 2), (k['wall_thickness'] / 2, k['half_width_y'], height / 2)))
    elif variant != 'original':
        raise ValueError(variant)
    return dict(problem=f'e1_ur5_{variant}', sphere=[], cylinder=[], box=b, valid=True)

def inflate_scene(p, counter=PADDING):
    """Inflate scene."""
    if counter == 0:
        return p
    return dict(p, box=[dict(b, half_extents=[h + counter for h in b['half_extents']]) for b in p['box']])
_HOME_CONFIGURATION = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
LINK_RADIUS = np.asarray(vamp.ur5.eefk(list(_HOME_CONFIGURATION)), float)[:3, :3]

def _error(q, p, R):
    T = np.asarray(vamp.ur5.eefk(list(q)), float)
    e_p = p - T[:3, 3]
    e_r = 0.5 * sum((np.cross(T[:3, k], R[:, k]) for k in range(3)))
    return np.concatenate([e_p, e_r])

def ik(p, q0, R=LINK_RADIUS, env=None, iterations=300):
    """Ik."""
    q = np.array(q0, float)
    for _ in range(iterations):
        e = _error(q, p, R)
        if np.linalg.norm(e[:3]) < 1e-05 and np.linalg.norm(e[3:]) < 1e-05:
            break
        J = np.empty((6, 6))
        h = 1e-06
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = h
            J[:, j] = (e - _error(q + dq, p, R)) / h
        q = q + J.T @ np.linalg.solve(J @ J.T + 1e-06 * np.eye(6), e)
    q = (q + np.pi) % (2 * np.pi) - np.pi
    e = _error(q, p, R)
    if np.linalg.norm(e[:3]) > 0.0001 or np.linalg.norm(e[3:]) > 0.0001:
        return None
    if env is not None and (not vamp.ur5.validate(list(q), env)):
        return None
    return q

def configurations(env=None, pick_clearance=PICK_CLEARANCE):
    """Configurations."""
    (hx, hy) = GRANOLA_BOX['center']
    (gx, gy) = BASKET['center']
    start_position = np.array([0.15, 0.35, 1.45])
    pick_position = np.array([hx, hy, TABLE_HEIGHT + 2 * GRANOLA_BOX['half_extents'][2] + pick_clearance + FINGERTIP_OFFSET])
    place_position = np.array([gx, gy, TABLE_HEIGHT + BASKET['wall_thickness'] + 0.04 + FINGERTIP_OFFSET])
    output = []
    for p in (start_position, pick_position, place_position):
        q0 = _HOME_CONFIGURATION.copy()
        q0[0] = np.arctan2(p[1], p[0]) - np.pi / 2
        q = ik(p, q0, env=env)
        if q is None:
            raise RuntimeError(f'Inverse kinematics failed at {p}')
        output.append(q)
    return tuple(output)
if __name__ == '__main__':
    import sys
    p = create_scene(sys.argv[1] if len(sys.argv) > 1 else 'original')
    env = vamp.problem_dict_to_vamp(inflate_scene(p))
    (s, k, d) = configurations(env)
    for (name, q) in (('start', s), ('pick', k), ('place', d)):
        T = np.asarray(vamp.ur5.eefk(list(q)), float)
        print(f'{name:6s} q = {np.round(q, 3)}  ee = {np.round(T[:3, 3], 3)}  accepted_count le (edge counter) {vamp.ur5.validate(list(q), env)}')
