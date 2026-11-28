"""Showcase 2: UR5 + Robotiq 2F-85 sorts three cubes from tray A into tray B through a cluttered cell.

Every motion (approach, descend, lift, transfer with the cube in hand, place, retreat) is planned online by FLASK
(native, VAMP, lazy-certified; the held cube is part of the collision model while it is carried) and tracked in
MuJoCo with the UR5's torque limits. Grasps close the fingers by contact; see simulation/showcase/arms.py for the
weld that is switched on once the finger encoders report a stall.

    python -m simulation.showcase.ur5_pickplace --seed 1 --output showcase/showcase_2/ur5/seed_1
"""
import argparse
import json
from pathlib import Path

import numpy as np

from simulation.showcase import arms as A
from simulation.showcase.manip import Cell

H = 0.86                                    # table top (experiments/ur5_scene.TABLE_HEIGHT)
HALF = (0.022, 0.022, 0.022)                # cubes, 4.4 cm
TRAY = dict(A=(-0.38, 0.52), B=(0.38, 0.52))
COLORS = dict(table=(0.80, 0.78, 0.74, 1), tray=(0.16, 0.42, 0.72, 1), post=(0.35, 0.37, 0.40, 1),
              arch=(0.92, 0.55, 0.16, 1), bar=(0.92, 0.55, 0.16, 1), pillar=(0.35, 0.37, 0.40, 1))
CUBES = dict(red=((-0.44, 0.46), (0.32, 0.46), (0.85, 0.22, 0.20, 1)),
             green=((-0.32, 0.46), (0.44, 0.46), (0.22, 0.65, 0.30, 1)),
             blue=((-0.38, 0.57), (0.38, 0.57), (0.20, 0.40, 0.85, 1)))
HOME = np.array([0.0, -1.9, 1.75, -1.42, -1.57, 0.0])
ARM = A.Arm('ur5', 'ur5/')


def _box(name, c, h):
    return dict(name=name, position=list(c), half_extents=list(h), orientation_euler_xyz=(0., 0., 0.))


def scene():
    b = [_box('table', (0.0, 0.6, H - 0.03), (0.75, 0.32, 0.03))]
    for k, (cx, cy) in TRAY.items():
        r, d, h = 0.15, 0.01, 0.04
        b.append(_box(f'tray{k}_floor', (cx, cy, H + d / 2), (r + d, r + d, d / 2)))
        for s in (-1, 1):
            b.append(_box(f'tray{k}_wx{s:+d}', (cx + s * (r + d / 2), cy, H + h / 2), (d / 2, r + d, h / 2)))
            b.append(_box(f'tray{k}_wy{s:+d}', (cx, cy + s * (r + d / 2), H + h / 2), (r + d, d / 2, h / 2)))
    b.append(_box('post_center', (0.0, 0.52, H + 0.20), (0.03, 0.03, 0.20)))
    b.append(_box('bar_center', (0.0, 0.66, H + 0.385), (0.03, 0.14, 0.015)))     # from the post top towards the back
    for s in (-1, 1):
        b.append(_box(f'arch_post{s:+d}', (s * 0.34, 0.84, H + 0.27), (0.025, 0.025, 0.27)))
    b.append(_box('arch_beam', (0.0, 0.84, H + 0.56), (0.37, 0.025, 0.025)))
    b.append(_box('pillar_front', (0.16, 0.30, H + 0.17), (0.025, 0.025, 0.17)))
    return b


def color(name):
    for k, c in COLORS.items():
        if name.startswith(k):
            return c
    return (0.6, 0.6, 0.6, 1)


def objects():
    return {k: dict(position=[x, y, H + 0.01 + HALF[2]], half_extents=list(HALF), rgba=c, mass=0.08)
            for k, ((x, y), _, c) in CUBES.items()}


CAMERA = ([0.0, 0.55, 1.0], 1.8, 125, -28)         # renderer: lookat, distance, azimuth, elevation


def targets():
    return [(gx, gy, H + 0.012) for (_, (gx, gy), _) in CUBES.values()]


def placed(data):
    """Cubes resting within 2 cm of their slot in tray B (renderer HUD)."""
    ok = sum(np.linalg.norm(data.body(f'object_{k}').xpos[:2] - np.array(g)) < 0.02 and data.body(f'object_{k}').xpos[2] < H + 0.06
             for k, (_, g, _) in CUBES.items())
    return int(ok), len(CUBES)


def cameras():
    return [('top', (0, 0.55, 2.7), (1, 0, 0, 0, 1, 0), 40)]


def build_model():
    return A.build([ARM], scene(), objects(), color, cameras=cameras())


def queue(env):
    """IK for every grasp/place pose (gripper down, fingers along x), then the action list."""
    q, Q = HOME, []
    rot = A.down(np.pi / 2)                 # fingers close along y, away from the trays' side walls
    for name, ((x, y), (gx, gy), _) in CUBES.items():
        z = H + 0.01 + HALF[2]
        poses = dict(pre_pick=(x, y, z + 0.13), pick=(x, y, z + 0.004), pre_place=(gx, gy, z + 0.13), place=(gx, gy, z + 0.006))
        sol = {}
        for k, p in poses.items():
            sol[k] = A.ik(ARM, np.array(p), rot, q if k == 'pre_pick' else sol.get('pre_pick', q))
            if sol[k] is None:
                raise RuntimeError(f'IK failed for {name} {k}')
        Q += [('move', sol['pre_pick'], f'approach {name}'),
              ('move', sol['pick'], f'descend to {name}', (name,)),
              ('close', name, f'grasp {name}'),
              ('move', sol['pre_pick'], f'lift {name}', (name, 'trayA_floor', 'table')),
              ('move', sol['pre_place'], f'carry {name}'),
              ('move', sol['place'], f'lower {name}', ('trayB_floor', 'table')),
              ('open', name, f'release {name}', (gx, gy, z)),
              ('move', sol['pre_place'], 'retreat', (name,))]
        q = sol['pre_place']
    Q.append(('move', HOME, 'home'))
    return Q


def run(seed, output):
    output.mkdir(parents=True, exist_ok=False)
    model = build_model()
    cell = Cell(model, [ARM], [HOME], [queue(None)], scene(), objects(), seed)
    cell.run(max_time=90.)
    d = cell.d
    final = {k: d.body(f'object_{k}').xpos.tolist() for k in CUBES}
    errors = {k: float(np.linalg.norm(np.array(final[k][:2]) - np.array(CUBES[k][1]))) for k in CUBES}
    log = {k: np.asarray(v) for k, v in cell.log.items() if k != 'phase'}
    np.savez_compressed(output / 'execution.npz', **log, phase=np.array(['|'.join(p) for p in cell.log['phase']]))
    summary = dict(robot='UR5 + Robotiq 2F-85', task='pick-and-place, 3 cubes', seed=seed, motions=len(cell.plans),
                   placed=sum(e < 0.02 for e in errors.values()), place_error_m=errors,
                   contact_pairs=sorted(map(list, cell.bad_pairs)), contacts=len(cell.bad_pairs),
                   plan_us=[p['plan_us'] for p in cell.plans], total_us=[p['total_us'] for p in cell.plans],
                   total_ms=sum(p['total_us'] for p in cell.plans) / 1e3, duration_s=cell.t,
                   planner='FLASK-RRTConnect + Alg. 4 (native, VAMP), lazy-certified; held cube in the collision model',
                   grasp='fingers close by contact (Menagerie 2F-85); weld on after the encoders report a stall')
    (output / 'plans.json').write_text(json.dumps(cell.plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k not in ('plan_us', 'total_us')}))
    return summary


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
