"""Showcase 2, multi-agent manipulation: a UR5 (Robotiq 2F-85) and a Franka Panda pass three bars from tray A to
tray B by hand-over. Neither arm reaches the other's tray, so every bar must change hands mid-air above a divider.

Prioritized space-time planning (simulation/showcase/manip.py): every motion is planned by FLASK when it starts and
must avoid the other arm's committed motion (its swept VAMP spheres from now to the end of its trajectory, its rest
pose and the object it holds). The arms work in parallel: the UR5 fetches the next bar while the Panda places the
previous one. The hand-over is synchronized by events: the UR5 presents the bar lying flat, holding its near end from
above; the Panda takes the far end from above; the UR5 lets go only after the Panda's encoders report a grasp, and
the Panda leaves only after the UR5 has moved clear.

    python -m simulation.showcase.multiarm_handover --seed 1 --output showcase/showcase_2/multiarm/seed_1
"""
import argparse
import json
from pathlib import Path

import numpy as np

from simulation.showcase import arms as A
from simulation.showcase.manip import PADDING, Cell

H = 0.86
HALF = (0.12, 0.02, 0.02)                   # bars 24 x 4 x 4 cm, lying along x
GRIP = 0.085                                # each arm grasps 8.5 cm from the bar centre: grips stay > 4 cm apart
UR5 = A.Arm('ur5', 'ur5/', (0., 0., 0.), 0.)
PANDA = A.Arm('panda', 'panda/', (1.0, 0., H), 0.)
ARMS = [UR5, PANDA]
TRAY = dict(A=(-0.30, 0.50), B=(1.30, 0.50))
BARS = dict(red=((-0.30, 0.40), (1.30, 0.37), (0.85, 0.22, 0.20, 1)),
            green=((-0.30, 0.50), (1.30, 0.50), (0.22, 0.65, 0.30, 1)),
            blue=((-0.30, 0.60), (1.30, 0.63), (0.20, 0.40, 0.85, 1)))
HANDOVER = np.array([0.50, 0.45, H + 0.30])  # bar centre during the hand-over
UR5_HOME = np.array([0.0, -1.9, 1.75, -1.42, -1.57, 0.0])
PANDA_HOME = np.array([0.0, -0.6, 0.0, -2.4, 0.0, 1.9, 0.785])
COLORS = dict(table=(0.80, 0.78, 0.74, 1), tray=(0.16, 0.42, 0.72, 1), divider=(0.92, 0.55, 0.16, 1),
              post=(0.35, 0.37, 0.40, 1), beam=(0.92, 0.55, 0.16, 1))
CAMERA = ([0.5, 0.45, 1.05], 2.25, 70, -38)


def _box(name, c, h):
    return dict(name=name, position=list(c), half_extents=list(h), orientation_euler_xyz=(0., 0., 0.))


def scene():
    b = [_box('table', (0.5, 0.55, H - 0.03), (1.05, 0.36, 0.03))]
    for k, (cx, cy) in TRAY.items():
        rx, ry, d, h = 0.18, 0.18 if k == 'A' else 0.26, 0.01, 0.04   # tray B wider: the Panda hand spans +-10 cm in y
        b.append(_box(f'tray{k}_floor', (cx, cy, H + d / 2), (rx + d, ry + d, d / 2)))
        for s in (-1, 1):
            b.append(_box(f'tray{k}_wx{s:+d}', (cx + s * (rx + d / 2), cy, H + h / 2), (d / 2, ry + d, h / 2)))
            b.append(_box(f'tray{k}_wy{s:+d}', (cx, cy + s * (ry + d / 2), H + h / 2), (rx + d, d / 2, h / 2)))
    b.append(_box('divider', (0.5, 0.58, H + 0.08), (0.012, 0.30, 0.08)))
    for x in (0.18, 0.82):
        b.append(_box(f'post_{x:.2f}', (x, 0.76, H + 0.24), (0.025, 0.025, 0.24)))
    b.append(_box('beam', (0.5, 0.76, H + 0.495), (0.345, 0.025, 0.015)))
    return b


def color(name):
    for k, c in COLORS.items():
        if name.startswith(k):
            return c
    return (0.6, 0.6, 0.6, 1)


def objects():
    return {k: dict(position=[x, y, H + 0.01 + HALF[2]], half_extents=list(HALF), rgba=c, mass=0.08)
            for k, ((x, y), _, c) in BARS.items()}


def targets():
    return [(gx, gy, H + 0.012) for (_, (gx, gy), _) in BARS.values()]


def placed(data):
    ok = sum(np.linalg.norm(data.body(f'object_{k}').xpos[:2] - np.array(g)) < 0.02 and data.body(f'object_{k}').xpos[2] < H + 0.1
             for k, (_, g, _) in BARS.items())
    return int(ok), len(BARS)


def build_model():
    def decorate(w, asset):
        import xml.etree.ElementTree as ET
        ET.SubElement(w, 'geom', name='ur5_pedestal', type='cylinder', size='0.09 0.455', pos='0 0 0.455',
                      rgba='.28 .30 .33 1', contype='0', conaffinity='0')
        ET.SubElement(w, 'geom', name='panda_pedestal', type='cylinder', size='0.11 0.43', pos=f'1.0 0 0.43',
                      rgba='.28 .30 .33 1', contype='0', conaffinity='0')
    return A.build(ARMS, scene(), objects(), color, decorate=decorate,
                   cameras=[('top', (0.5, 0.45, 3.0), (1, 0, 0, 0, 1, 0), 45)])


def queues():
    """Both arms grasp from above with fingers closing along y; the UR5 holds the bar's -x end, the Panda its +x end."""
    ur, pa = [], []
    q_ur, q_pa = UR5_HOME, PANDA_HOME
    top_u, top_p = A.down(np.pi / 2), A.down(np.pi / 2, 'panda')
    zc = H + 0.01 + HALF[2]
    present = A.ik(UR5, HANDOVER - [GRIP, 0., 0.], top_u, UR5_HOME, restarts=30)
    clear_ur5 = A.ik(UR5, HANDOVER - [GRIP + 0.06, 0., -0.14], top_u, present)
    # the Panda's take poses keep clear of the presenting UR5 (its VAMP spheres + the planning padding)
    clear = A.environment(PANDA, scene(), PADDING + 0.005, UR5.world_spheres(present))
    pre_take = A.ik(PANDA, HANDOVER + [GRIP, 0., 0.12], top_p, PANDA_HOME, env=clear, restarts=40)
    take = A.ik(PANDA, HANDOVER + [GRIP, 0., 0.004], top_p, pre_take, env=clear, restarts=40)
    for k, v in dict(present=present, clear_ur5=clear_ur5, pre_take=pre_take, take=take).items():
        if v is None:
            raise RuntimeError(f'IK failed: {k}')
    for name, ((x, y), (gx, gy), _) in BARS.items():
        pick = np.array([x - GRIP, y, zc + 0.004])
        place = np.array([gx + GRIP, gy, zc + 0.006])
        p = dict(pre_pick=A.ik(UR5, pick + [0, 0, 0.13], top_u, q_ur))
        p['pick'] = A.ik(UR5, pick, top_u, p['pre_pick'])
        p['pre_place'] = A.ik(PANDA, place + [0., 0., 0.13], top_p, take)
        p['place'] = A.ik(PANDA, place, top_p, p['pre_place'])
        for k, v in p.items():
            if v is None:
                raise RuntimeError(f'IK failed: {name} {k}')
        ur += [('move', p['pre_pick'], f'approach {name}'),
               ('move', p['pick'], f'descend to {name}', (name,)),
               ('close', name, f'grasp {name}'),
               ('move', p['pre_pick'], f'lift {name}', (name, 'trayA_floor', 'table')),
               ('move', present, f'present {name}'),
               ('signal', f'ready_{name}'),
               ('wait', f'taken_{name}'),
               ('open', name, f'hand over {name}', None),
               ('move', clear_ur5, 'clear', (name,)),
               ('signal', f'released_{name}')]                       # the Panda leaves once the UR5 is clear
        pa += [('wait', f'ready_{name}'),
               ('move', pre_take, f'reach {name}', (name,)),
               ('move', take, f'hand {name}', (name,)),
               ('close', name, f'hand take {name}'),
               ('signal', f'taken_{name}'),
               ('wait', f'released_{name}'),
               ('move', pre_take, f'withdraw {name}', (name,)),
               ('move', p['pre_place'], f'carry {name}'),
               ('move', p['place'], f'lower {name}', ('trayB_floor', 'table')),
               ('open', name, f'release {name}', (gx, gy, zc)),
               ('move', p['pre_place'], 'retreat', (name,))]
        q_ur = p['pre_pick']
    ur.append(('move', UR5_HOME, 'home'))
    pa.append(('move', PANDA_HOME, 'home'))
    return ur, pa


def run(seed, output):
    output.mkdir(parents=True, exist_ok=False)
    model = build_model()
    qu, qp = queues()
    cell = Cell(model, ARMS, [UR5_HOME, PANDA_HOME], [qu, qp], scene(), objects(), seed)
    cell.run(max_time=150.)
    d = cell.d
    final = {k: d.body(f'object_{k}').xpos.tolist() for k in BARS}
    errors = {k: float(np.linalg.norm(np.array(final[k][:2]) - np.array(BARS[k][1]))) for k in BARS}
    flat = {k: float(abs(d.body(f'object_{k}').xmat.reshape(3, 3)[2, 2])) for k in BARS}     # 1 = lying flat
    log = {k: np.asarray(v) for k, v in cell.log.items() if k != 'phase'}
    np.savez_compressed(output / 'execution.npz', **log, phase=np.array(['|'.join(p) for p in cell.log['phase']]))
    # independent check of the executed motion: smallest distance between the two arms' VAMP spheres (1 kHz log at 100 Hz)
    io = [A.ArmIO(a, model) for a in ARMS]
    gaps = []
    for q in log['qpos'][::2]:
        S1, S2 = UR5.world_spheres(q[io[0].qadr]), PANDA.world_spheres(q[io[1].qadr])
        D = np.linalg.norm(S1[:, None, :3] - S2[None, :, :3], axis=2) - S1[:, None, 3] - S2[None, :, 3]
        gaps.append(D.min())
    summary = dict(robots='UR5 + Robotiq 2F-85, Franka Panda', task='hand-over pick-and-place, 3 bars (24 cm)', seed=seed,
                   motions=len(cell.plans), placed=sum(e < 0.02 for e in errors.values()), place_error_m=errors,
                   lying_flat=flat, contact_pairs=sorted(map(list, cell.bad_pairs)), contacts=len(cell.bad_pairs),
                   min_arm_sphere_gap_m=float(np.min(gaps)),
                   total_ms={a.prefix.rstrip('/'): sum(p['total_us'] for p in cell.plans if p['arm'] == a.prefix.rstrip('/')) / 1e3 for a in ARMS},
                   total_us=[p['total_us'] for p in cell.plans] + [f['plan_us'] for st in cell.st for f in st.failed],
                   replans_after_waiting=sum(len(st.failed) for st in cell.st),
                   failed_plan_ms=sum(f['plan_us'] for st in cell.st for f in st.failed) / 1e3, duration_s=cell.t,
                   planner='FLASK-RRTConnect + Alg. 4 (native, VAMP), lazy-certified; prioritized space-time planning '
                           '(the other arm\'s committed motion as swept spheres); held objects in the collision model')
    (output / 'plans.json').write_text(json.dumps(cell.plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'total_us'}))
    return summary


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
