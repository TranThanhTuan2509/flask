"""Showcase: UR5 moving through a cluttered sorting cell, every motion planned online by FLASK (native, VAMP).

Each leg: FLASK-RRTConnect + Alg. 4 in the delta-inflated scene with the certified profile (no false accepts in the
acceptance tests); the trajectory is tracked in MuJoCo by experiments/ur5_mujoco.Tracker (Ruckig + computed torque,
acceleration feed-forward) with the UR5's torque limits.

    python -m simulation.showcase.ur5_cell --seed 1 --output showcase/showcase_1/ur5/seed_1
"""
import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import vamp

from experiments import ur5_mujoco as E1
from experiments.ur5_scene import FINGERTIP_OFFSET, PADDING, TABLE_HEIGHT, _box, _HOME_CONFIGURATION, ik, inflate_scene
from flask_repro import mbm, native, robot as R

H = TABLE_HEIGHT
COLORS = dict(table=(0.80, 0.78, 0.74, 1), bin=(0.16, 0.42, 0.72, 1), post=(0.35, 0.37, 0.40, 1), arch=(0.92, 0.55, 0.16, 1),
              item=(0.85, 0.25, 0.25, 1))


def scene():
    b = [_box('table', (0.0, 0.6, H - 0.03), (0.75, 0.32, 0.03))]
    for name, cx in (('bin_left', -0.42), ('bin_right', 0.42)):
        r, d, h, cy = 0.12, 0.02, 0.16, 0.50
        b.append(_box(f'{name}_bottom', (cx, cy, H + d / 2), (r + d, r + d, d / 2)))
        for s in (-1, 1):
            b.append(_box(f'{name}_wx{s:+d}', (cx + s * (r + d / 2), cy, H + h / 2), (d / 2, r + d, h / 2)))
            b.append(_box(f'{name}_wy{s:+d}', (cx, cy + s * (r + d / 2), H + h / 2), (r + d, d / 2, h / 2)))
    b.append(_box('post_center', (0.0, 0.50, H + 0.21), (0.03, 0.03, 0.21)))
    for s in (-1, 1):
        b.append(_box(f'arch_post{s:+d}', (s * 0.34, 0.80, H + 0.27), (0.025, 0.025, 0.27)))
    b.append(_box('arch_beam', (0.0, 0.80, H + 0.56), (0.37, 0.025, 0.025)))
    for k, (x, y) in enumerate([(-0.46, 0.47), (-0.38, 0.55), (0.40, 0.46), (0.46, 0.55)]):
        b.append(_box(f'item_{k}', (x, y, H + 0.02 + 0.03), (0.03, 0.03, 0.03)))
    return dict(problem='ur5_sorting_cell', sphere=[], cylinder=[], box=b, valid=True)


TARGETS = [(-0.42, 0.50, 0.24), (0.30, 0.34, 0.20), (0.0, 0.66, 0.30), (0.42, 0.50, 0.24), (-0.30, 0.66, 0.20),
           (-0.10, 0.34, 0.10)]   # (x, y, fingertip height above the table)


def color(name):
    for k in ('table', 'bin', 'post', 'arch', 'item'):
        if name.startswith(k):
            return COLORS[k]
    return (0.6, 0.6, 0.6, 1)


def build_model(sc):
    def extra(root):
        world = root.find('worldbody')
        for g in world.findall('geom'):
            if g.get('name', '').startswith('scene_'):
                g.set('rgba', ' '.join(map(str, color(g.get('name')[6:]))))
        asset = root.find('asset') if root.find('asset') is not None else ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'texture', name='cell_floor', type='2d', builtin='checker', rgb1='.46 .47 .49', rgb2='.42 .43 .45',
                      mark='edge', markrgb='.36 .37 .39', width='512', height='512')
        ET.SubElement(asset, 'material', name='cell_floor', texture='cell_floor', texrepeat='4 4', texuniform='true', reflectance='.06')
        ET.SubElement(asset, 'texture', name='cell_sky', type='skybox', builtin='gradient', rgb1='.62 .70 .80', rgb2='.92 .94 .96',
                      width='512', height='3072')
        ET.SubElement(world, 'geom', name='floor', type='plane', size='3 3 0.05', pos='0 0.4 0', material='cell_floor')
        ET.SubElement(world, 'geom', name='pedestal', type='cylinder', size='0.09 0.455', pos='0 0 0.455', rgba='.28 .30 .33 1',
                      contype='0', conaffinity='0')
        for x in (-0.70, 0.70):
            for y in (0.32, 0.88):
                ET.SubElement(world, 'geom', name=f'table_leg_{x:+.1f}_{y:.2f}', type='box', size='0.03 0.03 0.40',
                              pos=f'{x} {y} 0.40', rgba='.55 .56 .58 1', contype='0', conaffinity='0')
        ET.SubElement(world, 'light', name='key', pos='1.2 -0.6 3.2', dir='-0.35 0.3 -1', diffuse='.6 .6 .58', cutoff='45',
                      exponent='3', castshadow='true')
        ET.SubElement(world, 'light', name='fill', pos='-1.5 1.5 2.5', dir='0.5 -0.4 -1', diffuse='.25 .25 .27', castshadow='false')
        ET.SubElement(world, 'camera', name='top', pos='0 0.55 2.6', xyaxes='1 0 0 0 1 0', fovy='38')
        for k, (x, y, dz) in enumerate(TARGETS):
            ET.SubElement(world, 'site', name=f'target_{k}', pos=f'{x} {y} {H + dz - FINGERTIP_OFFSET + 0.155}', size='.018', rgba='.15 .85 .65 .6')
        vis = root.find('visual') if root.find('visual') is not None else ET.SubElement(root, 'visual')
        ET.SubElement(vis, 'global', offwidth='3840', offheight='2160')
        ET.SubElement(vis, 'quality', shadowsize='8192')
        ET.SubElement(vis, 'headlight', ambient='.32 .32 .34', diffuse='.35 .35 .35', specular='.05 .05 .05')
    return E1.build_model(sc, extra=extra, collision_geometry='mesh')


def goal_configurations(env):
    qs = []
    for x, y, dz in TARGETS:
        p = np.array([x, y, H + dz + FINGERTIP_OFFSET])
        q0 = _HOME_CONFIGURATION.copy(); q0[0] = np.arctan2(y, x) - np.pi / 2
        q = ik(p, q0, env=env)
        if q is None:
            raise RuntimeError(f'IK failed for target {(x, y, dz)}')
        qs.append(q)
    return qs


def ee(q):
    return np.asarray(vamp.ur5.eefk(list(q)), float)[:3, 3]


def run(seed, output):
    output.mkdir(parents=True, exist_ok=False)
    rb = R.get_robot('ur5'); sc = scene()
    planning = inflate_scene(sc, PADDING + E1.CERTIFY_DELTA)
    ev = vamp.problem_dict_to_vamp(planning); penv = mbm.native_environment(planning); senv = mbm.native_environment(sc)
    # nominal planning (default peak_speed validation) in the PADDING-inflated scene; the returned path is then
    # certified in the scene inflated by PADDING + delta (lazy certification: replans if certification fails)
    nominal_env = mbm.native_environment(inflate_scene(sc, PADDING))
    settings = native.settings(**E1.FLASK_SETTINGS)
    native.plan_lazy_certified(rb, nominal_env, penv, start_q := np.array([0.0, -1.9, 1.75, -1.42, -1.57, 0.0]),
                               [start_q], settings, seed=0, certify_delta=E1.CERTIFY_DELTA)   # warm-up, not timed
    goals = goal_configurations(ev)
    start = np.array([0.0, -1.9, 1.75, -1.42, -1.57, 0.0])
    model = build_model(sc); data = mujoco.MjData(model)
    E1.settle(model, data, start, rb.n)
    tracker = E1.Tracker(model, data, rb, start)
    log = dict(time=[], q=[], qd=[], tau=[], contact=[], leg=[])
    plans, t, current = [], 0., start
    for leg, goal in enumerate(goals):
        t0 = time.perf_counter()
        d = native.plan_lazy_certified(rb, nominal_env, penv, current, [goal], settings, seed=seed * 100 + leg,
                                       certify_delta=E1.CERTIFY_DELTA)
        wall = (time.perf_counter() - t0) * 1e3
        if not d['solved']:
            raise RuntimeError(f'leg {leg}: plan failed')
        traj = d['trajectory']; T = traj.duration()
        ts = np.linspace(0, T, 300); Q = traj.evaluate(ts)[0]
        dense = traj.evaluate(np.arange(0., T, E1.DT_CONTROL))[0]          # reference vs the UNPADDED scene
        refcol = int(native.module(rb).count_collisions(senv, dense.tolist())[0])
        plans.append(dict(leg=leg, t_start=t, duration=T, plan_us=d['plan_ns'] / 1e3, shortcut_us=d['shortcut_ns'] / 1e3,
                          total_us=(d['plan_ns'] + d['shortcut_ns']) / 1e3, rejected_paths=int(d.get('rejected_paths', 0)),
                          wall_ms=wall, segments=len(d['T']), ee_path=[ee(q).tolist() for q in Q],
                          waypoints=np.asarray(d['Q']).tolist(), reference_collision_samples=int(refcol)))
        for k in range(int((T + E1.PAUSE) / E1.DT_CONTROL)):
            tk = min(k * E1.DT_CONTROL, T)
            q, v, a = (x[0] for x in traj.evaluate([tk])[:3])
            if k * E1.DT_CONTROL > T:
                v, a = np.zeros_like(v), np.zeros_like(a)
            tracker.step(q, v, a)
            depth, _ = E1.contacts(model, data)
            t += E1.DT_CONTROL
            if k % 5 == 0:
                log['time'].append(t); log['q'].append(data.qpos[:rb.n].copy()); log['qd'].append(data.qvel[:rb.n].copy())
                log['tau'].append(data.ctrl[:rb.n].copy()); log['contact'].append(depth > 0); log['leg'].append(leg)
        current = goal
    arrays = {k: np.asarray(v) for k, v in log.items()}
    arrays['ee'] = np.array([ee(q) for q in arrays['q']])
    np.savez_compressed(output / 'execution.npz', **arrays)
    summary = dict(robot='UR5', seed=seed, legs=len(plans), contact_samples=int(arrays['contact'].sum()),
                   reference_collision_samples=sum(p['reference_collision_samples'] for p in plans),
                   plan_us=[p['plan_us'] for p in plans], shortcut_us=[p['shortcut_us'] for p in plans],
                   max_track_error_rad=None, planner='FLASK-RRTConnect + Alg. 4 (native, VAMP), lazy-certified', total_us=[p['total_us'] for p in plans])
    (output / 'plans.json').write_text(json.dumps(plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
