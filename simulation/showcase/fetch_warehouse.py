"""Showcase: Fetch mobile manipulator in a warehouse. FLASK plans both the base and the arm.

Base: FLASK BVP-SST* on the unicycle flat model (paper limits |v| <= 1, |omega| <= 1.5), uniformly time-scaled to
the executed limits (0.7 m/s, 1.0 rad/s); a Kanayama law turns it into diff-drive commands for a velocity-servoed
planar base (x, y, yaw joints). Arm: FLASK-RRTConnect + Alg. 4 (native, VAMP Fetch sphere model: torso + 7 arm
joints) in the base frame, tracked by position servos with gravity compensation.

    python -m simulation.showcase.fetch_warehouse --seed 1 --output showcase/showcase_1/fetch/seed_1
"""
import argparse
import dataclasses
import json
import time
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

import mujoco
import numpy as np
import vamp

from flask_repro import mbm, native, robot as R
from flask_repro.flat_models import Unicycle
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings, plan, validate
from simulation.robot_assets import urdf_root
from simulation.showcase.tracking import kanayama

ROOT = Path(__file__).resolve().parents[2]
BOUNDS = ([0., 0.], [12., 8.])
RADIUS = 0.42
RACK, CARTON, STEEL, TABLE = (0.20, 0.36, 0.62, 1), (0.70, 0.55, 0.36, 1), (0.40, 0.42, 0.45, 1), (0.82, 0.80, 0.76, 1)
# (name, shape, centre, size (box xy) or radius, height, rgba)
OBSTACLES = [
    ('rack_a', 'box', (3.0, 2.6), (3.6, 0.6), 2.0, RACK), ('rack_b', 'box', (3.0, 5.4), (3.6, 0.6), 2.0, RACK),
    ('rack_c', 'box', (7.6, 1.3), (2.8, 0.6), 2.0, RACK), ('rack_d', 'box', (7.6, 4.3), (0.6, 2.2), 2.0, RACK),
    ('rack_e', 'box', (7.4, 7.25), (3.0, 0.6), 2.0, RACK),
    ('pallet_a', 'box', (5.5, 4.0), (1.0, 1.0), 0.8, CARTON), ('pallet_b', 'box', (10.2, 1.3), (1.0, 1.0), 0.6, CARTON),
    ('column_a', 'cyl', (5.4, 1.1), 0.2, 3.0, STEEL), ('column_b', 'cyl', (9.3, 6.1), 0.2, 3.0, STEEL),
    ('table', 'box', (10.9, 4.2), (0.7, 1.4), 0.74, TABLE),
]
ITEMS = [((10.80, 4.42), (0.07, 0.07, 0.10), (0.85, 0.25, 0.22, 1)), ((10.83, 3.95), (0.07, 0.07, 0.10), (0.20, 0.65, 0.35, 1))]
START, STATION, DOCK = (1.0, 1.2, 0.0), (9.88, 4.2, 0.0), (1.1, 6.9, np.pi)
V_MAX, W_MAX = 0.7, 1.0
ARM_PADDING = 0.04   # m; localization (2 cm, 1 deg at ~0.9 m reach) + arm servo lag
TUCK = np.array([0.05, 1.32, 1.40, -0.20, 1.72, 0.0, 1.66, 0.0])
# arm targets in the base frame: gripper forward (x), 16 cm above the table top, above each item
REACH = [np.array([0.92, 0.22, 0.95]), np.array([0.92, -0.26, 0.95])]


def build_root(obstacles=None, items=None):
    """MJCF tree of the scene (showcase_2 reuses it with its own obstacles and items)."""
    obstacles = OBSTACLES if obstacles is None else obstacles
    items = ITEMS if items is None else items
    root = urdf_root('fetch')
    w = root.find('worldbody')
    base = next(b for b in w.iter('body') if b.get('name') == 'base_link')
    for j, (kind, axis) in enumerate([('slide', '1 0 0'), ('slide', '0 1 0'), ('hinge', '0 0 1')]):
        base.insert(j, ET.Element('joint', name=['base_x', 'base_y', 'base_yaw'][j], type=kind, axis=axis, damping='20'))
    # Collision geometry of the robot = VAMP's Fetch sphere model (resources/fetch/fetch_spherized.urdf), the model
    # the planner certifies against. MuJoCo would otherwise use convex hulls of the URDF meshes, which over-fill
    # non-convex links (gripper, forearm) and report contacts that the real geometry does not have. With matched
    # geometry a contact can only come from execution error (tracking, localization), which is what we evaluate.
    for g in root.iter('geom'):
        if g.get('group') == '3':
            g.set('contype', '0'); g.set('conaffinity', '0')
    bodies = {b.get('name'): b for b in w.iter('body')}
    sph = ET.parse(ROOT / 'third_party/vamp/resources/fetch/fetch_spherized.urdf')
    for k, link in enumerate(sph.getroot().iter('link')):
        for j, c in enumerate(link.iter('collision')):
            sphere = c.find('geometry/sphere')
            if sphere is None or link.get('name') not in bodies:
                continue
            o = c.find('origin'); xyz = o.get('xyz') if o is not None else '0 0 0'
            ET.SubElement(bodies[link.get('name')], 'geom', name=f'vamp_{link.get("name")}_{j}', type='sphere',
                          size=sphere.get('radius'), pos=xyz, contype='2', conaffinity='1', group='3', mass='0',
                          rgba='.2 .5 .9 .3')
    ET.SubElement(base, 'site', name='base_imu', pos='0 0 0.3', size='0.01', rgba='0 0 0 0', group='5')
    sensor = root.find('sensor') if root.find('sensor') is not None else ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'gyro', name='base_gyro', site='base_imu')
    for b in base.iter('body'):
        if b.get('name') != 'base_link':
            b.set('gravcomp', '1')
        name = b.get('name', '')
        for g in b.findall('geom'):
            if g.get('group') == '2' and ('base' in name or 'torso_fixed' in name or 'bellows' in name):
                g.set('rgba', '0.22 0.24 0.27 1')
            elif g.get('group') == '2' and ('shoulder_lift' in name or 'elbow' in name or 'wrist_flex' in name):
                g.set('rgba', '0.20 0.42 0.70 1')
    for jn in root.iter('joint'):
        if jn.get('name') in vamp.fetch.joint_names():
            jn.set('damping', '8' if jn.get('name') != 'torso_lift_joint' else '200'); jn.set('armature', '0.05')
    asset = root.find('asset')
    ET.SubElement(asset, 'texture', name='wh_floor', type='2d', builtin='checker', rgb1='.50 .51 .52', rgb2='.46 .47 .48',
                  mark='edge', markrgb='.40 .41 .42', width='512', height='512')
    ET.SubElement(asset, 'material', name='wh_floor', texture='wh_floor', texrepeat='1 1', texuniform='true', reflectance='.06')
    ET.SubElement(asset, 'texture', name='wh_sky', type='skybox', builtin='gradient', rgb1='.60 .68 .78', rgb2='.93 .95 .97',
                  width='512', height='3072')
    ET.SubElement(w, 'geom', name='floor', type='plane', size='10 8 0.05', pos='6 4 0', material='wh_floor', contype='0', conaffinity='0')
    for line_y in (1.95, 6.05):                                   # painted aisle lines (visual)
        ET.SubElement(w, 'geom', type='box', size='5.5 0.03 0.001', pos=f'5.5 {line_y} 0.002', rgba='.95 .78 .15 1', contype='0', conaffinity='0')
    for name, shape, (x, y), size, h, rgba in obstacles:
        attrs = dict(name=f'scene_{name}', contype='1', conaffinity='2', rgba=' '.join(map(str, rgba)))
        if shape == 'cyl':
            ET.SubElement(w, 'geom', type='cylinder', size=f'{size} {h / 2}', pos=f'{x} {y} {h / 2}', **attrs)
        else:
            ET.SubElement(w, 'geom', type='box', size=f'{size[0] / 2} {size[1] / 2} {h / 2}', pos=f'{x} {y} {h / 2}', **attrs)
        if name.startswith('rack'):          # open shelving drawn inside the (invisible) collision box
            w[-1].set('rgba', '0 0 0 0')
            sx, sy = size; rng = np.random.default_rng(zlib.crc32(name.encode()))
            vis = dict(contype='0', conaffinity='0')
            for cx in (x - sx / 2 + 0.03, x + sx / 2 - 0.03):
                for cy in (y - sy / 2 + 0.03, y + sy / 2 - 0.03):
                    ET.SubElement(w, 'geom', type='box', size=f'0.03 0.03 {h / 2}', pos=f'{cx} {cy} {h / 2}', rgba='.18 .34 .62 1', **vis)
            for z in (0.08, 0.68, 1.28, 1.88):
                ET.SubElement(w, 'geom', type='box', size=f'{sx / 2} {sy / 2} 0.02', pos=f'{x} {y} {z}', rgba='.92 .55 .12 1', **vis)
                if z > 1.8:
                    continue
                long_x = sx >= sy; L = sx if long_x else sy; u = -L / 2 + 0.08
                while u < L / 2 - 0.35:
                    a, b, c = rng.uniform(0.25, 0.45), min(sy, sx) * rng.uniform(0.55, 0.8), rng.uniform(0.25, 0.48)
                    px, py = (x + u + a / 2, y) if long_x else (x, y + u + a / 2)
                    shade = rng.uniform(0.85, 1.05)
                    ET.SubElement(w, 'geom', type='box', size=f'{(a if long_x else b) / 2} {(b if long_x else a) / 2} {c / 2}',
                                  pos=f'{px} {py} {z + 0.02 + c / 2}', rgba=f'{0.72 * shade} {0.56 * shade} {0.36 * shade} 1', **vis)
                    u += a + rng.uniform(0.05, 0.2)
    for k, ((x, y), (sx, sy, sz), rgba) in enumerate(items):
        ET.SubElement(w, 'geom', name=f'scene_item_{k}', type='box', size=f'{sx / 2} {sy / 2} {sz / 2}', pos=f'{x} {y} {0.74 + sz / 2}',
                      rgba=' '.join(map(str, rgba)), contype='1', conaffinity='2')
    ET.SubElement(w, 'light', name='key', directional='true', pos='4 -3 8', dir='0.25 0.45 -1', diffuse='.55 .55 .53', castshadow='true')
    ET.SubElement(w, 'camera', name='top', pos='6 4 14', xyaxes='1 0 0 0 1 0', fovy='8.6', projection='orthographic')
    vis = root.find('visual') if root.find('visual') is not None else ET.SubElement(root, 'visual')
    ET.SubElement(vis, 'global', offwidth='3840', offheight='2160')
    ET.SubElement(vis, 'quality', shadowsize='8192')
    ET.SubElement(vis, 'headlight', ambient='.30 .30 .32', diffuse='.30 .30 .30', specular='.05 .05 .05')
    act = root.find('actuator') if root.find('actuator') is not None else ET.SubElement(root, 'actuator')
    for name, kv in [('base_x', 3000), ('base_y', 3000), ('base_yaw', 800)]:
        ET.SubElement(act, 'velocity', name=f'v_{name}', joint=name, kv=str(kv), ctrlrange='-2 2', ctrllimited='true')
    for name in vamp.fetch.joint_names():
        ET.SubElement(act, 'position', name=f'p_{name}', joint=name, kp='20000' if name == 'torso_lift_joint' else '600')
    opt = root.find('option') if root.find('option') is not None else ET.SubElement(root, 'option')
    opt.set('timestep', '0.002'); opt.set('integrator', 'implicitfast')
    return root


def build_model():
    return mujoco.MjModel.from_xml_string(ET.tostring(build_root(), encoding='unicode'))


def planning_world():
    boxes = [((x, y), (2 * s, 2 * s) if shape == 'cyl' else s) for _, shape, (x, y), s, _, _ in OBSTACLES]
    world = World(BOUNDS[0], BOUNDS[1], boxes, [], RADIUS)
    return world, world      # exact boxes (VAMP cuboids) instead of the conservative sphere cover


def arm_environment(pose):
    """Scene boxes in the (estimated) base frame at `pose`, padded by ARM_PADDING to absorb localization and servo error."""
    x0, y0, th = pose; c, s = np.cos(th), np.sin(th); boxes = []
    def add(name, centre, half):
        half = [h + ARM_PADDING for h in half]
        dx, dy = centre[0] - x0, centre[1] - y0
        boxes.append(dict(name=name, position=[c * dx + s * dy, -s * dx + c * dy, centre[2]], half_extents=list(half),
                          orientation_euler_xyz=(0., 0., -th)))
    for name, shape, (x, y), size, h, _ in OBSTACLES:
        half = (size, size, h / 2) if shape == 'cyl' else (size[0] / 2, size[1] / 2, h / 2)
        add(name, (x, y, h / 2), half)
    for k, ((x, y), (sx, sy, sz), _) in enumerate(ITEMS):
        add(f'item_{k}', (x, y, 0.74 + sz / 2), (sx / 2, sy / 2, sz / 2))
    return mbm.native_environment(dict(problem='fetch', sphere=[], cylinder=[], box=boxes))


def ik(p, q0, iterations=400):
    """Damped least squares on gripper position + orientation (gripper x forward, z up), base frame."""
    q = np.array(q0, float); lo, hi = np.asarray(vamp.fetch.lower_bounds()), np.asarray(vamp.fetch.upper_bounds())
    Rd = np.eye(3)
    def err(q):
        T = np.asarray(vamp.fetch.eefk(list(q)))
        return np.r_[p - T[:3, 3], 0.5 * sum(np.cross(T[:3, k], Rd[:, k]) for k in range(3))]
    for _ in range(iterations):
        e = err(q)
        if np.linalg.norm(e) < 1e-6:
            break
        J = np.empty((6, 8)); h = 1e-6
        for j in range(8):
            dq = np.zeros(8); dq[j] = h; J[:, j] = (e - err(q + dq)) / h
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e), lo, hi)
    return q if np.linalg.norm(err(q)) < 1e-4 else None


def base_settings():
    p = json.loads((ROOT / 'config/low_dof_v10_order2.json').read_text())
    o = p['case_overrides']['unicycle_wall']['settings']
    # first solution + Alg. 4 is what the robot executes; the shown planning time is the time of that very plan
    return PlannerSettings(**(p['settings'] | o | dict(algorithm='sst_bvp', budget_s=5.0, stop_first=True)))


def run(seed, output):
    output.mkdir(parents=True, exist_ok=False)
    model = build_model(); data = mujoco.MjData(model)
    rb = R.get_robot('fetch'); mod = native.module(rb)
    arm_idx = np.array([model.joint(n).qposadr[0] for n in vamp.fetch.joint_names()])
    data.qpos[0:3] = START; data.qpos[arm_idx] = TUCK; data.ctrl[3:] = TUCK; mujoco.mj_forward(model, data)
    world, cover = planning_world(); uni = Unicycle(velocity_max=1.0, angular_velocity_max=1.5, lift_speed=0.3, reverse=False)
    plan(uni, cover, np.asarray(START), np.asarray(START) + [0.5, 0, 0], base_settings(), 0)     # warm-up, not timed
    scene_ids = {i for i in range(model.ngeom) if model.geom(i).name.startswith('scene_')}
    # non-privileged base state: AMCL/SLAM-like localization + gyro (simulation/showcase/sensors.py); arm joints from
    # encoders. Ground-truth base pose is used only to evaluate.
    from simulation.showcase.sensors import IMU, PlanarEstimator, PoseFix, encoder
    est = PlanarEstimator(PoseFix(model, data, 'base_imu', 'slam', seed=seed), IMU(model, data, 'base_gyro', seed=seed))
    for _ in range(100):
        est.update(data.time); mujoco.mj_step(model, data)
    goals_base = [np.asarray(START, float)]
    log = dict(time=[], qpos=[], phase=[], step=[], error=[], estimate=[])
    plans, t, contacts = [], 0., 0
    dt = model.opt.timestep

    def record(phase, step, err):
        log['time'].append(t); log['qpos'].append(data.qpos.copy()); log['phase'].append(phase)
        log['step'].append(step); log['error'].append(err); log['estimate'].append(est.x.copy())

    def drive(goal, step):
        nonlocal t, contacts
        start = goals_base[-1]                                     # previous commanded base goal
        rec = plan(uni, cover, start, np.asarray(goal), base_settings(), seed * 100 + step)
        if not (rec.solved and validate(rec, uni, world, base_settings())['valid']):
            raise RuntimeError(f'base plan {step} failed: {rec.reason}')
        traj = rec.trajectory; T0 = traj.duration; ts = np.linspace(0, T0, 400)
        ctrl = uni.recover(traj.jet(ts, 2))['control']
        k = max(1.0, np.abs(ctrl[:, 0]).max() / V_MAX, np.abs(ctrl[:, 1]).max() / W_MAX); T = T0 * k
        plans.append(dict(step=step, kind='base', t_start=t, duration=T, first_ms=rec.first_solution_ns / 1e6,
                          plan_ms=rec.plan_ns / 1e6, post_ms=rec.postprocess_ns / 1e6, length_m=float(traj.length(2)), path=traj.jet(ts, 1)[:, 0, :].tolist()))
        for n in range(int((T + 1.0) / dt)):
            tk = min(n * dt, T)
            r = uni.recover(traj.jet([tk / k], 2)); ref = r['state'][0]; v_r, w_r = r['control'][0] / k
            if n * dt > T:
                v_r = w_r = 0.
            pose = est.update(data.time)
            v, w, _ = kanayama(pose, ref, v_r, w_r)
            err = float(np.linalg.norm(ref[:2] - data.qpos[0:2]))         # vs ground truth (evaluation)
            th = pose[2]
            data.ctrl[0:3] = [v * np.cos(th), v * np.sin(th), w]
            mujoco.mj_step(model, data); t += dt
            contacts += any(c.geom1 in scene_ids or c.geom2 in scene_ids for c in data.contact[:data.ncon])
            if n % 10 == 0:
                record('base', step, err)
        data.ctrl[0:3] = 0
        goals_base.append(np.asarray(goal, float))

    commanded = [TUCK.copy()]

    def arm(goal_q, step):
        nonlocal t, contacts
        # start from the last commanded configuration (servo error must not push the start outside joint limits)
        base_est = est.update(data.time)
        env = arm_environment(base_est); q0 = commanded[-1]
        d = native.plan(rb, env, q0, [goal_q], native.settings(range=1.0, velocity_sample_scale=0.0, dynamic_domain=True,
                                                                  prefix_steering=False), seed=seed * 100 + step)
        if not d['solved']:
            raise RuntimeError(f'arm plan {step} failed')
        commanded.append(np.asarray(goal_q, float))
        traj = d['trajectory']; T = traj.duration(); ts = np.linspace(0, T, 200)
        ee = []
        for q in traj.evaluate(ts)[0]:
            Tb = np.asarray(vamp.fetch.eefk(list(q))); x0, y0, th = base_est
            p = Tb[:3, 3]; ee.append([float(x0 + np.cos(th) * p[0] - np.sin(th) * p[1]), float(y0 + np.sin(th) * p[0] + np.cos(th) * p[1]), float(p[2])])
        plans.append(dict(step=step, kind='arm', t_start=t, duration=T, plan_us=d['plan_ns'] / 1e3, shortcut_us=d['shortcut_ns'] / 1e3,
                          ee_path=ee))
        for n in range(int((T + 0.6) / dt)):
            q = traj.evaluate([min(n * dt, T)])[0][0]
            data.ctrl[3:] = q
            est.update(data.time)                                    # localization keeps running during arm motions
            mujoco.mj_step(model, data); t += dt
            contacts += any(c.geom1 in scene_ids or c.geom2 in scene_ids for c in data.contact[:data.ncon])
            if n % 10 == 0:
                record('arm', step, float(np.linalg.norm(encoder(data.qpos[arm_idx]) - q)))

    drive(STATION, 0)
    env = arm_environment(est.update(data.time)); goals = []
    for p in REACH:                    # IK seeds in a fixed order: previous goal, then the tucked pose
        for seed_q in ([goals[-1]] if goals else []) + [TUCK]:
            g = ik(p, seed_q)
            if g is not None and mod.validate(env, g.tolist()):
                goals.append(g); break
        else:
            raise RuntimeError(f'no collision-free IK for {p}')
    arm(goals[0], 1); arm(goals[1], 2); arm(TUCK, 3)
    drive(DOCK, 4)
    arrays = {k: np.asarray(v) for k, v in log.items()}
    np.savez_compressed(output / 'execution.npz', **arrays)
    summary = dict(robot='Fetch', seed=seed, steps=len(plans), contact_steps=int(contacts),
                   base_first_ms=[p['first_ms'] for p in plans if p['kind'] == 'base'],
                   base_plan_ms=[p['plan_ms'] for p in plans if p['kind'] == 'base'],
                   base_post_ms=[p['post_ms'] for p in plans if p['kind'] == 'base'],
                   arm_shortcut_us=[p['shortcut_us'] for p in plans if p['kind'] == 'arm'],
                   arm_plan_us=[p['plan_us'] for p in plans if p['kind'] == 'arm'],
                   max_base_error_m=float(arrays['error'][arrays['phase'] == 'base'].max()),
                   planner='FLASK: base BVP-SST* (unicycle), arm RRTConnect + Alg. 4 (native, VAMP Fetch)')
    (output / 'plans.json').write_text(json.dumps(plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
