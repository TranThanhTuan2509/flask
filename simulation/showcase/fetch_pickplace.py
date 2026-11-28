"""Showcase 2: Fetch mobile manipulator picks a carton (5 x 5 x 18 cm) from table A and places it on table B across the warehouse.

FLASK plans every motion: the base with BVP-SST* on the unicycle flat model (as fetch_warehouse), the arm with
FLASK-RRTConnect + Alg. 4 (native, VAMP Fetch: torso + 7 joints) in the estimated base frame. While the carton is
held it is part of the arm's collision model (VAMP attachment) for the lift, tuck and place motions.

Simulation side: the Fetch fingers, fixed in the planner's URDF, are prismatic joints with position servos here. They
close on the carton by contact; a weld is switched on once the finger encoders report a stall short of fully closed
(as in simulation/showcase/arms.py). Base state from SLAM-like localization + gyro, arm state from encoders.

    python -m simulation.showcase.fetch_pickplace --seed 1 --output showcase/showcase_2/fetch/seed_1
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import vamp
from scipy.spatial.transform import Rotation

from flask_repro import mbm, native, robot as R
from flask_repro.flat_models import Unicycle
from flask_repro.flat_planner import World
from flask_repro.planning import plan, validate
from simulation.showcase import fetch_warehouse as W
from simulation.showcase.arms import held_spheres, object_T
from simulation.showcase.tracking import kanayama

TABLE_H = 0.74
HALF = (0.025, 0.025, 0.09)                         # carton 5 x 5 x 18 cm (2 cm finger clearance per side)
CARTON = (10.63, 4.25, TABLE_H + HALF[2])           # on table A, near its edge facing the robot
PLACE = (3.0, 7.34, TABLE_H + HALF[2])              # on table B (upper aisle), 14 cm in from its edge (a marker)
# fetch_warehouse's scene with its two columns moved next to the walls: the aisles stay wide enough for the 0.5 m
# footprint the base has while it carries the carton
MOVED = dict(column_a=(5.4, 0.45), column_b=(11.5, 7.5))   # columns next to the walls (showcase_2 layout)
OBSTACLES = [o if o[0] not in MOVED else (o[0], o[1], MOVED[o[0]], *o[3:]) for o in W.OBSTACLES] + [
             ('table_b', 'box', (3.0, 7.55), (1.4, 0.7), TABLE_H, W.TABLE),
                           ('crate', 'box', (3.2, 0.7), (0.6, 0.6), 0.9, W.CARTON)]      # extra clutter, lower aisle
STATION_A, STATION_B = (9.88, 4.25, 0.0), (3.0, 6.55, np.pi / 2)
GRASP_DZ = 0.03                                     # pinch 3 cm above the carton centre (6 cm below its top)
THIN_PADDING = 0.005
DROP = 0.025                                        # release height of the carton bottom above table B
GRASP_DX = 0.018                                    # carton centre 1.8 cm beyond the pinch: the palm stays ~1 cm clear
CARRY_POINT = np.array([0.34, -0.15, 0.85])        # gripper while driving with the carton (base frame)
CARRY_RADIUS = 0.45                                # base footprint while carrying: arm + carton reach 0.425 m
FINGER_OPEN, FINGER_STROKE = 0.0, 0.05
DEBUG = False


def build_model():
    root = W.build_root(OBSTACLES, [])
    w = root.find('worldbody')
    bodies = {b.get('name'): b for b in w.iter('body')}
    act = root.find('actuator')
    for side, sign in (('r', -1), ('l', 1)):          # r finger sits at +y and closes along -y; l the opposite
        b = bodies[f'{side}_gripper_finger_link']
        ET.SubElement(b, 'joint', name=f'{side}_finger_slide', type='slide', axis=f'0 {sign} 0',
                      range=f'0 {FINGER_STROKE}', limited='true', damping='5', armature='0.01')
        ET.SubElement(b, 'geom', name=f'{side}_finger_pad', type='box', size='0.022 0.008 0.014',
                      pos=f'0 {sign * 0.0085} 0', rgba='.15 .15 .17 1', friction='1.5 .02 .001',
                      contype='4', conaffinity='4', mass='0.01', group='1')
        ET.SubElement(act, 'position', name=f'p_{side}_finger', joint=f'{side}_finger_slide', kp='400',
                      forcerange='-40 40', forcelimited='true', ctrlrange=f'0 {FINGER_STROKE}')
    body = ET.SubElement(w, 'body', name='object_carton', pos=' '.join(map(str, CARTON)))
    ET.SubElement(body, 'freejoint', name='free_carton')
    ET.SubElement(body, 'geom', name='object_carton', type='box', size=' '.join(map(str, HALF)), rgba='.85 .25 .22 1',
                  mass='0.12', contype='5', conaffinity='5', friction='1.2 .02 .0005')
    marker = ET.SubElement(w, 'body', name='place_marker', pos=f'{PLACE[0]} {PLACE[1]} {TABLE_H + 0.001}')
    ET.SubElement(marker, 'geom', name='place_marker', type='cylinder', size='0.045 0.001', rgba='.1 .65 .55 .9',
                  contype='0', conaffinity='0', group='1')
    eq = ET.SubElement(root, 'equality')
    ET.SubElement(eq, 'weld', name='weld_carton', body1='gripper_link', body2='object_carton', active='false',
                  solref='0.01 1')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def base_settings():
    """v2 profile (v2d) of BVP-SST* for the unicycle (forward chart, as the Fetch drives forward); first solution +
    Alg. 4 / L7 is what the robot executes."""
    from dataclasses import replace
    from flask_repro.profiles import load_profile
    return replace(load_profile('unicycle_wall', 'sst_bvp').settings, budget_s=5.0, stop_first=True)


def arm_environment(pose, carry_T=None, ignore_carton=False, ignore=(), thin=()):
    """Scene boxes (and the carton unless carried or approached) in the estimated base frame, padded as fetch_warehouse;
    boxes in `thin` get a 5 mm padding (the short vertical lift / place moves next to their support)."""
    x0, y0, th = pose; c, s = np.cos(th), np.sin(th); boxes = []

    def add(name, centre, half):
        half = [h + (THIN_PADDING if name in thin else W.ARM_PADDING) for h in half]
        dx, dy = centre[0] - x0, centre[1] - y0
        boxes.append(dict(name=name, position=[c * dx + s * dy, -s * dx + c * dy, centre[2]], half_extents=list(half),
                          orientation_euler_xyz=(0., 0., -th)))
    for name, shape, (x, y), size, h, _ in OBSTACLES:
        if name in ignore:
            continue
        half = (size, size, h / 2) if shape == 'cyl' else (size[0] / 2, size[1] / 2, h / 2)
        add(name, (x, y, h / 2), half)
    if carry_T is None and not ignore_carton:
        add('carton', CARTON_BELIEF[0], HALF)
    env = mbm.native_environment(dict(problem='fetch', sphere=[], cylinder=[], box=boxes))
    if carry_T is not None:
        env.attach(held_spheres(HALF, carry_T))
    return env


CARTON_BELIEF = [np.array(CARTON)]                  # where the robot believes the carton is (known map, then placed)


def ik(p, q0, env=None, restarts=20):
    """Gripper x forward (base frame), z up; position p in the base frame. Rotation-vector error (no 180-degree flips)."""
    lo, hi = np.asarray(vamp.fetch.lower_bounds()), np.asarray(vamp.fetch.upper_bounds())
    rng = np.random.default_rng(0); best = None
    def err(q):
        T = np.asarray(vamp.fetch.eefk(list(q)))
        return np.r_[p - T[:3, 3], Rotation.from_matrix(T[:3, :3].T).as_rotvec()]
    for r in range(restarts):
        q = np.array(q0, float) + (0 if r == 0 else rng.normal(0, 0.3, 8))
        for _ in range(400):
            e = err(q)
            if np.linalg.norm(e) < 1e-7:
                break
            J = np.empty((6, 8)); h = 1e-6
            for j in range(8):
                dq = np.zeros(8); dq[j] = h; J[:, j] = (e - err(q + dq)) / h
            q = np.clip(q + np.clip(J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e), -.3, .3), lo, hi)
        if np.linalg.norm(err(q)) > 1e-4 or (env is not None and not vamp_validate(env, q)):
            continue
        if best is None or np.linalg.norm(q - q0) < np.linalg.norm(best - q0):
            best = q
    return best


def ik_line(q0, p0, p1, steps=6):
    """IK along the straight segment p0 -> p1, each solve seeded by the previous one (restarts off): the end pose is
    then on the same branch as q0, so the short move between them stays close to the segment."""
    q = q0
    for f in np.linspace(0, 1, steps + 1)[1:]:
        q = ik(p0 + f * (np.asarray(p1) - p0), q, restarts=1)
        if q is None:
            return None
    return q


def vamp_validate(env, q):
    return native.module(R.get_robot('fetch')).validate(env, list(map(float, q)))


def to_base(pose, p):
    x0, y0, th = pose; c, s = np.cos(th), np.sin(th); dx, dy = p[0] - x0, p[1] - y0
    return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])


def run(seed, output):
    output.mkdir(parents=True, exist_ok=False)
    CARTON_BELIEF[0] = np.array(CARTON)
    model = build_model(); data = mujoco.MjData(model)
    rb = R.get_robot('fetch'); mod = native.module(rb)
    arm_idx = np.array([model.joint(n).qposadr[0] for n in vamp.fetch.joint_names()])
    arm_act = np.array([model.actuator(f'p_{n}').id for n in vamp.fetch.joint_names()])
    fq = np.array([model.joint(f'{s}_finger_slide').qposadr[0] for s in ('r', 'l')])
    fa = np.array([model.actuator(f'p_{s}_finger').id for s in ('r', 'l')])
    weld = model.equality('weld_carton').id
    gripper_body, carton_body = model.body('gripper_link').id, model.body('object_carton').id
    data.qpos[0:3] = W.START; data.qpos[arm_idx] = W.TUCK; data.ctrl[arm_act] = W.TUCK; mujoco.mj_forward(model, data)
    world = World(W.BOUNDS[0], W.BOUNDS[1], [((x, y), (2 * s, 2 * s) if sh == 'cyl' else s) for _, sh, (x, y), s, _, _ in OBSTACLES], [], W.RADIUS)
    # the base planner's disk covers the tucked arm (fetch_warehouse); while carrying it covers arm + carton
    uni = Unicycle(velocity_max=1.0, angular_velocity_max=1.5, lift_speed=0.3, reverse=False)
    plan(uni, world, np.asarray(W.START), np.asarray(W.START) + [0.5, 0, 0], base_settings(), 0)   # warm-up
    scene_ids = {i for i in range(model.ngeom) if model.geom(i).name.startswith('scene_')}
    robot_ids = {i for i in range(model.ngeom) if model.geom(i).name.startswith('vamp_') or model.geom(i).name.endswith('finger_pad')}
    carton_id = model.geom('object_carton').id
    from simulation.showcase.sensors import IMU, PlanarEstimator, PoseFix, encoder
    est = PlanarEstimator(PoseFix(model, data, 'base_imu', 'slam', seed=seed), IMU(model, data, 'base_gyro', seed=seed))
    for _ in range(100):
        est.update(data.time); mujoco.mj_step(model, data)
    log = dict(time=[], qpos=[], phase=[], error=[], grip=[], weld=[], step=[], estimate=[])
    plans, t, bad = [], 0., set()
    dt = model.opt.timestep
    state = dict(base=np.asarray(W.START, float), q=W.TUCK.copy(), carrying=False, closing=False, phase='start')

    def step_sim(phase, err=0.):
        nonlocal t
        data.ctrl[fa] = FINGER_STROKE if state['closing'] else FINGER_OPEN
        mujoco.mj_step(model, data); t += dt
        for c in data.contact[:data.ncon]:
            a, b = c.geom1, c.geom2
            if c.dist >= 0:
                continue
            pair = {a, b}
            if pair & scene_ids and pair & robot_ids:
                bad.add(tuple(sorted((model.geom(a).name, model.geom(b).name))))
                if DEBUG and len(bad) < 6:
                    print('contact', phase, round(t, 3), model.geom(a).name, model.geom(b).name)
            if carton_id in pair and pair & scene_ids and state['carrying'] and phase not in ('lift', 'place', 'release'):
                bad.add(tuple(sorted((model.geom(a).name, model.geom(b).name))))
            if carton_id in pair and pair & robot_ids and not (state['carrying'] or state['closing'] or phase in ('release', 'retract')):
                bad.add(tuple(sorted((model.geom(a).name, model.geom(b).name))))
                if DEBUG and len(bad) < 4:
                    print('contact', phase, round(t, 3), model.geom(a).name, model.geom(b).name, np.round(data.xpos[carton_body], 3))
        if int(round(t / dt)) % 10 == 0:
            log['time'].append(t); log['qpos'].append(data.qpos.copy()); log['phase'].append(phase); log['error'].append(err)
            log['grip'].append(float(np.mean(data.qpos[fq]) / FINGER_STROKE)); log['weld'].append(int(data.eq_active[weld]))
            log['step'].append(max(len(plans) - 1, 0)); log['estimate'].append(est.x.copy())

    world_carry = World(world.lower, world.upper, world.boxes, [], CARRY_RADIUS)

    def drive(goal, step, label):
        start = state['base']
        wd = world_carry if state['carrying'] else world          # larger footprint while the arm holds the carton
        rec = plan(uni, wd, start, np.asarray(goal), base_settings(), seed * 100 + step)
        if not (rec.solved and validate(rec, uni, wd, base_settings())['valid']):
            raise RuntimeError(f'base plan {step} failed: {rec.reason}')
        traj = rec.trajectory; T0 = traj.duration; ts = np.linspace(0, T0, 400)
        ctrl = uni.recover(traj.jet(ts, 2))['control']
        k = max(1.0, np.abs(ctrl[:, 0]).max() / W.V_MAX, np.abs(ctrl[:, 1]).max() / W.W_MAX); T = T0 * k
        plans.append(dict(step=step, kind='base', label=label, t_start=t, duration=T, plan_ms=rec.plan_ns / 1e6,
                          post_ms=rec.postprocess_ns / 1e6, length_m=float(traj.length(2)), path=traj.jet(ts, 1)[:, 0, :].tolist()))
        for n in range(int((T + 1.0) / dt)):
            tk = min(n * dt, T)
            r = uni.recover(traj.jet([tk / k], 2)); ref = r['state'][0]; v_r, w_r = r['control'][0] / k
            if n * dt > T:
                v_r = w_r = 0.
            pose = est.update(data.time)
            v, w, _ = kanayama(pose, ref, v_r, w_r)
            th = pose[2]; data.ctrl[0:3] = [v * np.cos(th), v * np.sin(th), w]
            step_sim(label, float(np.linalg.norm(ref[:2] - data.qpos[0:2])))
        data.ctrl[0:3] = 0
        state['base'] = np.asarray(goal, float)

    def arm(goal_q, step, label, ignore_carton=False, ignore=(), thin=()):
        base_est = est.update(data.time)
        env = arm_environment(base_est, state.get('grip_T') if state['carrying'] else None, ignore_carton, ignore, thin)
        d = native.plan(rb, env, state['q'], [goal_q], native.settings(range=1.0, velocity_sample_scale=0.0, dynamic_domain=True,
                                                                         prefix_steering=False), seed=seed * 100 + step)
        if not d['solved']:
            raise RuntimeError(f'arm plan {label} failed: {d.get("reason")}')
        traj = d['trajectory']; T = traj.duration(); ts = np.linspace(0, T, 200)
        ee = []
        for q in traj.evaluate(ts)[0]:
            Tb = np.asarray(vamp.fetch.eefk(list(q))); x0, y0, th = base_est
            p = Tb[:3, 3]; ee.append([float(x0 + np.cos(th) * p[0] - np.sin(th) * p[1]), float(y0 + np.sin(th) * p[0] + np.cos(th) * p[1]), float(p[2])])
        plans.append(dict(step=step, kind='arm', label=label, t_start=t, duration=T, plan_us=d['plan_ns'] / 1e3,
                          shortcut_us=d['shortcut_ns'] / 1e3, carrying=state['carrying'], ee_path=ee))
        for n in range(int((T + 0.4) / dt)):
            q = traj.evaluate([min(n * dt, T)])[0][0]
            data.ctrl[arm_act] = q
            est.update(data.time)
            step_sim(label, float(np.linalg.norm(encoder(data.qpos[arm_idx]) - q)))
        state['q'] = np.asarray(goal_q, float)

    def grasp(label):
        state['closing'] = True; t0 = t; hist = []
        while True:
            step_sim(label); est.update(data.time)
            o = float(np.mean(data.qpos[fq]) / FINGER_STROKE); hist.append((t, o))
            recent = [x for (tt, x) in hist if tt >= t - 0.1]
            if t - t0 > 0.25 and len(recent) > 20 and max(recent) - min(recent) < 2e-3 and o < 0.93:
                break
            if t - t0 > 2.0:
                raise RuntimeError(f'grasp not detected (closure {o:.3f}); carton {np.round(data.xpos[carton_body], 3)} '
                                   f'(believed {np.round(CARTON_BELIEF[0], 3)}), gripper {np.round(data.xpos[gripper_body], 3)}, '
                                   f'base {np.round(data.qpos[:3], 3)} est {np.round(est.x, 3)}')
        r1 = Rotation.from_quat(data.xquat[gripper_body][[1, 2, 3, 0]]); r2 = Rotation.from_quat(data.xquat[carton_body][[1, 2, 3, 0]])
        model.eq_data[weld, 3:6] = r1.inv().apply(data.xpos[carton_body] - data.xpos[gripper_body])
        model.eq_data[weld, 6:10] = (r1.inv() * r2).as_quat()[[3, 0, 1, 2]]; model.eq_data[weld, 10] = 1.
        data.eq_active[weld] = 1
        # planned grasp (for the attachment): believed carton pose relative to the commanded end effector, base frame
        Tee = np.asarray(vamp.fetch.eefk(list(state['q'])))
        state['grip_T'] = np.linalg.inv(Tee) @ object_T(to_base(state['base'], CARTON_BELIEF[0]), -state['base'][2])
        state['carrying'] = True

    def release(label):
        data.eq_active[weld] = 0; state['closing'] = False; t0 = t
        while t - t0 < 0.4 or np.mean(data.qpos[fq]) / FINGER_STROKE > 0.1:
            step_sim(label); est.update(data.time)
            if t - t0 > 3.0:
                raise RuntimeError('gripper did not open')
        state['carrying'] = False; CARTON_BELIEF[0] = np.array(PLACE)

    # ---- mission ----
    drive(STATION_A, 0, 'drive to table A')
    pose = est.update(data.time)
    # the carton is located relative to the base by the head camera (detection noise 4 mm), not through the map:
    # localization error (~2 cm) is as large as the finger clearance
    from simulation.showcase.sensors import detect_relative
    seen = detect_relative(model, data, 'base_link', 'object_carton', seed=seed)
    CARTON_BELIEF[0] = np.array([pose[0] + np.cos(pose[2]) * seen[0] - np.sin(pose[2]) * seen[1],
                                 pose[1] + np.sin(pose[2]) * seen[0] + np.cos(pose[2]) * seen[1], seen[2]])
    pick = seen + [-GRASP_DX, 0, GRASP_DZ]
    q_pre = ik(pick - [0.11, 0, 0], W.TUCK, arm_environment(pose))       # clear of the (padded) carton
    q_pick = ik_line(q_pre, pick - [0.11, 0, 0], pick) if q_pre is not None else None      # straight in
    q_lift = ik_line(q_pick, pick, pick + [0, 0, 0.10]) if q_pick is not None else None   # straight up (ignores the table)
    for k, v in dict(pre=q_pre, pick=q_pick, lift=q_lift).items():
        if v is None:
            raise RuntimeError(f'IK failed: {k}')
    arm(q_pre, 1, 'reach')
    arm(q_pick, 2, 'approach', ignore_carton=True)
    grasp('grasp')
    arm(q_lift, 3, 'lift', ignore=('table',))      # the carton starts on the table; straight up (ik_line)
    e_self = arm_environment(est.update(data.time), state['grip_T'])       # carton attached: self-collision too
    q_carry = ik(CARRY_POINT, q_lift, e_self, restarts=30)
    if q_carry is None:
        q_carry = ik(CARRY_POINT, W.TUCK, e_self, restarts=30)
    if q_carry is None:
        raise RuntimeError('IK failed: carry')
    arm(q_carry, 4, 'carry pose')
    drive(STATION_B, 5, 'drive to table B')
    pose = est.update(data.time)
    # the place marker on table B is located by the head camera like the carton (detection noise 4 mm)
    mark = detect_relative(model, data, 'base_link', 'place_marker', seed=seed + 1000)
    place = mark + [-GRASP_DX, 0, HALF[2] - 0.001 + GRASP_DZ + DROP]
    q_pre_place = ik(place + [0, 0, 0.10], state['q'])
    q_place = ik_line(q_pre_place, place + [0, 0, 0.10], place) if q_pre_place is not None else None   # straight down
    CARTON_BELIEF[0] = np.array(PLACE)                 # where it will stand: the retreat pose must clear it
    q_back = ik(place - [0.12, 0, 0], q_place if q_place is not None else W.TUCK, arm_environment(pose, None), restarts=30)
    for k, v in dict(pre_place=q_pre_place, place=q_place, back=q_back).items():
        if v is None:
            raise RuntimeError(f'IK failed: {k}')
    arm(q_pre_place, 6, 'present')
    arm(q_place, 7, 'place', thin=('table_b',))
    release('release')
    arm(q_back, 8, 'retract', ignore_carton=True)
    arm(W.TUCK, 9, 'tuck')
    arrays = {k: np.asarray(v) for k, v in log.items()}
    np.savez_compressed(output / 'execution.npz', **arrays)
    final = data.body('object_carton').xpos.copy()
    err = float(np.linalg.norm(final[:2] - np.array(PLACE[:2])))
    summary = dict(robot='Fetch', task='pick a carton on table A, place it on table B', seed=seed, steps=len(plans),
                   placed=int(err < 0.03 and abs(final[2] - PLACE[2]) < 0.02), place_error_m=err,
                   upright=float(abs(data.body('object_carton').xmat.reshape(3, 3)[2, 2])),
                   contact_pairs=sorted(map(list, bad)), contacts=len(bad),
                   base_plan_ms=[p['plan_ms'] for p in plans if p['kind'] == 'base'],
                   base_post_ms=[p['post_ms'] for p in plans if p['kind'] == 'base'],
                   arm_plan_us=[p['plan_us'] for p in plans if p['kind'] == 'arm'],
                   arm_shortcut_us=[p['shortcut_us'] for p in plans if p['kind'] == 'arm'],
                   total_ms=sum(p.get('plan_ms', 0) + p.get('post_ms', 0) + (p.get('plan_us', 0) + p.get('shortcut_us', 0)) / 1e3 for p in plans),
                   max_base_error_m=float(arrays['error'][np.char.startswith(arrays['phase'].astype(str), 'drive')].max()),
                   duration_s=t,
                   planner='FLASK: base BVP-SST* (unicycle, v2 profile), arm RRTConnect + Alg. 4 (native, VAMP Fetch, carton attached)')
    (output / 'plans.json').write_text(json.dumps(plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items()}))
    return summary


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
