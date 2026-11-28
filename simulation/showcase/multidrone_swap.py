"""Showcase 2, multi-agent aerial: three Skydio X2 quadrotors (paper model, full order r = 4) swap sides of a hall
through the two windows of a dividing wall. Red and green fly head-on through the same side, so they must share a
window in time.

Prioritized space-time planning (revised prioritized planning, Cap et al. 2015): drones are planned in priority order
with FLASK-RRT on a single forward tree (v2 profile of RRTConnect; every node has an absolute time). Drone k must keep clear of
  - the trajectories of drones 1..k-1 on the common clock, and of where they then hover (exact polynomial clearance,
    PlannerSettings.moving_obstacles, src/flask/flat_native.hh), and
  - the start positions of drones k+1..n (they wait there until they leave).
The declared retry rule: if a drone finds no plan within its budget, the next priority order of PRIORITY_ORDERS is
tried from scratch (at most three orders). Each returned trajectory is interval-certified (state and control limits),
the pairwise clearance is re-checked independently at 1 kHz, and every drone is flown in MuJoCo at the same time with
its own geometric controller on noisy motion-capture + gyro estimates.

    python -m simulation.showcase.multidrone_swap --seed 1 --output showcase/showcase_2/multidrone/seed_1
"""
import argparse
import json
import time
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from flask_repro.flat_models import Quad3D
from flask_repro.flat_planner import World
from flask_repro.interval_validation import audit_segment
from flask_repro.planning import hold_piece, moving_pieces, plan, space_time_clearance, validate
from flask_repro.profiles import load_profile
from simulation.controllers import quadrotor_tracking_torque
from simulation.physics import desired_attitude
from simulation.robot_assets import load_robot

BOUNDS = ([0., 0., 0.], [10., 6., 4.])
RADIUS = 0.47            # Skydio X2 rotor discs reach 0.372 m, + 10 cm tracking margin (as drone_hall)
MARGIN = 0.10            # extra drone-drone clearance on top of 2 x RADIUS
WALL_X, WINDOWS, WIN = 5.0, [(1.8, 1.7), (4.2, 2.5)], 1.5      # window centres (y, z), side length
PILLARS = [((2.6, 3.0), 0.22), ((7.4, 3.0), 0.22), ((7.6, 0.9), 0.18)]
# goals are distinct from every start (a lower-priority drone waits at its start until it leaves); red and green fly
# in opposite directions through the same side of the hall, blue crosses the upper window
DRONES = dict(red=((1.0, 1.0, 1.4), (9.0, 3.2, 1.8), (0.88, 0.20, 0.18, 1)),
              green=((9.0, 1.0, 1.6), (1.0, 3.2, 2.0), (0.20, 0.68, 0.30, 1)),
              blue=((1.0, 5.0, 2.4), (9.0, 5.0, 2.6), (0.20, 0.42, 0.90, 1)))
PRIORITY_ORDERS = [('red', 'green', 'blue'), ('green', 'blue', 'red'), ('blue', 'red', 'green')]
WALL, FRAME, PILLAR = (0.70, 0.82, 0.92, 0.22), (0.12, 0.62, 0.86, 1), (0.42, 0.44, 0.48, 1)
CAMERA = ([5.0, 3.0, 1.6], 9.5, -60, -28)


def obstacle_boxes():
    t, (y0w, z0w), (y1w, z1w) = 0.2, WINDOWS[0], WINDOWS[1]
    H, Y = BOUNDS[1][2], BOUNDS[1][1]
    h = WIN / 2
    boxes = []
    # wall split in y bands around the windows; each window band is closed above and below the window
    cuts = sorted([0., y0w - h, y0w + h, y1w - h, y1w + h, Y])
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b - a <= 1e-9:
            continue
        mid = (a + b) / 2
        win = next(((y, z) for y, z in WINDOWS if abs(y - mid) < h - 1e-9), None)
        if win is None:
            boxes.append(((WALL_X, mid, H / 2), (t, b - a, H)))
        else:
            z = win[1]
            boxes.append(((WALL_X, mid, (z - h) / 2), (t, b - a, z - h)))
            boxes.append(((WALL_X, mid, (z + h + H) / 2), (t, b - a, H - z - h)))
    boxes += [((x, y, H / 2), (2 * r, 2 * r, H)) for (x, y), r in PILLARS]
    return boxes


def build_model():
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.001
    v = spec.visual
    v.global_.offwidth, v.global_.offheight = 3840, 2160; v.quality.shadowsize = 8192
    v.headlight.ambient = [0.30, 0.30, 0.32]; v.headlight.diffuse = [0.35, 0.35, 0.35]; v.headlight.specular = [0.05] * 3
    spec.add_texture(name='floor', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.36, 0.38, 0.42], rgb2=[0.32, 0.34, 0.38], mark=mujoco.mjtMark.mjMARK_EDGE,
                     markrgb=[0.26, 0.28, 0.31], width=512, height=512)
    spec.add_material(name='floor', textures=['', 'floor'], texrepeat=[1, 1], texuniform=True, reflectance=0.08)
    spec.add_texture(name='sky', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.22, 0.26, 0.32], rgb2=[0.06, 0.07, 0.09], width=512, height=3072)
    w = spec.worldbody
    w.add_geom(name='ground', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[9, 7, 0.05], pos=[5, 3, 0], material='floor')
    nwall = len(obstacle_boxes()) - len(PILLARS)
    for k, (c, s) in enumerate(obstacle_boxes()):
        if k >= nwall:
            (x, y), r = PILLARS[k - nwall]
            w.add_geom(name=f'scene_pillar_{k}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[r, BOUNDS[1][2] / 2, 0],
                       pos=[x, y, BOUNDS[1][2] / 2], rgba=PILLAR)
        else:
            w.add_geom(name=f'scene_wall_{k}', type=mujoco.mjtGeom.mjGEOM_BOX, size=np.asarray(s) / 2, pos=c, rgba=WALL)
    h = WIN / 2
    for (y, z) in WINDOWS:                       # visual window frames on both wall faces (no contact)
        for dx in (-0.12, 0.12):
            for c, sz in [((WALL_X + dx, y, z - h - 0.04), (0.02, h + 0.08, 0.04)), ((WALL_X + dx, y, z + h + 0.04), (0.02, h + 0.08, 0.04)),
                          ((WALL_X + dx, y - h - 0.04, z), (0.02, 0.04, h + 0.08)), ((WALL_X + dx, y + h + 0.04, z), (0.02, 0.04, h + 0.08))]:
                w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=sz, pos=c, rgba=FRAME, contype=0, conaffinity=0, group=1)
    for name, (_, g, c) in DRONES.items():
        w.add_site(name=f'goal_{name}', pos=list(g), size=[0.13, 0, 0], rgba=list(c[:3]) + [0.45])
    for j, (x, y) in enumerate([(2.5, 1.5), (2.5, 4.5), (7.5, 1.5), (7.5, 4.5)]):
        w.add_light(name=f'lamp_{j}', pos=[x, y, 3.9], dir=[0, 0, -1], diffuse=[0.28, 0.28, 0.28], castshadow=j == 1,
                    cutoff=70, exponent=2)
    w.add_light(name='key', type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, pos=[5, -4, 9], dir=[0.1, 0.5, -1],
                diffuse=[0.45, 0.45, 0.45], castshadow=True)
    w.add_camera(name='top', pos=[5, 3, 13.5], xyaxes=[1, 0, 0, 0, 1, 0], fovy=50)
    root, _ = load_robot('drone')
    for name, ((x, y, z), _, c) in DRONES.items():
        sub = mujoco.MjSpec.from_string(ET.tostring(root, encoding='unicode'))
        body = next(b for b in sub.bodies if b.name == 'airframe')
        body.add_site(name='thrust_frame', size=[0.005, 0, 0], rgba=[0, 0, 0, 0], group=5)
        body.add_geom(name='tint', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.06, 0, 0], pos=[0, 0, 0.05], rgba=list(c),
                      contype=0, conaffinity=0, group=1)
        frame = spec.worldbody.add_frame(pos=[x, y, z])
        frame.attach_body(body, f'{name}/', '')
        spec.add_sensor(name=f'{name}/imu_gyro', type=mujoco.mjtSensor.mjSENS_GYRO, objtype=mujoco.mjtObj.mjOBJ_SITE,
                        objname=f'{name}/thrust_frame')
        for an, gear, lim in [('thrust', [0, 0, 1, 0, 0, 0], [0, 14.715]), ('roll', [0, 0, 0, 1, 0, 0], [-2, 2]),
                              ('pitch', [0, 0, 0, 0, 1, 0], [-2, 2]), ('yaw', [0, 0, 0, 0, 0, 1], [-2, 2])]:
            a = spec.add_actuator(name=f'{name}/{an}', target=f'{name}/thrust_frame', trntype=mujoco.mjtTrn.mjTRN_SITE)
            a.gear = gear; a.ctrlrange = lim; a.ctrllimited = True
    return spec.compile()


def hover(p, yaw=0.):
    return np.r_[p, Rotation.from_euler('z', yaw).as_quat(), np.zeros(6)]


def settings():
    # v2 profile (v2d) of RRTConnect for the full quadrotor model (r = 4, L7 shortcut rounds), run as a single forward
    # tree (rrt_forward): RRTConnect's goal tree has no absolute time, which space-time planning needs
    s = load_profile('quad3d_window', 'rrt_connect').settings
    # rho = 100 (time weight of the LQMT cost, Eq. 20) for brisk flight, as in drone_hall
    return replace(s, algorithm='rrt_forward', budget_s=10.0, goal_tolerance=0.25, stop_first=True, moving_margin=MARGIN, rho=100.)


def plan_team(order, seed):
    quad, world, cfg = Quad3D(), World(BOUNDS[0], BOUNDS[1], obstacle_boxes(), [], RADIUS), settings()
    records, pieces = {}, []
    for k, name in enumerate(order):
        later = tuple(hold_piece(np.array(DRONES[o][0]), RADIUS) for o in order[k + 1:])   # they wait at their starts
        c = replace(cfg, moving_obstacles=tuple(pieces) + later)
        t0 = time.perf_counter()
        r = plan(quad, world, hover(np.array(DRONES[name][0])), hover(np.array(DRONES[name][1])), c, seed * 100 + k)
        wall = (time.perf_counter() - t0) * 1e3
        if not r.solved:
            return None, dict(failed=name, reason=r.reason)
        records[name] = (r, wall, k)
        pieces += list(moving_pieces(r.trajectory, RADIUS))
    return records, None


def run(seed, output):
    output.mkdir(parents=True, exist_ok=False)
    quad = Quad3D(); world = World(BOUNDS[0], BOUNDS[1], obstacle_boxes(), [], RADIUS)
    # warm-up (not timed)
    plan(quad, world, hover(np.array([1., 1., 1.])), hover(np.array([2., 1., 1.])), replace(settings(), budget_s=1.), 0)
    attempts = []
    for order in PRIORITY_ORDERS:
        records, fail = plan_team(order, seed)
        attempts.append(dict(order=order, failed=None if fail is None else fail))
        if records is not None:
            break
    if records is None:
        raise RuntimeError(f'no priority order solved: {attempts}')
    names = list(order)
    plans = []
    for name in names:
        r, wall, k = records[name]
        chk = validate(r, quad, world, settings())
        interval = [audit_segment(e.coefficients, e.duration, 'quad3d')['status'] for e in r.trajectory.edges]
        if not chk['valid'] or any(s != 'clear' for s in interval):
            raise RuntimeError(f'{name}: not certified {chk.get("errors")} {interval}')
        T = r.trajectory.duration; ts = np.linspace(0, T, 400)
        plans.append(dict(drone=name, priority=k + 1, duration=T, plan_ms=r.plan_ns / 1e6, post_ms=r.postprocess_ns / 1e6,
                          wall_ms=wall, iterations=r.iterations, edges=len(r.trajectory.edges),
                          length_m=float(r.trajectory.length(3)), moving_rejections=int(r.rejections['moving']),
                          path=r.trajectory.jet(ts, 0)[:, 0, :3].tolist(), interval=interval))
    trajs = [records[n][0].trajectory for n in names]
    gap, where = space_time_clearance(trajs, [RADIUS] * len(trajs))         # independent 1 kHz check
    # ---- fly all drones at once ----
    model = build_model(); data = mujoco.MjData(model)
    from simulation.showcase.sensors import IMU, PoseFix, RigidEstimator
    J = np.diag([.1, .1, .2])
    ids = {}
    for n in names:
        jq = model.joint(f'{n}/aerial_free').qposadr[0]; jv = model.joint(f'{n}/aerial_free').dofadr[0]
        ids[n] = (jq, jv, [model.actuator(f'{n}/{a}').id for a in ('thrust', 'roll', 'pitch', 'yaw')])
        data.qpos[jq:jq + 3] = DRONES[n][0]; data.qpos[jq + 3:jq + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    est = {n: RigidEstimator(PoseFix(model, data, f'{n}/thrust_frame', 'mocap', seed=seed * 10 + i),
                             IMU(model, data, f'{n}/imu_gyro', seed=seed * 10 + i)) for i, n in enumerate(names)}
    for _ in range(50):                                   # hover while the estimators initialize
        for n in names:
            est[n].update(data.time); data.ctrl[ids[n][2]] = [9.81, 0, 0, 0]
        mujoco.mj_step(model, data)
    body = {n: {i for i in range(model.ngeom) if model.body(model.geom_bodyid[i]).name.startswith(f'{n}/')} for n in names}
    scene_ids = {i for i in range(model.ngeom) if model.geom(i).name.startswith('scene_')}
    log = dict(time=[], qpos=[], ref=[], error=[]); contacts = 0; t = 0.
    T_end = max(tr.duration for tr in trajs) + 1.5
    while t < T_end:
        refs, errs = [], []
        for n, tr in zip(names, trajs):
            jq, jv, act = ids[n]
            tk = min(t, tr.duration)                       # the trajectory ends at rest on the goal: hover there
            jet = tr.jet([tk], 4)[0]; des = quad.recover(jet[None])
            target = des['position'][0]; vel = des['velocity'][0]
            p, v, R, omega = est[n].update(data.time)
            acc = jet[2, :3] + 4 * (target - p) + 3 * (vel - v) + [0, 0, 9.81]
            Rd = desired_attitude(acc, jet[0, 3])
            sk = Rd.T @ R - R.T @ Rd; att = np.array([sk[2, 1], sk[0, 2], sk[1, 0]]) / 2
            tau = quadrotor_tracking_torque(R, des['rotation'][0], omega, des['omega'][0], des['omega_dot'][0], J, att)
            if np.linalg.norm(tau) > 2:
                tau *= 2 / np.linalg.norm(tau)
            data.ctrl[act] = np.r_[float(np.clip(acc @ R[:, 2], 0, 14.715)), tau]
            refs.append(target); errs.append(float(np.linalg.norm(target - data.qpos[jq:jq + 3])))
        mujoco.mj_step(model, data); t += model.opt.timestep
        for c in data.contact[:data.ncon]:
            a, b = c.geom1, c.geom2
            owner = {n for n in names if a in body[n] or b in body[n]}
            if (owner and (a in scene_ids or b in scene_ids)) or len(owner) == 2:     # drone-scene or drone-drone
                contacts += 1
        if int(round(t * 1000)) % 20 == 0:
            log['time'].append(t); log['qpos'].append(data.qpos.copy()); log['ref'].append(np.array(refs)); log['error'].append(errs)
    arrays = {k: np.asarray(v) for k, v in log.items()}
    np.savez_compressed(output / 'execution.npz', **arrays, names=np.array(names))
    P = np.stack([arrays['qpos'][:, ids[n][0]:ids[n][0] + 3] for n in names], 1)
    D = [np.linalg.norm(P[:, i] - P[:, j], axis=1) for i in range(len(names)) for j in range(i + 1, len(names))]
    summary = dict(robots='3 x Skydio X2 (paper quadrotor model, r = 4)', task='cross the hall through two windows, head-on traffic', seed=seed,
                   priority_order=names, priority_attempts=attempts,
                   plan_ms={p['drone']: p['plan_ms'] for p in plans}, post_ms={p['drone']: p['post_ms'] for p in plans},
                   total_ms=sum(p['plan_ms'] + p['post_ms'] for p in plans),
                   planned_min_surface_gap_m=gap, planned_min_gap_where=where,
                   flown_min_centre_distance_m=float(min(d.min() for d in D)), contact_steps=int(contacts),
                   max_error_m=float(arrays['error'].max()), mean_error_m=float(arrays['error'].mean()),
                   interval_certified=all(s == 'clear' for p in plans for s in p['interval']),
                   planner='FLASK-RRT forward tree (v2 profile, Quad3D r = 4), prioritized space-time planning with exact '
                           'polynomial clearance to higher-priority drones and lower-priority start positions',
                   sensing='per drone: motion capture 100 Hz / 2 mm / 0.3 deg / 10 ms + gyro; alpha-beta filter')
    (output / 'plans.json').write_text(json.dumps(plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items()}))
    return summary


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
