"""Showcase: Skydio X2 (paper's 1 kg quadrotor model) flying FLASK trajectories through a hall of windowed walls.

FLASK plans each leg with the full quadrotor model (flat output (x, y, z, yaw), order r = 4, RRTConnect + Alg. 4).
Every returned trajectory is re-checked by the interval oracle before it is flown. A geometric SE(3) controller
(simulation/physics.py: desired_attitude, controllers.quadrotor_tracking_torque) tracks it in MuJoCo at 1 kHz with
aggregate thrust (<= 1.5 mg) and body torque (|tau| <= 2 N m) actuators.

    python -m simulation.showcase.drone_hall --seed 1 --output showcase/showcase_1/drone/seed_1
"""
import argparse
import dataclasses
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from flask_repro.flat_models import Quad3D
from flask_repro.flat_planner import World
from flask_repro.interval_validation import audit_segment
from flask_repro.planning import PlannerSettings, plan, validate
from simulation.controllers import quadrotor_tracking_torque
from simulation.physics import desired_attitude
from simulation.robot_assets import load_robot

ROOT = Path(__file__).resolve().parents[2]
BOUNDS = ([0., 0., 0.], [16., 8., 5.])
RADIUS = 0.47          # m: Skydio X2 collision geometry reaches 0.372 m (rotor discs) + 10 cm tracking margin
WALL, FRAME, PILLAR = (0.70, 0.82, 0.92, 0.22), (0.12, 0.62, 0.86, 1), (0.42, 0.44, 0.48, 1)
# partitions: (x, window centre y, window centre z, window size)
PARTITIONS = [(4.0, 2.0, 1.5, 1.8), (8.5, 6.0, 3.4, 1.8), (13.0, 3.2, 2.2, 1.8)]
PILLARS = [((6.0, 3.0), 0.25), ((6.6, 5.6), 0.25), ((10.6, 2.0), 0.25), ((11.0, 5.2), 0.25), ((2.2, 5.6), 0.25)]
WAYPOINTS = [(1.2, 4.0, 1.0), (15.0, 6.2, 1.6), (14.8, 1.4, 3.6), (1.4, 6.8, 3.8)]


def obstacle_boxes():
    """(centre, size) boxes, exactly the planning geometry (pillars as bounding boxes)."""
    boxes = []
    t = 0.2
    for x, wy, wz, s in PARTITIONS:
        y0, y1, z0, z1 = wy - s / 2, wy + s / 2, wz - s / 2, wz + s / 2
        boxes += [((x, y0 / 2, 2.5), (t, y0, 5.0)), ((x, (y1 + 8) / 2, 2.5), (t, 8 - y1, 5.0)),
                  ((x, wy, z0 / 2), (t, s, z0)), ((x, wy, (z1 + 5) / 2), (t, s, 5 - z1))]
    boxes += [((x, y, 2.5), (2 * r, 2 * r, 5.0)) for (x, y), r in PILLARS]
    return boxes


def build_model():
    root, _ = load_robot('drone')
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding='unicode'))
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
    w.add_geom(name='ground', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[12, 8, 0.05], pos=[8, 4, 0], material='floor')
    for k, (c, s) in enumerate(obstacle_boxes()):
        pillar = k >= 4 * len(PARTITIONS)
        if pillar:
            (x, y), r = PILLARS[k - 4 * len(PARTITIONS)]
            w.add_geom(name=f'scene_pillar_{k}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[r, 2.5, 0], pos=[x, y, 2.5], rgba=PILLAR)
        else:
            w.add_geom(name=f'scene_wall_{k}', type=mujoco.mjtGeom.mjGEOM_BOX, size=np.asarray(s) / 2, pos=c, rgba=WALL)
    for i, (x, wy, wz, s) in enumerate(PARTITIONS):   # visual window frames on the wall faces (no contact)
        for dx in (-0.12, 0.12):
            for c, sz in [((x + dx, wy, wz - s / 2 - 0.04), (0.02, s / 2 + 0.08, 0.04)), ((x + dx, wy, wz + s / 2 + 0.04), (0.02, s / 2 + 0.08, 0.04)),
                          ((x + dx, wy - s / 2 - 0.04, wz), (0.02, 0.04, s / 2 + 0.08)), ((x + dx, wy + s / 2 + 0.04, wz), (0.02, 0.04, s / 2 + 0.08))]:
                w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=sz, pos=c, rgba=FRAME, contype=0, conaffinity=0, group=1)
    for i, (x, y, z) in enumerate(WAYPOINTS[1:]):
        w.add_site(name=f'goal_{i}', pos=[x, y, z], size=[0.12, 0, 0], rgba=[0.15, 0.85, 0.65, 0.6])
    for j, (x, y) in enumerate([(2, 2), (6, 6), (10, 2), (14, 6)]):
        w.add_light(name=f'lamp_{j}', pos=[x, y, 4.9], dir=[0, 0, -1], diffuse=[0.30, 0.30, 0.30], castshadow=j == 1,
                    cutoff=70, exponent=2)
    w.add_light(name='key', type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, pos=[8, -4, 10], dir=[0.1, 0.5, -1],
                diffuse=[0.45, 0.45, 0.45], castshadow=True)
    body = next(b for b in spec.bodies if b.name == 'airframe')
    body.add_site(name='thrust_frame', size=[0.005, 0, 0], rgba=[0, 0, 0, 0], group=5)
    spec.add_sensor(name='imu_gyro', type=mujoco.mjtSensor.mjSENS_GYRO, objtype=mujoco.mjtObj.mjOBJ_SITE, objname='thrust_frame')
    body.add_camera(name='fpv', pos=[0.12, 0, 0.02], xyaxes=[0, -1, 0, 0, 0.2, 1], fovy=85)
    w.add_camera(name='side', pos=[8, -7.5, 5.5], xyaxes=[1, 0, 0, 0, 0.55, 0.83], fovy=55)
    for name, gear, lim in [('thrust', [0, 0, 1, 0, 0, 0], [0, 14.715]), ('roll', [0, 0, 0, 1, 0, 0], [-2, 2]),
                            ('pitch', [0, 0, 0, 0, 1, 0], [-2, 2]), ('yaw', [0, 0, 0, 0, 0, 1], [-2, 2])]:
        a = spec.add_actuator(name=name, target='thrust_frame', trntype=mujoco.mjtTrn.mjTRN_SITE)
        a.gear = gear; a.ctrlrange = lim; a.ctrllimited = True
    return spec.compile()


def settings():
    p = json.loads((ROOT / 'config/low_dof_v10_order4.json').read_text())
    o = p['case_overrides']['quad3d_block']['settings']
    # showcase: tighter goal region (0.25 m instead of Dynoplan's 0.7 m); RRTConnect as recommended in the README;
    # rho = 100 (time weight of the LQMT cost, Eq. 20) for brisker flight than the benchmark's rho = 1
    return PlannerSettings(**(p['settings'] | o | dict(algorithm='rrt_connect', budget_s=10.0, goal_tolerance=0.25, rho=100.0)))


def hover(p, yaw=0.):
    q = Rotation.from_euler('z', yaw).as_quat()
    return np.r_[p, q, 0, 0, 0, 0, 0, 0]


def run(seed, output):
    output.mkdir(parents=True, exist_ok=False)
    model = build_model(); data = mujoco.MjData(model)
    quad = Quad3D(); cfg = settings()
    world = World(BOUNDS[0], BOUNDS[1], obstacle_boxes(), [], RADIUS)
    cover = world      # exact boxes (VAMP cuboids): ~6x faster than the conservative sphere cover, same validity
    plan(quad, cover, hover(np.asarray(WAYPOINTS[0])), hover(np.asarray(WAYPOINTS[1])), cfg, 0)   # warm-up, not timed
    data.qpos[:3] = WAYPOINTS[0]; data.qpos[3:7] = [1, 0, 0, 0]; mujoco.mj_forward(model, data)
    J = np.diag([.1, .1, .2])
    # non-privileged state: motion-capture fixes + gyro, filtered (simulation/showcase/sensors.py)
    from simulation.showcase.sensors import IMU, PoseFix, RigidEstimator
    est = RigidEstimator(PoseFix(model, data, 'thrust_frame', 'mocap', seed=seed), IMU(model, data, 'imu_gyro', seed=seed))
    for _ in range(50):                                   # hover in place while the estimator initializes
        est.update(data.time); data.ctrl[:] = [9.81, 0, 0, 0]; mujoco.mj_step(model, data)
    scene_ids = {i for i in range(model.ngeom) if model.geom(i).name.startswith('scene_')}
    log = dict(time=[], qpos=[], ref=[], error=[], est_error=[], thrust=[], leg=[])
    plans, contacts, t = [], 0, 0.
    for leg, goal in enumerate(WAYPOINTS[1:]):
        start = hover(np.asarray(WAYPOINTS[leg], float))          # previous commanded goal (no simulator state)
        t0 = time.perf_counter()
        rec = plan(quad, cover, start, hover(np.asarray(goal)), cfg, seed * 100 + leg)
        wall_ms = (time.perf_counter() - t0) * 1e3
        if not rec.solved:
            raise RuntimeError(f'leg {leg}: {rec.reason}')
        chk = validate(rec, quad, world, cfg)
        interval = [audit_segment(e.coefficients, e.duration, 'quad3d')['status'] for e in rec.trajectory.edges]
        if not chk['valid'] or any(s != 'clear' for s in interval):
            raise RuntimeError(f'leg {leg}: not certified {chk.get("errors")} {interval}')
        traj = rec.trajectory; T = traj.duration
        ts = np.linspace(0, T, 500); jets = traj.jet(ts, 4)
        frames = quad.recover(jets[::25])
        plans.append(dict(leg=leg, t_start=t, duration=T, plan_ms=rec.plan_ns / 1e6, post_ms=rec.postprocess_ns / 1e6,
                          wall_ms=wall_ms, iterations=rec.iterations, edges=len(traj.edges), length_m=float(traj.length(3)),
                          path=jets[:, 0, :3].tolist(), frames_pos=frames['position'].tolist(),
                          frames_rot=frames['rotation'].tolist(), interval=interval))
        for k in range(int((T + 1.0) / model.opt.timestep)):
            tk = min(k * model.opt.timestep, T)
            jet = traj.jet([tk], 4)[0]; des = quad.recover(jet[None])
            target = des['position'][0]; vel = des['velocity'][0]
            p, v, R, omega = est.update(data.time)
            acc = jet[2, :3] + 4 * (target - p) + 3 * (vel - v) + [0, 0, 9.81]
            Rd = desired_attitude(acc, jet[0, 3])
            sk = Rd.T @ R - R.T @ Rd; att = np.array([sk[2, 1], sk[0, 2], sk[1, 0]]) / 2
            tau = quadrotor_tracking_torque(R, des['rotation'][0], omega, des['omega'][0], des['omega_dot'][0], J, att)
            if np.linalg.norm(tau) > 2:
                tau *= 2 / np.linalg.norm(tau)
            thrust = float(np.clip(acc @ R[:, 2], 0, 14.715))
            data.ctrl[:] = np.r_[thrust, tau]
            mujoco.mj_step(model, data); t += model.opt.timestep
            contacts += any(c.geom1 in scene_ids or c.geom2 in scene_ids for c in data.contact[:data.ncon])
            if k % 20 == 0:
                log['time'].append(t); log['qpos'].append(data.qpos.copy()); log['ref'].append(target.copy())
                log['error'].append(float(np.linalg.norm(target - data.qpos[:3])))         # vs ground truth (evaluation)
                log['est_error'].append(float(np.linalg.norm(p - data.qpos[:3])))
                log['thrust'].append(thrust / 9.81); log['leg'].append(leg)
    arrays = {k: np.asarray(v) for k, v in log.items()}
    np.savez_compressed(output / 'execution.npz', **arrays)
    summary = dict(robot='Skydio X2 (paper quadrotor model)', seed=seed, legs=len(plans), contact_steps=int(contacts),
                   max_error_m=float(arrays['error'].max()), mean_error_m=float(arrays['error'].mean()),
                   plan_ms=[p['plan_ms'] for p in plans], post_ms=[p['post_ms'] for p in plans],
                   max_estimate_error_m=float(arrays['est_error'].max()),
                   sensing='motion capture 100 Hz / 2 mm / 0.3 deg / 10 ms + gyro (noise, bias); alpha-beta velocity filter',
                   planner='FLASK RRTConnect + Alg. 4, Quad3D order 4; interval-certified')
    (output / 'plans.json').write_text(json.dumps(plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
