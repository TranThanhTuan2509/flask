"""Showcase: Unitree G1 humanoid walking a FLASK base trajectory through a courtyard.

FLASK (unicycle flat model, flat output (x, y), BVP-SST*) plans each leg online from the robot's measured pose. A
Kanayama tracking law turns the planned (x, y, theta, v, omega) into body velocity commands, and Unitree's released
walking policy (simulation/legged/g1_policy.py) executes them in MuJoCo at 500 Hz. Every number shown in the video
comes from the log written here.

    python -m simulation.showcase.g1_courtyard --seed 1 --output showcase/showcase_1/g1/seed_1
"""
import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from flask_repro.flat_models import Unicycle
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings, plan, validate
from simulation.legged.g1_policy import RLGYM, G1Walker, base_pose
from simulation.showcase.tracking import kanayama  # noqa: F401  (re-exported for older callers)

ROOT = Path(__file__).resolve().parents[2]
BOUNDS = ([0., 0.], [14., 9.])
RADIUS = 0.55                       # m: G1 upper body reaches 0.39 m from the pelvis axis (arms fixed forward) + 16 cm tracking margin
CONCRETE, WOOD, STEEL, GREEN, STONE = (0.62, 0.63, 0.65, 1), (0.55, 0.40, 0.27, 1), (0.30, 0.42, 0.58, 1), (0.30, 0.44, 0.28, 1), (0.72, 0.70, 0.66, 1)
# (name, shape, centre xy, size xy (box) or radius (cylinder), height, rgba). Cylinders are planned as their
# bounding squares (conservative); boxes exactly.
OBSTACLES = [
    ('pillar_a', 'cyl', (2.6, 2.2), 0.25, 2.2, CONCRETE), ('pillar_b', 'cyl', (6.4, 7.7), 0.25, 2.2, CONCRETE),
    ('pillar_c', 'cyl', (9.6, 2.6), 0.25, 2.2, CONCRETE), ('pillar_d', 'cyl', (12.0, 6.4), 0.25, 2.2, CONCRETE),
    ('barrel_a', 'cyl', (4.2, 3.6), 0.30, 0.9, STEEL), ('barrel_b', 'cyl', (4.85, 3.95), 0.30, 0.9, STEEL),
    ('barrel_c', 'cyl', (10.6, 5.0), 0.30, 0.9, STEEL),
    ('crate_a', 'box', (3.2, 7.35), (1.2, 0.9), 0.9, WOOD), ('crate_b', 'box', (8.5, 3.5), (1.0, 1.0), 0.8, WOOD),
    ('crate_c', 'box', (11.2, 1.4), (1.0, 0.8), 0.6, WOOD),
    ('bench_a', 'box', (6.6, 1.5), (1.6, 0.5), 0.45, (0.42, 0.30, 0.20, 1)),
    ('bench_b', 'box', (8.4, 7.6), (1.6, 0.5), 0.45, (0.42, 0.30, 0.20, 1)),
    ('wall_a', 'box', (5.6, 4.0), (0.25, 2.6), 1.0, STONE), ('wall_b', 'box', (9.0, 5.2), (2.6, 0.25), 1.0, STONE),
    ('planter_a', 'box', (1.6, 5.2), (1.4, 0.7), 0.5, GREEN), ('planter_b', 'box', (12.8, 3.8), (0.8, 1.6), 0.5, GREEN),
]
WAYPOINTS = [(0.9, 0.9, 0.0), (3.4, 6.05, 0.0), (7.0, 6.3, -1.3), (8.6, 1.0, 0.3), (12.2, 2.2, 1.4), (13.0, 8.0, 1.57)]
SLEW = np.array([0.05, 0.05, 0.08])
SLOW = 1.4   # uniform time scaling of each FLASK plan (same path; v/1.4, a/1.96) to leave command headroom   # max command change per 20 ms policy step
V_BASE, A_BASE = 0.7, 1.0           # base limits for the point model (m/s, m/s^2)
V_MAX, W_MAX = 0.6, 0.9            # executed limits, inside the range the G1 policy tracks
# FLASK plans with the paper's unicycle (|v| <= 1, |omega| <= 1.5); each plan is then uniformly time-scaled by
# k = max(1, v_peak / V_MAX, omega_peak / W_MAX), which keeps the path and divides v, omega by k.


def build_model():
    spec = mujoco.MjSpec.from_file(str(RLGYM / 'resources/robots/g1_description/scene.xml'))
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 3840, 2160
    spec.visual.quality.shadowsize = 8192
    spec.visual.headlight.ambient = [0.22, 0.22, 0.24]; spec.visual.headlight.diffuse = [0.30, 0.30, 0.30]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    w = spec.worldbody
    for g in list(w.geoms):
        if g.name == 'floor':
            spec.delete(g)
    for t in list(spec.textures):
        if t.name == 'groundplane':
            t.rgb1, t.rgb2, t.mark = [0.47, 0.48, 0.50], [0.43, 0.44, 0.46], mujoco.mjtMark.mjMARK_EDGE
            t.markrgb = [0.36, 0.37, 0.39]
        if t.type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
            t.builtin, t.rgb1, t.rgb2 = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, [0.55, 0.68, 0.84], [0.92, 0.94, 0.97]
    for m in spec.materials:
        if m.name == 'groundplane':
            m.texrepeat = [1, 1]; m.reflectance = 0.05
    w.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[16, 12, 0.05], pos=[7, 4.5, 0], material='groundplane')
    for name, shape, (x, y), size, h, rgba in OBSTACLES:
        if shape == 'cyl':
            w.add_geom(name=f'scene_{name}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[size, h / 2, 0], pos=[x, y, h / 2], rgba=rgba)
        else:
            w.add_geom(name=f'scene_{name}', type=mujoco.mjtGeom.mjGEOM_BOX, size=[size[0] / 2, size[1] / 2, h / 2], pos=[x, y, h / 2], rgba=rgba)
    for k, (x, y, sx, sy) in enumerate([(7, -0.45, 14.9, 0.3), (7, 9.45, 14.9, 0.3), (-0.45, 4.5, 0.3, 9.6), (14.45, 4.5, 0.3, 9.6)]):
        w.add_geom(name=f'border_{k}', type=mujoco.mjtGeom.mjGEOM_BOX, size=[sx / 2, sy / 2, 0.25], pos=[x, y, 0.25], rgba=STONE)
    for i, (x, y, _) in enumerate(WAYPOINTS[1:]):
        w.add_site(name=f'goal_{i}', pos=[x, y, 0.012], size=[0.32, 0.004, 0], type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   rgba=[0.10, 0.62, 0.52, 0.45])
    w.add_camera(name='top', pos=[7, 4.5, 14], quat=[1, 0, 0, 0], fovy=10.4,
                 proj=mujoco.mjtProjection.mjPROJ_ORTHOGRAPHIC)
    pelvis = next(b for b in spec.bodies if b.name == 'pelvis')
    pelvis.add_site(name='imu', pos=[0, 0, 0], size=[0.01, 0, 0], rgba=[0, 0, 0, 0], group=5)
    spec.add_sensor(name='imu_gyro', type=mujoco.mjtSensor.mjSENS_GYRO, objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')
    w.add_light(name='sun', type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, pos=[3, -2, 8], dir=[-0.3, 0.4, -1], diffuse=[0.55, 0.54, 0.50],
                castshadow=True)
    return spec.compile()


def planning_world():
    boxes = [((x, y), (2 * size, 2 * size) if shape == 'cyl' else size) for _, shape, (x, y), size, _, _ in OBSTACLES]
    world = World(BOUNDS[0], BOUNDS[1], boxes, [], RADIUS)
    return world, world.sphere_cover(0.15)


def settings(seed_budget=3.0):
    """BVP-SST* run anytime for a fixed budget (best path kept); the first-solution time is logged separately."""
    p = json.loads((ROOT / 'config/low_dof_v10_order2.json').read_text())
    o = p['case_overrides']['unicycle_wall']['settings']
    return PlannerSettings(**(p['settings'] | o | dict(algorithm='sst_bvp', budget_s=seed_budget, stop_first=False)))


def point_robot():
    """FLASK's fully actuated point model for the omnidirectional base: flat output = (x, y, z), r = 2, z pinned."""
    import dataclasses
    from flask_repro import native, robot as R
    rb = dataclasses.replace(R.get_robot('sphere'), q_lo=np.array([BOUNDS[0][0], BOUNDS[0][1], 0.79]),
                             q_hi=np.array([BOUNDS[1][0], BOUNDS[1][1], 0.81]), v_max=np.array([V_BASE, V_BASE, 0.05]),
                             a_max=np.array([A_BASE, A_BASE, 0.1]))
    native.module(rb).set_radius(RADIUS)
    return rb


def base_environment():
    """Obstacles as 2 m tall prisms (low objects are never passed over); cylinders by their bounding boxes."""
    from flask_repro import mbm
    boxes = []
    for name, shape, (x, y), size, h, _ in OBSTACLES:
        sx, sy = (2 * size, 2 * size) if shape == 'cyl' else size
        boxes.append(dict(name=name, position=[x, y, 1.0], half_extents=[sx / 2, sy / 2, 1.0], orientation_euler_xyz=(0., 0., 0.)))
    return mbm.native_environment(dict(problem='g1_courtyard', sphere=[], cylinder=[], box=boxes))


def run(seed, output):
    from flask_repro import native
    output.mkdir(parents=True, exist_ok=False)
    model = build_model(); data = mujoco.MjData(model)
    walker = G1Walker(model, data)
    # non-privileged measurements (simulation/showcase/sensors.py): the policy and the tracker never read the
    # floating-base state; ground truth below is used only to evaluate the run
    from simulation.showcase.sensors import IMU, PlanarEstimator, PoseFix, encoder
    rng = np.random.default_rng(seed + 7)
    imu = IMU(model, data, 'imu_gyro', seed=seed)
    ahrs = PoseFix(model, data, 'imu', 'ahrs', seed=seed)
    slam = PoseFix(model, data, 'imu', 'slam', seed=seed + 3)
    estimator = PlanarEstimator(slam, imu)
    def read_quat():
        f = ahrs.update(data.time)
        q = np.zeros(4); mujoco.mju_mat2Quat(q, np.ascontiguousarray(f[2]).ravel()); return q
    walker.read_gyro = imu.gyro
    walker.read_quat = read_quat
    walker.read_joints = lambda: (encoder(data.qpos[7:19]), data.qvel[6:18] + rng.normal(0, 0.01, 12))
    data.qpos[0:2] = WAYPOINTS[0][:2]; yaw = WAYPOINTS[0][2]
    data.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    mujoco.mj_forward(model, data)
    rb = point_robot(); env = base_environment(); limits = native.joint_limits(rb)
    st = native.settings(range=2.0, velocity_sample_scale=0.0, dynamic_domain=True, prefix_steering=False)
    p0 = np.array([*WAYPOINTS[0][:2], 0.8])
    native.plan(rb, env, p0, [p0 + [0.3, 0, 0]], st, seed=0, limits=limits)      # warm-up, not timed
    for _ in range(int(1.0 / model.opt.timestep)):     # settle standing; the estimator converges meanwhile
        walker.step(); estimator.update(data.time)
    log = dict(time=[], qpos=[], command=[], ref=[], track_error=[], estimate_error=[], leg=[])
    plans, contacts = [], 0
    obstacle_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'scene_{o[0]}') for o in OBSTACLES}
    t = 0.; heading = base_pose(data)[2]
    for leg, goal in enumerate(WAYPOINTS[1:]):
        # plan from the previous commanded goal (always collision-free); the tracker absorbs the small offset
        start = np.array([WAYPOINTS[leg][0], WAYPOINTS[leg][1], 0.8])
        d = native.plan(rb, env, start, [np.array([goal[0], goal[1], 0.8])], st, seed=seed * 100 + leg, limits=limits)
        if not d['solved']:
            raise RuntimeError(f'leg {leg}: plan failed')
        traj = d['trajectory']; T = traj.duration() * SLOW; ts = np.linspace(0, traj.duration(), 400)
        path = traj.evaluate(ts)[0][:, :2]
        plans.append(dict(leg=leg, t_start=t, duration=T, plan_us=d['plan_ns'] / 1e3, shortcut_us=d['shortcut_ns'] / 1e3,
                          start=start.tolist(), goal=list(goal), path=path.tolist(),
                          length_m=float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))))
        for k in range(int((T + 1.5) / model.opt.timestep)):
            tk = min(k * model.opt.timestep, T)
            if k % walker.decimation == 0:
                q, v = (x[0] for x in traj.evaluate([tk / SLOW])[:2]); v = v / SLOW
                if k * model.opt.timestep > T:
                    v = np.zeros(3)
                pose = estimator.update(data.time)                       # SLAM + gyro estimate, not ground truth
                e = q[:2] - pose[:2]
                cmd = v[:2] + 1.8 * e                                    # world-frame velocity command
                if np.linalg.norm(v[:2]) > 0.15:                         # face the direction of travel
                    heading = np.arctan2(v[1], v[0])
                dyaw = np.arctan2(np.sin(heading - pose[2]), np.cos(heading - pose[2]))
                c, s_ = np.cos(pose[2]), np.sin(pose[2])
                # stay inside the envelope the walking policy handles robustly, and slew-limit every 20 ms
                # (saturated vx = 0.9, vy = 0.6, omega = 1 together made the robot fall in 2/10 runs)
                want = np.array([np.clip(c * cmd[0] + s_ * cmd[1], -0.3, 0.7), np.clip(-s_ * cmd[0] + c * cmd[1], -0.3, 0.3),
                                 np.clip(2.0 * dyaw, -0.8, 0.8)])
                walker.command[:] = walker.command + np.clip(want - walker.command, -SLEW, SLEW)
                truth = base_pose(data)                                   # evaluation only
                ref, err = np.r_[q[:2], heading], float(np.linalg.norm(q[:2] - truth[:2]))
                est_err = float(np.linalg.norm(pose[:2] - truth[:2]))
            walker.step(); t += model.opt.timestep
            contacts += any(c.geom1 in obstacle_ids or c.geom2 in obstacle_ids for c in data.contact[:data.ncon])
            if k % 10 == 0:          # 50 Hz log
                log['time'].append(t); log['qpos'].append(data.qpos.copy()); log['command'].append(walker.command.copy())
                log['ref'].append(ref.copy()); log['track_error'].append(err); log['estimate_error'].append(est_err); log['leg'].append(leg)
        walker.command[:] = 0
    arrays = {k: np.asarray(v) for k, v in log.items()}
    np.savez_compressed(output / 'execution.npz', **arrays)
    summary = dict(robot='Unitree G1', seed=seed, legs=len(plans), contact_steps=int(contacts),
                   min_pelvis_height=float(arrays['qpos'][:, 2].min()), max_track_error_m=float(arrays['track_error'].max()),
                   mean_track_error_m=float(arrays['track_error'].mean()), fell=bool(arrays['qpos'][:, 2].min() < 0.5),
                   max_estimate_error_m=float(arrays['estimate_error'].max()),
                   sensing='IMU gyro (noise+bias), AHRS attitude, SLAM-like pose 30 Hz / 2 cm / 1 deg / 40 ms, 17-bit encoders',
                   plan_us=[p['plan_us'] for p in plans], shortcut_us=[p['shortcut_us'] for p in plans],
                   planner='FLASK-RRTConnect + Alg. 4, fully actuated point base (r = 2, native)',
                   locomotion='Unitree unitree_rl_gym G1 policy (vx, vy, yaw rate commands)')
    (output / 'plans.json').write_text(json.dumps(plans) + '\n')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))
    return summary


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--seed', type=int, default=1); a.add_argument('--output', type=Path, required=True)
    args = a.parse_args(); run(args.seed, args.output)
