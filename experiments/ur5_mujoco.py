"""UR5 pick-and-place tracking in MuJoCo with rigid-body dynamics (paper Sec. VII-C1, Figs. 1, 7, 8).

Scientific simulation, NOT the physical UR5 experiment and NOT a showcase rendering:
  * plant: UR5 (pinned VAMP URDF inertias, Robotiq 85 fixed) in MuJoCo, 1 ms physics step;
    joint servo  tau = M(q)(a_c + Kv(v_c - qdot) + Kq(q_c - q)) + bias(q, qdot), saturated at the UR5
    joint torque limits (150, 150, 150, 28, 28, 28) N m (stand-in for the robot's internal servo);
  * command: identical for both methods, as in the paper's baseline pipeline: every 2 ms Ruckig
    (velocity interface, jerk-limited) turns u = v_d + Kp (q_d - q) into the servo command (q_c, v_c, a_c);
  * references: FLASK-RRTConnect trajectory vs the VAMP geometric path sampled with constant velocity over
    the same per-segment durations (paper: "duration similar to ours");
  * scene: table, basket (static), granola box and block stack as FREE rigid bodies, so collisions can
    push/topple them (Fig. 1b). Collision = any robot/scene contact; objects displaced > 1 cm reported.

Pre-declared conditions (fixed before running, never tuned on outcomes): narrow-passage scene 're',
both planners plan on the same scene inflated only by the certified-checking delta, controller gains
below, held-out seeds 2001+.

Usage: python -m experiments.ur5_mujoco --trial_count 20 --output results/ur5_sim_v7/e1
"""
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import fire
import mujoco
import numpy as np
import ruckig
import vamp

ROOT = Path(os.environ.get('FLASK_REPRO_ROOT', Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'experiments'))
from flask_repro import mbm, native, robot as R
from flask_repro.provenance import save_manifest
from simulation.robot_assets import load_robot
import ur5_scene

DT_CONTROL, DT_PHYSICS = .002, .001
PAUSE, TAIL = .5, 1.
KV, KQ, KP, J_MAX = 60., 900., 5., 50.
TORQUE_LIMIT = np.array([150., 150., 150., 28., 28., 28.])
DYNAMIC_OBJECTS = ('granola',) + tuple(f'block_{k}' for k in range(4))
CERTIFY_DELTA = .001
FLASK_SETTINGS = dict(prefix_steering=False, velocity_sample_scale=0., range=2., neighbor_mode='A',
                      max_ns=int(1e9), max_iterations=int(1e7), max_samples=100000)


def vamp_spheres():
    """(link, centre, radius) of the VAMP UR5 collision model (resources/ur5/ur5_spherized.urdf)."""
    urdf = ET.parse(mbm.VAMP_ROOT / 'resources/ur5/ur5_spherized.urdf').getroot()
    out = []
    for link in urdf.iter('link'):
        for c in link.iter('collision'):
            sphere = c.find('geometry/sphere')
            if sphere is None:
                continue
            origin = c.find('origin')
            xyz = [float(x) for x in (origin.get('xyz') if origin is not None else '0 0 0').split()]
            out.append((link.get('name'), xyz, float(sphere.get('radius'))))
    return out


def build_model(scene, extra=None, collision_geometry='mesh'):
    """MuJoCo model of the UR5 plus the scene; `extra(root)` may add bodies before compilation.

    collision_geometry 'mesh': URDF collision meshes (v7). 'spheres' (v9): exactly the planner's VAMP sphere
    model, so a contact can only come from tracking deviation, never from a planner/plant geometry mismatch."""
    root, _ = load_robot('ur5')
    world = root.find('worldbody')
    for geom in world.iter('geom'):  # robot collision geoms collide with the scene only
        if geom.get('group') == '3' and collision_geometry == 'mesh':
            geom.set('contype', '2'); geom.set('conaffinity', '1')
        else:
            geom.set('contype', '0'); geom.set('conaffinity', '0')
    if collision_geometry == 'spheres':
        bodies = {b.get('name'): b for b in world.iter('body')}
        for k, (link, xyz, radius) in enumerate(vamp_spheres()):
            ET.SubElement(bodies[link], 'geom', name=f'vamp_sphere_{k}', type='sphere', size=str(radius),
                          pos=' '.join(map(str, xyz)), contype='2', conaffinity='1', group='3', mass='0', rgba='.2 .5 .9 .3')
    elif collision_geometry != 'mesh':
        raise ValueError(collision_geometry)
    for box in scene['box']:
        attrs = dict(name=f"scene_{box['name']}", type='box', size=' '.join(map(str, box['half_extents'])),
                     contype='1', conaffinity='2', rgba='.6 .5 .4 1', friction='0.8 0.01 0.0001')
        if box['name'] in DYNAMIC_OBJECTS:
            body = ET.SubElement(world, 'body', name=f"object_{box['name']}", pos=' '.join(map(str, box['position'])))
            ET.SubElement(body, 'freejoint', name=f"free_{box['name']}")
            g = ET.SubElement(body, 'geom', **attrs)
            g.set('mass', '0.15' if box['name'] == 'granola' else '0.1')
            g.set('contype', '3'); g.set('conaffinity', '3')
        else:
            ET.SubElement(world, 'geom', pos=' '.join(map(str, box['position'])), **attrs)
    if extra is not None:
        extra(root)
    actuator = root.find('actuator')
    for name, limit in zip(vamp.ur5.joint_names(), TORQUE_LIMIT):
        ET.SubElement(actuator, 'motor', name=f'motor_{name}', joint=name, ctrlrange=f'{-limit} {limit}', ctrllimited='true')
    option = root.find('option') if root.find('option') is not None else ET.SubElement(root, 'option')
    option.set('timestep', str(DT_PHYSICS)); option.set('integrator', 'implicitfast')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


class FlaskReference:
    def __init__(self, segments):
        self.segments = segments
        self.starts = np.cumsum([0.] + [s.duration() + (PAUSE if k == 0 else 0.) for k, s in enumerate(segments)])

    def duration(self):
        return float(self.starts[-1])

    def evaluate(self, t):
        for k, s in enumerate(self.segments):
            if t <= self.starts[k] + s.duration() or k == len(self.segments) - 1:
                tt = min(max(t - self.starts[k], 0.), s.duration())
                q, v, a = (x[0] for x in s.evaluate([tt])[:3])
                if t - self.starts[k] > s.duration():
                    v, a = np.zeros_like(v), np.zeros_like(a)
                return q, v, a
            if t <= self.starts[k + 1]:
                return s.Q[-1], np.zeros(s.Q.shape[1]), np.zeros(s.Q.shape[1])


class GeometricReference:
    """Constant-speed sampling of a piecewise-linear path (velocity jumps at corners)."""

    def __init__(self, waypoints, durations):
        self.w, self.T = waypoints, durations
        self.starts = np.cumsum([0.] + [T + (PAUSE if k == 0 else 0.) for k, T in enumerate(durations)])
        self.L = [np.r_[0., np.cumsum(np.linalg.norm(np.diff(w, axis=0), axis=1))] for w in waypoints]

    def duration(self):
        return float(self.starts[-1])

    def evaluate(self, t):
        n = self.w[0].shape[1]
        for k, (w, T, L) in enumerate(zip(self.w, self.T, self.L)):
            if t <= self.starts[k] + T or k == len(self.w) - 1:
                tt = t - self.starts[k]
                if tt >= T:
                    return w[-1], np.zeros(n), np.zeros(n)
                s = L[-1] * max(tt, 0.) / T
                i = min(np.searchsorted(L, s, side='right') - 1, len(w) - 2)
                u = (w[i + 1] - w[i]) / (L[i + 1] - L[i])
                return w[i] + u * (s - L[i]), u * L[-1] / T, np.zeros(n)
            if t <= self.starts[k + 1]:
                return w[-1], np.zeros(n), np.zeros(n)


class Tracker:
    """Ruckig velocity command + computed-torque joint servo on a MuJoCo UR5 (one control period per step())."""

    def __init__(self, model, data, rb, q0):
        self.model, self.data, self.rb, self.n = model, data, rb, rb.n
        self.otg, self.inp, self.out = ruckig.Ruckig(self.n, DT_CONTROL), ruckig.InputParameter(self.n), ruckig.OutputParameter(self.n)
        self.inp.control_interface = ruckig.ControlInterface.Velocity
        self.inp.max_velocity, self.inp.max_acceleration = rb.v_max.tolist(), rb.a_max.tolist()
        self.inp.max_jerk = [J_MAX] * self.n
        self.inp.current_position, self.inp.current_velocity, self.inp.current_acceleration = list(q0), [0.] * self.n, [0.] * self.n
        self.M = np.zeros((model.nv, model.nv))

    def step(self, qd, vd, ad=None):
        """ad (v10, optional): reference acceleration passed to Ruckig as target acceleration (feedforward)."""
        n, data = self.n, self.data
        u = np.clip(vd + KP * (qd - data.qpos[:n]), -self.rb.v_max, self.rb.v_max)
        target_acceleration = [0.] * n if ad is None else np.clip(ad, -self.rb.a_max, self.rb.a_max).tolist()
        self.inp.target_velocity, self.inp.target_acceleration = u.tolist(), target_acceleration
        result = self.otg.update(self.inp, self.out)
        if result not in (ruckig.Result.Working, ruckig.Result.Finished):
            raise RuntimeError(f'Ruckig error {result}')
        qc, vc, ac = np.asarray(self.out.new_position), np.asarray(self.out.new_velocity), np.asarray(self.out.new_acceleration)
        self.out.pass_to_input(self.inp)
        saturated = False
        for _ in range(int(round(DT_CONTROL / DT_PHYSICS))):
            mujoco.mj_fullM(self.model, data, self.M)
            tau = self.M[:n, :n] @ (ac + KV * (vc - data.qvel[:n]) + KQ * (qc - data.qpos[:n])) + data.qfrc_bias[:n]
            saturated |= bool(np.any(np.abs(tau) > TORQUE_LIMIT))
            data.ctrl[:n] = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
            mujoco.mj_step(self.model, data)
        return saturated


def contacts(model, data):
    depth, pairs = 0., set()
    for c in data.contact[:data.ncon]:
        names = [model.geom(int(g)).name for g in (c.geom1, c.geom2)]
        if c.dist < 0 and any(nm.startswith('scene_') for nm in names) and any(not nm.startswith('scene_') for nm in names):
            depth = max(depth, -c.dist); pairs.add(tuple(sorted(names)))
    return depth, pairs


def settle(model, data, q0, n):
    data.qpos[:n] = q0
    mujoco.mj_forward(model, data)
    for _ in range(300):  # free objects come to rest on their supports
        data.qpos[:n] = q0; data.qvel[:n] = 0.
        mujoco.mj_step(model, data)


def simulate(model, reference, rb, on_tick=None, duration=None, acceleration_feedforward=False):
    data = mujoco.MjData(model)
    n = rb.n
    q0 = reference.evaluate(0.)[0]
    settle(model, data, q0, n)
    objects = {name: data.body(f'object_{name}').xpos.copy() for name in DYNAMIC_OBJECTS}
    tracker = Tracker(model, data, rb, q0)
    steps = int(np.ceil(((reference.duration() if duration is None else duration) + TAIL) / DT_CONTROL))
    log = dict(t=np.zeros(steps), q=np.zeros((steps, n)), qd=np.zeros((steps, n)), tau=np.zeros((steps, n)),
               contact=np.zeros(steps, bool), penetration=np.zeros(steps), saturated=np.zeros(steps, bool))
    pairs = set()
    for k in range(steps):
        t = k * DT_CONTROL
        if on_tick is not None:
            on_tick(t, data)
        qd, vd, ad = reference.evaluate(t)
        sat = tracker.step(qd, vd, ad if acceleration_feedforward else None)
        depth, p = contacts(model, data); pairs |= p
        log['t'][k], log['q'][k], log['qd'][k], log['tau'][k] = t, data.qpos[:n], qd, data.ctrl[:n]
        log['contact'][k], log['penetration'][k], log['saturated'][k] = depth > 0, depth, sat
    displaced = {name: float(np.linalg.norm(data.body(f'object_{name}').xpos - p0)) for name, p0 in objects.items()}
    return log, sorted(pairs), displaced


def plan_flask(rb, env, a, b, seed, limits):
    d = native.plan(rb, env, a, [b], native.settings(**FLASK_SETTINGS), seed=seed, limits=limits)
    return (d['trajectory'], (d['plan_ns'] + d['shortcut_ns']) / 1e6) if d['solved'] else (None, None)


def plan_geometric(ev, a, b, seed):
    m, planner, ps, simp = vamp.configure_robot_and_planner_with_kwargs('ur5', 'rrtc')
    sampler = m.halton(); sampler.skip(1000 * seed)
    r = planner(list(a), [list(b)], ev, ps, sampler)
    if not r.solved:
        return None, None
    s = m.simplify(r.path, ev, simp, sampler)
    return np.asarray(s.path.numpy(), float), (r.nanoseconds + s.nanoseconds) / 1e6


def reference_collisions(reference, rb, environment, step=DT_CONTROL):
    """v10: collisions of the REFERENCE itself (no tracking) against the unpadded scene, sampled every control period.
    Separates contacts caused by the planned path from contacts caused by tracking overshoot."""
    times = np.arange(0., reference.duration() + 1e-12, step)
    samples = [reference.evaluate(t)[0].tolist() for t in times]
    return int(native.module(rb).count_collisions(environment, samples)[0])


def main(trial_count=20, seed0=2001, output=None, plot=True, variant='re', padding=0., pick_clearance=None, collision_geometry='mesh',
         acceleration_feedforward=False, random_tasks=False):
    """random_tasks (acceptance V6): per trial, the two goal configurations are drawn uniformly from the joint box
    (seeded by the trial seed) and kept only if collision-free in the planning scene; the start stays fixed."""
    rb = R.get_robot('ur5')
    scene = ur5_scene.create_scene(variant)
    pick_clearance = ur5_scene.PICK_CLEARANCE if pick_clearance is None else pick_clearance
    planning_scene = ur5_scene.inflate_scene(scene, padding + CERTIFY_DELTA)
    ev = vamp.problem_dict_to_vamp(planning_scene)
    planning_env = mbm.native_environment(planning_scene)
    start_q, pick_q, place_q = ur5_scene.configurations(ev, pick_clearance)
    limits = native.joint_limits(rb, 'certified', CERTIFY_DELTA)
    model = build_model(scene, collision_geometry=collision_geometry)
    scene_env = mbm.native_environment(scene)
    output = Path(output or ROOT / 'results/ur5_sim_v7/e1')
    output.mkdir(parents=True, exist_ok=False)
    save_manifest(output / 'manifest.json', dict(variant=variant, trial_count=trial_count, seed0=seed0,
        gains=dict(Kv=KV, Kq=KQ, Kp=KP, j_max=J_MAX), planner_settings=FLASK_SETTINGS, certified_delta=CERTIFY_DELTA,
        padding=padding, pick_clearance=pick_clearance, random_tasks=random_tasks, collision_geometry=collision_geometry, physics_step=DT_PHYSICS, control_period=DT_CONTROL,
        acceleration_feedforward=acceleration_feedforward, reference_collision_check='VAMP spheres, unpadded scene, every control period (v10)',
        torque_limit=TORQUE_LIMIT.tolist(), scene=scene,
        disclosure="MuJoCo rigid-body simulation of a UR5 (VAMP URDF inertias); not the authors' physical robot, camera or controller"))
    rows = []
    t_start = time.time()
    for trial in range(trial_count):
        seed = seed0 + trial
        if random_tasks:
            rng = np.random.default_rng(seed); mod = native.module(rb); goals = []
            while len(goals) < 2:
                q = rng.uniform(rb.q_lo, rb.q_hi)
                if mod.validate(planning_env, q.tolist()):
                    goals.append(q)
            pick_q, place_q = goals
        f1, mf1 = plan_flask(rb, planning_env, start_q, pick_q, seed, limits)
        f2, mf2 = plan_flask(rb, planning_env, pick_q, place_q, seed + 10007, limits)
        w1, mg1 = plan_geometric(ev, start_q, pick_q, seed)
        w2, mg2 = plan_geometric(ev, pick_q, place_q, seed + 10007)
        if None in (f1, f2) or w1 is None or w2 is None:
            rows.append(dict(trial=trial, seed=seed, planning_failed=True)); continue
        references = dict(flask=FlaskReference([f1, f2]), geometric=GeometricReference([w1, w2], [f1.duration(), f2.duration()]))
        row = dict(trial=trial, seed=seed, goals=[pick_q.tolist(), place_q.tolist()], flask_plan_ms=mf1 + mf2, geometric_plan_ms=mg1 + mg2, durations=[f1.duration(), f2.duration()])
        for name, ref in references.items():
            log, pairs, displaced = simulate(model, ref, rb, acceleration_feedforward=acceleration_feedforward)
            err = np.linalg.norm(log['q'] - log['qd'], axis=1)
            row[name] = dict(max_error=float(err.max()), mean_error=float(err.mean()),
                             reference_collision_samples=reference_collisions(ref, rb, scene_env),
                             contact_steps=int(log['contact'].sum()), max_penetration_m=float(log['penetration'].max()),
                             first_contact_s=float(log['t'][log['contact']][0]) if log['contact'].any() else None,
                             contact_pairs=[list(p) for p in pairs], objects_displaced_m=displaced,
                             toppled_or_pushed=[k for k, v in displaced.items() if v > .01],
                             torque_saturation_fraction=float(log['saturated'].mean()))
            np.savez_compressed(output / f'trial_{trial:02d}_{name}.npz', **log)
        rows.append(row)
        F, G = row['flask'], row['geometric']
        print(f"trial {trial:2d}: FLASK max err {F['max_error']:.4f} contacts {F['contact_steps']:4d} pushed {F['toppled_or_pushed']} | "
              f"geometric max err {G['max_error']:.4f} contacts {G['contact_steps']:4d} pushed {G['toppled_or_pushed']}  [{time.time() - t_start:.0f}s]", flush=True)
    (output / 'trials.json').write_text(json.dumps(rows, indent=1))
    ok = [r for r in rows if not r.get('planning_failed')]
    summary = dict(trials=len(rows), simulated=len(ok))
    for name in ('flask', 'geometric'):
        summary[name] = dict(collision_trials=sum(r[name]['contact_steps'] > 0 for r in ok),
                             reference_collision_trials=sum(r[name]['reference_collision_samples'] > 0 for r in ok),
                             tracking_only_collision_trials=sum(r[name]['contact_steps'] > 0 and r[name]['reference_collision_samples'] == 0 for r in ok),
                             object_disturbed_trials=sum(bool(r[name]['toppled_or_pushed']) for r in ok),
                             max_error_median=float(np.median([r[name]['max_error'] for r in ok])),
                             max_error_p90=float(np.percentile([r[name]['max_error'] for r in ok], 90)),
                             mean_error_median=float(np.median([r[name]['mean_error'] for r in ok])))
    summary['flask_plan_ms_median'] = float(np.median([r['flask_plan_ms'] for r in ok]))
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    if plot and ok:
        plot_figure7(output, ok)


def plot_figure7(output, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    row = max(rows, key=lambda r: (r['geometric']['contact_steps'], r['geometric']['max_error']))
    z = {name: np.load(output / f"trial_{row['trial']:02d}_{name}.npz") for name in ('flask', 'geometric')}
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.3), constrained_layout=True)
    for i, (name, title) in enumerate((('flask', '(a) Tracking FLASK trajectory'), ('geometric', '(b) Tracking a geometric path'))):
        ax[i].plot(z[name]['t'], z[name]['qd'][:, 2], '-', color='#2a78d6', lw=1.2, label='q3 (desired)')
        ax[i].plot(z[name]['t'], z[name]['q'][:, 2], '--', color='#eb6834', lw=1.2, label='q3 (actual)')
        c = z[name]['contact']
        if c.any():
            ax[i].plot(z[name]['t'][c], z[name]['q'][c, 2], '.', color='#e34948', ms=2, label='robot/scene contact')
        ax[i].set_title(title); ax[i].set_xlabel('Time (s)'); ax[i].set_ylabel('Joint angle (rad)'); ax[i].legend(frameon=False, fontsize=8)
    for name, style, color in (('flask', '-', '#2a78d6'), ('geometric', '--', '#eb6834')):
        ax[2].plot(z[name]['t'], np.linalg.norm(z[name]['q'] - z[name]['qd'], axis=1), style, color=color, lw=1.1, label=f'tracking error ({name})')
    ax[2].set_title('(c) Tracking error'); ax[2].set_xlabel('Time (s)'); ax[2].set_ylabel('||q_actual - q_desired|| (rad)')
    ax[2].legend(frameon=False, fontsize=8)
    fig.savefig(output / 'figure7_reproduction.png', dpi=150); fig.savefig(output / 'figure7_reproduction.pdf')


if __name__ == '__main__':
    fire.Fire(main)
