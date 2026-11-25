"""Online reactive pick-place-reset loop with a moving obstacle in MuJoCo (paper Sec. VII-C2, Figs. 9-10).

Unlike experiments/reactive_experiment.py (offline planning then kinematic replay), sensing, planning
and control run together in simulated time:
  * physics/control: MuJoCo UR5 at 1 ms with the E1 Ruckig + joint-servo tracker at 500 Hz, always
    tracking the latest committed motion schedule;
  * sensing: the tube pose is captured at 30 Hz and becomes usable after a fixed 33 ms latency;
    the planner never sees the true current pose;
  * planning: every 50 ms the latest *available* capture is turned into a point cloud -> CAPT
    environment; the current plan is checked and FLASK-RRTConnect replans from the predicted reference
    state when needed (decision logic of reactive_experiment.run); the new motion takes effect after
    max(START_DELAY, measured planning time); braking when no valid plan exists;
  * obstacle_policy='yield' (v7 default, diagnostic only): the tube waits when it would come closer than
    GOAL_CLEARANCE to the robot's ACTUAL configuration. obstacle_policy='independent' (v10): the tube follows its
    fixed script regardless of the robot, so avoidance is the planner's job;
  * late_plan_policy='insert' (v7 default): a plan finishing after the predicted start time is still inserted at
    that time (logged). 'discard' (v10): such a plan is not used (its start state no longer matches the schedule);
  * latency_inflation (v10): the sensed tube is inflated by TUBE_SPEED x sensor age in addition to the per-cycle
    motion already in TUBE_POINT_RADIUS.
Robot/scene and robot/tube contacts are counted from MuJoCo. Simulated; not the authors' setup.

Usage: python -m experiments.ur5_reactive_mujoco --trial_count 5 --output results/ur5_sim_v7/e2
"""
import json
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import fire
import mujoco
import numpy as np
import vamp

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'experiments')); sys.path.insert(0, str(ROOT / 'simulation'))
from flask_repro import native, robot as R
from flask_repro.provenance import save_manifest
import reactive_experiment as RX
import reactive_scene as CE
import ur5_scene
import ur5_mujoco as E1

SENSE_PERIOD, SENSE_LATENCY = 1 / 30, .033
MAX_NS = 30000000


def add_tube(root):
    body = ET.SubElement(root.find('worldbody'), 'body', name='tube', mocap='true', pos='0 0 -5')
    C = CE._LOCAL_CENTERS
    for k in range(0, len(C) - 1, 4):
        a, b = C[k], C[min(k + 4, len(C) - 1)]
        ET.SubElement(body, 'geom', name=f'scene_tube_{k:03d}', type='capsule', size=str(CE.TUBE_RADIUS),
                      fromto=' '.join(map(str, np.r_[a, b])), contype='1', conaffinity='2', rgba='.2 .2 .2 1')


def quaternion_wxyz(x):
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(CE.rotation_matrix(x[3], x[4])).as_quat()
    return np.r_[q[3], q[:3]]


def run_online(seed, rb, limits, dm, qs, trail, model, settings, obstacle_policy='yield', late_plan_policy='insert', latency_inflation=False):
    if obstacle_policy not in ('yield', 'independent') or late_plan_policy not in ('insert', 'discard'):
        raise ValueError('obstacle_policy must be yield|independent; late_plan_policy insert|discard')
    mod = native.module(rb)
    data = mujoco.MjData(model)
    n = rb.n
    E1.settle(model, data, qs[0], n)
    tracker = E1.Tracker(model, data, rb, qs[0])
    tube = model.body('tube').mocapid[0]
    schedule = RX.MotionSchedule(qs[0])
    goal_index, arrival_time, tube_time, stop_count = 0, None, 0., 0
    captures = []                      # (capture time, tube pose)
    next_capture = 0.
    cycles = int(round(CE.TOTAL_TIME / CE.REPLAN_PERIOD))
    ticks_per_cycle = int(round(CE.REPLAN_PERIOD / E1.DT_CONTROL))
    L = {k: np.full(cycles, np.nan) for k in ('time_s', 'plan_ns', 'shortcut_ns', 'capt_ns', 'sensor_age_s', 'decision',
                                               'contact_steps', 'commanded_speed_max')}  # v10: per-cycle contact attribution
    contact_steps, max_depth, pairs = 0, 0., set()
    errors = []
    for k in range(cycles):
        t = k * CE.REPLAN_PERIOD
        # ---- sense (30 Hz) with latency; planner uses the newest capture old enough to be available
        x = trail.pose(tube_time)
        while next_capture <= t + 1e-12:
            captures.append((next_capture, x)); next_capture += SENSE_PERIOD
        usable = [c for c in captures if c[0] <= t - SENSE_LATENCY] or captures[:1]
        capture_time, sensed = usable[-1]
        env, cs = dm.build(sensed, CE.TUBE_SPEED * (t - capture_time) if latency_inflation else 0.)
        L['time_s'][k], L['capt_ns'][k], L['sensor_age_s'][k] = t, cs['capt_ns'], t - capture_time
        te = t + CE.START_DELAY
        m = schedule.current(t)
        decided = False
        if m.category == 'iterations' and t >= m.end_time:
            if arrival_time is None:
                arrival_time = m.end_time
            if t < arrival_time + CE.PAUSE:
                L['decision'][k] = RX.DECISION_CODES['at_goal']; decided = True
            else:
                goal_index += 1; arrival_time = None
                schedule.insert(RX.MotionSegment(t, 'build', q=m.q)); m = schedule.current(t)
        if not decided:
            goal = qs[RX.TASK_ORDER[goal_index % 3]]
            start_valid, _ = RX._check(mod, env, m, t, te, limits)
            previous_valid, _ = RX._check(mod, env, m, te, np.inf, limits) if start_valid else (False, 0)
            if m.category == 'iterations' and te >= m.end_time and start_valid and previous_valid:
                L['decision'][k] = RX.DECISION_CODES['arriving']
            else:
                predicted_q, predicted_v, _ = schedule.state(te)
                d = dict(solved=False)
                if mod.validate(env, predicted_q.tolist()):
                    d = native.plan(rb, env, predicted_q, [goal], settings, seed=seed * 1000003 + k, v_start=predicted_v, limits=limits)
                    L['plan_ns'][k] = d['plan_ns']
                late = False
                if d['solved']:
                    L['shortcut_ns'][k] = d['shortcut_ns']
                    effective = t + max(CE.START_DELAY, (d['plan_ns'] + d['shortcut_ns']) / 1e9)
                    if effective > te + 1e-12:  # plan ready later than predicted: replan start would mismatch
                        errors.append(dict(cycle=k, reason='planning_slower_than_start_delay'))
                        late = late_plan_policy == 'discard'
                if d['solved'] and not late:
                    schedule.insert(RX.MotionSegment(te, 'iterations', trajectory=d['trajectory']))
                    L['decision'][k] = RX.DECISION_CODES['changed']
                elif start_valid and previous_valid:
                    L['decision'][k] = RX.DECISION_CODES['keep_plan' if m.category == 'iterations' else 'stationary']
                elif m.category == 'iterations':
                    brake_time = te if start_valid else t
                    q_b, v_b, _ = schedule.state(brake_time)
                    segment, _ = RX._brake(rb, mod, env, limits, q_b, v_b, brake_time)
                    schedule.insert(segment); stop_count += 1
                    L['decision'][k] = RX.DECISION_CODES['brake']
                else:
                    L['decision'][k] = RX.DECISION_CODES['stopped_by_obstacle']
        # ---- physics/control until the next planning cycle; the tube advances unless too close to the ACTUAL robot
        for j in range(ticks_per_cycle):
            tt = t + j * E1.DT_CONTROL
            nxt = tube_time + E1.DT_CONTROL
            if obstacle_policy == 'independent' or \
               CE.tube_clearance(data.qpos[:n], trail.pose(nxt)) >= CE.GOAL_CLEARANCE or \
               CE.tube_clearance(data.qpos[:n], trail.pose(nxt)) >= CE.tube_clearance(data.qpos[:n], trail.pose(tube_time)):
                tube_time = nxt
            pose = trail.pose(tube_time)
            data.mocap_pos[tube] = pose[:3]; data.mocap_quat[tube] = quaternion_wxyz(pose)
            qd, vd, _ = schedule.state(tt)
            tracker.step(qd, vd)
            L['commanded_speed_max'][k] = max(np.nan_to_num(L['commanded_speed_max'][k]), float(np.max(np.abs(vd))))
            depth, p = E1.contacts(model, data)
            if depth > 0:
                contact_steps += 1; max_depth = max(max_depth, depth); pairs |= p
                L['contact_steps'][k] = np.nan_to_num(L['contact_steps'][k]) + 1
    L['goals_reached'] = goal_index; L['stop_count'] = stop_count
    return L, dict(contact_steps=contact_steps, max_penetration_m=max_depth, contact_pairs=sorted(pairs), timing_violations=errors)


def main(trial_count=5, seed0=2001, output=None, obstacle_policy='yield', late_plan_policy='insert', latency_inflation=False):
    rb = R.get_robot('ur5')
    limits = native.joint_limits(rb)
    settings = native.settings(**(E1.FLASK_SETTINGS | dict(max_ns=MAX_NS)))
    p = CE.create_scene()
    qs = CE.configurations(vamp.problem_dict_to_vamp(ur5_scene.inflate_scene(p)))
    dm = RX.EnvironmentBuilder(p)
    trail = CE.TubeMotion()
    model = E1.build_model(p, add_tube)
    output = Path(output or ROOT / 'results/ur5_sim_v7/e2')
    output.mkdir(parents=True, exist_ok=False)
    save_manifest(output / 'manifest.json', dict(trial_count=trial_count, seed0=seed0, planner_settings=E1.FLASK_SETTINGS | dict(max_ns=MAX_NS),
        sense_period_s=SENSE_PERIOD, sense_latency_s=SENSE_LATENCY, replan_period_s=CE.REPLAN_PERIOD, start_delay_s=CE.START_DELAY,
        total_time_s=CE.TOTAL_TIME, tube_speed=CE.TUBE_SPEED, obstacle_policy=obstacle_policy, late_plan_policy=late_plan_policy,
        latency_inflation=latency_inflation, gains=dict(Kv=E1.KV, Kq=E1.KQ, Kp=E1.KP, j_max=E1.J_MAX),
        disclosure="Synthetic point cloud and tube motion; online loop in simulated time; MuJoCo rigid-body execution; not the authors' setup"))
    rows = []
    for trial in range(trial_count):
        seed = seed0 + trial
        t0 = time.time()
        L, c = run_online(seed, rb, limits, dm, qs, trail, model, settings, obstacle_policy, late_plan_policy, latency_inflation)
        planned = np.isfinite(L['plan_ns']); solved = np.isfinite(L['shortcut_ns'])
        row = dict(trial=trial, seed=seed, goals_reached=int(L['goals_reached']), brakes=int(L['stop_count']),
                   planning_calls=int(planned.sum()), solved_calls=int(solved.sum()),
                   planning_us=dict(median=float(np.median(L['plan_ns'][planned]) / 1e3), mean=float(np.mean(L['plan_ns'][planned]) / 1e3),
                                    p95=float(np.percentile(L['plan_ns'][planned], 95) / 1e3)),
                   simplification_us=dict(median=float(np.median(L['shortcut_ns'][solved]) / 1e3), mean=float(np.mean(L['shortcut_ns'][solved]) / 1e3)),
                   capt_ms_median=float(np.nanmedian(L['capt_ns']) / 1e6), wall_s=time.time() - t0, **c)
        hit = np.nan_to_num(L['contact_steps']); still = np.nan_to_num(L['commanded_speed_max']) < 1e-6
        # v10: a contact in a cycle whose commanded reference never moves is the obstacle hitting a stopped robot
        row.update(contact_steps_robot_stationary=int(hit[still].sum()), contact_steps_robot_moving=int(hit[~still].sum()),
                   decisions_with_contact={str(int(d)): int(n) for d, n in zip(*np.unique(L['decision'][hit > 0], return_counts=True))
                                           if np.isfinite(d)})
        rows.append(row)
        np.savez_compressed(output / f'trial_{trial:02d}.npz', **{k: v for k, v in L.items() if isinstance(v, np.ndarray)})
        print(f"trial {trial}: goals {row['goals_reached']} | plan median {row['planning_us']['median']:.0f} us mean {row['planning_us']['mean']:.0f} | "
              f"simplify median {row['simplification_us']['median']:.1f} us | brakes {row['brakes']} | contacts {row['contact_steps']} "
              f"| timing violations {len(c['timing_violations'])} [{row['wall_s']:.0f}s]", flush=True)
        (output / 'trials.json').write_text(json.dumps(rows, indent=1))
    summary = dict(trials=len(rows), collision_trials=sum(r['contact_steps'] > 0 for r in rows),
                   goals_reached=[r['goals_reached'] for r in rows],
                   trials_with_contact_while_moving=sum(r['contact_steps_robot_moving'] > 0 for r in rows),
                   contact_steps_stationary=[r['contact_steps_robot_stationary'] for r in rows],
                   contact_steps_moving=[r['contact_steps_robot_moving'] for r in rows],
                   decision_codes=RX.DECISION_CODES,
                   planning_us_median=float(np.median([r['planning_us']['median'] for r in rows])),
                   planning_us_mean=float(np.mean([r['planning_us']['mean'] for r in rows])),
                   simplification_us_median=float(np.median([r['simplification_us']['median'] for r in rows])),
                   paper=dict(planning_us=90, simplification_us=40, total_ms=.13))
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    plot_figure10(output)


def plot_figure10(output, trial=0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    z = np.load(output / f'trial_{trial:02d}.npz')
    fig, ax = plt.subplots(figsize=(7, 3), constrained_layout=True)
    ax.semilogy(z['time_s'], z['shortcut_ns'] / 1e9, '--', color='#eb6834', lw=1., label='simplification time (s)')
    ax.semilogy(z['time_s'], z['plan_ns'] / 1e9, '-', color='#2a78d6', lw=1., label='planning time (s)')
    ax.set_ylim(1e-7, 1e-2); ax.set_xlabel('Time (s)'); ax.grid(alpha=.3); ax.legend(fontsize=8, frameon=False)
    ax.set_title('Reproduction of Fig. 10 (simulated UR5, online loop in MuJoCo)', fontsize=9, loc='left')
    fig.savefig(output / 'figure10_reproduction.png', dpi=150); fig.savefig(output / 'figure10_reproduction.pdf')


if __name__ == '__main__':
    fire.Fire(main)
