"""Offline kinematic reactive diagnostic; not the paper's physical feedback loop.

Scripted obstacle motion is independent of the robot by default. Historical
cooperative obstacle yielding requires an explicit diagnostic opt-in and must
never substantiate reactive avoidance or execution performance.
"""
import bisect
import json
import sys
import time
from pathlib import Path
import fire
import numpy as np
import vamp
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'simulation'))
sys.path.insert(0, str(ROOT / 'experiments'))
from flask_repro import _core, mbm, native, robot as R
from flask_repro.reference import Trajectory
import reactive_scene as CE
import ur5_scene
from tracking_experiment import track
MAX_NS = 30000000
TASK_ORDER = (1, 2, 0)
DECISION_CODES = {'at_goal': 0, 'changed': 1, 'keep_plan': 2, 'brake': 3, 'brake_immediately': 4, 'continue_braking': 5, 'stationary': 6, 'arriving': 7, 'stopped_by_obstacle': 8}

class MotionSegment:
    __slots__ = ('t0', 'category', 'trajectory', 'q', 'end_time')

    def __init__(self, t0, category, trajectory=None, q=None):
        (self.t0, self.category, self.trajectory) = (t0, category, trajectory)
        self.q = np.asarray(q if q is not None else trajectory.Q[-1], float)
        self.end_time = t0 + (trajectory.duration() if trajectory is not None else 0.0)

    def state(self, t):
        n = self.q.shape[0]
        if self.trajectory is None or t >= self.end_time:
            return (self.q, np.zeros(n), np.zeros(n))
        (q, v, a) = (x[0] for x in self.trajectory.evaluate([max(t - self.t0, 0.0)]))
        return (q, v, a)

class MotionSchedule:
    """MotionSchedule state and operations."""

    def __init__(self, q0):
        self.pieces = [MotionSegment(0.0, 'build', q=q0)]
        self.critical_points = [0.0]

    def insert(self, m):
        while self.critical_points and self.critical_points[-1] >= m.t0 - 1e-12 and (len(self.pieces) > 1):
            self.pieces.pop()
            self.critical_points.pop()
        self.pieces.append(m)
        self.critical_points.append(m.t0)

    def current(self, t):
        return self.pieces[bisect.bisect_right(self.critical_points, t + 1e-12) - 1]

    def state(self, t):
        return self.current(t).state(t)

    def evaluate(self, t):
        return self.state(t)

    def duration(self):
        return CE.TOTAL_TIME

class EnvironmentBuilder:

    def __init__(self, p):
        self.table = CE.table_box(p)
        self.static_points = CE.static_points(p)
        (self.r_min, self.r_max) = vamp.ur5.min_max_radii()

    def build(self, tube_pose, extra_tube_radius=0.0):
        """extra_tube_radius (v10): additional tube inflation, e.g. TUBE_SPEED x sensor age."""
        e = _core.Environment()
        b = self.table
        e.add_cuboid(list(b['position']), list(b['orientation_euler_xyz']), [h + CE.PADDING for h in b['half_extents']])
        tube_points = CE.tube_surface(tube_pose)
        (l1, x1, n1) = e.add_pointcloud(self.static_points, self.r_min, self.r_max, CE.STATIC_POINT_RADIUS, CE.FILTER_SPACING, CE.CULL_R, list(CE.FILTER_ORIGIN))
        (l2, x2, n2) = e.add_pointcloud(tube_points, self.r_min, self.r_max, CE.TUBE_POINT_RADIUS + extra_tube_radius, CE.FILTER_SPACING, CE.CULL_R, list(CE.FILTER_ORIGIN))
        cb = e.prepare()
        return (e, dict(filter_ns=l1 + l2, build_ns=x1 + x2, simd_conversion_ns=cb, capt_ns=l1 + l2 + x1 + x2 + cb, static_point_count=n1, tube_point_count=n2))

def _check(mod, env, m, ta, tb, limits):
    """ check."""
    if m.trajectory is None or ta >= m.end_time:
        return (bool(mod.validate(env, m.q.tolist())), 0)
    trajectory = m.trajectory
    (ok, nanoseconds) = mod.validate_interval(env, trajectory.Q.tolist(), trajectory.V.tolist(), trajectory.T.tolist(), ta - m.t0, min(tb, m.end_time) - m.t0, limits)
    if ok and tb > m.end_time:
        ok = bool(mod.validate(env, m.q.tolist()))
    return (bool(ok), nanoseconds)

def _brake(rb, mod, env, limits, q, v, t0):
    """ brake."""
    if np.linalg.norm(v) < 1e-09:
        return (MotionSegment(t0, 'build', q=q), bool(mod.validate(env, q.tolist())))
    Tb = float(np.max(np.abs(v) / rb.a_max))
    q1 = q + v * Tb / 2
    trajectory = Trajectory(np.array([q, q1]), np.array([v, np.zeros_like(v)]), np.array([Tb]))
    ok = bool(mod.flask_cc(env, q.tolist(), v.tolist(), q1.tolist(), [0.0] * len(q), Tb, limits, True))
    return (MotionSegment(t0, 'brake', trajectory=trajectory), ok)

def obstacle_clock(previous, simulation_time, q, trail, diagnostic_yield=False):
    """Keep the obstacle on its fixed script unless cooperation is requested."""
    if not diagnostic_yield:
        return simulation_time, False
    candidate = previous + CE.REPLAN_PERIOD
    clearance = CE.tube_clearance(q, trail.pose(candidate))
    if clearance >= CE.GOAL_CLEARANCE or clearance >= CE.tube_clearance(q, trail.pose(previous)):
        return candidate, False
    return previous, True


def run(seed, rb, limits, dm, qs, trail, progress=True, diagnostic_obstacle_yield=False):
    mod = native.module(rb)
    settings = native.settings(neighbor_mode='A', max_ns=MAX_NS, max_iterations=int(10000000.0))
    schedule = MotionSchedule(qs[0])
    (goal_index, arrival_time) = (0, None)
    N = int(round(CE.TOTAL_TIME / CE.REPLAN_PERIOD))
    L = {k: np.zeros(N) for k in ('time_s', 'capt_ns', 'filter_ns', 'build_ns', 'simd_conversion_ns', 'static_point_count', 'tube_point_count', 'iterations', 'solved', 'plan_ns', 'shortcut_ns', 'node_count', 'raw_node_count', 'previous_valid', 'start_valid', 'decision', 'goal_index', 'brake_valid', 'validation_ns', 'insertion', 'start_feasible', 'tube_time', 'obstacle_wait')}
    stop_count = 0
    t_wall = time.time()
    tube_time = 0.0
    for k in range(N):
        t = k * CE.REPLAN_PERIOD
        te = t + CE.START_DELAY
        if k > 0:
            current_q = schedule.state(t)[0]
            tube_time, waited = obstacle_clock(tube_time, t, current_q, trail, diagnostic_obstacle_yield)
            L['obstacle_wait'][k] = int(waited)
        x = trail.pose(tube_time)
        L['tube_time'][k] = tube_time
        (env, cs) = dm.build(x)
        for (a, b) in cs.items():
            L[a][k] = b
        L['time_s'][k] = t
        L['insertion'][k] = trail.insertion_active(tube_time)
        L['brake_valid'][k] = np.nan
        L['plan_ns'][k] = L['shortcut_ns'][k] = np.nan
        m = schedule.current(t)
        if m.category == 'iterations' and t >= m.end_time:
            if arrival_time is None:
                arrival_time = m.end_time
            if t < arrival_time + CE.PAUSE:
                (ok, _) = _check(mod, env, m, t, te, limits)
                L['decision'][k] = DECISION_CODES['at_goal'] if ok else DECISION_CODES['stopped_by_obstacle']
                L['previous_valid'][k] = ok
                L['start_valid'][k] = ok
                L['goal_index'][k] = TASK_ORDER[goal_index % 3]
                continue
            goal_index += 1
            arrival_time = None
            schedule.insert(MotionSegment(t, 'build', q=m.q))
            m = schedule.current(t)
        goal = qs[TASK_ORDER[goal_index % 3]]
        L['goal_index'][k] = TASK_ORDER[goal_index % 3]
        (start_valid, ns1) = _check(mod, env, m, t, te, limits)
        (previous_valid, ns2) = _check(mod, env, m, te, np.inf, limits) if start_valid else (False, 0)
        (L['start_valid'][k], L['previous_valid'][k], L['validation_ns'][k]) = (start_valid, previous_valid, ns1 + ns2)
        if m.category == 'iterations' and te >= m.end_time:
            if start_valid and previous_valid:
                L['decision'][k] = DECISION_CODES['arriving']
                continue
        (predicted_q, predicted_v, _) = schedule.state(te)
        L['start_feasible'][k] = mod.validate(env, predicted_q.tolist())
        d = dict(solved=False)
        if L['start_feasible'][k]:
            d = native.plan(rb, env, predicted_q, [goal], settings, seed=seed * 1000003 + k, v_start=predicted_v, limits=limits)
            L['iterations'][k] = 1
            L['plan_ns'][k] = d['plan_ns']
            L['solved'][k] = d['solved']
        if d['solved']:
            L['shortcut_ns'][k] = d['shortcut_ns']
            L['node_count'][k] = len(d['T']) + 1
            L['raw_node_count'][k] = len(d['raw_durations']) + 1
        planning_active = m.category == 'iterations'
        if d['solved']:
            schedule.insert(MotionSegment(te, 'iterations', trajectory=d['trajectory']))
            L['decision'][k] = DECISION_CODES['changed']
        elif start_valid and previous_valid:
            L['decision'][k] = DECISION_CODES['keep_plan'] if m.category == 'iterations' else DECISION_CODES['stationary']
        elif planning_active:
            brake_time = te if start_valid else t
            (brake_q, brake_v, _) = schedule.state(brake_time)
            (mh, hok) = _brake(rb, mod, env, limits, brake_q, brake_v, brake_time)
            schedule.insert(mh)
            L['decision'][k] = DECISION_CODES['brake'] if start_valid else DECISION_CODES['brake_immediately']
            L['brake_valid'][k] = hok
            stop_count += 1
        else:
            L['decision'][k] = DECISION_CODES['continue_braking'] if m.category == 'brake' and t < m.end_time else DECISION_CODES['stopped_by_obstacle']
        if progress and k % 400 == 0:
            print(f'  seed {seed} t {t:5.1f}s goal {goal_index} build {stop_count} [{time.time() - t_wall:.0f}s]', flush=True)
    L['stop_count'] = stop_count
    L['goals_reached'] = goal_index
    L['loop_wall_s'] = time.time() - t_wall
    return (schedule, L)

def _cluster(b):
    b = np.asarray(b, bool)
    return int(np.sum(b[1:] & ~b[:-1]) + (1 if b.size and b[0] else 0))

def check_contacts(mod, actual_environment, X, Q, V):
    """Check contacts."""
    static_collision = np.array([mod.count_collisions(actual_environment, [q.tolist()])[0] > 0 for q in Q], bool)
    clearance = np.empty(len(Q))
    for (i, (x, q)) in enumerate(zip(X, Q)):
        S = CE.robot_spheres(q)
        C = CE.tube_centers(x)
        clearance[i] = (np.linalg.norm(S[:, None, :3] - C[None], axis=2) - S[:, None, 3] - CE.TUBE_RADIUS).min()
    tube_collision = clearance < 0
    moving = np.linalg.norm(V, axis=1) > 0.001
    return dict(static_collision=static_collision, tube_collision=tube_collision, tube_clearance=clearance, moving=moving)

def summarize_contacts(dv):
    collisions = dv['static_collision'] | dv['tube_collision']
    return dict(collision_samples=int(collisions.sum()), collision_events=_cluster(collisions), static_collision_events=_cluster(dv['static_collision']), tube_collision_events=_cluster(dv['tube_collision']), moving_collision_events=_cluster(collisions & dv['moving']), stopped_collision_events=_cluster(collisions & ~dv['moving']), minimum_tube_clearance_cm=float(dv['tube_clearance'].min() * 100))

def main(trial_count=5, seed0=0, plot=True, j_max=50.0, Kp=5.0, output=None, diagnostic_obstacle_yield=False):
    from flask_repro.provenance import save_manifest
    rb = R.get_robot('ur5')
    mod = native.module(rb)
    limits = native.joint_limits(rb)
    p = CE.create_scene()
    inflated_environment = vamp.problem_dict_to_vamp(ur5_scene.inflate_scene(p))
    qs = CE.configurations(inflated_environment)
    dm = EnvironmentBuilder(p)
    trail = CE.TubeMotion()
    actual_environment = mbm.native_environment(p)
    (e0, _) = dm.build(trail.pose(0.0))
    for (name, q) in zip(('start', 'pick', 'place'), qs):
        assert mod.validate(e0, q.tolist()), f'Pose {name} is invalid in the CAPT environment'
    output = Path(output) if output else ROOT / 'results/legacy' / 'e2'
    if (output / 'summary.json').exists():
        raise FileExistsError(output)
    output.mkdir(parents=True, exist_ok=True)
    obstacle_policy = 'cooperative_yield_diagnostic' if diagnostic_obstacle_yield else 'fixed_script_independent_of_robot'
    save_manifest(output / 'summary.json', dict(trial_count=trial_count, seed0=seed0, j_max=j_max, Kp=Kp, velocity_sample_scale=native.DEFAULT_VELOCITY_SCALE, scene=p, unavailable=['author obstacle motion', 'camera depth/latency', 'controller settings'], experiment='synthetic point cloud, offline kinematic replay; no physical feedback', obstacle_policy=obstacle_policy, planning_latency_applied=False))
    (output / 'edge.json').write_text(json.dumps(dict(edge=p, qs=[q.tolist() for q in qs], insertion=CE.INSERTION_POSES, tube_schedule=CE.MOTION_SCHEDULE, retreat=CE.RETREAT, retreat_offsets=CE.RETREAT_OFFSETS, tube=dict(r=CE.TUBE_RADIUS, length=CE.TUBE_LENGTH, arc_radius=CE.ARC_RADIUS)), default=float))
    total = []
    for trial in range(trial_count):
        seed = seed0 + trial
        (schedule, L) = run(seed, rb, limits, dm, qs, trail, diagnostic_obstacle_yield=diagnostic_obstacle_yield)
        t2 = time.time()
        (t, Qd, Qt) = track(schedule, rb, j_max, Kp)
        Vd = np.gradient(Qd, t, axis=0)
        sampled_tube_times = np.interp(t, L['time_s'], L['tube_time'], right=np.nan)
        sampled_tube_times = np.where(np.isnan(sampled_tube_times), L['tube_time'][-1] + (t - L['time_s'][-1]), sampled_tube_times)
        X = np.array([trail.pose(parameterization) for parameterization in sampled_tube_times])
        reference_contacts = check_contacts(mod, actual_environment, X, Qd, Vd)
        actual_contacts = check_contacts(mod, actual_environment, X, Qt, Vd)
        iterations = L['iterations'] > 0
        solved = iterations & (L['solved'] > 0)
        x = dict(trial=trial, seed=seed, stop_count=int(L['stop_count']), goals_reached=int(L['goals_reached']), planning_calls=int(iterations.sum()), solved_calls=int(solved.sum()), direct_fraction=float(np.mean(L['raw_node_count'][solved] == 2)), planning_median_us=float(np.median(L['plan_ns'][iterations]) / 1000.0), planning_mean_us=float(np.mean(L['plan_ns'][iterations]) / 1000.0), planning_max_us=float(np.max(L['plan_ns'][iterations]) / 1000.0), shortcut_median_us=float(np.median(L['shortcut_ns'][solved]) / 1000.0), shortcut_mean_us=float(np.mean(L['shortcut_ns'][solved]) / 1000.0), capt_median_ms=float(np.median(L['capt_ns']) / 1000000.0), capt_max_ms=float(np.max(L['capt_ns']) / 1000000.0), filter_median_ms=float(np.median(L['filter_ns']) / 1000000.0), build_median_ms=float(np.median(L['build_ns']) / 1000000.0), simd_conversion_median_ms=float(np.median(L['simd_conversion_ns']) / 1000000.0), point_count_median=float(np.median(L['static_point_count'] + L['tube_point_count'])), blocked_previous_plans=int(np.sum((L['previous_valid'] == 0) | (L['start_valid'] == 0))), braking_collisions=int(np.nansum(L['brake_valid'] == 0)), obstacle_wait_s=float(L['obstacle_wait'].sum() * CE.REPLAN_PERIOD), obstacle_end_time_s=float(L['tube_time'][-1]), decision={kk: int(np.sum(L['decision'] == v)) for (kk, v) in DECISION_CODES.items()}, reference=summarize_contacts(reference_contacts), track=summarize_contacts(actual_contacts), max_tracking_error=float(np.linalg.norm(Qt - Qd, axis=1).max()), planning_loop_wall_s=float(L['loop_wall_s']), evaluation_wall_s=time.time() - t2)
        total.append(x)
        X = X.astype(np.float32)
        np.savez_compressed(output / f'trial_{trial:02d}.npz', t=t, Qd=Qd.astype(np.float32), Qt=Qt.astype(np.float32), tube_pose=X, **{f'cycle_{kk if kk not in {"plan_ns", "node_count", "raw_node_count", "tube_time"} else {"plan_ns":"planning_ns", "node_count":"node_count", "raw_node_count":"raw_node_count", "tube_time":"tube_time_s"}[kk]}': v for (kk, v) in L.items() if isinstance(v, np.ndarray)}, **{f'reference_{kk}': v for (kk, v) in reference_contacts.items()}, **{f'actual_{kk}': v for (kk, v) in actual_contacts.items()})
        print(f"trial {trial}: reached {x['goals_reached']} goal | iterations {x['planning_calls']} (solved {x['solved_calls']}, direct connections {x['direct_fraction'] * 100:.0f}%) | iterations median {x['planning_median_us']:.0f} us, mean {x['planning_mean_us']:.0f}, shortcut {x['shortcut_median_us']:.1f} us | CAPT {x['capt_median_ms']:.2f} ms | build {x['stop_count']} | reference contacts {x['reference']['collision_events']} track {x['track']['collision_events']} (clearance tube min {x['track']['minimum_tube_clearance_cm']:.1f} cm) [{x['planning_loop_wall_s']:.0f}+{x['evaluation_wall_s']:.0f}s]", flush=True)
        (output / 'summary.json').write_text(json.dumps(total, indent=1))
    if plot and total:
        plot_reactive_timing(output, 0)
    print(f'-> {output}')

def plot_reactive_timing(output, trial=0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    z = np.load(output / f'trial_{trial:02d}.npz')
    t = z['cycle_time_s']
    iterations = z['cycle_iterations'] > 0
    solved = iterations & (z['cycle_solved'] > 0)
    (fig, (a1, a2)) = plt.subplots(2, 1, figsize=(8, 4.6), sharex=True, gridspec_kw=dict(height_ratios=[2.2, 1]), constrained_layout=True)
    insertion = z['cycle_insertion'] > 0
    for ax in (a1, a2):
        for (s, e) in _intervals(insertion, t):
            ax.axvspan(s, e, color='#888', alpha=0.1, lw=0)
    rg = np.where(solved, z['cycle_shortcut_ns'], np.nan) / 1000000000.0
    pl = np.where(iterations, z['cycle_planning_ns'], np.nan) / 1000000000.0
    a1.semilogy(t, rg, '--', color='#d62728', lw=1.2, label='simplification time (s)')
    a1.semilogy(t, pl, '-', color='#1f3fbf', lw=1.2, label='planning time (s)')
    build = np.isin(z['cycle_decision'], [DECISION_CODES['brake'], DECISION_CODES['brake_immediately']])
    if build.any():
        a1.plot(t[build], np.full(build.sum(), 2e-06), 'v', color='#c0202a', ms=4, label='Braking required')
    a1.set_ylim(1e-06, 0.005)
    a1.grid(alpha=0.3, which='major')
    a1.legend(loc='upper right', fontsize=8)
    a1.set_title('UR5 kinematic reconstruction: planning and shortcut time at 20 Hz (shading: obstacle insertion)', fontsize=9)
    a2.semilogy(t, z['cycle_capt_ns'] / 1000000000.0, color='#555', lw=1.0, label='CAPT filtering, construction, and SIMD conversion (s)')
    a2.grid(alpha=0.3)
    a2.legend(loc='upper right', fontsize=8)
    a2.set_xlabel('Time (s)')
    fig.savefig(output / 'reactive_summary.png', dpi=160)

def _intervals(b, t):
    (output, s) = ([], None)
    for (i, v) in enumerate(b):
        if v and s is None:
            s = t[i]
        if not v and s is not None:
            output.append((s, t[i]))
            s = None
    if s is not None:
        output.append((s, t[-1]))
    return output
if __name__ == '__main__':
    fire.Fire(main)
