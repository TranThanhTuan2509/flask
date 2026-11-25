"""Tracking experiment: FLASK implementation and evaluation utilities."""
import json
import sys
import time
from pathlib import Path
import fire
import numpy as np
import ruckig
import vamp
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'simulation'))
sys.path.insert(0, str(ROOT / 'experiments'))
from flask_repro import mbm, native, robot as R
from flask_repro.provenance import save_manifest
from flask_repro.reference import Trajectory
import ur5_scene
from time_scaling import rescale
DT = 0.002
PAUSE = 0.5
TAIL = 1.0

class FlatReference:

    def __init__(self, segments):
        self.segments = segments
        self.critical_points = np.cumsum([0.0] + [d.duration() + (PAUSE if k == 0 else 0.0) for (k, d) in enumerate(segments)])

    def duration(self):
        return float(self.critical_points[-1])

    def evaluate(self, t):
        for (k, d) in enumerate(self.segments):
            if t <= self.critical_points[k] + d.duration() or k == len(self.segments) - 1:
                tt = min(max(t - self.critical_points[k], 0.0), d.duration())
                (q, v, a) = (x[0] for x in d.evaluate([tt]))
                if t - self.critical_points[k] > d.duration():
                    (v, a) = (np.zeros_like(v), np.zeros_like(a))
                return (q, v, a)
            if t <= self.critical_points[k + 1]:
                return (d.Q[-1], np.zeros(d.Q.shape[1]), np.zeros(d.Q.shape[1]))
        raise AssertionError

class GeometricReference:
    """GeometricReference state and operations."""

    def __init__(self, wps, Ts):
        (self.wps, self.Ts) = (wps, Ts)
        self.critical_points = np.cumsum([0.0] + [T + (PAUSE if k == 0 else 0.0) for (k, T) in enumerate(Ts)])
        self.L = [np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(w, axis=0), axis=1))]) for w in wps]

    def duration(self):
        return float(self.critical_points[-1])

    def evaluate(self, t):
        n = self.wps[0].shape[1]
        for (k, (w, T, L)) in enumerate(zip(self.wps, self.Ts, self.L)):
            if t <= self.critical_points[k] + T or k == len(self.wps) - 1:
                tt = t - self.critical_points[k]
                if tt >= T:
                    return (w[-1], np.zeros(n), np.zeros(n))
                s = L[-1] * max(tt, 0.0) / T
                i = min(np.searchsorted(L, s, side='right') - 1, len(w) - 2)
                u = (w[i + 1] - w[i]) / (L[i + 1] - L[i])
                return (w[i] + u * (s - L[i]), u * L[-1] / T, np.zeros(n))
            if t <= self.critical_points[k + 1]:
                return (w[-1], np.zeros(n), np.zeros(n))
        raise AssertionError

def track(tc, rb, j_max, Kp=5.0):
    n = rb.n
    otg = ruckig.Ruckig(n, DT)
    inp = ruckig.InputParameter(n)
    out = ruckig.OutputParameter(n)
    inp.control_interface = ruckig.ControlInterface.Velocity
    inp.max_velocity = rb.v_max.tolist()
    inp.max_acceleration = rb.a_max.tolist()
    inp.max_jerk = [j_max] * n
    (q0, _, _) = tc.evaluate(0.0)
    inp.current_position = q0.tolist()
    inp.current_velocity = [0.0] * n
    inp.current_acceleration = [0.0] * n
    N = int(np.ceil((tc.duration() + TAIL) / DT)) + 1
    t = np.arange(N) * DT
    Qd = np.empty((N, n))
    Qt = np.empty((N, n))
    Qt[0] = q0
    Qd[0] = q0
    for k in range(1, N):
        (q, v, _) = tc.evaluate(t[k - 1])
        u = v + Kp * (q - np.asarray(inp.current_position))
        inp.target_velocity = np.clip(u, -rb.v_max, rb.v_max).tolist()
        inp.target_acceleration = [0.0] * n
        r = otg.update(inp, out)
        if r not in (ruckig.Result.Working, ruckig.Result.Finished):
            raise RuntimeError(f'Ruckig update failed with result {r} at time {t[k]:.3f} s')
        Qt[k] = out.new_position
        Qd[k] = tc.evaluate(t[k])[0]
        out.pass_to_input(inp)
    return (t, Qd, Qt)

def plan_flask(rb, ec, a, b, seed, velocity_sample_scale=native.DEFAULT_VELOCITY_SCALE):
    settings = native.settings(neighbor_mode='A', max_ns=int(1000000000.0), max_iterations=int(10000000.0), velocity_sample_scale=velocity_sample_scale)
    d = native.plan(rb, ec, a, [b], settings, seed=seed)
    if not d['solved']:
        return (None, None)
    return (d['trajectory'], (d['plan_ns'] + d['shortcut_ns']) / 1000000.0)

def plan_geometric(ev, a, b, seed):
    (m, planner, ps, simp) = vamp.configure_robot_and_planner_with_kwargs('ur5', 'rrtc')
    sampler = m.halton()
    sampler.reset()
    sampler.skip(1000 * seed)
    r = planner(list(a), [list(b)], ev, ps, sampler)
    if not r.solved:
        return (None, None)
    sp = m.simplify(r.path, ev, simp, sampler)
    return (np.asarray(sp.path.numpy(), float), (r.nanoseconds + sp.nanoseconds) / 1000000.0)

def _contacts_per_state(mod, ec, Q):
    return np.array([mod.count_collisions(ec, [q.tolist()])[0] > 0 for q in Q], bool)

def main(trial_count=20, mode='original', j_max=50.0, Kp=5.0, seed0=0, plot=True, variant='original', counter=None, pick_clearance=None, output=None, velocity_sample_scale=native.DEFAULT_VELOCITY_SCALE):
    if mode not in ('original', 'rescale'):
        raise ValueError(mode)
    rb = R.get_robot('ur5')
    mod = native.module(rb)
    p = ur5_scene.create_scene(variant)
    if counter is None:
        counter = ur5_scene.PADDING
    if pick_clearance is None:
        pick_clearance = ur5_scene.PICK_CLEARANCE
    planning_scene = ur5_scene.inflate_scene(p, counter)
    ev = vamp.problem_dict_to_vamp(planning_scene)
    planning_environment = mbm.native_environment(planning_scene)
    ec = mbm.native_environment(p)
    (start_q, pick_q, place_q) = ur5_scene.configurations(ev, pick_clearance)
    output = Path(output) if output else ROOT / 'results/legacy' / ('e1' if variant == 'original' else f'e1_{variant}') / f'{mode}_j{j_max:g}_Kp{Kp:g}'
    if (output / 'summary.json').exists():
        raise FileExistsError(output)
    output.mkdir(parents=True, exist_ok=True)
    (output.parent / 'edge.json').write_text(json.dumps(p, default=float))
    (output.parent / 'iterations.json').write_text(json.dumps(dict(counter=counter, pick_clearance=pick_clearance)))
    total = []
    failures = []
    save_manifest(output / 'summary.json', dict(trial_count=trial_count, seed0=seed0, mode=mode, j_max=j_max, Kp=Kp, dt=DT, counter=counter, pick_clearance=pick_clearance, velocity_sample_scale=velocity_sample_scale, scene=p, controller='Ruckig velocity tracking, kinematic integration', unavailable=['author scene poses', 'controller gains', 'jerk limit', 'robot feedback/latency']))
    t0 = time.time()
    for trial in range(trial_count):
        seed = seed0 + trial
        (f1, ms1) = plan_flask(rb, planning_environment, start_q, pick_q, seed, velocity_sample_scale)
        (f2, ms2) = plan_flask(rb, planning_environment, pick_q, place_q, seed, velocity_sample_scale)
        (w1, mg1) = plan_geometric(ev, start_q, pick_q, seed)
        (w2, mg2) = plan_geometric(ev, pick_q, place_q, seed)
        if f1 is None or f2 is None or w1 is None or (w2 is None):
            failures.append(dict(trial=trial, seed=seed, reason='planning_failed', flask=[f1 is not None, f2 is not None], geometric=[w1 is not None, w2 is not None]))
            print(f'trial {trial}: planning failed (FLASK {f1 is not None}/{f2 is not None}, geometric {w1 is not None}/{w2 is not None})', flush=True)
            continue
        time_scales = []
        if mode == 'rescale':
            (f1, s1) = rescale(f1, rb)
            (f2, s2) = rescale(f2, rb)
            time_scales = [s1, s2]
        tf = FlatReference([f1, f2])
        tg = GeometricReference([w1, w2], [f1.duration(), f2.duration()])
        result = {}
        for (name, tc) in (('flask', tf), ('geometric', tg)):
            (t, Qd, Qt) = track(tc, rb, j_max, Kp)
            e = np.linalg.norm(Qt - Qd, axis=1)
            reference_collision = _contacts_per_state(mod, ec, Qd)
            actual_collision = _contacts_per_state(mod, ec, Qt)
            ee_d = np.array([np.asarray(vamp.ur5.eefk(list(q)), float)[:3, 3] for q in Qd[::5]])
            ee_t = np.array([np.asarray(vamp.ur5.eefk(list(q)), float)[:3, 3] for q in Qt[::5]])
            e_ee = np.linalg.norm(ee_t - ee_d, axis=1)
            result[name] = dict(T=tc.duration(), max_error=float(e.max()), mean_error=float(e.mean()), max_end_effector_error=float(e_ee.max()), actual_collision_samples=int(actual_collision.sum()), reference_collision_samples=int(reference_collision.sum()), first_collision_time=float(t[actual_collision][0]) if actual_collision.any() else None)
            np.savez_compressed(output / f'trial_{trial:02d}_{name}.npz', t=t, Qd=Qd, Qt=Qt, actual_collision=actual_collision, reference_collision=reference_collision)
        result.update(trial=trial, seed=seed, flask_ms=ms1 + ms2, geometric_ms=mg1 + mg2, time_scales=time_scales, flask_node_count=[len(f1.Q), len(f2.Q)], waypoint_count=[len(w1), len(w2)])
        total.append(result)
        (F, G) = (result['flask'], result['geometric'])
        print(f"trial {trial:2d}: T {F['T']:5.1f}s | FLASK max_error {F['max_error']:.4f} rad ({F['max_end_effector_error'] * 1000.0:5.1f} mm) collisions {F['actual_collision_samples']:4d} | geometric max_error {G['max_error']:.4f} rad ({G['max_end_effector_error'] * 1000.0:5.1f} mm) collisions {G['actual_collision_samples']:4d} (reference collisions {G['reference_collision_samples']})  [{time.time() - t0:.0f}s]", flush=True)
    (output / 'summary.json').write_text(json.dumps(total, indent=1))
    (output / 'attempts.json').write_text(json.dumps(dict(attempted=trial_count, successful=len(total), failures=failures), indent=2))
    n = len(total)
    if n:
        for name in ('flask', 'geometric'):
            collisions = sum((x[name]['actual_collision_samples'] > 0 for x in total))
            vd = sum((x[name]['reference_collision_samples'] > 0 for x in total))
            print(f"{name:8s}: reference collisions {vd}/{n} | {collisions}/{n} trial collisions (actual_state, sphere VAMP) | max_error median {np.median([x[name]['max_error'] for x in total]):.4f} rad, end effector {np.median([x[name]['max_end_effector_error'] for x in total]) * 1000.0:.1f} mm")
    if plot and n:
        plot_tracking(output, total)
    print(f'-> {output}')

def plot_tracking(output, total):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x = max(total, key=lambda x: (x['geometric']['actual_collision_samples'], x['geometric']['max_error']))
    (fig, ax) = plt.subplots(figsize=(8, 3.2), constrained_layout=True)
    for (name, color, label) in (('geometric', '#d9731a', 'Geometric reference (constant velocity)'), ('flask', '#2a7de1', 'FLASK')):
        z = np.load(output / f"trial_{x['trial']:02d}_{name}.npz")
        e = np.linalg.norm(z['Qt'] - z['Qd'], axis=1)
        ax.plot(z['t'], e, color=color, lw=1.2, label=label)
        if z['actual_collision'].any():
            ax.plot(z['t'][z['actual_collision']], e[z['actual_collision']], '.', color='#c0202a', ms=2.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Joint tracking error norm (rad)')
    ax.set_title(f"E1 trial {x['trial']}: Ruckig tracking error (red points: VAMP sphere collisions)")
    ax.legend(frameon=False)
    ax.grid(alpha=0.3)
    fig.savefig(output / 'tracking_error.png', dpi=150)
if __name__ == '__main__':
    fire.Fire(main)
