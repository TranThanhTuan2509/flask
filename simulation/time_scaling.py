"""Time scaling: FLASK implementation and evaluation utilities."""
import json
import sys
import time
from pathlib import Path
import fire
import numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from flask_repro import mbm, native, robot as R
from flask_repro.reference import Trajectory
DT = 0.001

def time_scale(Q, V, T, v_max, a_max):
    """Time scale."""
    Tc = T[:, None]
    (v0, v1) = (V[:-1], V[1:])
    dp = Q[1:] - Q[:-1] - Tc * v0
    dv = v1 - v0
    c3 = -2.0 * dp / Tc ** 3 + dv / Tc ** 2
    c2 = 3.0 * dp / Tc ** 2 - dv / Tc
    vm = np.maximum(np.abs(v0), np.abs(v1))
    with np.errstate(divide='ignore', invalid='ignore'):
        ts = -c2 / (3.0 * c3)
        vd = v0 - c2 ** 2 / (3.0 * c3)
    inside = np.isfinite(ts) & (ts > 0) & (ts < Tc)
    vm = np.where(inside, np.maximum(vm, np.abs(vd)), vm)
    am = np.maximum(np.abs(2.0 * c2), np.abs(6.0 * c3 * Tc + 2.0 * c2))
    return max(float(np.max(vm / v_max)), float(np.sqrt(np.max(am / a_max))))

def rescale(trajectory, rb):
    s = time_scale(trajectory.Q, trajectory.V, trajectory.T, rb.v_max, rb.a_max)
    return (Trajectory(trajectory.Q, trajectory.V / s, trajectory.T * s), s)

def main(robot='ur5'):
    rb = R.get_robot(robot)
    settings = native.settings(velocity_sample_scale=0.25, neighbor_mode='A', max_ns=int(1000000000.0), max_iterations=int(10000000.0))
    D = ROOT / 'results/legacy' / 'mujoco_mbm' / robot
    start_chain = json.loads((D / 'inventory.json').read_text())
    dataset = mbm.load_dataset(robot)
    for x in start_chain:
        p = dataset[x['env']][x['problem']]
        ec = mbm.native_environment(p)
        d = native.plan(rb, ec, np.asarray(p['start'], float), [np.asarray(g, float) for g in p['goals']], settings, seed=x['problem'])
        trajectory = d['trajectory']
        previous = np.load(D / f"problem_{x['index']:02d}.npz")
        assert abs(trajectory.duration() - previous['tf'][-1]) < 0.002, 'Retiming reconstruction changed the original trajectory'
        time_scale(trajectory.Q, trajectory.V, trajectory.T, rb.v_max, rb.a_max)
        a = time.perf_counter_ns()
        (qm, s) = rescale(trajectory, rb)
        nanoseconds = time.perf_counter_ns() - a
        tf = np.append(np.arange(0.0, qm.duration(), DT), qm.duration())
        (Q, V, A) = qm.evaluate(tf)
        (pv, pa) = (np.max(np.abs(V) / rb.v_max), np.max(np.abs(A) / rb.a_max))
        assert pv <= 1 + 1e-06 and pa <= 1 + 1e-06, (pv, pa)
        assert np.allclose(Q[-1], previous['Qf'][-1], atol=1e-06)
        x.update(time_scales=s, rescaling_ms=nanoseconds / 1000000.0, flask_rescaled_planning_ms=x['flask_ms'] + nanoseconds / 1000000.0, flask_rescaled_duration_s=float(tf[-1]))
        np.savez_compressed(D / f"problem_{x['index']:02d}_rescaled.npz", tf=tf, Qf=Q)
        print(f"#{x['index']:2d}: s = {s:.3f}  T {x['T_flask']:5.1f} -> {tf[-1]:4.1f} s (baseline {x['T_baseline']:.1f} s) | iterations {x['flask_ms']:.3f} + rescale {nanoseconds / 1000.0:.0f} us = {x['flask_rescaled_planning_ms']:.3f} ms (baseline {x['ms_baseline']:.2f})")
    (D / 'inventory.json').write_text(json.dumps(start_chain, indent=1, ensure_ascii=False))
if __name__ == '__main__':
    fire.Fire(main)
