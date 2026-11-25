"""Diagnose trajectory length: FLASK implementation and evaluation utilities."""
import csv
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'simulation'))
from flask_repro import mbm, native, lqmt, robot as R
from flask_repro.reference import Trajectory
from time_scaling import time_scale
rb = R.get_robot('ur5')
mod = native.module(rb)
limits = native.joint_limits(rb, 'published_resolution')
settings = native.settings(velocity_sample_scale=0.25, neighbor_mode='A', max_ns=int(1000000000.0), max_iterations=int(10000000.0))
base = {}
with open(ROOT / 'results/legacy/baseline_ur5_gh1_chord_length.csv') as f:
    for r in csv.DictReader(f):
        base[r['env'], int(r['problem'])] = (float(r['length_rad']), float(r['duration_s']))

def v_spline(Q, T):
    M = len(T)
    n = Q.shape[1]
    V = np.zeros_like(Q)
    if M < 2:
        return V
    A = np.zeros((M - 1, M - 1))
    B = np.zeros((M - 1, n))
    for (k, i) in enumerate(range(1, M)):
        (h0, h1) = (T[i - 1], T[i])
        A[k, k] = 2 * (h0 + h1)
        if k > 0:
            A[k, k - 1] = h1
        if k < M - 2:
            A[k, k + 1] = h0
        B[k] = 3 * (h1 * (Q[i] - Q[i - 1]) / h0 + h0 * (Q[i + 1] - Q[i]) / h1)
    V[1:M] = np.linalg.solve(A, B)
    return V

def valid(Q, V, T, ec):
    return all((mod.flask_cc(ec, Q[i].tolist(), V[i].tolist(), Q[i + 1].tolist(), V[i + 1].tolist(), float(T[i]), limits) for i in range(len(T))))

def arc_lengths(Q, V, T):
    return Trajectory(Q, V, T).length()

def rescaled_duration(Q, V, T):
    return float(np.sum(T)) * time_scale(Q, V, T, rb.v_max, rb.a_max)
def run(output):
    """Run the historical sensitivity diagnostic into a fresh output file."""
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    rows = []
    for (name, i, s, g, _ev, ec) in mbm.paired_problems('ur5'):
        d = native.plan(rb, ec, s, g, settings, seed=i)
        if not d['solved']:
            continue
        trajectory = d['trajectory']
        (Q, V, T) = (trajectory.Q, trajectory.V, trajectory.T)
        (Lb, Tb) = base[name, i]
        poly = float(np.sum(np.linalg.norm(np.diff(Q, axis=0), axis=1)))
        cos = []
        for k in range(1, len(Q) - 1):
            ch = Q[k + 1] - Q[k - 1]
            if np.linalg.norm(V[k]) > 1e-09:
                cos.append(float(V[k] @ ch / np.linalg.norm(V[k]) / np.linalg.norm(ch)))
        pause_duration = sum((lqmt.optimal_duration(Q[k], np.zeros(6), Q[k + 1], np.zeros(6)) for k in range(len(T))))
        VA = v_spline(Q, T)
        okA = valid(Q, VA, T, ec)
        (VC, TC) = (V.copy(), T.copy())
        for _ in range(5):
            VC = v_spline(Q, TC)
            TC = np.array([lqmt.optimal_duration(Q[k], VC[k], Q[k + 1], VC[k + 1]) for k in range(len(TC))])
        VC = v_spline(Q, TC)
        okC = valid(Q, VC, TC, ec)
        rows.append(dict(env=name, problem=i, node=len(Q), raw_nodes=len(d['raw_positions']), L=arc_lengths(Q, V, T), Lp=poly, Lb=Lb, T=float(T.sum()), Tb=Tb, stop_duration=pause_duration, scaled_duration=rescaled_duration(Q, V, T), cos=np.mean(cos) if cos else np.nan, vnorm=float(np.mean(np.abs(V[1:-1]) / rb.v_max)) if len(Q) > 2 else np.nan, okA=okA, LA=arc_lengths(Q, VA, T) if okA else np.nan, spline_scaled_duration=rescaled_duration(Q, VA, T) if okA else np.nan, okC=okC, LC=arc_lengths(Q, VC, TC) if okC else np.nan, TC=float(TC.sum()), alternating_scaled_duration=rescaled_duration(Q, VC, TC) if okC else np.nan))
    import pandas as pd
    df = pd.DataFrame(rows)
    df.to_csv(output, index=False)
    md = lambda c: float(np.nanmedian(df[c]))
    print(f"problem: {len(df)}   nodes after Algorithm 4 median {md('node'):.0f} (raw {md('raw_nodes'):.0f})")
    print(f"Median length: FLASK curve {md('L'):.2f} | waypoint polyline {md('Lp'):.2f} | baseline {md('Lb'):.2f}")
    print(f'  curve/polyline ratio {np.nanmedian(df.L / df.Lp):.3f}   polyline/baseline {np.nanmedian(df.Lp / df.Lb):.3f}')
    print(f'Cosine between node velocity and waypoint chord mean {np.nanmean(df.cos):.2f}   |V|/vmax mean {np.nanmean(df.vnorm):.3f}')
    print(f"Median duration: FLASK {md('T'):.1f} s | stop at each node {md('stop_duration'):.1f} s | rescale uniform {md('scaled_duration'):.1f} s | baseline {md('Tb'):.1f} s")
    print(f"what-if A (V spline): accepted {df.okA.mean() * 100:.0f} %  length {md('LA'):.2f}  rescale {md('spline_scaled_duration'):.1f} s")
    print(f"what-if C (V spline + alternating optimal duration): accepted {df.okC.mean() * 100:.0f} %  length {md('LC'):.2f}  T(rho=1) {md('TC'):.1f}  rescale {md('alternating_scaled_duration'):.1f} s")
    for (e, g) in df.groupby('env'):
        print(f"  {e:17s} node {g.node.median():.0f}  L {g.L.median():5.2f} Lp {g.Lp.median():5.2f} Lb {g.Lb.median():5.2f}  T {g['T'].median():4.1f} rescale {g.scaled_duration.median():3.1f} | A ok {g.okA.mean() * 100:3.0f}% L {g.LA.median():5.2f} rescale {g.spline_scaled_duration.median():3.1f} | C ok {g.okC.mean() * 100:3.0f}% L {g.LC.median():5.2f} rescale {g.alternating_scaled_duration.median():3.1f} | base T {g.Tb.median():.1f}")


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output)
