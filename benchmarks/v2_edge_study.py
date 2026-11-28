"""flask_v2 Phase 1: edge-level mechanism study for the unicycle |omega| <= 1.5 rejections (no planner change).

A free-space tree is grown the way the v10 RRTConnect (rrt_mode 2) grows it on the unicycle: uniform position in the
6 x 6 m world, isotropic velocity in the annulus speed_min <= |v| <= 1, nearest node by the v10 heading-aware metric
(metric_mode 2), target position clipped to the 1.5 m range with the sampled velocity kept, optimal-T BVP (rho = 1).
An edge is valid when |v| <= 1, |omega| <= 1.5 and the speed never vanishes inside the edge (dense check, 801 samples).
Obstacles are left out on purpose: the study isolates the dynamic-limit rejections (96-98% of all rejections in v10).

Levers (plan file, Phase 1):
  L1  parent selection: k metric-nearest candidates re-ranked by the paper's LQMT cost C_loc(T*) (Eq. 24)
  L2  speed lower bound of the velocity proposal (annulus)
  L3  duration multipliers: if the T* edge is invalid, try fixed-time edges (Example 4) at c * T*
  L4  clipping: compare clipped (velocity kept) edges with unclipped edges
Two edge populations are measured: EXTEND (node -> clipped sample) and CONNECT (node -> exact node of the other tree,
as in the RRTConnect connect step, which must hit the other tree's state exactly).

Usage: python -m benchmarks.v2_edge_study --output results/diagnostics/v2_mechanism/edge_study.json
"""
import argparse
import itertools
import json
import time
from pathlib import Path

import numpy as np

from flask_repro import _core

V_MAX, W_MAX, RHO, RANGE, LO, HI = 1., 1.5, 1., 1.5, 0., 6.
DERIV_W, TURN_R = .2, V_MAX / W_MAX           # v10 metric_mode 2 settings (derivative_metric_weight, max_speed/max_yaw)
S = np.linspace(0., 1., 801)


def metric(a, b):
    base = np.sqrt(np.sum((a[:2] - b[:2]) ** 2) + DERIV_W * np.sum((a[2:] - b[2:]) ** 2))
    da = a[2:] / max(np.linalg.norm(a[2:]), 1e-300); db = b[2:] / max(np.linalg.norm(b[2:]), 1e-300)
    return np.hypot(base, TURN_R * np.linalg.norm(da - db))


def edge(a, b, T=0.):
    e = _core.flat_bvp([a[:2].tolist(), a[2:].tolist()], [b[:2].tolist(), b[2:].tolist()], RHO, T)
    return e


def check(e):
    """Returns (valid, cause) with cause in {'ok', 'speed', 'omega', 'stop', 'degenerate'}."""
    T = e['duration']
    if not (T > 0) or not np.isfinite(T):
        return False, 'degenerate'
    c = np.asarray(e['coefficients'])
    v = sum(np.outer(k * S ** (k - 1), c[k]) for k in range(1, 4)) / T
    acc = sum(np.outer(k * (k - 1) * S ** (k - 2), c[k]) for k in range(2, 4)) / T ** 2
    sp = np.linalg.norm(v, axis=1)
    if sp[1:-1].min() < 1e-6:
        return False, 'stop'
    if sp.max() > V_MAX + 1e-9:
        return False, 'speed'
    om = (v[:, 0] * acc[:, 1] - v[:, 1] * acc[:, 0]) / np.maximum(sp ** 2, 1e-300)
    if np.abs(om).max() > W_MAX + 1e-9:
        return False, 'omega'
    return True, 'ok'


def cost(e):
    return e['effort'] + RHO * e['duration']


def steer(a, b, multipliers):
    """Optimal-T edge, then (L3) fixed-time edges at c * T*; returns (edge, valid, cause, bvp_calls)."""
    e = edge(a, b); ok, cause = check(e); calls = 1
    T0 = e['duration']
    for c in multipliers:
        if ok or not (T0 > 0):
            break
        e2 = edge(a, b, c * T0); calls += 1
        ok2, cause2 = check(e2)
        if ok2:
            return e2, True, 'ok', calls
    return e, ok, cause, calls


def run(cfg, seed, iterations):
    rng = np.random.default_rng(seed)
    speed_min, k, mult = cfg['speed_min'], cfg['k'], cfg['multipliers']

    def sample():
        r = np.sqrt(rng.uniform(speed_min ** 2, V_MAX ** 2)); t = rng.uniform(0, 2 * np.pi)
        return np.array([*rng.uniform(LO, HI, 2), r * np.cos(t), r * np.sin(t)])

    trees = [[sample()], [sample()]]
    st = dict(extend=0, extend_ok=0, connect=0, connect_ok=0, bvp_calls=0, clipped=0, clipped_ok=0, unclipped=0,
              unclipped_ok=0, causes={}, edge_len=[], seconds=0.)
    t0 = time.perf_counter()

    def parent(tree, z):
        A = np.asarray(tree)
        base2 = np.sum((A[:, :2] - z[:2]) ** 2, 1) + DERIV_W * np.sum((A[:, 2:] - z[2:]) ** 2, 1)
        dirs = A[:, 2:] / np.linalg.norm(A[:, 2:], axis=1, keepdims=True)
        d = np.sqrt(base2 + TURN_R ** 2 * np.sum((dirs - z[2:] / np.linalg.norm(z[2:])) ** 2, 1))
        cand = np.argsort(d, kind='stable')[:k]
        if k == 1:
            return int(cand[0])
        costs = []
        for i in cand:
            e = edge(tree[i], z); st['bvp_calls'] += 1
            costs.append(cost(e) if e['duration'] > 0 else np.inf)
        return int(cand[int(np.argmin(costs))])

    for it in range(iterations):
        tree, other = trees[it % 2], trees[1 - it % 2]
        z = sample()
        i = parent(tree, z)
        dz = z.copy(); dp = np.linalg.norm(z[:2] - tree[i][:2]); clipped = dp > RANGE
        if clipped:
            dz[:2] = tree[i][:2] + (z[:2] - tree[i][:2]) * RANGE / dp
        e, ok, cause, calls = steer(tree[i], dz, mult)
        st['bvp_calls'] += calls; st['extend'] += 1; st['extend_ok'] += ok
        st['clipped' if clipped else 'unclipped'] += 1; st[('clipped' if clipped else 'unclipped') + '_ok'] += ok
        st['causes'][cause] = st['causes'].get(cause, 0) + 1
        if not ok:
            continue
        tree.append(dz); st['edge_len'].append(min(dp, RANGE))
        # CONNECT: other tree's parent for the new node, exact target state
        j = parent(other, dz)
        e, ok, cause, calls = steer(other[j], dz, mult)
        st['bvp_calls'] += calls; st['connect'] += 1; st['connect_ok'] += ok
    st['seconds'] = time.perf_counter() - t0
    return st


def summarize(rows):
    tot = lambda key: sum(r[key] for r in rows)
    causes = {}
    for r in rows:
        for c, n in r['causes'].items():
            causes[c] = causes.get(c, 0) + n
    return dict(extend_valid=tot('extend_ok') / max(tot('extend'), 1),
                connect_valid=tot('connect_ok') / max(tot('connect'), 1),
                clipped_valid=tot('clipped_ok') / max(tot('clipped'), 1),
                unclipped_valid=tot('unclipped_ok') / max(tot('unclipped'), 1),
                bvp_calls_per_valid_extend=tot('bvp_calls') / max(tot('extend_ok'), 1),
                seconds_per_valid_extend=sum(r['seconds'] for r in rows) / max(tot('extend_ok'), 1),
                mean_valid_edge_length=float(np.mean([x for r in rows for x in r['edge_len']])) if tot('extend_ok') else None,
                rejection_causes={c: n / max(tot('extend'), 1) for c, n in sorted(causes.items())},
                extend_attempts=tot('extend'), connect_attempts=tot('connect'))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seeds', type=int, nargs='+', default=[1001, 1002, 1003, 1004, 1005])
    p.add_argument('--iterations', type=int, default=1500)
    a = p.parse_args()
    grid = dict(speed_min=[.01, .2, .4], k=[1, 5, 10, 20], multipliers=[(), (1.5, 2., 3.)])
    out = dict(declared_rule='keep a lever for Phase 3 only if it raises the valid-edge fraction (extend or connect) '
                             'by >= 2x over the v10 baseline (speed_min .01, k 1, no multipliers) while the time per '
                             'valid extend grows < 2x', setup=dict(v_max=V_MAX, omega_max=W_MAX, rho=RHO, range=RANGE,
                             world=[LO, HI], derivative_metric_weight=DERIV_W, turn_radius=TURN_R,
                             seeds=a.seeds, iterations=a.iterations), results={})
    for sm, k, mult in itertools.product(*grid.values()):
        cfg = dict(speed_min=sm, k=k, multipliers=list(mult))
        name = f'speed_min={sm} k={k} mult={"/".join(map(str, mult)) or "none"}'
        out['results'][name] = cfg | summarize([run(cfg, s, a.iterations) for s in a.seeds])
        r = out['results'][name]
        print(f'{name:40s} extend {r["extend_valid"]:.3f} connect {r["connect_valid"]:.3f} '
              f'clip {r["clipped_valid"]:.3f}/{r["unclipped_valid"]:.3f} calls/valid {r["bvp_calls_per_valid_extend"]:.1f} '
              f'ms/valid {1e3 * r["seconds_per_valid_extend"]:.2f}', flush=True)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(out, indent=1) + '\n')


if __name__ == '__main__':
    main()
