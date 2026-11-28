"""flask_v2 Phase 1, lever L5: pseudo-control proposal bound for the unicycle propagation planners (no planner change).

A free-space propagation tree is grown as SIMD-only SST* / DP-RRT grow it on the unicycle (order 2): pick a node,
draw a constant pseudo-control w (flat acceleration) uniformly in [-b, b]^2 and a duration in [0, 0.3] s, propagate
the flat chain (Eq. 29) and accept the edge when |v| <= 1, |omega| <= 1.5 and the speed does not vanish.
Node choice is uniform over the tree (the sampling-based selection is not what L5 changes).
b = 2.0 is the v10 setting (a unitless reconstruction); b = omega_max * v_max = 1.5 is the largest lateral
acceleration the unicycle can use at full speed; b = 1.0 is reported for the trend.
"""
import argparse
import json
from pathlib import Path

import numpy as np

V_MAX, W_MAX, T_MAX = 1., 1.5, .3
S = np.linspace(0., 1., 201)


def valid(v0, w, T):
    v = v0[None, :] + np.outer(S * T, w)
    sp = np.linalg.norm(v, axis=1)
    if sp.min() < 1e-6 or sp.max() > V_MAX + 1e-9:
        return False
    om = (v[:, 0] * w[1] - v[:, 1] * w[0]) / sp ** 2
    return np.abs(om).max() <= W_MAX + 1e-9


def run(bound, seed, iterations):
    rng = np.random.default_rng(seed)
    nodes = [(np.zeros(2), np.array([.6, 0.]))]                # start lift 0.6 m/s (v10 unicycle profile)
    ok = 0
    for _ in range(iterations):
        p0, v0 = nodes[rng.integers(len(nodes))]
        w = rng.uniform(-bound, bound, 2); T = rng.uniform(0., T_MAX)
        if valid(v0, w, T):
            ok += 1
            nodes.append((p0 + v0 * T + .5 * w * T * T, v0 + w * T))
    P = np.array([p for p, _ in nodes]); V = np.array([np.linalg.norm(v) for _, v in nodes])
    return dict(accept=ok / iterations, spread=float(np.sqrt(np.mean(np.sum((P - P.mean(0)) ** 2, 1)))),
                median_speed=float(np.median(V)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    out = dict(declared_rule='keep L5 only if the acceptance rate rises >= 2x over b = 2.0 without shrinking the tree '
                             'spread (coverage per iteration) below the b = 2.0 value', results={})
    for b in (2., 1.5, 1.):
        rows = [run(b, s, 4000) for s in range(1001, 1006)]
        out['results'][f'b={b}'] = {k: float(np.mean([r[k] for r in rows])) for k in rows[0]}
        print(b, out['results'][f'b={b}'])
    a.output.write_text(json.dumps(out, indent=1) + '\n')


if __name__ == '__main__':
    main()
