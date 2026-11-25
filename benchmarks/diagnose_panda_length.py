"""Diagnose why FLASK Table II paths are longer than the paper's (development seeds 1001-1003 only).

For each problem: raw tree path length, length after one Alg. 4 pass (paper), after repeated passes until
convergence (diagnostic), node counts, number of goals, and the geometric VAMP (RRTC + simplify) length on
the same problem. Extension range 2.0 (VAMP C++ RRTCSettings default) vs 1.0 (VAMP Python robot default).
Rest-to-rest edges are straight joint-space segments, so the length is the sum of segment norms.

Usage: python -m benchmarks.diagnose_panda_length --output results/diagnostics/panda_length_v9 --cpu 3
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
import vamp

from flask_repro import mbm, native, robot

ROOT = Path(__file__).resolve().parents[1]
PROFILE = json.loads((ROOT / 'config/manipulator_final_v9.json').read_text())


def seg_length(Q):
    Q = np.asarray(Q)
    return float(np.sum(np.linalg.norm(np.diff(Q, axis=0), axis=1))) if len(Q) > 1 else 0.


def main(args):
    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})
    args.output.mkdir(parents=True, exist_ok=True)
    rb = robot.get_robot('panda'); mod = native.module(rb); limits = native.joint_limits(rb)
    data = mbm.load_dataset('panda')
    module, planner, vs, simp = vamp.configure_robot_and_planner_with_kwargs('panda', 'rrtc')
    out = {}
    for env in args.environments:
        for rng in (2.0, 1.0):
            s = native.settings(**(PROFILE['settings'] | dict(range=rng)))
            rows = []
            for index, problem in enumerate(data[env][:args.problems]):
                if not problem['valid']:
                    continue
                ec = mbm.native_environment(problem)
                ev = vamp.problem_dict_to_vamp(problem)
                for seed in (1001, 1002, 1003):
                    r = native.plan(rb, ec, problem['start'], problem['goals'], s, seed=seed)
                    if not r['solved']:
                        continue
                    Q, V, T = r['Q'], r['V'], r['T']
                    passes, prev = 1, len(Q)
                    while True:  # diagnostic only: repeat Alg. 4 until no node is removed
                        Q2, V2, T2 = mod.shortcut(ec, Q, V, T, limits, s.rho)[:3]
                        passes += 1
                        if len(Q2) >= len(Q) or passes > 10:
                            break
                        Q, V, T = Q2, V2, T2
                    sampler = module.halton(); sampler.skip((seed - 1) * 1000)
                    g = planner(list(problem['start']), [list(x) for x in problem['goals']], ev, vs, sampler)
                    geo = module.simplify(g.path, ev, simp, sampler).path.numpy() if g.solved else None
                    rows.append(dict(problem=index, seed=seed, goals=len(problem['goals']), iterations=r['iterations'],
                                     raw_nodes=len(r['raw_positions']), raw_length=seg_length(r['raw_positions']),
                                     alg4_nodes=len(r['Q']), alg4_length=seg_length(r['Q']), converged_length=seg_length(Q),
                                     vamp_geometric_length=seg_length(geo) if geo is not None else None))
            agg = {k: float(np.mean([x[k] for x in rows if x[k] is not None])) for k in
                   ['goals', 'iterations', 'raw_nodes', 'raw_length', 'alg4_nodes', 'alg4_length', 'converged_length', 'vamp_geometric_length']}
            out[f'{env}|range{rng}'] = dict(summary=agg, rows=rows)
            print(env, rng, {k: round(v, 2) for k, v in agg.items()}, flush=True)
    (args.output / 'panda_length.json').write_text(json.dumps(out, indent=1) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cpu', type=int)
    p.add_argument('--problems', type=int, default=40)
    p.add_argument('--environments', nargs='+', default=['box', 'bookshelf_thin', 'cage'])
    main(p.parse_args())
