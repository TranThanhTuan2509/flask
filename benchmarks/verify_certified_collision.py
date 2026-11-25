"""Negative-control replay: every stored Panda trajectory that dense sampling found colliding must be
rejected by the certified (sweep-bounded) environment check on the delta-inflated scene.

Replays stored NPZ trajectories only (no planning, no timing).
Usage: python -m benchmarks.verify_certified_collision --output results/verification/certified_collision_replay.json
"""
import argparse
import json
from pathlib import Path

import numpy as np

from flask_repro import certify, mbm, native, robot

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ['results/paper_campaigns/2026-11-23_native_v1/table_ii_panda',
           'results/paper_campaigns/2026-11-23_native_v2/table_ii_panda_v2_res1',
           'results/paper_campaigns/2026-11-23_native_v2/table_ii_panda_v2_res4',
           'results/paper_campaigns/2026-11-23_corrective_v3/panda_velocity0_prefix1_res16_v3']


def main(args):
    rb = robot.get_robot('panda')
    mod = native.module(rb)
    limits = native.joint_limits(rb, 'certified', args.delta)
    dataset = mbm.load_dataset('panda')
    records = []
    for source in SOURCES:
        directory = ROOT / source
        if not (directory / 'attempts.jsonl').exists():
            continue
        for line in (directory / 'attempts.jsonl').open():
            row = json.loads(line)
            if row.get('method') != 'flask_rrt_connect' or not row.get('collision'):
                continue
            with np.load(directory / row['trajectory_artifact']) as z:
                Q, V, T = z['Q'], z['V'], z['T']
            problem = dataset[row['environment']][row['problem']]
            env = mbm.native_environment(certify.inflate_problem(problem, args.delta))
            rejected = [i for i in range(len(T))
                        if not mod.flask_cc(env, Q[i].tolist(), V[i].tolist(), Q[i + 1].tolist(), V[i + 1].tolist(),
                                            float(T[i]), limits, False)]
            records.append(dict(source=source, environment=row['environment'], problem=row['problem'], seed=row['seed'],
                                edges=len(T), rejected_edges=rejected, rejected=bool(rejected)))
    summary = dict(delta_m=args.delta, reach=list(certify.reach('panda')), trajectories=len(records),
                   rejected=sum(r['rejected'] for r in records),
                   accepted_known_colliding=[r for r in records if not r['rejected']], records=records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=1) + '\n')
    print(json.dumps({k: summary[k] for k in ['delta_m', 'trajectories', 'rejected']}),
          'accepted:', [(r['source'].split('/')[-1], r['environment'], r['problem'], r['seed']) for r in summary['accepted_known_colliding']])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=ROOT / 'results/verification/certified_collision_replay.json')
    p.add_argument('--delta', type=float, default=native.CERTIFY_DELTA)
    main(p.parse_args())
