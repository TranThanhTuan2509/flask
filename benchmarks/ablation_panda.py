"""Development ablation of reconstructed FLASK-RRTConnect settings on the Table II Panda problems.

Uses development seeds only (config/seed_registry.json policy). Each configuration runs the native
planner + Alg. 4 shortcut on every eligible MotionBenchMaker pair and records timing, search counters,
trajectory length, analytic limits and dense (1 ms / 128 samples per rad) collision sampling.
This is a diagnostic study, not a frozen measurement campaign.

Usage: python -m benchmarks.ablation_panda --output DIR --configs FILE.json --seeds 1001 [--cpu 3]
Config file: list of {"name": str, "settings": {...native.settings kwargs...},
                      "limits": {...JointLimits overrides...}, "a_max_scale": float (optional)}
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np

from flask_repro import certify, lqmt, mbm, native, robot, trajectory
from flask_repro.provenance import distribution, manifest
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json


def run_config(cfg, rb, problems, seeds):
    settings = native.settings(**cfg.get('settings', {}))
    certified = cfg.get('profile') == 'certified'
    delta = cfg.get('certify_delta', native.CERTIFY_DELTA)
    limits = native.joint_limits(rb, 'certified' if certified else 'published_resolution', delta)
    for key, value in cfg.get('limits', {}).items():
        setattr(limits, key, value)
    if 'a_max_scale' in cfg:
        limits.a_max = [float(a * cfg['a_max_scale']) for a in rb.a_max]
    rows = []
    for env, index, problem, ec in problems:
        for seed in seeds:
            plan_env = mbm.native_environment(certify.inflate_problem(problem, delta)) if certified else ec
            r = native.plan(rb, plan_env, problem['start'], problem['goals'], settings, seed=seed, limits=limits)
            row = dict(environment=env, problem=index, seed=seed, solved=bool(r['solved']),
                       plan_ms=r['plan_ns'] / 1e6, shortcut_ms=r['shortcut_ns'] / 1e6,
                       total_ms=(r['plan_ns'] + r['shortcut_ns']) / 1e6, iterations=r['iterations'],
                       edge_checks=r['edge_checks'], nodes=r['start_nodes'] + r['goal_nodes'])
            if r['solved']:
                path = r['trajectory']
                dense = trajectory.evaluate_flask(path, rb, ec)
                accel = max(float(np.max(np.abs(lqmt.evaluate(e.coef, np.array([0., e.T]))[2]) - rb.a_max))
                            for e in path.edges()) if len(path.T) else 0.
                row.update(length=dense['length'], duration=dense['duration'], collision=bool(dense['collision']),
                           accel_excess=accel, segments=len(path.T))
            rows.append(row)
    return rows


def summary(rows):
    out = {}
    for env in mbm.ENVIRONMENTS + ['overall']:
        g = rows if env == 'overall' else [r for r in rows if r['environment'] == env]
        s = [r for r in g if r['solved']]
        out[env] = dict(attempts=len(g), solved=len(s), collisions=sum(r['collision'] for r in s),
                        total_ms=distribution([r['total_ms'] for r in s]),
                        length=distribution([r['length'] for r in s]),
                        edge_checks=distribution([r['edge_checks'] for r in s]))
    return out


def main(args):
    os.sched_setaffinity(0, {args.cpu})
    args.output.mkdir(parents=True, exist_ok=True)
    configs = json.loads(args.configs.read_text())
    rb = robot.get_robot('panda')
    dataset = mbm.load_dataset('panda')
    problems = [(env, i, p, mbm.native_environment(p)) for env in mbm.ENVIRONMENTS
                for i, p in enumerate(dataset[env]) if p['valid'] and (args.problem_count is None or i < args.problem_count)]
    (args.output / 'manifest.json').write_text(json.dumps(manifest(dict(configs=configs, seeds=args.seeds,
        study='development_ablation_not_final_timing')), indent=2) + '\n')
    table = []
    for cfg in configs:
        rows = run_config(cfg, rb, problems, args.seeds)
        (args.output / f"{cfg['name']}.jsonl").write_text(''.join(json.dumps(r) + '\n' for r in rows))
        s = summary(rows)
        (args.output / f"{cfg['name']}.summary.json").write_text(json.dumps(s, indent=2) + '\n')
        o = s['overall']
        f = lambda x, spec='.3f': 'nan' if x is None else format(x, spec)
        line = (f"{cfg['name']:28s} solved {o['solved']}/{o['attempts']} coll {o['collisions']:3d} "
                f"mean {f(o['total_ms']['mean'])} p50 {f(o['total_ms']['p50'])} p75 {f(o['total_ms']['p75'])} "
                f"len {f(o['length']['mean'], '.2f')} checks p50 {f(o['edge_checks']['p50'], '.0f')}")
        print(line, flush=True)
        per_env = ' | '.join(f"{e[:10]} {f(s[e]['total_ms']['p50'], '.2f')}/{f(s[e]['total_ms']['mean'], '.1f')} {s[e]['solved']}"
                             for e in mbm.ENVIRONMENTS)
        print('    ' + per_env, flush=True)
        table.append(dict(name=cfg['name'], overall=o))
    (args.output / 'table.json').write_text(json.dumps(table, indent=2) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--configs', type=Path, required=True)
    p.add_argument('--seeds', type=int, nargs='+', default=[1001])
    p.add_argument('--cpu', type=int, default=default_cpu())
    p.add_argument('--problem-count', type=int)
    main(p.parse_args())
