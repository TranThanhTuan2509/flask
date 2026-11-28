"""flask_v2 generalization check, declared in config/v2_generalization_rule.json before it ran.

v1 (literal) and v2 settings on unicycle problems never used to choose a setting: three unseen DynoBench maps
and 100 random scenes, both flat charts, all four planners. Every returned path is validated on the original model.

    python -m benchmarks.v2_generalization --output results/diagnostics/v2_generalization
"""
import argparse
import json
import multiprocessing as mp
import os
import time
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RULE = json.loads((ROOT / 'config/v2_generalization_rule.json').read_text())
REGISTRY = json.loads((ROOT / 'config/seed_registry.json').read_text())
PROFILES = dict(v1='config/low_dof_v1_literal_v2d.json', v2='config/low_dof_v2d_order2.json')
MAPS = RULE['problems']['maps']
PLANNERS = RULE['settings']['planners']
BUDGET = RULE['settings']['budget_s']
CORES = [0, 2, 4, 6, 8, 10, 12, 14]


def problem(kind, key, chart, version, algorithm):
    """(model, planning world, original world, start, goal, settings) built from the unicycle family settings."""
    from flask_repro.dynobench import load
    from flask_repro.flat_models import Unicycle
    from flask_repro.flat_planner import World
    from flask_repro.planning import PlannerSettings
    from verification.v5_random_low_dof import scene
    p = json.loads((ROOT / PROFILES[version]).read_text())
    ov = p['case_overrides']['unicycle_bugtrap']                  # family settings (Bugtrap == Wall except the chart)
    geometry = p['geometry'] | ov.get('geometry', {})
    boundary = p['boundary'] | ov.get('boundary', {})
    if kind == 'map':
        model, planning, original, start, goal, _ = load(key, **geometry, lift_speed=boundary['lift_speed'])
    else:
        lo, hi, boxes, start, goal = scene(np.random.default_rng(key), 'unicycle')
        original = World(lo, hi, boxes, [], geometry['radius'])
        planning = original.sphere_cover(geometry['sphere_spacing'], geometry['shell_only'], geometry['shell_keep_boxes'])
        planning.prepare_simd()
        model = Unicycle(); model.lift_speed = boundary['lift_speed']
    model.reverse = chart == 'backward'
    for k, v in (p.get('model', {}) | ov.get('model', {})).items():
        setattr(model, k, v)
    raw = p['settings'] | ov.get('settings', {}) | ov.get('algorithm_settings', {}).get(algorithm, {})
    cfg = PlannerSettings(**(raw | dict(algorithm=algorithm, budget_s=BUDGET)))
    if geometry.get('shell_only') and not geometry.get('shell_keep_boxes', True):
        thickness = original.shell_thickness(geometry['sphere_spacing'])
        if not cfg.validation_spacing < thickness / 1.01:
            raise ValueError(f'validation spacing does not resolve the shell thickness {thickness}')
    return model, planning, original, start, goal, cfg


def attempt(kind, key, chart, version, algorithm, seed):
    from flask_repro.planning import plan, validate
    row = dict(kind=kind, problem=key, chart=chart, version=version, algorithm=algorithm, seed=seed)
    try:
        model, planning, original, start, goal, cfg = problem(kind, key, chart, version, algorithm)
        r = plan(model, planning, start, goal, cfg, seed)
        row.update(solved=bool(r.solved), ms=(r.plan_ns + r.postprocess_ns) / 1e6, reason=r.reason, iterations=r.iterations)
        if r.solved:
            v = validate(r, model, original, cfg)
            row.update(valid=bool(v['valid']), errors=None if v['valid'] else v['errors'],
                       length=float(r.trajectory.length(2)), duration=float(r.trajectory.duration))
    except Exception as e:                                     # a crash is a finding, never a skipped attempt
        row.update(solved=False, crashed=True, error=repr(e), trace=traceback.format_exc()[-1500:])
    return row


def worker(job, cores):
    core = cores.get()
    os.sched_setaffinity(0, {core})
    try:
        kind, key, chart, algorithm, seeds = job
        rows = []
        for seed in seeds:                                     # v1 and v2 back to back on the same core (paired)
            for version in ('v1', 'v2'):
                rows.append(attempt(kind, key if kind == 'map' else seed, chart, version, algorithm, seed) | dict(core=core))
        return rows
    finally:
        cores.put(core)


def jobs():
    out = []
    for chart in ('forward', 'backward'):
        for algorithm in PLANNERS:
            for m in MAPS:
                out.append(('map', m, chart, algorithm, REGISTRY['generalization_v2_maps']))
            random_seeds = REGISTRY['generalization_v2_random']
            for i in range(0, len(random_seeds), 25):             # 4 chunks per cell to balance the workers
                out.append(('random', 'random', chart, algorithm, random_seeds[i:i + 25]))
    return out


def summarize(rows):
    cells = {}
    for r in rows:
        cells.setdefault((r['problem'] if r['kind'] == 'map' else 'random', r['chart'], r['algorithm']), []).append(r)
    table = []
    for (problem_name, chart, algorithm), rs in sorted(cells.items()):
        by = {v: {r['seed']: r for r in rs if r['version'] == v} for v in ('v1', 'v2')}
        stat = {}
        for v in ('v1', 'v2'):
            xs = list(by[v].values())
            stat[v] = dict(attempts=len(xs), solved=sum(x['solved'] for x in xs), crashed=sum(bool(x.get('crashed')) for x in xs),
                           invalid_returns=sum(x['solved'] and not x.get('valid') for x in xs),
                           median_censored_ms=float(np.median([x['ms'] if x['solved'] else BUDGET * 1e3 for x in xs])))
        both = [s for s in by['v1'] if by['v1'][s]['solved'] and by['v2'].get(s, {}).get('solved')]
        for v in ('v1', 'v2'):
            stat[v]['paired_mean_length'] = float(np.mean([by[v][s]['length'] for s in both])) if both else None
        allowance = 2 if problem_name != 'random' else 5
        table.append(dict(problem=problem_name, chart=chart, algorithm=algorithm, paired_solved=len(both), **stat,
                          G2=stat['v2']['solved'] >= stat['v1']['solved'] - allowance,
                          G3_eligible=stat['v1']['solved'] + stat['v2']['solved'] > 0,
                          G3=stat['v2']['median_censored_ms'] < stat['v1']['median_censored_ms']))
    g1 = all(c['v2']['invalid_returns'] == 0 and c['v2']['crashed'] == 0 for c in table)
    g2_fail = [c for c in table if not c['G2']]
    eligible = [c for c in table if c['G3_eligible']]
    g3_share = sum(c['G3'] for c in eligible) / len(eligible) if eligible else 0.
    g3 = g3_share >= .8
    if g1 and not g2_fail and g3:
        verdict = 'GENERALIZES'
    elif not g1 or len(g2_fail) > len(table) / 2 or g3_share <= .5:
        verdict = 'DOES_NOT_GENERALIZE'
    else:
        verdict = 'PARTIAL'
    return dict(rule='config/v2_generalization_rule.json', verdict=verdict, G1=g1, G2_failing_cells=len(g2_fail),
                G3_share=g3_share, G3=g3, cells=table,
                v1_invalid_returns=sum(c['v1']['invalid_returns'] for c in table),
                v1_crashed=sum(c['v1']['crashed'] for c in table))


def markdown(s):
    lines = [f"# v2 generalization check: **{s['verdict']}**", '',
             f"Rule: `{s['rule']}` (declared before running). G1 safety (0 invalid v2 returns, 0 crashes): **{s['G1']}**. "
             f"G2 cells failing (v2 solves fewer than v1 minus allowance): **{s['G2_failing_cells']}**. "
             f"G3 share of cells where v2 is faster (median, unsolved = 10 s): **{100 * s['G3_share']:.0f} %** (need >= 80 %).",
             f"v1 for reference: {s['v1_invalid_returns']} invalid returns, {s['v1_crashed']} crashes.", '',
             '| Problem | Chart | Planner | v1 solved | v2 solved | v1 median ms | v2 median ms | Paired length v1 / v2 (m) | G2 | G3 |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for c in s['cells']:
        L = lambda v: '-' if c[v]['paired_mean_length'] is None else f"{c[v]['paired_mean_length']:.2f}"
        lines.append(f"| {c['problem']} | {c['chart']} | {c['algorithm']} | {c['v1']['solved']}/{c['v1']['attempts']} | "
                     f"{c['v2']['solved']}/{c['v2']['attempts']} | {c['v1']['median_censored_ms']:.2f} | "
                     f"{c['v2']['median_censored_ms']:.2f} | {L('v1')} / {L('v2')} | {'ok' if c['G2'] else 'FAIL'} | "
                     f"{('ok' if c['G3'] else 'no') if c['G3_eligible'] else 'n/a'} |")
    return '\n'.join(lines) + '\n'


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    manager = mp.Manager(); cores = manager.Queue()
    for c in CORES:
        cores.put(c)
    todo = jobs(); rows = []; t0 = time.time()
    with mp.Pool(len(CORES)) as pool, (a.output / 'rows.jsonl').open('w') as log:
        for i, rs in enumerate(pool.imap_unordered(_run, [(j, cores) for j in todo])):
            for r in rs:
                log.write(json.dumps(r) + '\n')
            log.flush(); rows += rs
            print(f'[{i + 1}/{len(todo)}] {time.time() - t0:7.0f} s', flush=True)
    s = summarize(rows)
    (a.output / 'summary.json').write_text(json.dumps(s, indent=1) + '\n')
    (a.output / 'RESULT.md').write_text(markdown(s))
    print(markdown(s))


def _run(arg):
    return worker(*arg)


if __name__ == '__main__':
    main()
