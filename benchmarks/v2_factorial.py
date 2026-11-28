"""flask_v2 Phase 3: lever factorial on the development seeds, selection by config/v2_selection_rule.json.

Runs every configuration of the declared grid serially on the measurement CPU, validates each returned path on the
original model, then applies the declared rule and writes selection.json next to the raw rows.

    python -m benchmarks.v2_factorial --output results/diagnostics/v2_factorial
"""
import argparse
import itertools
import json
import os
from pathlib import Path

import numpy as np

from flask_repro.dynobench import load
from flask_repro.flat_models import Unicycle
from flask_repro.planning import PlannerSettings, plan, validate

ROOT = Path(__file__).resolve().parents[1]
RULE = json.loads((ROOT / 'config/v2_selection_rule.json').read_text())
UNICYCLE = ['unicycle_bugtrap', 'unicycle_wall']
QUADS = ['quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
L3 = [(), (1.5, 2., 3.)]


def grid(family, algorithm):
    """Yields (lever names, settings overrides) for the declared grid."""
    if algorithm == 'rrt_connect':
        for k, m in itertools.product([1, 5, 10, 20], L3):
            yield [n for n, on in (('L1k%d' % k, k > 1), ('L3', bool(m))) if on], dict(parent_candidates=k, duration_multipliers=m)
    elif family == 'unicycle':
        for m, b, prop in itertools.product(L3, [2., 1.], [(0., .3), (.2, 2.)]):
            names = [n for n, on in (('L3', bool(m)), ('L5', b == 1.), ('L6', prop[0] > 0)) if on]
            yield names, dict(duration_multipliers=m, control_bound=b, propagation_min=prop[0], propagation_max=prop[1])
    else:
        for m in L3:
            yield (['L3'] if m else []), dict(duration_multipliers=m)


def case_setup(case):
    profile = json.loads((ROOT / RULE['base_profiles']['unicycle' if case in UNICYCLE else 'quadrotor_order4']).read_text())
    ov = profile.get('case_overrides', {}).get(case, {})
    geometry = profile['geometry'] | ov.get('geometry', {})
    boundary = profile['boundary'] | ov.get('boundary', {})
    model, planning, original, start, goal, _ = load(case, **geometry, lift_speed=boundary['lift_speed'])
    if isinstance(model, Unicycle):
        model.reverse = boundary['reverse_chart']
    for key, value in (profile.get('model', {}) | ov.get('model', {})).items():
        setattr(model, key, value)
    return model, planning, original, start, goal, profile['settings'] | ov.get('settings', {})


def select(rows):
    """Applies rule steps 1-3 to the rows of one (case, algorithm)."""
    by = {}
    for r in rows:
        by.setdefault(r['config'], []).append(r)
    stats = {c: dict(levers=rs[0]['levers'], valid=sum(x['valid'] for x in rs), n=len(rs),
                     mean_ms=float(np.mean([x['ms'] for x in rs])), median_ms=float(np.median([x['ms'] for x in rs])),
                     mean_length=float(np.nanmean([x['length'] if x['length'] is not None else np.nan for x in rs])))
             for c, rs in by.items()}
    best_valid = max(s['valid'] for s in stats.values())
    eligible = {c: s for c, s in stats.items() if s['valid'] == best_valid}
    winner = min(eligible, key=lambda c: eligible[c]['mean_ms'])
    simpler = [c for c, s in eligible.items() if len(s['levers']) < len(eligible[winner]['levers'])
               and s['mean_ms'] <= 1.1 * eligible[winner]['mean_ms']]
    if simpler:
        winner = min(simpler, key=lambda c: (len(stats[c]['levers']), stats[c]['mean_ms']))
    return winner, stats


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cpu', type=int, default=json.loads((ROOT / 'config/machine.json').read_text())['measurement_cpu'])
    a = p.parse_args()
    os.sched_setaffinity(0, {a.cpu})
    a.output.mkdir(parents=True, exist_ok=False)
    jobs = [(c, alg) for c in UNICYCLE for alg in ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']] + \
           [(c, alg) for c in QUADS for alg in ['rrt_connect', 'sst_bvp']]
    rows = []
    with (a.output / 'rows.jsonl').open('w') as log:
        for case, algorithm in jobs:
            family = 'unicycle' if case in UNICYCLE else 'quadrotor_order4'
            model, planning, original, start, goal, base = case_setup(case)
            for levers, extra in grid(family, algorithm):
                cfg = PlannerSettings(**(base | dict(algorithm=algorithm) | extra))
                config = '+'.join(levers) or 'v1'
                for seed in RULE['seeds']:
                    r = plan(model, planning, start, goal, cfg, seed)
                    v = validate(r, model, original, cfg)
                    row = dict(family=family, case=case, algorithm=algorithm, config=config, levers=levers, seed=seed,
                               solved=r.solved, valid=bool(r.solved and v['valid']),
                               ms=(r.plan_ns + r.postprocess_ns) / 1e6,
                               length=r.trajectory.length(len(original.lower)) if r.solved else None,
                               duration=r.trajectory.duration if r.solved else None,
                               iterations=r.iterations, rejections=r.rejections,
                               diagnostics={k: v for k, v in r.diagnostics.items() if not isinstance(v, (list, dict))})
                    rows.append(row)
                    log.write(json.dumps(row) + '\n'); log.flush()
                s = [x for x in rows if x['case'] == case and x['algorithm'] == algorithm and x['config'] == config]
                print(f'{case:17s} {algorithm:11s} {config:14s} valid {sum(x["valid"] for x in s)}/{len(s)} '
                      f'mean {np.mean([x["ms"] for x in s]):9.3f} ms', flush=True)
    per_case = {}
    for case, algorithm in jobs:
        winner, stats = select([x for x in rows if x['case'] == case and x['algorithm'] == algorithm])
        per_case[f'{case}/{algorithm}'] = dict(winner=winner, stats=stats)
    frozen = {}
    for family, cases in (('unicycle', UNICYCLE), ('quadrotor_order4', QUADS)):
        for algorithm in sorted({alg for c, alg in jobs if c in cases}):
            winners = [per_case[f'{c}/{algorithm}']['winner'] for c in cases]
            counts = {w: winners.count(w) for w in winners}
            top = max(counts.values())
            tied = [w for w, n in counts.items() if n == top]
            total = lambda w: sum(per_case[f'{c}/{algorithm}']['stats'][w]['mean_ms'] for c in cases)
            frozen[f'{family}/{algorithm}'] = min(tied, key=total)
    (a.output / 'selection.json').write_text(json.dumps(dict(rule='config/v2_selection_rule.json', per_case=per_case,
                                                             frozen=frozen), indent=1) + '\n')
    print(json.dumps(frozen, indent=1))


if __name__ == '__main__':
    main()
