"""flask_v2 lever L7 (Alg. 4 over subdivided edges): factorial on development seeds, selection by
config/v2_shortcut_rule.json (declared before this ran).

Serial, pinned to the measurement CPU. Every returned path is validated on the original model.

    python -m benchmarks.v2_shortcut_factorial --output results/diagnostics/v2_shortcut/factorial
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np

from flask_repro.planning import plan, validate
from flask_repro.profiles import load_profile

ROOT = Path(__file__).resolve().parents[1]
RULE = json.loads((ROOT / 'config/v2_shortcut_rule.json').read_text())
SEEDS = json.loads((ROOT / 'config/seed_registry.json').read_text())['development_v2_shortcut']
GRID = [tuple(g) for g in RULE['grid']['shortcut_subdivisions x shortcut_rounds']]
FAMILIES = {('unicycle', a): ['unicycle_bugtrap', 'unicycle_wall'] for a in ('rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp')}
FAMILIES |= {('quad_r4', a): ['quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window'] for a in ('rrt_connect', 'sst_bvp')}
CAP, GAIN, TIE = 1.5, 0.95, 0.01


def run(case, algorithm, m, rounds):
    s = load_profile(case, algorithm, shortcut_subdivisions=m, shortcut_rounds=rounds)
    rows = []
    for seed in SEEDS:
        r = plan(s.model, s.planning, s.start, s.goal, s.settings, seed)
        row = dict(case=case, algorithm=algorithm, m=m, rounds=rounds, seed=seed, solved=bool(r.solved),
                   ms=(r.plan_ns + r.postprocess_ns) / 1e6, post_ms=r.postprocess_ns / 1e6)
        if r.solved:
            v = validate(r, s.model, s.original, s.settings)
            row.update(valid=bool(v['valid']), length=float(r.trajectory.length(len(s.original.lower))),
                       duration=float(r.trajectory.duration), edges=len(r.trajectory.edges))
        rows.append(row)
    return rows


def stats(rows):
    solved = [r for r in rows if r['solved']]
    return dict(solved=len(solved), valid=sum(r['valid'] for r in solved), solved_seeds=sorted(r['seed'] for r in solved),
                mean_ms=float(np.mean([r['ms'] for r in rows])), mean_post_ms=float(np.mean([r['post_ms'] for r in rows])),
                mean_length=float(np.mean([r['length'] for r in solved])) if solved else None,
                max_length=float(np.max([r['length'] for r in solved])) if solved else None,
                mean_duration=float(np.mean([r['duration'] for r in solved])) if solved else None)


def select(table):
    """table[(case, algorithm)][(m, rounds)] = stats. Applies rule steps 1-5 per (family, algorithm)."""
    out = {}
    for (family, algorithm), cases in FAMILIES.items():
        per = {}
        for g in GRID:
            ok, norm, time = True, [], 0.
            for case in cases:
                base, x = table[case, algorithm][(1, 1)], table[case, algorithm][g]
                eligible = (set(base['solved_seeds']) <= set(x['solved_seeds']) and x['valid'] == x['solved']
                            and x['mean_ms'] <= CAP * base['mean_ms'])
                ok &= eligible
                norm.append(x['mean_length'] / base['mean_length'])
                time += x['mean_ms']
            per[g] = dict(eligible_on_every_case=bool(ok), normalized_length=float(np.mean(norm)), summed_mean_ms=time)
        eligible = {g: v for g, v in per.items() if v['eligible_on_every_case'] and g != (1, 1)}
        choice = (1, 1)
        if eligible:
            best = min(v['normalized_length'] for v in eligible.values())
            near = {g: v for g, v in eligible.items() if v['normalized_length'] <= best + TIE}
            g = min(near, key=lambda g: near[g]['summed_mean_ms'])
            if eligible[g]['normalized_length'] <= GAIN:
                choice = g
        out[f'{family}/{algorithm}'] = dict(choice=list(choice), per_config={f'{g[0]}x{g[1]}': v for g, v in per.items()})
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cpu', type=int, default=json.loads((ROOT / 'config/machine.json').read_text()).get('measurement_cpu', 10))
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    os.sched_setaffinity(0, {a.cpu})
    table = {}
    with (a.output / 'rows.jsonl').open('w') as log:
        for (family, algorithm), cases in FAMILIES.items():
            for case in cases:
                table[case, algorithm] = {}
                for m, rounds in GRID:
                    rows = run(case, algorithm, m, rounds)
                    for r in rows:
                        log.write(json.dumps(r) + '\n')
                    log.flush()
                    table[case, algorithm][(m, rounds)] = s = stats(rows)
                    print(f'{case}/{algorithm} {m}x{rounds}: solved {s["solved"]} valid {s["valid"]} '
                          f'length {s["mean_length"]} time {s["mean_ms"]:.3f} ms', flush=True)
    sel = select(table)
    raw = {f'{c}/{al}': {f'{g[0]}x{g[1]}': v for g, v in t.items()} for (c, al), t in table.items()}
    (a.output / 'selection.json').write_text(json.dumps(dict(rule='config/v2_shortcut_rule.json', seeds=SEEDS,
                                                             selection=sel, stats=raw), indent=1) + '\n')
    for k, v in sel.items():
        print(k, '->', v['choice'])


if __name__ == '__main__':
    main()
