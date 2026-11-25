"""v8 development ablation on seeds 1001-1010 (work counters decide; never distance to the paper).

1. Unicycle stop-turn-go: RRTConnect extension range {0.35, 1.5, 6.0}; all four variants at the chosen range.
2. Aerial pure-sphere shell cover vs filled cover: search work must be identical (equivalent geometry).

Selection rule (declared in the v8 plan before running): among settings solving 10/10 development seeds
with 100% independent original-model validation, choose the least mean iteration count.

Usage: python -m benchmarks.ablation_v8 --output results/diagnostics/ablation_v8 --cpu 3
"""
import argparse
import json
import os
import statistics as st
from pathlib import Path

from flask_repro.dynobench import load
from flask_repro.planning import PlannerSettings, plan, validate

ROOT = Path(__file__).resolve().parents[1]
SEEDS = list(range(1001, 1011))
BASE = json.loads((ROOT / 'config/low_dof_final_v7_order2.json').read_text())


def run(case, settings, rest=False, geometry=None, order=None):
    ov = BASE['case_overrides'][case]
    model, planning, original, start, goal, _ = load(case, **(geometry or BASE['geometry']), lift_speed=1.0)
    if rest:
        model.rest_turns = True
    if order:
        model.order = order
    cfg = PlannerSettings(**(BASE['settings'] | ov['settings'] | settings))
    rows = []
    for seed in SEEDS:
        r = plan(model, planning, start, goal, cfg, seed)
        v = validate(r, model, original, cfg, **({} if rest or case.startswith('unicycle') else dict(allow_reduced_model=True)))
        rows.append(dict(seed=seed, solved=r.solved, valid=v['valid'], errors=v['errors'][:3], iterations=r.iterations,
                         edges=r.rejections.get('edges'), plan_ms=r.plan_ns / 1e6, total_ms=(r.plan_ns + r.postprocess_ns) / 1e6,
                         length=r.trajectory.length(len(original.lower)) if r.solved else None,
                         turn_time_s=(v.get('stop_turn') or {}).get('turn_time_s')))
    ok = [x for x in rows if x['valid']]
    return dict(rows=rows, valid=len(ok), mean_iterations=st.mean(x['iterations'] for x in rows),
                mean_length=st.mean(x['length'] for x in ok) if ok else None,
                median_total_ms_development_only=st.median(x['total_ms'] for x in ok) if ok else None)


def main(args):
    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})
    args.output.mkdir(parents=True, exist_ok=False)
    rest = dict(unicycle_rest_turns=True, rest_sample_probability=1.0, metric_mode=0, speed_min=0.)
    out = dict(seeds=SEEDS, base_profile='config/low_dof_final_v7_order2.json', range={}, variants={}, shell={})
    for case in ['unicycle_wall', 'unicycle_bugtrap']:
        for rng in (0.35, 1.5, 6.0):
            out['range'][f'{case}|{rng}'] = run(case, rest | dict(extension_range=rng, algorithm='rrt_connect'), rest=True)
            print(case, rng, {k: v for k, v in out['range'][f'{case}|{rng}'].items() if k != 'rows'}, flush=True)
    eligible = {}
    for rng in (0.35, 1.5, 6.0):
        rows = [out['range'][f'{c}|{rng}'] for c in ['unicycle_wall', 'unicycle_bugtrap']]
        if all(r['valid'] == len(SEEDS) for r in rows):
            eligible[rng] = sum(r['mean_iterations'] for r in rows)
    chosen = min(eligible, key=eligible.get)
    out['chosen_extension_range'] = chosen
    for case in ['unicycle_wall', 'unicycle_bugtrap']:
        for alg in ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']:
            out['variants'][f'{case}|{alg}'] = run(case, rest | dict(extension_range=chosen, algorithm=alg), rest=True)
            print(case, alg, {k: v for k, v in out['variants'][f'{case}|{alg}'].items() if k != 'rows'}, flush=True)
    for case in ['quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']:
        for alg in ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']:
            full = run(case, dict(algorithm=alg), order=2)
            shell = run(case, dict(algorithm=alg), order=2, geometry=BASE['geometry'] | dict(shell_only=True, shell_keep_boxes=False))
            same = all((a['iterations'], a['edges'], a['solved']) == (b['iterations'], b['edges'], b['solved'])
                       for a, b in zip(full['rows'], shell['rows']))
            out['shell'][f'{case}|{alg}'] = dict(identical_work=same, full=full, shell=shell)
            print(case, alg, 'identical', same, full['median_total_ms_development_only'], shell['median_total_ms_development_only'], flush=True)
    out['all_shell_identical'] = all(v['identical_work'] for v in out['shell'].values())
    (args.output / 'ablation_v8.json').write_text(json.dumps(out, indent=1) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cpu', type=int)
    main(p.parse_args())
