"""Development ablation of reconstructed low-DOF (Table I) settings. Development seeds only.

Every returned trajectory is independently validated (flask_repro.planning.validate). Timings are
success-conditioned and shown with success counts. Not a frozen measurement campaign.

Usage: python -m benchmarks.ablation_low_dof --output DIR --configs FILE.json --seeds 1001 1002 [--cpu 3]
Config file: list of {"name", "cases": [...], "algorithms": [...], "settings": {...PlannerSettings...},
                      "boundary": {"lift_speed": float, "reverse_chart": bool}, "geometry": {...},
                      "case_overrides": {case: {"settings": {}, "boundary": {}}}}
"""
import argparse
import json
import os
from dataclasses import asdict
from pathlib import Path

import numpy as np

from flask_repro.dynobench import CASES, load
from flask_repro.flat_models import Unicycle
from flask_repro.planning import PlannerSettings, plan, validate
from flask_repro.provenance import manifest
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

PAPER = {'unicycle_bugtrap': dict(rrt_connect=6.0, sst_bvp=48.6, sst_simd=407, rrt_dp=565),
         'unicycle_wall': dict(rrt_connect=0.6, sst_bvp=11.0, sst_simd=30, rrt_dp=25),
         'quad2d_bugtrap': dict(rrt_connect=2.7, sst_bvp=45.0, sst_simd=388, rrt_dp=909),
         'quad2d_hole': dict(rrt_connect=0.4, sst_bvp=270, sst_simd=1813, rrt_dp=2133),
         'quad3d_block': dict(rrt_connect=0.9, sst_bvp=1.4, sst_simd=4752, rrt_dp=7602),
         'quad3d_window': dict(rrt_connect=0.5, sst_bvp=22.6, sst_simd=629, rrt_dp=2631)}


def main(args):
    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})
    args.output.mkdir(parents=True, exist_ok=True)
    configs = json.loads(args.configs.read_text())
    (args.output / 'manifest.json').write_text(json.dumps(manifest(dict(configs=configs, seeds=args.seeds,
        study='development_ablation_not_final_timing')), indent=2) + '\n')
    for cfg in configs:
        if 'base_profile' in cfg:  # merge a frozen profile, then the config's own overrides
            base = json.loads(Path(cfg['base_profile']).read_text())
            merged = dict(geometry=base.get('geometry', {}) | cfg.get('geometry', {}),
                          boundary=base.get('boundary', {}) | cfg.get('boundary', {}),
                          settings=base.get('settings', {}) | cfg.get('settings', {}), model=cfg.get('model', {}))
            co = {}
            for case in set(base.get('case_overrides', {})) | set(cfg.get('case_overrides', {})):
                b, c = base.get('case_overrides', {}).get(case, {}), cfg.get('case_overrides', {}).get(case, {})
                co[case] = {k: b.get(k, {}) | c.get(k, {}) for k in ['geometry', 'boundary', 'settings', 'model']}
            cfg = cfg | merged | dict(case_overrides=co)
        rows = []
        for case in cfg.get('cases', list(CASES)):
            o = cfg.get('case_overrides', {}).get(case, {})
            geometry = dict(radius=.1, sphere_spacing=.15) | cfg.get('geometry', {}) | o.get('geometry', {})
            boundary = dict(lift_speed=.3, reverse_chart=False) | cfg.get('boundary', {}) | o.get('boundary', {})
            model, planning, original, start, goal, _ = load(case, **geometry, lift_speed=boundary['lift_speed'])
            if isinstance(model, Unicycle):
                model.reverse = boundary['reverse_chart']
            for key, value in (cfg.get('model', {}) | o.get('model', {})).items():
                setattr(model, key, value)
            for algorithm in cfg.get('algorithms', ['rrt_connect']):
                settings = PlannerSettings(**(cfg.get('settings', {}) | o.get('settings', {}) | dict(algorithm=algorithm)))
                for seed in args.seeds:
                    record = plan(model, planning, start, goal, settings, seed)
                    v = validate(record, model, original, settings)
                    reduced = validate(record, model, original, settings, allow_reduced_model=True)['valid'] \
                        if getattr(model, 'order', 4) < 4 and hasattr(model, 'gravity') else v['valid']
                    rows.append(dict(case=case, algorithm=algorithm, seed=seed, solved=record.solved, valid=v['valid'],
                                     reduced_order=v.get('reduced_order'), protocol_valid=bool(reduced),
                                     nodes=record.nodes,
                                     errors=v['errors'][:3], plan_ms=record.plan_ns / 1e6, post_ms=record.postprocess_ns / 1e6,
                                     first_ms=record.first_solution_ns / 1e6, iterations=record.iterations,
                                     length=record.trajectory.length(len(original.lower)) if record.solved else None,
                                     rejections=record.rejections))
        (args.output / f"{cfg['name']}.jsonl").write_text(''.join(json.dumps(r) + '\n' for r in rows))
        print(f"== {cfg['name']}", flush=True)
        for case in dict.fromkeys(r['case'] for r in rows):
            for algorithm in dict.fromkeys(r['algorithm'] for r in rows if r['case'] == case):
                g = [r for r in rows if r['case'] == case and r['algorithm'] == algorithm]
                ok = [r for r in g if r['protocol_valid']]
                full = sum(r['valid'] for r in g)
                work = np.median([r['iterations'] for r in g]); edges = np.median([r['rejections'].get('edges', 0) for r in g])
                t = np.median([r['plan_ms'] + r['post_ms'] for r in ok]) if ok else float('nan')
                L = np.mean([r['length'] for r in ok]) if ok else float('nan')
                errs = sorted({e for r in g if r['solved'] and not r['valid'] for e in r['errors']})
                print(f"  {case:17s} {algorithm:12s} valid {len(ok)}/{len(g)} (full-model {full}) solved {sum(r['solved'] for r in g)} it50 {work:.0f} edges50 {edges:.0f}"
                      f"  median {t:9.2f} ms (paper {PAPER[case][algorithm]})  len {L:6.2f}"
                      + (f"  invalid:{errs}" if errs else ''), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--configs', type=Path, required=True)
    p.add_argument('--seeds', type=int, nargs='+', default=[1001, 1002, 1003])
    p.add_argument('--cpu', type=int, default=default_cpu())
    main(p.parse_args())
