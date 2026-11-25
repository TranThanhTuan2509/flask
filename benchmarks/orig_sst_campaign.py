"""Supplementary first-party original-coordinate SST* reconstruction campaign.

This does not execute the original Dynoplan/OMPL implementation used for Table I.

Same scenes, sphere geometry, workspace, limits and DynoBench goal regions as the FLASK low-DOF
campaign. Anytime protocol of the paper: report the first-solution time and the length of the best
(final) solution within the budget (180 s). Every final solution is independently re-integrated and
validated against the original boxes (flask_repro.original_sst.validate).

Usage: python -m benchmarks.orig_sst_campaign --output DIR --profile config/orig_sst_measurement_v2.json --cpu 3
"""
import argparse
import json
import os
from pathlib import Path

from flask_repro import original_sst as O
from flask_repro.dynobench import CASES, load
from flask_repro.provenance import distribution, manifest
from benchmarks.native_campaign import hardware_snapshot
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

ROOT = Path(__file__).resolve().parents[1]


def main(args):
    os.sched_setaffinity(0, {args.cpu})
    profile = json.loads(args.profile.read_text())
    seeds = args.seeds or profile['seeds']
    args.output.mkdir(parents=True, exist_ok=False)
    provenance = manifest(dict(profile=profile, seeds=seeds, cases=args.cases), [args.profile])
    provenance['hardware_before'] = hardware_snapshot()
    (args.output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')
    rows = []
    with (args.output / 'attempts.jsonl').open('w') as stream:
        for case in args.cases:
            o = profile.get('case_overrides', {}).get(case, {})
            geometry = profile['geometry'] | o.get('geometry', {})
            _, planning, original, start, goal, _ = load(case, **geometry)
            settings = O.OrigSettings(**(profile['settings'] | o.get('settings', {})))
            for seed in seeds:
                r = O.plan(case, planning, start, goal, settings, seed)
                v = O.validate(case, r, original, start, goal)
                row = dict(case=case, seed=seed,method='reconstructed_original_coordinate_sst_star',
                           upstream_baseline=False,solved=r['solved'], valid=v['valid'], errors=v['errors'],
                           first_ms=r['first_ns'] / 1e6 if r['solved'] else None, plan_ms=r['plan_ns'] / 1e6,
                           iterations=r['iterations'], nodes=r['nodes'], best_cost=r['best_cost'] if r['solved'] else None,
                           final_length=v['length'], final_duration=v['duration'],
                           improvements=r['improvements'], controls=r['controls'], steps=r['steps'])
                rows.append(row)
                stream.write(json.dumps(row) + '\n')
                stream.flush()
                print(json.dumps({k: row[k] for k in ['case', 'seed', 'solved', 'valid', 'first_ms', 'final_length']}), flush=True)
    summary = {}
    for case in args.cases:
        g = [r for r in rows if r['case'] == case]
        ok = [r for r in g if r['valid']]
        summary[case] = dict(attempts=len(g), valid=len(ok), first_ms=distribution([r['first_ms'] for r in ok]),
                             final_length=distribution([r['final_length'] for r in ok]))
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    after = manifest(dict(profile=profile, seeds=seeds, cases=args.cases))
    provenance.update(hardware_after=hardware_snapshot(), completed_attempts=len(rows),
                      source_changed_during_campaign=[k for k in set(after['source_sha256']) | set(provenance['source_sha256'])
                                                      if after['source_sha256'].get(k) != provenance['source_sha256'].get(k)])
    (args.output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--profile', type=Path, required=True)
    p.add_argument('--cases', nargs='+', default=list(CASES))
    p.add_argument('--seeds', nargs='+', type=int)
    p.add_argument('--cpu', type=int, default=default_cpu())
    main(p.parse_args())
