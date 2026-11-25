"""Revalidate stored polynomial trajectories without replanning or changing timings."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import numpy as np
from flask_repro.dynobench import load
from flask_repro.flat_bvp import FlatEdge
from flask_repro.flat_models import Unicycle
from flask_repro.planning import PlannerSettings,RunRecord,Trajectory,validate
from flask_repro.provenance import manifest,file_hash
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json


def main(args):
    os.sched_setaffinity(0,{args.cpu})
    args.output.mkdir(parents=True,exist_ok=False)
    source_manifest=json.loads((args.source/'manifest.json').read_text())
    profile=source_manifest['configuration']['profile']
    records=[]
    provenance=manifest(dict(purpose='posthoc_original_model_validation_not_replanning',source=str(args.source)),
                        [args.source/'manifest.json'])
    with (args.output/'records.jsonl').open('w') as stream:
        for path in sorted(args.source.glob('*_seed_*.json')):
            old=json.loads(path.read_text())
            override=profile.get('case_overrides',{}).get(old['case'],{})
            model,_,original,start,goal,_=load(old['case'],simd=False,**old['geometry'],lift_speed=old['boundary']['lift_speed'])
            if isinstance(model,Unicycle):model.reverse=old['boundary']['reverse_chart']
            for k,v in (profile.get('model',{})|override.get('model',{})).items():setattr(model,k,v)
            edges=[FlatEdge(np.asarray(e['coefficients']),e['duration'],e['order']) for e in old['trajectory']]
            record=RunRecord(**{k:old[k] for k in ['solved','plan_ns','postprocess_ns','first_solution_ns',
                'iterations','nodes','terminal_error','seed','settings']},trajectory=Trajectory(edges,model.lift(start)),
                start_state=start.tolist(),goal_state=goal.tolist())
            result=validate(record,model,original,PlannerSettings(**old['settings']))
            row=dict(case=old['case'],algorithm=old['algorithm'],seed=old['seed'],solved=old['solved'],
                     original_artifact=str(path),original_sha256=file_hash(path),
                     historical_validation=old['validation'],original_model_validation=result)
            records.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
    summary={}
    for key in sorted({(r['case'],r['algorithm']) for r in records}):
        g=[r for r in records if (r['case'],r['algorithm'])==key]
        summary['/'.join(key)]=dict(attempts=len(g),returned=sum(r['solved'] for r in g),
            historical_valid=sum(r['historical_validation']['valid'] for r in g),
            original_model_valid=sum(r['original_model_validation']['original_model_valid'] for r in g),
            errors=dict(Counter(e for r in g for e in r['original_model_validation']['errors'])))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    provenance['artifact_sha256']={p.name:file_hash(p) for p in args.output.iterdir()}
    (args.output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--cpu',type=int,default=default_cpu())
    main(p.parse_args())
