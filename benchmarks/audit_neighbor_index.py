"""Interleaved scan/index comparisons with identical native search work.

All six paper-case candidates and four FLASK variants are retained. This is a
capped-work diagnostic, not a replacement first-solution reliability campaign.
"""
import argparse
from dataclasses import replace
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import numpy as np

from benchmarks.audit_interleaved_work import signature
from benchmarks.native_campaign import hardware_snapshot
from flask_repro.dynobench import CASES,load
from flask_repro.flat_models import Unicycle
from flask_repro.planning import ALGORITHMS,PlannerSettings,plan,validate
from flask_repro.provenance import distribution,manifest

ROOT=Path(__file__).resolve().parents[1]


def run(output,iterations=20000,factor='indexed_search',cases=None,algorithms=None,profile_path=None):
    if factor not in ['indexed_search','early_constraint_screening'] or iterations<=0:
        raise ValueError('A supported factor and positive iteration cap are required')
    cases=cases or list(CASES);algorithms=algorithms or list(ALGORITHMS)
    output=Path(output);output.mkdir(parents=True,exist_ok=False);os.sched_setaffinity(0,{3})
    profile_path=Path(profile_path) if profile_path is not None else ROOT/'config/low_dof_stability_v4.json'
    profile=json.loads(profile_path.read_text())
    configuration=dict(purpose='interleaved_native_neighbor_index_audit',profile=profile,
        factor=factor,cases=cases,algorithms=algorithms,iterations=iterations,seeds=[5001,5002],cycles=2,order=['disabled','enabled','enabled','disabled'],
        safety_budget_s=300.,timing='Native planning only; postprocessing and independent validation separate')
    before=manifest(configuration,[profile_path]);before['hardware_before']=hardware_snapshot()
    (output/'manifest.json').write_text(json.dumps(before,indent=2)+'\n')
    rows=[];groups=[];validation_jobs=[]
    with (output/'attempts.jsonl').open('w') as stream:
        for case in cases:
            override=profile['case_overrides'][case];boundary=profile['boundary']|override.get('boundary',{})
            model,world,original,start,goal,_=load(case,**profile['geometry'],lift_speed=boundary['lift_speed'])
            if isinstance(model,Unicycle):model.reverse=boundary['reverse_chart']
            for name,value in (profile.get('model',{})|override.get('model',{})).items():setattr(model,name,value)
            for algorithm in algorithms:
                cfg=PlannerSettings(**(profile['settings']|override.get('settings',{})|dict(
                    algorithm=algorithm,max_iterations=iterations,budget_s=300.,stop_first=False)))
                if factor=='early_constraint_screening':cfg=replace(cfg,indexed_search=True)
                for enabled in [False,True]:
                    plan(model,world,start,goal,replace(cfg,**{factor:enabled},max_iterations=500),5000)
                for seed in [5001,5002]:
                    expected=None;ratios=[];group=[]
                    for cycle in range(2):
                        block=[]
                        for order,enabled in enumerate([False,True,True,False]):
                            conditions=hardware_snapshot()
                            record=plan(model,world,start,goal,replace(cfg,**{factor:enabled}),seed)
                            digest=signature(record)
                            if expected is None:expected=digest
                            row=dict(case=case,algorithm=algorithm,seed=seed,cycle=cycle,order=order,
                                factor=factor,enabled=enabled,solved=record.solved,iterations=record.iterations,nodes=record.nodes,
                                checked_edges=record.rejections['edges'],plan_ns=record.plan_ns,postprocess_ns=record.postprocess_ns,
                                explicit_metric_evaluations=record.diagnostics['exact_metric_evaluations'],
                                signature=digest,work_and_trajectory_match=digest==expected,hardware_before=conditions)
                            rows.append(row);group.append(row);block.append(row)
                            stream.write(json.dumps(row)+'\n');stream.flush()
                            if cycle==0 and order in (0,1):validation_jobs.append((case,algorithm,seed,enabled,record,model,original,cfg))
                        ratios.append(float(np.mean([r['plan_ns'] for r in block if not r['enabled']])/
                                            np.mean([r['plan_ns'] for r in block if r['enabled']])))
                    result=dict(case=case,algorithm=algorithm,seed=seed,solved=group[0]['solved'],
                        all_work_and_trajectory_equal=all(r['work_and_trajectory_match'] for r in group),cycle_disabled_over_enabled_ratios=ratios)
                    groups.append(result);print(json.dumps(result),flush=True)
                    if not result['all_work_and_trajectory_equal']:
                        raise AssertionError('Indexed search changed a search/trajectory signature; retain partial evidence for diagnosis')
    validations=[]
    for case,algorithm,seed,enabled,record,model,original,cfg in validation_jobs:
        checked=validate(record,model,original,cfg)
        validations.append(dict(case=case,algorithm=algorithm,seed=seed,factor=factor,enabled=enabled,solved=record.solved,validation=checked))
    after=manifest(configuration)
    changed=[name for name in set(before['source_sha256'])|set(after['source_sha256'])
             if before['source_sha256'].get(name)!=after['source_sha256'].get(name)]
    invalid=[r for r in validations if r['solved'] and not r['validation']['valid']]
    summary=[]
    for case in cases:
        for algorithm in algorithms:
            selected=[r for r in groups if r['case']==case and r['algorithm']==algorithm]
            summary.append(dict(case=case,algorithm=algorithm,
                disabled_over_enabled=distribution([ratio for r in selected for ratio in r['cycle_disabled_over_enabled_ratios']]),
                solved_seed_groups=sum(r['solved'] for r in selected)))
    result=dict(completed_at=datetime.now(timezone.utc).isoformat(),attempts=len(rows),groups=groups,summary=summary,
        source_changed_during_audit=changed,returned_validation_failures=invalid,
        status='passed_work_equivalence' if not changed and not invalid else 'requires_diagnosis',
        limitations=['Capped workloads include no-solution trials','Logical CPU affinity does not isolate the SMT sibling',
                    'Explicit metric counter excludes index-internal coordinate-distance operations','No universal hardware scaling factor'],
        showcase_locked=True)
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    before.update(completed_attempts=len(rows),source_changed_during_audit=changed,hardware_after=hardware_snapshot())
    (output/'manifest.json').write_text(json.dumps(before,indent=2)+'\n')
    if result['status']!='passed_work_equivalence':raise AssertionError('Index audit requires diagnosis')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--iterations',type=int,default=20000)
    parser.add_argument('--factor',choices=['indexed_search','early_constraint_screening'],default='indexed_search')
    parser.add_argument('--cases',nargs='+',choices=list(CASES))
    parser.add_argument('--algorithms',nargs='+',choices=list(ALGORITHMS))
    parser.add_argument('--profile',type=Path)
    args=parser.parse_args();run(args.output,args.iterations,args.factor,args.cases,args.algorithms,args.profile)
