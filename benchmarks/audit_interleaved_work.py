"""Interleaved fixed-work native scalar/SIMD diagnostic on all six paper cases.

This diagnostic controls search work rather than imposing a CPU-sensitive short
deadline. It is not a new Table I reliability campaign or a paper-speed claim.
Warmups are excluded. Every pair retains failures and verifies the exact search
and polynomial outputs; scalar/vector rejection counters can have different
time/constraint priorities and sample-batch widths.
"""
import argparse
from dataclasses import replace
from datetime import datetime,timezone
import hashlib,json,os,subprocess
from pathlib import Path
import numpy as np
from benchmarks.native_campaign import hardware_snapshot
from flask_repro.dynobench import CASES,load
from flask_repro.flat_models import Unicycle
from flask_repro.planning import PlannerSettings,plan,validate
from flask_repro.provenance import manifest,distribution

ROOT=Path(__file__).resolve().parents[1]


def signature(record):
    values=dict(solved=record.solved,iterations=record.iterations,nodes=record.nodes,
                checked_edges=record.rejections['edges'],trajectory=record.trajectory.to_records())
    return hashlib.sha256(json.dumps(values,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def run(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);os.sched_setaffinity(0,{3})
    for unit in ['flask-corrective-v3-resume.service','flask-mobile-boundary-followup.service']:
        if subprocess.run(['systemctl','--user','is-active','--quiet',unit]).returncode==0:
            raise RuntimeError('A timing campaign/diagnostic service is active')
    profile_path=ROOT/'config/low_dof_measurement_v3.json'
    profile=json.loads(profile_path.read_text());seeds=[2001,2002];methods=['rrt_connect','sst_simd']
    configuration=dict(purpose='interleaved_fixed_work_diagnostic',cases=list(CASES),methods=methods,
        seeds=seeds,iterations=5000,safety_budget_s=300.,cycles=2,order=['simd','scalar','scalar','simd'],
        warmup_iterations=500,claim_scope='Matching capped search workloads, including no-solution outcomes; not full Table I performance or reliability.')
    before=manifest(configuration,[profile_path,ROOT/'config/seed_registry.json']);before['hardware_before']=hardware_snapshot()
    (output/'manifest.json').write_text(json.dumps(before,indent=2)+'\n')
    rows=[];groups=[];to_validate=[]
    with (output/'attempts.jsonl').open('w') as stream:
        for case in CASES:
            override=profile.get('case_overrides',{}).get(case,{})
            geometry=profile['geometry']|override.get('geometry',{})
            boundary=profile['boundary']|override.get('boundary',{})
            model,world,original,start,goal,_=load(case,**geometry,lift_speed=boundary['lift_speed'])
            if isinstance(model,Unicycle):model.reverse=boundary['reverse_chart']
            for k,v in (profile.get('model',{})|override.get('model',{})).items():setattr(model,k,v)
            for method in methods:
                cfg=PlannerSettings(**(profile['settings']|override.get('settings',{})|dict(
                    algorithm=method,max_iterations=5000,budget_s=300.,stop_first=False)))
                # Instruction/data warmup, excluded from the recorded timings.
                for backend in ['simd','scalar']:
                    plan(model,world,start,goal,replace(cfg,max_iterations=500,validation_backend=backend),2000)
                for seed in seeds:
                    reference_signature=None;group_rows=[];paired=[]
                    for cycle in range(2):
                        block=[]
                        for order,backend in enumerate(['simd','scalar','scalar','simd']):
                            conditions=hardware_snapshot()
                            record=plan(model,world,start,goal,replace(cfg,validation_backend=backend),seed)
                            fingerprint=signature(record)
                            if reference_signature is None:reference_signature=fingerprint
                            row=dict(case=case,method=method,seed=seed,cycle=cycle,order=order,backend=backend,
                                solved=record.solved,iterations=record.iterations,nodes=record.nodes,
                                checked_edges=record.rejections['edges'],plan_ns=record.plan_ns,
                                postprocess_ns=record.postprocess_ns,signature=fingerprint,
                                search_and_trajectory_match=fingerprint==reference_signature,hardware_before=conditions)
                            rows.append(row);group_rows.append(row);block.append(row)
                            stream.write(json.dumps(row)+'\n');stream.flush()
                            if cycle==0 and order in [0,1]:to_validate.append((case,method,seed,backend,record,model,original,cfg))
                        vector=[r['plan_ns'] for r in block if r['backend']=='simd']
                        scalar=[r['plan_ns'] for r in block if r['backend']=='scalar']
                        paired.append(float(np.mean(scalar)/np.mean(vector)))
                    groups.append(dict(case=case,method=method,seed=seed,
                        all_work_and_trajectory_equal=all(r['search_and_trajectory_match'] for r in group_rows),
                        solved=group_rows[0]['solved'],iterations=group_rows[0]['iterations'],
                        cycle_scalar_over_simd_ratios=paired))
                    print(json.dumps(groups[-1]),flush=True)
    # Additional validation occurs after all measurements, outside every timer.
    validations=[]
    for case,method,seed,backend,record,model,original,cfg in to_validate:
        result=validate(record,model,original,replace(cfg,validation_backend=backend))
        validations.append(dict(case=case,method=method,seed=seed,backend=backend,solved=record.solved,validation=result))
    after=manifest(configuration)
    changed=[name for name in set(before['source_sha256'])|set(after['source_sha256'])
             if before['source_sha256'].get(name)!=after['source_sha256'].get(name)]
    bad=[r for r in validations if r['solved'] and not r['validation']['original_model_valid']]
    summary=[]
    for case in CASES:
        for method in methods:
            g=[r for r in groups if r['case']==case and r['method']==method]
            summary.append(dict(case=case,method=method,seed_groups=len(g),
                both_backend_solved=sum(r['solved'] for r in g),
                all_work_and_trajectory_equal=all(r['all_work_and_trajectory_equal'] for r in g),
                interleaved_scalar_over_simd=distribution([x for r in g for x in r['cycle_scalar_over_simd_ratios']])))
    result=dict(completed_at=datetime.now(timezone.utc).isoformat(),attempts=len(rows),groups=groups,summary=summary,
        source_changed_during_audit=changed,returned_validation_failures=bad,validation=validations,
        status='passed_work_equivalence' if not changed and not bad and all(r['search_and_trajectory_match'] for r in rows) else 'requires_diagnosis',
        limitations=['Logical CPU affinity only; SMT sibling and OS scheduling not reserved',
            'Interleaving reduces drift and order bias but does not establish a universal acceleration factor',
            'Capped development workloads include failures and do not replace thirty-seed first-solution comparisons',
            'Absolute times must not be normalized to the paper CPU with a single multiplier'],showcase_locked=True)
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    before['completed_attempts']=len(rows);before['source_changed_during_audit']=changed;before['hardware_after']=hardware_snapshot()
    (output/'manifest.json').write_text(json.dumps(before,indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],attempts=len(rows),returned_validation_failures=len(bad),source_changes=changed)),flush=True)
    if result['status']!='passed_work_equivalence':raise AssertionError('Interleaved work audit needs diagnosis')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True,type=Path)
    run(parser.parse_args().output)
