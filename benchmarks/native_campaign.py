"""Checkpointed native low-DOF experiments with independent validation.

Rendering and simulation are never imported. Development runs are explicitly
separate from a final campaign. The latter requires the complete registered
seed list and single-CPU affinity, and rejects a changed source fingerprint.
"""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import json
import os
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from flask_repro.dynobench import CASES,load,ROOT as DATASET_ROOT
from flask_repro.flat_models import Unicycle
from flask_repro.planning import ALGORITHMS,PlannerSettings,plan,validate
from flask_repro.provenance import distribution,file_hash,manifest
from flask_repro.scales import rest_propagation_scale

ROOT=Path(__file__).resolve().parents[1]


def configuration_diagnostics(model,start,cfg):
    """Report a sufficient obstruction to SST's initial root expansion.

    A positive-cost candidate inside the zero-cost root witness's pruning ball
    cannot replace its representative. SST* can eventually shrink that ball;
    therefore this is a phase diagnostic, not a proof of permanent failure.
    """
    if cfg.algorithm not in ('sst_bvp','sst_simd') or cfg.metric_mode!=0:
        return dict(rest_root_scale_applicable=False)
    lifted=np.asarray(model.lift(start),float)
    if np.any(lifted[1:]!=0.):
        return dict(rest_root_scale_applicable=False)
    bounds=cfg.control_bounds or (cfg.control_bound,)*model.outputs
    scale=rest_propagation_scale(model.order,bounds,cfg.propagation_min,
                                 cfg.propagation_max,cfg.derivative_metric_weight)
    return dict(rest_root_scale_applicable=True,
        maximum_initial_step_metric=scale.maximum,rms_initial_step_metric=scale.rms,
        initial_phase_blocked_by_radius=bool(np.nextafter(scale.maximum,np.inf)<cfg.witness_radius),
        initial_witness_radius=cfg.witness_radius,
        scope='Euclidean metric, zero initial derivatives, unconditional proposal box; later SST-star phases shrink the radius')


def hardware_snapshot():
    values={}
    for pattern in ('/sys/class/thermal/thermal_zone*/temp',
                    '/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq'):
        import glob
        for name in glob.glob(pattern):
            try:values[name]=Path(name).read_text().strip()
            except OSError:pass
    from benchmarks.measurement_env import check,snapshot
    environment=snapshot(sorted(os.sched_getaffinity(0))[0],load_seconds=1.)
    return dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),
                affinity=sorted(os.sched_getaffinity(0)),sensors=values,
                environment=environment,environment_problems=check(environment))


def summarize(records):
    groups={}
    for row in records:
        key=row['case']+'/'+row['algorithm'];groups.setdefault(key,[]).append(row)
    result={}
    for key,rows in groups.items():
        good=[r for r in rows if r['validation']['valid']];n=len(rows);k=len(good)
        z=1.959963984540054;fraction=k/n;denominator=1+z*z/n
        center=(fraction+z*z/(2*n))/denominator
        half=z*np.sqrt(fraction*(1-fraction)/n+z*z/(4*n*n))/denominator
        result[key]=dict(attempts=n,returned_solutions=sum(r['solved'] for r in rows),
            validated_solutions=k,validated_success_rate=fraction,
            success_rate_wilson_95=[max(0.,center-half),min(1.,center+half)],
            all_attempt_planning_ms=distribution([r['plan_ns']/1e6 for r in rows]),
            valid_solution_planning_ms=distribution([r['plan_ns']/1e6 for r in good]),
            valid_solution_total_ms=distribution([(r['plan_ns']+r['postprocess_ns'])/1e6 for r in good]),
            valid_solution_length_m=distribution([r['length_m'] for r in good]))
    return result


def run(args):
    profile=json.loads(args.profile.read_text())
    registry=json.loads((ROOT/'config/seed_registry.json').read_text())
    seeds=args.seeds if args.seeds is not None else registry[profile.get('seed_set','low_dof')]
    if not seeds or len(set(seeds))!=len(seeds) or min(seeds)<0:raise ValueError('Seeds must be nonempty, unique, and nonnegative')
    if args.cpu is not None:os.sched_setaffinity(0,{args.cpu})
    if args.final:
        if seeds!=registry[profile.get('seed_set','low_dof')] or len(os.sched_getaffinity(0))!=1:
            raise ValueError('Final campaigns require the full seed registry and single-CPU affinity')
        if profile.get('frozen') is not True:
            raise ValueError('Final campaigns require an explicitly frozen reviewed profile')
    args.output.mkdir(parents=True,exist_ok=False)
    configuration=dict(profile=profile,seeds=seeds,cases=args.cases,algorithms=args.algorithms,
        campaign_class='final_controlled' if args.final else args.campaign_class,
        isolation_note='Process affinity alone does not establish an isolated core; competing processes and SMT siblings must be reviewed separately')
    input_files=[args.profile,ROOT/'config/seed_registry.json',ROOT/'docs/sources/flask_v4.pdf']
    input_files += [DATASET_ROOT/'envs'/CASES[case][0] for case in args.cases]
    provenance=manifest(configuration,input_files)
    provenance['native_binary_sha256']={str(p.relative_to(ROOT)):file_hash(p) for p in (ROOT/'flask_repro').glob('_core*.so')}
    provenance['hardware_before']=hardware_snapshot()
    (args.output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    records=[]
    for case in args.cases:
        override=profile.get('case_overrides',{}).get(case,{})
        geometry=profile['geometry']|override.get('geometry',{})
        boundary=profile['boundary']|override.get('boundary',{})
        model,planning,original,start,goal,input_provenance=load(case,**geometry,lift_speed=boundary['lift_speed'])
        if geometry.get('shell_only') and not geometry.get('shell_keep_boxes',True):
            # Pure-sphere shell: refuse unless every validator step is shorter than the shell layer.
            thickness=original.shell_thickness(geometry.get('sphere_spacing',.15))
            for algorithm in args.algorithms:
                spacing=(profile['settings']|override.get('settings',{})).get('validation_spacing',.015)
                if not spacing<thickness/1.01:
                    raise ValueError(f'{case}: validation spacing {spacing} does not resolve shell thickness {thickness}')
            input_provenance['shell_thickness_m']=thickness
        if boundary.get('rest_turns'):
            model.rest_turns=True
        if isinstance(model,Unicycle):model.reverse=boundary['reverse_chart']
        for key,value in (profile.get('model',{})|override.get('model',{})).items():setattr(model,key,value)
        input_provenance['derivative_order']=model.order
        input_provenance['dynamics']=f'Flatness reconstruction with integrator-chain order {model.order}'
        for algorithm in args.algorithms:
            # flask_v2: optional per-planner settings (levers frozen per algorithm by results/diagnostics/v2_factorial)
            cfg=PlannerSettings(**(profile['settings']|override.get('settings',{})|override.get('algorithm_settings',{}).get(algorithm,{})|dict(algorithm=algorithm)))
            diagnostics=configuration_diagnostics(model,start,cfg)
            repeats=int(profile.get('timing_repeats',1))
            for seed in seeds:
                begin=perf_counter_ns();record=plan(model,planning,start,goal,cfg,seed)
                wrapper_ns=perf_counter_ns()-begin
                # v9: identical-work repetitions of the same seed; the median time damps sub-millisecond
                # scheduling noise. The first run stays the primary record; work must match exactly.
                repeat_plan=[record.plan_ns];repeat_post=[record.postprocess_ns];repeat_identical=True
                for _ in range(repeats-1):
                    again=plan(model,planning,start,goal,cfg,seed)
                    repeat_identical&=(again.iterations,again.rejections,again.solved,len(again.trajectory.edges))==\
                        (record.iterations,record.rejections,record.solved,len(record.trajectory.edges))
                    repeat_plan.append(again.plan_ns);repeat_post.append(again.postprocess_ns)
                begin=perf_counter_ns();validation=validate(record,model,original,cfg)
                if getattr(model,'order',4)<4 and hasattr(model,'gravity'):
                    # Paper-protocol (reduced pseudo-control order) check, reported beside the full-model result.
                    validation['protocol_validation']=validate(record,model,original,cfg,allow_reduced_model=True)
                validation_ns=perf_counter_ns()-begin
                length=record.trajectory.length(len(original.lower)) if record.solved else None
                row=dict(case=case,algorithm=algorithm,seed=seed,solved=record.solved,
                    settings=asdict(cfg),input_provenance=input_provenance,
                    boundary=boundary,geometry=geometry,validation=validation,
                    plan_ns=record.plan_ns,postprocess_ns=record.postprocess_ns,
                    first_solution_ns=record.first_solution_ns,wrapper_ns=wrapper_ns,
                    independent_validation_ns=validation_ns,iterations=record.iterations,
                    nodes=record.nodes,terminal_error=record.terminal_error,
                    failure_reason=record.reason if not record.solved else None,
                    duration_s=record.trajectory.duration if record.solved else None,
                    length_m=length,rejections=record.rejections,
                    search_diagnostics=record.diagnostics,
                    configuration_diagnostics=diagnostics,
                    trajectory=record.trajectory.to_records(),
                    timing_repeats=dict(count=repeats,identical_work=repeat_identical,plan_ns=repeat_plan,postprocess_ns=repeat_post,
                        median_total_ns=float(np.median(np.add(repeat_plan,repeat_post))),
                        median_plan_ns=float(np.median(repeat_plan))))
                records.append(row)
                (args.output/f'{case}_{algorithm}_seed_{seed:04d}.json').write_text(json.dumps(row,indent=2)+'\n')
                (args.output/'summary.json').write_text(json.dumps(summarize(records),indent=2)+'\n')
                print(json.dumps(dict(case=case,algorithm=algorithm,seed=seed,solved=record.solved,
                    valid=validation['valid'],plan_ms=record.plan_ns/1e6,iterations=record.iterations)),flush=True)
    after=manifest(configuration)
    changed=[name for name in set(provenance['source_sha256'])|set(after['source_sha256'])
             if after['source_sha256'].get(name)!=provenance['source_sha256'].get(name)]
    provenance['hardware_after']=hardware_snapshot();provenance['source_changed_during_campaign']=changed
    provenance['completed_attempts']=len(records)
    provenance['attempt_sha256']={p.name:file_hash(p) for p in args.output.glob('*_seed_*.json')}
    (args.output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    if changed:raise RuntimeError('Source changed during campaign; results are not accepted as a frozen campaign')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--profile',type=Path,default=ROOT/'config/native_low_dof_development.json')
    parser.add_argument('--cases',nargs='+',choices=list(CASES),default=list(CASES))
    parser.add_argument('--algorithms',nargs='+',choices=list(ALGORITHMS),default=list(ALGORITHMS))
    parser.add_argument('--seeds',nargs='+',type=int)
    parser.add_argument('--cpu',type=int)
    parser.add_argument('--final',action='store_true')
    parser.add_argument('--campaign-class',choices=['development_not_final_timing','paper_case_reconstruction_measurement'],default='development_not_final_timing')
    run(parser.parse_args())
