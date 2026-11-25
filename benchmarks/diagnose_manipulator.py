"""Development traces separating manipulator limit and collision rejections.

Velocity-scale profiles are sensitivity experiments, not confirmed author
samplers. The zero-velocity profile restricts the sampled state space.
"""
import argparse
import json
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from flask_repro import mbm,native,robot,trajectory
from flask_repro.provenance import save_manifest


def run(output,environment,indices,iterations,budget,disable_dynamic_domain=False,legacy_steering=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    rb=robot.get_robot('panda');dataset=mbm.load_dataset('panda');records=[]
    for index in indices:
        problem=dataset[environment][index]
        if not problem['valid']:raise ValueError('Use the inventory audit for source-invalid entries')
        ec=mbm.native_environment(problem)
        for scale in (1.,.25,0.):
            settings=native.settings(velocity_sample_scale=scale,max_iterations=iterations,
                                     max_ns=int(budget*1e9),dynamic_domain=not disable_dynamic_domain,
                                     prefix_steering=not legacy_steering)
            begin=perf_counter_ns()
            result=native.plan(rb,ec,problem['start'],problem['goals'],settings,seed=index,
                              limits=native.joint_limits(rb,'published_resolution'))
            wrapper_ns=perf_counter_ns()-begin
            row=dict(purpose='failure_diagnosis_not_final_timing',environment=environment,index=index,
                     seed=index,velocity_sample_scale=scale,
                     sampling_status='restricted_zero_velocity' if scale==0. else 'reconstructed_full_or_bounded_velocity',
                     max_iterations=iterations,budget_s=budget,validation_profile='published_resolution',
                     dynamic_domain=not disable_dynamic_domain,
                     prefix_steering=not legacy_steering,
                     wrapper_ns=wrapper_ns,
                     **{key:result[key] for key in ['solved','plan_ns','shortcut_ns','iterations',
                         'start_nodes','goal_nodes','edge_checks','batch_checks','limit_rejections',
                         'collision_rejections','task_rejections','dynamic_domain_rejections',
                         'steering_failures','accepted_extensions','connection_attempts']})
            if result['solved']:
                row['dense_sample_check']=trajectory.evaluate_flask(result['trajectory'],rb,ec)
                row['all_edges_strictly_accepted']=all(native.module(rb).flask_cc(
                    ec,*[x.tolist() for x in (edge.q0,edge.v0,edge.q1,edge.v1)],edge.T,
                    native.joint_limits(rb,'strict_development')) for edge in result['trajectory'].edges())
                path=output/f'{environment}_{index}_velocity_{scale:g}.npz'
                np.savez_compressed(path,**{key:result[key] for key in ['Q','V','T','raw_positions','raw_velocities','raw_durations']})
            records.append(row);(output/'records.json').write_text(json.dumps(records,indent=2)+'\n')
            print(json.dumps(row),flush=True)
    save_manifest(output/'records.json',dict(purpose='failure_diagnosis_not_final_timing',
                  environment=environment,indices=indices,max_iterations=iterations,budget_s=budget,
                  dynamic_domain=not disable_dynamic_domain,prefix_steering=not legacy_steering),
                  [mbm.VAMP_ROOT/'resources/panda/problems.pkl'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--environment',default='cage',choices=mbm.ENVIRONMENTS)
    parser.add_argument('--indices',nargs='+',type=int,default=[0,20,40,60,80,99])
    parser.add_argument('--iterations',type=int,default=3000)
    parser.add_argument('--budget',type=float,default=1.)
    parser.add_argument('--disable-dynamic-domain',action='store_true')
    parser.add_argument('--legacy-steering',action='store_true')
    args=parser.parse_args();run(args.output,args.environment,args.indices,args.iterations,args.budget,args.disable_dynamic_domain,args.legacy_steering)
