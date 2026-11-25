"""Diagnose the unreported mobile boundary-control and flatness-chart choices.

The paper's Eq. (10) permits either fixed direction chart. These profiles do
not infer which chart or nonzero boundary speed the authors used. No chart
switch, zero-speed crossing, limit relaxation, or scene modification is made.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter_ns

from flask_repro.dynobench import load
from flask_repro.planning import PlannerSettings, plan, validate
from flask_repro.provenance import save_manifest


def run(output, speeds, seeds, budget, iterations, metric_weight):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    records=[]
    for case in ('unicycle_bugtrap','unicycle_wall'):
        for speed in speeds:
            for reverse in (False,True):
                model,planning,original,start,goal,provenance=load(case,lift_speed=speed)
                model.reverse=reverse
                for seed in seeds:
                    settings=PlannerSettings(budget_s=budget,max_iterations=iterations,
                                             derivative_metric_weight=metric_weight)
                    begin=perf_counter_ns();result=plan(model,planning,start,goal,settings,seed)
                    wrapper_ns=perf_counter_ns()-begin
                    validation=validate(result,model,original,settings)
                    record=dict(purpose='unpublished_boundary_sensitivity_not_confirmed_reproduction',
                                case=case,seed=seed,boundary_speed=speed,reverse_chart=reverse,
                                settings=asdict(settings),input_provenance=provenance,
                                solved=result.solved,validation=validation,
                                plan_ns=result.plan_ns,postprocess_ns=result.postprocess_ns,
                                wrapper_ns=wrapper_ns,iterations=result.iterations,nodes=result.nodes,
                                rejections=result.rejections,edges=result.trajectory.to_records())
                    records.append(record);(output/'records.json').write_text(json.dumps(records,indent=2)+'\n')
                    print(json.dumps(dict(case=case,speed=speed,reverse=reverse,seed=seed,
                                          solved=result.solved,valid=validation['valid'],
                                          iterations=result.iterations)),flush=True)
    save_manifest(output/'records.json',dict(purpose='unpublished_boundary_sensitivity_not_confirmed_reproduction',
                  speeds=speeds,seeds=seeds,budget_s=budget,max_iterations=iterations,
                  derivative_metric_weight=metric_weight),[Path('docs/sources/flask_v4.pdf')])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--speeds',nargs='+',type=float,default=[.03,.1,.3,.6])
    parser.add_argument('--seeds',nargs='+',type=int,default=[0,1])
    parser.add_argument('--budget',type=float,default=3.)
    parser.add_argument('--iterations',type=int,default=20000)
    parser.add_argument('--metric-weight',type=float,default=.2)
    args=parser.parse_args();run(args.output,args.speeds,args.seeds,args.budget,args.iterations,args.metric_weight)
