"""Record deterministic native unicycle search traces before sampler changes."""
import argparse,json,time
from pathlib import Path
from dataclasses import asdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from flask_repro.dynobench import load
from flask_repro.planning import plan,validate,PlannerSettings
from flask_repro.provenance import save_manifest
ROOT=Path(__file__).resolve().parents[1]

def run(output,budget=2.,seed=0,lift_speed=.3):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);records=[]
    for case in ('unicycle_bugtrap','unicycle_wall'):
        model,world,original,start,goal,provenance=load(case,lift_speed=lift_speed)
        settings=PlannerSettings(budget_s=budget,trace=True)
        begin=time.perf_counter_ns();result=plan(model,world,start,goal,settings,seed);wrapper_ns=time.perf_counter_ns()-begin
        validation=validate(result,model,original,settings) if result.solved else dict(valid=False,errors=['no_solution'])
        traces={}
        fig,(ax,hist)=plt.subplots(1,2,figsize=(12,5))
        for center,size in original.boxes:
            center=np.array(center);size=np.array(size);ax.add_patch(plt.Rectangle(center-size/2,*size,color='#73828a'))
        for tree,color in [('start','#16849b'),('goal','#d98243')]:
            states=np.array(result.diagnostics[tree+'_states']).reshape(-1,4);parents=np.array(result.diagnostics[tree+'_parents'])
            speeds=np.linalg.norm(states[:,2:],axis=1);traces[tree+'_states']=states;traces[tree+'_parents']=parents
            indices=np.linspace(0,len(states)-1,min(len(states),750)).astype(int)
            ax.quiver(states[indices,0],states[indices,1],states[indices,2],states[indices,3],color=color,angles='xy',scale_units='xy',scale=3,alpha=.5,label=tree.title()+' tree')
            hist.hist(speeds,bins=np.linspace(0,1.05,40),histtype='step',lw=1.7,color=color,label=tree.title()+' tree')
        np.savez_compressed(output/(case+'_trace.npz'),**traces)
        ax.scatter([start[0],goal[0]],[start[1],goal[1]],s=50,c=['#16849b','#d98243'],edgecolor='black');ax.set(xlim=original.lower[[0]][0:1].tolist()+original.upper[[0]][0:1].tolist(),ylim=[original.lower[1],original.upper[1]],aspect='equal',xlabel='x (m)',ylabel='y (m)');ax.legend()
        hist.set(xlabel='Flat speed magnitude (m/s)',ylabel='Allocated nodes');hist.legend();fig.suptitle(case.replace('_',' ').title()+f' | solved={result.solved}; independently valid={validation["valid"]}')
        fig.tight_layout()
        for extension in ('png','pdf'):fig.savefig(output/(case+'_search.'+extension),dpi=180)
        plt.close(fig)
        stats={}
        for tree in ('start','goal'):
            state=traces[tree+'_states'];speed=np.linalg.norm(state[:,2:],axis=1)
            parent_counts=np.bincount(traces[tree+'_parents'])
            stats[tree]=dict(nodes=len(state),distinct_states=len(np.unique(state,axis=0)),largest_parent_child_count=int(parent_counts.max()),position_min=state[:,:2].min(0).tolist(),position_max=state[:,:2].max(0).tolist(),speed_percentiles=np.percentile(speed,[0,10,50,90,100]).tolist(),speed_below_01_fraction=float(np.mean(speed<.1)))
        record=dict(case=case,seed=seed,solved=result.solved,validation=validation,plan_ns=result.plan_ns,postprocess_ns=result.postprocess_ns,wrapper_ns=wrapper_ns,iterations=result.iterations,nodes=result.nodes,rejections=result.rejections,trace_statistics=stats,settings=asdict(settings),input_provenance=provenance,edges=result.trajectory.to_records())
        records.append(record);print(json.dumps({k:v for k,v in record.items() if k not in ['edges','settings','input_provenance']},indent=2),flush=True)
    path=output/'summary.json';path.write_text(json.dumps(records,indent=2)+'\n');save_manifest(path,dict(purpose='failure_diagnosis_not_final_timing',budget_s=budget,seed=seed,lift_speed=lift_speed),[ROOT/'docs/sources/flask_v4.pdf'])
    return records
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=ROOT/'results/diagnostics/unicycle_native');parser.add_argument('--budget',type=float,default=2);parser.add_argument('--seed',type=int,default=0);parser.add_argument('--lift-speed',type=float,default=.3);args=parser.parse_args();run(args.output,args.budget,args.seed,args.lift_speed)
