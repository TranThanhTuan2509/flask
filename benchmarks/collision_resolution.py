"""Study collision-resolution convergence on the four preserved counterexamples."""
import argparse,json,time
from pathlib import Path
import numpy as np
from flask_repro import native,mbm
from flask_repro.robot import get_robot
from flask_repro.provenance import save_manifest
ROOT=Path(__file__).resolve().parents[1]

def run(output):
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    witnesses=ROOT/'results/research/mbm_collision_witnesses'
    robot=get_robot('panda');module=native.module(robot);dataset=mbm.load_dataset('panda');rows=[]
    for file in sorted(witnesses.glob('*.npz')):
        environment,index=file.stem.rsplit('_',1);problem=dataset[environment][int(index)];world=mbm.native_environment(problem)
        with np.load(file) as data:
            states=data['collision_configurations'];edges=list(zip(data['Q'][:-1],data['V'][:-1],data['Q'][1:],data['V'][1:],data['T']))
            witness_detected=[not module.validate(world,q.tolist()) for q in states]
            if not all(witness_detected):raise AssertionError(f'Collision witness changed: {file}')
            for scale in (1,2,4,8,16):
                for bounded in (False,True):
                    limits=native.joint_limits(robot);limits.collision_resolution_scale=scale
                    limits.bound_validation_speed=bounded
                    if bounded:limits.max_validation_dt=.005
                    begin=time.perf_counter_ns()
                    valid=[module.flask_cc(world,*[np.asarray(v).tolist() for v in edge[:-1]],float(edge[-1]),limits) for edge in edges]
                    rows.append(dict(witness=file.name,scene=environment,pair_index=int(index),resolution_scale=scale,
                            samples_per_rad=module.resolution*scale,bounded_speed=bounded,max_dt_s=.005 if bounded else None,
                            all_edges_accepted=all(valid),accepted_edges=valid,check_ns=time.perf_counter_ns()-begin,
                            point_witnesses_detected=all(witness_detected)))
    output.write_text(json.dumps(dict(rows=rows,scope='four_preserved_counterexamples',continuous_safety_proof=False,
          conclusion='Rejecting these counterexamples is necessary but does not establish a universal collision certificate.'),indent=2)+'\n')
    save_manifest(output,dict(study='resolution_convergence',scales=[1,2,4,8,16]),list(witnesses.glob('*.npz')))
    return rows
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=ROOT/'results/validation/collision_resolution.json');args=parser.parse_args();print(json.dumps(run(args.output),indent=2))
