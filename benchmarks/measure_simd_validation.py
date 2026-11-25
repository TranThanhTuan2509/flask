"""Paired scalar/SIMD kernel measurements, separate from paper repeat campaigns.

Both backends validate identical coefficient arrays and distributed time grids.
Scalar compilation disables loop and SLP vectorization; it uses the same sine
polynomial and float precision. Native timings exclude Python conversion and
corpus generation. This measures validation kernels, not planner speedup.
"""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import numpy as np

from flask_repro import _core
from flask_repro.flat_bvp import fixed_time
from flask_repro.flat_models import Unicycle,Quad2D,Quad3D
from flask_repro.planning import PlannerSettings
from flask_repro.provenance import manifest,file_hash,distribution
from benchmarks.native_campaign import hardware_snapshot

ROOT=Path(__file__).resolve().parents[1]


def corpus(model,seed,count):
    rng=np.random.default_rng(seed);edges=[]
    for _ in range(count):
        start=np.zeros((model.order,model.outputs));goal=start.copy()
        start[0]=rng.uniform(-.35,.35,model.outputs);goal[0]=start[0]+rng.uniform(-.2,.2,model.outputs)
        if isinstance(model,Unicycle):
            start[1]=goal[1]=[.3,0.];goal[0]=start[0]+[1.2,0.]
        if isinstance(model,Quad3D):
            start[0,2]=1.5+rng.uniform(-.15,.15);goal[0,2]=start[0,2]+rng.uniform(-.2,.2)
            start[0,3]=goal[0,3]=rng.uniform(-np.pi,np.pi)
        edges.append(fixed_time(start,goal,4.))
    return edges


def environment(dimensions,obstacles):
    result=_core.Environment();geometry=[]
    if obstacles:
        for x in np.linspace(-.8,1.8,12):
            for y in [-1.,1.]:
                center=[float(x),y,1.5 if dimensions==3 else 0.]
                result.add_sphere(center,.06);geometry.append(dict(type='sphere',center=center,radius=.06))
        center=[.5,1.15,1.5 if dimensions==3 else 0.]
        half=[.3,.05,.5];orientation=[0.,0.,.1]
        result.add_cuboid(center,orientation,half)
        geometry.append(dict(type='cuboid',center=center,half_extents=half,orientation_euler_xyz=orientation))
    result.prepare();return result,geometry


def run(args):
    os.sched_setaffinity(0,{args.cpu})
    flags=(ROOT/'build/CMakeFiles/_core.dir/flags.make').read_text()
    if not (('no-tree-vectorize' in flags and 'no-tree-slp-vectorize' in flags) or
            ('no-vectorize' in flags and 'no-slp-vectorize' in flags)):
        raise RuntimeError('Scalar vectorization-disable flags are not recorded')
    args.output.mkdir(parents=True,exist_ok=False)
    configuration=dict(purpose='validation_kernel_ablation_not_paper_reproduction',
        seeds=args.seeds,edges_per_seed=args.edges,repetitions=args.repetitions,
        validation_dt_s=.005,validation_spacing_m=.015,
        precision='float32 with rounded square root and matched sine polynomial',
        measurement='Native complete-edge validation only; Python conversion excluded',
        isolation='One logical CPU affinity; physical-core isolation is not established',
        unchanged_components='Steering, search, and nearest-neighbor code are not timed by this kernel study')
    provenance=manifest(configuration);provenance['hardware_before']=hardware_snapshot()
    (args.output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    rows=[];settings=PlannerSettings(validation_dt=.005,validation_spacing=.015)
    for name,model in [('unicycle',Unicycle()),('quad2d',Quad2D()),('quad3d',Quad3D())]:
        dimensions=3 if model.outputs==4 else 2
        for seed in args.seeds:
            edges=corpus(model,seed,args.edges)
            coefficients=[edge.coefficients.tolist() for edge in edges];durations=[edge.duration for edge in edges]
            np.savez_compressed(args.output/f'{name}_seed_{seed}.npz',coefficients=np.asarray(coefficients),durations_s=durations)
            for workload in ['empty','obstacles']:
                scene,geometry=environment(dimensions,workload=='obstacles')
                measurements={}
                order=['scalar','simd'] if seed%2 else ['simd','scalar']
                for backend in order:
                    cfg=settings.native();cfg.scalar_validation=backend=='scalar'
                    cfg.max_speed=model.velocity_max;cfg.max_yaw_rate=model.angular_velocity_max
                    if isinstance(model,Quad3D):cfg.max_torque=model.torque_max
                    call=lambda repeats:_core.flat_check_edges(name,scene,coefficients,durations,[-3.]*dimensions,[3.]*dimensions,.1,cfg,repeats)
                    warmup=call(2)
                    if not all(warmup['valid']):raise RuntimeError(f'Invalid all-valid corpus: {name}/{seed}/{workload}/{backend}')
                    measurements[backend]=call(args.repetitions)
                if measurements['scalar']['valid']!=measurements['simd']['valid'] or measurements['scalar']['batches']!=8*measurements['simd']['batches']:
                    raise RuntimeError('Backends did not validate the same complete sample grid')
                row=dict(model=name,seed=seed,workload=workload,geometry=geometry,
                         settings=asdict(settings),backend_order=order,measurements=measurements,
                         paired_speedup=measurements['scalar']['validation_ns']/measurements['simd']['validation_ns'])
                rows.append(row);(args.output/'measurements.json').write_text(json.dumps(rows,indent=2)+'\n')
            print(f'{name} seed {seed}: both workloads complete',flush=True)
    summary={}
    for name in ['unicycle','quad2d','quad3d']:
        for workload in ['empty','obstacles']:
            selected=[r for r in rows if r['model']==name and r['workload']==workload]
            summary[name+'/'+workload]=dict(attempts=len(selected),paired_speedup=distribution([r['paired_speedup'] for r in selected]),
                scalar_us_per_edge=distribution([r['measurements']['scalar']['validation_ns']/args.repetitions/args.edges/1000. for r in selected]),
                simd_us_per_edge=distribution([r['measurements']['simd']['validation_ns']/args.repetitions/args.edges/1000. for r in selected]))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    after=manifest(configuration)
    changed=[name for name in set(provenance['source_sha256'])|set(after['source_sha256']) if provenance['source_sha256'].get(name)!=after['source_sha256'].get(name)]
    provenance['source_changed_during_measurement']=changed;provenance['hardware_after']=hardware_snapshot()
    provenance['artifact_sha256']={p.name:file_hash(p) for p in args.output.glob('*.npz')}
    provenance['artifact_sha256']['measurements.json']=file_hash(args.output/'measurements.json')
    provenance['artifact_sha256']['summary.json']=file_hash(args.output/'summary.json')
    (args.output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    if changed:raise RuntimeError('Source changed during paired kernel measurements')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--cpu',type=int,default=2)
    parser.add_argument('--seeds',nargs='+',type=int,default=list(range(1001,1031)))
    parser.add_argument('--edges',type=int,default=24)
    parser.add_argument('--repetitions',type=int,default=10)
    args=parser.parse_args()
    if not args.seeds or args.edges<=0 or args.repetitions<=0:raise ValueError('Nonempty seeds and positive workload sizes are required')
    run(args)
