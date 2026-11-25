"""Sequential corrective studies; no simulation, rendering, or concurrent workers.

All timing stages use one logical CPU and an unchanged runtime fingerprint.
The supplied profiles retain reconstructed settings; execution is not acceptance.
"""
import argparse
from datetime import datetime,timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
from flask_repro.provenance import manifest,file_hash
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

ROOT=Path(__file__).resolve().parents[1]


def main(args):
    os.sched_setaffinity(0,{args.cpu})
    args.output.mkdir(parents=True,exist_ok=False)
    lock=(args.output.parent/'.corrective_measurement.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
    def job(name,module,*arguments):
        return name,[sys.executable,'-m',module,*map(str,arguments)]
    driver=ROOT/'build/author_flask_budgeted/mbm_flat'
    old=ROOT/'results/paper_campaigns/2026-11-23_native_v2'
    replay=ROOT/'results/paper_campaigns/2026-11-23_native_v1/analysis/replay_resolution.py'
    jobs=[
        job('bvp_direct_rho1','benchmarks.author_reference','bvp','--output',args.output/'bvp_direct_rho1.json','--count',1000,'--rho',1),
        job('bvp_direct_rho12','benchmarks.author_reference','bvp','--output',args.output/'bvp_direct_rho12.json','--count',1000,'--rho',12),
        job('revalidate_v2','benchmarks.revalidate_low_dof','--source',old/'table_i_low_dof_v2','--output',args.output/'revalidate_v2','--cpu',args.cpu),
        job('revalidate_order4','benchmarks.revalidate_low_dof','--source',old/'table_i_order4_sensitivity','--output',args.output/'revalidate_order4','--cpu',args.cpu),
        ('collision_resolution_replay',[sys.executable,str(replay),'--output',str(args.output/'collision_resolution_replay'),'--cpu',str(args.cpu)]),
        job('author_budgeted_smoke','benchmarks.author_reference','run','--output',args.output/'author_budgeted_smoke',
            '--driver',driver,'--seeds',1001,'--environments','cage','--problem-count',2,'--budget',.005,'--cpu',args.cpu),
        job('author_budgeted_reference','benchmarks.author_reference','run','--output',args.output/'author_budgeted_reference',
            '--driver',driver,'--seeds',*range(1,11),'--budget',1.,'--accel',0,'--cpu',args.cpu),
        job('table_i_original_model','benchmarks.native_campaign','--output',args.output/'table_i_original_model',
            '--profile',ROOT/'config/low_dof_measurement_v3.json','--cpu',args.cpu,
            '--campaign-class','paper_case_reconstruction_measurement'),
    ]
    for velocity,prefix in [(0,0),(1,0),(0,1),(1,1)]:
        name=f'panda_velocity{velocity}_prefix{prefix}_res16_v3'
        jobs.append(job(name,'benchmarks.manipulator_campaign','--output',args.output/name,
            '--profile',ROOT/'config'/f'{name}.json','--cpu',args.cpu,'--methods','flask_rrt_connect'))
    jobs += [
        job('author_budgeted_acceleration_limits','benchmarks.author_reference','run','--output',args.output/'author_budgeted_acceleration_limits',
            '--driver',driver,'--seeds',*range(1,11),'--budget',1.,'--accel',1,'--cpu',args.cpu),
        job('table_i_original_model_scalar','benchmarks.native_campaign','--output',args.output/'table_i_original_model_scalar',
            '--profile',ROOT/'config/low_dof_measurement_v3_scalar.json','--cpu',args.cpu,
            '--campaign-class','paper_case_reconstruction_measurement'),
    ]
    configuration=dict(purpose='corrective_evidence_not_reproduction_acceptance',cpu=args.cpu,
        isolation='Single logical CPU; sibling core and operating-system background work are not exclusively isolated.',
        jobs=[dict(name=n,command=c) for n,c in jobs],showcase_locked=True)
    provenance=manifest(configuration,[driver,driver.parent/'build_manifest.json',
        ROOT/'benchmarks/author_flask/mbm_flat.cpp',replay])
    (args.output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    with tarfile.open(args.output/'runtime_source.tar.gz','w:gz') as archive:
        paths=[ROOT/name for name in provenance['source_sha256']]
        paths += [ROOT/'benchmarks/author_flask/mbm_flat.cpp',driver,driver.parent/'build_manifest.json',
                  driver.parent/'BudgetedRRTConnect.cpp',driver.parent/'BudgetedRRTConnect.h']
        for path in sorted(set(paths)):archive.add(path,arcname=str(path.relative_to(ROOT)),recursive=False)
    (args.output/'runtime_source.tar.gz').chmod(0o444)
    provenance['runtime_archive_sha256']=file_hash(args.output/'runtime_source.tar.gz')
    (args.output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    records=[]
    for name,command in jobs:
        now=manifest(configuration)
        changed=[k for k in set(now['source_sha256'])|set(provenance['source_sha256'])
                 if now['source_sha256'].get(k)!=provenance['source_sha256'].get(k)]
        if changed:
            (args.output/'runtime_change_abort.json').write_text(json.dumps(changed,indent=2)+'\n')
            raise RuntimeError('Runtime changed; no further timing jobs started')
        record=dict(job=name,command=command,start_utc=datetime.now(timezone.utc).isoformat())
        records.append(record)
        (args.output/'jobs.json').write_text(json.dumps(records,indent=2)+'\n')
        print('START '+name,flush=True)
        with (args.output/f'{name}.log').open('w') as log:
            result=subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
        record.update(end_utc=datetime.now(timezone.utc).isoformat(),exit_code=result.returncode)
        (args.output/'jobs.json').write_text(json.dumps(records,indent=2)+'\n')
        print('END '+name+' exit='+str(result.returncode),flush=True)
        # A correctness-driver exception needs inspection before more timings.
        if result.returncode and name not in ('author_budgeted_reference','author_budgeted_acceleration_limits'):
            raise RuntimeError('Corrective stage failed; inspect retained log before continuing')
    print('CORRECTIVE CAMPAIGN DONE',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cpu',type=int,default=default_cpu())
    main(p.parse_args())
