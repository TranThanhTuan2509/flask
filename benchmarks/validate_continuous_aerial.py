"""Validate saved full-order aerial trajectories with an interval oracle.

This read-only replay preserves planning timings and retains unsolved attempts.
It covers the published fixed Quad2D/Quad3D models, not arbitrary inertial models.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path

from flask_repro.interval_validation import audit_segment,iv
from flask_repro.provenance import file_hash,manifest


def run(sources,output,cpu=None):
    sources=[Path(p) for p in sources];output=Path(output)
    inventories=[]
    for source in sources:
        provenance=json.loads((source/'manifest.json').read_text())
        paths=sorted(source.glob('*_seed_*.json'))
        if provenance.get('completed_attempts')!=len(paths) or not paths:
            raise ValueError('A complete source campaign is required: '+str(source))
        if provenance.get('source_changed_during_campaign'):
            raise ValueError('Source campaign changed runtime: '+str(source))
        profile=provenance.get('configuration',{}).get('profile',{})
        if not isinstance(profile,dict):raise ValueError('A recorded model profile is required')
        base=profile.get('model',{});overrides={case:c.get('model',{}) for case,c in profile.get('case_overrides',{}).items()}
        # v10 contract: Quad2D may add the norm limits |v|<=velocity_max, |omega|<=angular_velocity_max
        # (flat_models.Quad2D); every other model override is still rejected.
        extra=set(base)-{'order'}
        if extra:raise ValueError('Profile-wide model overrides are unsupported: '+str(sorted(extra)))
        limits={}
        for case,model in overrides.items():
            allowed={'order','velocity_max','angular_velocity_max'} if case.startswith('quad2d') else {'order'}
            if set(model)-allowed:
                raise ValueError('Unsupported model override for '+case+': '+str(sorted(set(model)-allowed)))
            limits[case]={k:model[k] for k in ['velocity_max','angular_velocity_max'] if k in model}
        inventories.append((source,provenance,paths,limits))
    if cpu is not None:os.sched_setaffinity(0,{cpu})
    output.mkdir(parents=True,exist_ok=False)
    configuration=dict(scope='posthoc_full_order_aerial_state_control_bounds',sources=[str(p) for p in sources],
        arithmetic='mpmath private interval context',precision_decimal_digits=iv.dps,
        model='Published fixed inertial models; no extra tolerance on limits; Quad2D per-case |v|, |omega| norm limits from the profile',
        quad2d_limits={str(s):l for s,_,_,l in inventories},
        limitations=['Outside planning timing','Experimental interval backend, not a formal proof tool',
                    'No geometry, tracking, mesh or physical execution guarantee'])
    provenance=manifest(configuration,[p/'manifest.json' for p in sources]);rows=[];cache={}
    (output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    with (output/'attempts.jsonl').open('w') as stream:
        for source,frozen,paths,limits in inventories:
            for path in paths:
                digest=file_hash(path)
                if digest!=frozen['attempt_sha256'][path.name]:raise ValueError('Changed source artifact: '+str(path))
                record=json.loads(path.read_text());row={k:record[k] for k in ['case','algorithm','seed','solved']}
                row.update(source=str(source),attempt_sha256=digest)
                if not record['case'].startswith('quad'):row['status']='outside_aerial_scope'
                elif not record['solved']:row['status']='no_solution'
                elif not record['trajectory']:row['status']='unsupported_empty_trajectory'
                elif any(e['order']!=4 for e in record['trajectory']):row['status']='unsupported_reduced_order'
                else:
                    robot='quad3d' if record['case'].startswith('quad3d') else 'quad2d';checks=[]
                    for edge in record['trajectory']:
                        bound=limits.get(record['case']) or None
                        key=hashlib.sha256(json.dumps([robot,bound,edge],sort_keys=True).encode()).hexdigest()
                        if key not in cache:cache[key]=audit_segment(edge['coefficients'],edge['duration'],robot,limits=bound)
                        checks.append(cache[key])
                        if checks[-1]['status']!='clear':break
                    row['segments']=checks
                    row['status']=next((r['status'] for r in checks if r['status']!='clear'),'clear')
                rows.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
    after=manifest({})
    changed=[name for name in set(provenance['source_sha256'])|set(after['source_sha256'])
             if provenance['source_sha256'].get(name)!=after['source_sha256'].get(name)]
    if changed:raise RuntimeError('Runtime changed during validation')
    summary=dict(attempts=len(rows),outcomes=dict(Counter(r['status'] for r in rows)),unique_segments=len(cache),
        scope=configuration,raw_sha256=file_hash(output/'attempts.jsonl'),source_changed_during_validation=changed)
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sources',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cpu',type=int)
    args=parser.parse_args();run(args.sources,args.output,args.cpu)


if __name__=='__main__':main()
