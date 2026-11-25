"""Table II measurements with the authors' own FLASK implementation (KavrakiLab/ompl-flask).

The C++ driver `build/author_flask/mbm_flat` links the authors' OMPL fork and uses their
FlatStateSpace, minimum-effort steering, FlatVampMotionValidator, RRTConnect and reduceVertices.
This script only exports MotionBenchMaker problems, runs one driver process per seed (OMPL's seed is
process-global) pinned to one CPU, and validates every returned trajectory with the same independent
checks used for the native campaign (`benchmarks.manipulator_campaign.validate_flask`).

Subcommands:
  run   --output DIR --seeds 1 2 ... [--budget 1.0] [--accel 0|1] [--cpu 3] [--problem-count N]
  bvp   --output FILE [--count 1000] [--rho 1.0]       native/Python/author steering agreement
"""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np

from flask_repro import lqmt, mbm, native, robot
from flask_repro.provenance import distribution, file_hash, manifest
from flask_repro.reference import Trajectory
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'build/author_flask/mbm_flat'
AUTHOR_SOURCE = ROOT / 'third_party/ompl_flask_author'


def run_driver(command,raw,log_path,env,idle_s):
    """Bound a batch externally and preserve partial output on a stalled child."""
    started=time.monotonic();last_progress=started;last_size=0;watchdog=False
    with Path(log_path).open('w') as log:
        process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        try:
            while process.poll() is None:
                size=raw.stat().st_size if raw.exists() else 0
                now=time.monotonic()
                if size!=last_size:last_size=size;last_progress=now
                if now-last_progress>idle_s:
                    watchdog=True
                    break
                time.sleep(.05)
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait()
    return dict(returncode=process.returncode,idle_watchdog_triggered=watchdog,
                batch_wrapper_s=time.monotonic()-started,command=command)


def export_problems(path, environments, problem_count=None):
    dataset = mbm.load_dataset('panda')
    lines, inventory = [], []
    for env in environments:
        for index, p in enumerate(dataset[env]):
            if problem_count is not None and index >= problem_count:
                break
            inventory.append(dict(environment=env, problem=index, source_valid=bool(p['valid'])))
            if not p['valid']:
                continue
            lines.append(f"{env} {index} {len(p['goals'])}")
            lines.append(' '.join(repr(float(x)) for x in p['start']))
            lines += [' '.join(repr(float(x)) for x in g) for g in p['goals']]
            lines.append(str(len(p['sphere'])))
            lines += [' '.join(map(repr, [*map(float, o['position']), float(o['radius'])])) for o in p['sphere']]
            capsules, cuboids = [], []
            for o in p['cylinder']:
                if env == 'box':  # same conversion as the authors' MotionBenchmakerDemo and flask_repro.mbm
                    cuboids.append([*o['position'], *o['orientation_euler_xyz'], o['radius'], o['radius'], o['length'] / 2])
                else:
                    capsules.append([*o['position'], *o['orientation_euler_xyz'], o['radius'], o['length']])
            cuboids += [[*o['position'], *o['orientation_euler_xyz'], *o['half_extents']] for o in p['box']]
            lines.append(str(len(capsules)))
            lines += [' '.join(repr(float(x)) for x in c) for c in capsules]
            lines.append(str(len(cuboids)))
            lines += [' '.join(repr(float(x)) for x in c) for c in cuboids]
    Path(path).write_text('\n'.join(lines) + '\n')
    return inventory


def trajectory_from(block):
    """Rebuild the cubic Hermite edges from author waypoints (q, v) and durations.

    For order two the minimum-effort polynomial with fixed endpoints and duration is the unique cubic
    Hermite curve, so (Q, V, T) determines the authors' trajectory exactly."""
    Q = np.asarray(block['Q'], float).reshape(-1, 7)
    V = np.asarray(block['V'], float).reshape(-1, 7)
    T = np.asarray(block['T'], float)
    if len(T) != len(Q) - 1:  # FlatTrajectory drops rest-to-rest duplicates
        keep = [0] + [i for i in range(1, len(Q)) if not (np.array_equal(Q[i], Q[i - 1]) and np.array_equal(V[i], V[i - 1]))]
        Q, V = Q[keep], V[keep]
    if len(T) != len(Q) - 1:
        raise ValueError('Author trajectory has inconsistent waypoint and segment counts')
    return Trajectory(Q, V, T)


def run(args):
    from benchmarks.inventory_mbm import SphereOracle
    from benchmarks.manipulator_campaign import validate_flask
    from benchmarks.native_campaign import hardware_snapshot
    if args.budget<=0 or not np.isfinite(args.budget) or args.watchdog_idle_s<=args.budget:
        raise ValueError('Positive budget and an external idle timeout larger than the planning budget required')
    if not args.seeds or min(args.seeds)<1 or len(set(args.seeds))!=len(args.seeds):
        raise ValueError('Unique positive OMPL seeds required')
    args.output.mkdir(parents=True, exist_ok=False)
    problems_file = args.output / 'problems.txt'
    inventory = export_problems(problems_file, args.environments, args.problem_count)
    rb = robot.get_robot('panda')
    dataset = mbm.load_dataset('panda')
    oracle = SphereOracle(mbm.VAMP_ROOT / 'resources/panda/panda_spherized.urdf')
    profile = dict(additional_validation_profile='strict_development', independent_urdf_samples_per_trajectory=129)
    driver=args.driver.resolve()
    configuration = dict(method='author_flask_rrt_connect', driver=str(driver), author_commit=_git(AUTHOR_SOURCE),
                         author_vamp_commit=_git(AUTHOR_SOURCE / 'external/vamp'), seeds=args.seeds,
                         budget_s=args.budget, acceleration_limits=bool(args.accel), cpu=args.cpu,
                         rho=1.0, order=2, planner='og::RRTConnect default range (0.2 x extent)',
                         simplification='og::PathSimplifier::reduceVertices',
                         validation='FlatVampMotionValidator at 1/(32 x output extent)',
                         external_idle_watchdog_s=args.watchdog_idle_s,
                         author_experiment_configuration_confirmed=False)
    inputs=[driver,problems_file,ROOT/'benchmarks/author_flask/mbm_flat.cpp',
            ROOT/'build/ompl_flask_author/src/ompl/libompl.so.2.0.2']
    if (driver.parent/'build_manifest.json').exists():inputs.append(driver.parent/'build_manifest.json')
    provenance = manifest(configuration,inputs)
    provenance['hardware_before']=hardware_snapshot()
    (args.output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')
    (args.output / 'inventory.json').write_text(json.dumps(inventory, indent=2) + '\n')
    rows = [];process_events=[]
    with (args.output / 'attempts.jsonl').open('w') as stream:
        for seed in args.seeds:
            raw = args.output / f'driver_seed_{seed:02d}.jsonl'
            env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
            command=['taskset','-c',str(args.cpu),str(driver),'plan',str(problems_file),str(seed),
                     str(args.budget),'1' if args.accel else '0',str(raw)]+(['rest'] if args.rest else [])
            event=run_driver(command,raw,args.output/f'driver_seed_{seed:02d}.log',env,args.watchdog_idle_s)
            event['seed']=seed
            process_events.append(event)
            (args.output/'process_events.json').write_text(json.dumps(process_events,indent=2)+'\n')
            for line in (raw.read_text().splitlines() if raw.exists() else []):
                try:row = json.loads(line)
                except json.JSONDecodeError:
                    event['incomplete_raw_line']=True
                    continue
                problem = dataset[row['environment']][row['problem']]
                row['postprocess_ns'] = row.get('simplify_ns', 0)
                row['total_ns'] = row['plan_ns'] + row['postprocess_ns']
                validation_started=time.perf_counter_ns()
                if row['solved']:
                    path = trajectory_from(row)
                    ec = mbm.native_environment(problem)
                    validation = validate_flask(dict(trajectory=path), rb, ec, problem, oracle, profile)
                    acceleration = max((float(np.max(np.abs(lqmt.evaluate(e.coef, np.array([0., e.T]))[2]) - rb.a_max))
                                       for e in path.edges()),default=0.)
                    validation['max_acceleration_excess'] = acceleration
                    row.update(validation=validation, length_rad=validation['dense']['length'],
                               duration_s=validation['dense']['duration'], collision=validation['dense']['collision'])
                    for key in ['Q', 'V', 'T']:
                        row.pop(key)
                    row.pop('raw', None)
                else:
                    row.update(validation=dict(valid=False, errors=['no_solution']), length_rad=None,
                               duration_s=None, collision=None)
                row['additional_validation_ns']=time.perf_counter_ns()-validation_started
                rows.append(row)
                stream.write(json.dumps(row) + '\n')
            stream.flush()
            print(json.dumps(dict(seed=seed, attempts=len(rows), solved=sum(r['solved'] for r in rows))), flush=True)
    (args.output / 'summary.json').write_text(json.dumps(summarize(rows), indent=2) + '\n')
    after=manifest(configuration)
    changed=[name for name in set(provenance['source_sha256'])|set(after['source_sha256'])
             if provenance['source_sha256'].get(name)!=after['source_sha256'].get(name)]
    provenance.update(completed_attempts=len(rows),process_events=process_events,hardware_after=hardware_snapshot(),
                      expected_attempts=sum(p['source_valid'] for p in inventory)*len(args.seeds),
                      source_changed_during_campaign=changed,
                      artifact_sha256={p.name: file_hash(p) for p in args.output.glob('*.jsonl')})
    (args.output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')
    if changed or any(e['returncode']!=0 for e in process_events):
        raise RuntimeError('Reference campaign incomplete or source changed; partial data retained')


def summarize(rows):
    out = {}
    for env in mbm.ENVIRONMENTS + ['overall']:
        group = rows if env == 'overall' else [r for r in rows if r['environment'] == env]
        if not group:
            continue
        solved = [r for r in group if r['solved']]
        valid = [r for r in solved if r['validation']['valid']]
        out[env] = dict(attempts=len(group), solved=len(solved), validated=len(valid),
                        total_ms=distribution([r['total_ns'] / 1e6 for r in solved]),
                        plan_ms=distribution([r['plan_ns'] / 1e6 for r in solved]),
                        postprocess_ms=distribution([r['postprocess_ns'] / 1e6 for r in solved]),
                        length_rad=distribution([r['length_rad'] for r in solved]),
                        dense_collision_trajectories=sum(bool(r['collision']) for r in solved),
                        acceleration_violations=sum(r['validation']['max_acceleration_excess'] > 1e-6 for r in solved))
    return out


def bvp_compare(args):
    """Direct native C++/independent Python/author comparison of order-two steering."""
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rng = np.random.default_rng(20261123)
    rb = robot.get_robot('panda')
    pairs = []
    for i in range(args.count):
        q0 = rng.uniform(rb.q_lo, rb.q_hi);q1 = rng.uniform(rb.q_lo, rb.q_hi)
        scale = [1.0, 1e-3, 0.0][i % 3]  # include near-rest and rest boundaries
        v0 = rng.uniform(-rb.v_max, rb.v_max) * scale;v1 = rng.uniform(-rb.v_max, rb.v_max) * scale
        if i % 50 == 0:
            q1 = q0 + 1e-9 * rng.normal(size=7)  # near-degenerate displacement
        pairs.append(np.concatenate([q0, v0, q1, v1]))
    # Known repeated stationary minimum: J'(T)=rho*(T-1)^3*(T+3)/T^4.
    scale=np.sqrt(args.rho/12.)
    repeated=np.zeros((4,7));repeated[1,:3]=[4,1,1]
    repeated[2,0]=1;repeated[3,:3]=[0,-1,-1]
    pairs.append((repeated*scale).ravel())
    pairs_file = args.output.with_suffix('.pairs.txt')
    pairs_file.write_text('\n'.join(' '.join(repr(float(x)) for x in p) for p in pairs) + '\n')
    author_file = args.output.with_suffix('.author.jsonl')
    subprocess.run([str(DRIVER), 'bvp', str(pairs_file), str(args.rho), str(author_file)], check=True)
    results = [json.loads(l) for l in author_file.read_text().splitlines()]
    if len(results)!=len(pairs):raise RuntimeError('Author BVP output count mismatch')
    rows = []
    for p, a in zip(pairs, results):
        q0, v0, q1, v1 = p.reshape(4, 7)
        compiled=native.module(rb).lqmt(*[x.tolist() for x in (q0,v0,q1,v1)],args.rho)
        T = compiled[0] if compiled is not None else None
        python_T=lqmt.optimal_duration(q0,v0,q1,v1,args.rho)
        row = dict(native_T=T, author_T=a.get('T'), author_cost=a.get('cost'), author_found=a['found'],
                   case='repeated_stationary_minimum' if len(rows)==args.count else 'randomized_boundary')
        row.update(python_T=python_T,python_cost=lqmt.boundary_cost(q0,v0,q1,v1,python_T,args.rho) if python_T is not None else None)
        if T is not None:
            row['native_cost'] = compiled[3]
        if a['found'] and T is not None:
            row['rel_T'] = abs(T - a['T']) / max(a['T'], 1e-300)
            row['rel_cost'] = abs(row['native_cost'] - a['cost']) / max(a['cost'], 1e-300)
            mid = lqmt.evaluate(lqmt.hermite_coefficients(q0, v0, q1, v1, a['T']), np.array([0.5 * a['T']]))[0][0]
            row['mid_error_at_author_T'] = float(np.max(np.abs(mid - np.asarray(a['mid']))))
        rows.append(row)
    both = [r for r in rows if 'rel_T' in r]
    report = dict(count=len(rows),randomized_cases=args.count,adversarial_cases=1,both_found=len(both),
                  native_only=sum(r['native_T'] is not None and not r['author_found'] for r in rows),
                  author_only=sum(r['native_T'] is None and r['author_found'] for r in rows),
                  max_rel_T=max(r['rel_T'] for r in both), max_rel_cost=max(r['rel_cost'] for r in both),
                  native_cost_lower_or_equal=sum(r['native_cost'] <= r['author_cost'] * (1 + 1e-9) for r in both),
                  max_mid_error=max(r['mid_error_at_author_T'] for r in both),
                  disagreements=[dict(index=i, **r) for i, r in enumerate(rows) if r.get('rel_T', 0) > 1e-6][:20],
                  rho=args.rho,author_commit=_git(AUTHOR_SOURCE),
                  implementation='Direct compiled Panda lqmt binding versus author driver; Python solver recorded independently',
                  author_driver_sha256=file_hash(DRIVER),
                  repeated_stationary_minimum=rows[-1],
                  native_binary_sha256={p.name:file_hash(p) for p in (ROOT/'flask_repro').glob('_core*.so')})
    args.output.with_suffix('.rows.json').write_text(json.dumps(rows,indent=2)+'\n')
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'disagreements'}, indent=2))


def _git(path):
    try:
        return subprocess.run(['git', '-C', str(path), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    r = sub.add_parser('run')
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--seeds', type=int, nargs='+', required=True)
    r.add_argument('--budget', type=float, default=1.0)
    r.add_argument('--accel', type=int, choices=[0, 1], default=0)
    r.add_argument('--cpu', type=int, default=default_cpu())
    r.add_argument('--driver',type=Path,default=DRIVER)
    r.add_argument('--rest',action='store_true',help='zero-velocity proposals via the derivative sampler (reconstructed Table II protocol)')
    r.add_argument('--watchdog-idle-s',type=float,default=15.)
    r.add_argument('--problem-count', type=int)
    r.add_argument('--environments', nargs='+', default=mbm.ENVIRONMENTS)
    b = sub.add_parser('bvp')
    b.add_argument('--output', type=Path, required=True)
    b.add_argument('--count', type=int, default=1000)
    b.add_argument('--rho', type=float, default=1.0)
    a = parser.parse_args()
    run(a) if a.command == 'run' else bvp_compare(a)
