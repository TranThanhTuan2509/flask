"""Upstream Dynoplan SST baseline for Table I with models matched to the paper (supplementary to FLASK).

Runs the UNMODIFIED Dynoplan/OMPL SST search (build/seeded_baselines main_sst object, RNG seeding only)
linked against a copy of libdynobench in which exactly two objects are replaced by the geometry-support
fixes in results/baseline_adapter_preparation (unicycle radius read from YAML; Quad2D sphere actually
inserted). Upstream sources stay untouched. Model parameters are YAML-only:
  unicycle  |v| <= 1, |omega| <= 1.5, sphere radius 0.1                        (paper Sec. VII-A)
  quad2d    m .034, J 1e-4, l .1, motor force <= .65 m g (max_f 1.3), sphere 0.1
  quad3d    m 1, J diag(.1,.1,.2), total thrust <= 1.5 m g (max_f 1.5), sphere 0.1; per-axis torque <= 2 N m
            via arm_length .385 and t2t .272 (paper: |tau| <= 2 -- box vs ball is a disclosed approximation);
            |v| <= 4, |omega| <= 8 are per-component bounds upstream (paper: norms) and are re-checked as norms here.
Algorithm: OMPL fixed-radius SST via Dynoplan (the pinned Dynoplan has no SST*); DynoBench sst_v0 options;
paper anytime protocol: 180 s budget, first-solution time and final-solution length.

Usage: python -m benchmarks.upstream_sst_campaign build
       python -m benchmarks.upstream_sst_campaign run --output DIR --seeds 2001 ... --cpu 3 [--timelimit 180]
"""
import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import yaml
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'build/baselines'
SEEDED = ROOT / 'build/seeded_baselines'
PREP = ROOT / 'results/baseline_adapter_preparation'
OUT = ROOT / 'build/upstream_sst'
LIBS = ':'.join([str(ROOT / '.venv_baselines/lib/python3.10/site-packages/cmeel.prefix/lib'),
                 str(ROOT / 'third_party/baseline_prefix/usr/lib'), str(ROOT / 'third_party/baseline_prefix/usr/lib/x86_64-linux-gnu')])
CASES = {'unicycle_bugtrap': 'unicycle1_flask', 'unicycle_wall': 'unicycle1_flask', 'quad2d_bugtrap': 'quad2d_flask',
         'quad2d_hole': 'quad2d_flask', 'quad3d_block': 'quad3d_flask', 'quad3d_window': 'quad3d_flask'}
MODELS = {
    'unicycle1_flask': dict(dynamics='unicycle1', max_vel=1.0, min_vel=-1.0, max_angular_vel=1.5, min_angular_vel=-1.5,
                            shape='sphere', radius=0.1, size=[0.1], distance_weights=[1, .5], dt=.1),
    'quad2d_flask': dict(dynamics='quad2d', distance_weights=[1., .5, .2, .2], max_f=1.3, max_vel=4, max_angular_vel=8,
                         max_acc=20, max_angular_acc=20, dt=.01, m=.034, I=1e-4, l=.1, size=[.1], shape='sphere',
                         drag_against_vel=False),
    'quad3d_flask': dict(dynamics='quad3d', distance_weights=[1., .5, .1, .05], max_vel=4, max_angular_vel=8, max_acc=25,
                         max_angular_acc=20, motor_control=True, m=1.0, max_f=1.5, arm_length=.385, t2t=.272,
                         J_v=[.1, .1, .2], dt=.01, size=[.1], shape='sphere'),
}
SST_OPTIONS = {  # dynoplan benchmark/config/algs/sst_v0.yaml
    'unicycle1_flask': dict(goal_epsilon=.2, min_control_duration=2, max_control_duration=20),
    'quad2d_flask': dict(goal_epsilon=.5, goal_bias=.5, propagation_step_size=.01, min_control_duration=10,
                         max_control_duration=50, custom_sampling=False),
    'quad3d_flask': dict(goal_epsilon=.7, goal_bias=.2, propagation_step_size=.01, min_control_duration=5,
                         max_control_duration=50, pruning_radius=.1, selection_radius=1., custom_sampling=False),
}
# custom_sampling (DynoBench's near-hover control sampler, std 0.01) is disabled for the aerial models: with the
# paper's inertia J = diag(.1,.1,.2) every near-hover propagation stays inside the 0.1 pruning radius of the root
# and SST never grows (1 state after ~4e5 iterations; build/upstream_sst diagnostics). OMPL's default uniform
# control sampling, i.e. the paper's "SST* from OMPL" setting, restores growth.


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    lib = OUT / 'libdynobench_matched.a'
    shutil.copy(BASE / 'dynobench/libdynobench.a', lib)
    for name in ['unicycle1.cpp.o', 'planar_rotor.cpp.o']:
        shutil.copy(PREP / name, OUT / name)
    subprocess.run(['ar', 'r', str(lib), 'unicycle1.cpp.o', 'planar_rotor.cpp.o'], check=True, cwd=OUT)
    words = shlex.split((BASE / 'CMakeFiles/main_sst.dir/link.txt').read_text())
    exe = str(OUT / 'main_sst')
    command = []
    for w in words:
        if w.endswith('main_sst.cpp.o'):
            w = str(SEEDED / 'main_sst.cpp.o')
        elif w == 'dynobench/libdynobench.a':
            w = str(lib)
        elif w.endswith('.a') and not w.startswith('/'):
            w = str(BASE / w)
        elif w == 'main_sst':
            w = exe
        command.append(w)
    subprocess.run(command, check=True, cwd=BASE)
    (OUT / 'models').mkdir(exist_ok=True)
    for name, params in MODELS.items():
        (OUT / 'models' / f'{name}.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
    manifest = dict(executable_sha256=sha(exe), library_sha256=sha(lib),
                    replaced_objects={n: sha(OUT / n) for n in ['unicycle1.cpp.o', 'planar_rotor.cpp.o']},
                    seeded_main_object_sha256=sha(SEEDED / 'main_sst.cpp.o'), link_command=command, models=MODELS,
                    options=SST_OPTIONS, upstream_unmodified=True)
    (OUT / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({k: manifest[k] for k in ['executable_sha256', 'library_sha256']}))


def scene(case, directory):
    data = yaml.safe_load((PREP / 'scenes' / f'{case}.yaml').read_text())
    data['robots'][0]['type'] = CASES[case]
    path = directory / f'{case}.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


def length(states, case):
    s = np.asarray(states, float)
    d = 3 if case.startswith('quad3d') else 2
    return float(np.sum(np.linalg.norm(np.diff(s[:, :d], axis=0), axis=1))) if len(s) > 1 else 0.


def independent_check(case, traj, world, goal):
    """States against the ORIGINAL boxes, paper norm limits (quad3d) and the DynoBench goal region."""
    from flask_repro import original_sst as O
    s = np.asarray(traj['states'], float)
    d = 3 if case.startswith('quad3d') else 2
    errors = []
    if not world.positions_valid(s[:, :d], independent=True):
        errors.append('collision_or_bounds_at_states')
    if case.startswith('quad3d'):
        if np.max(np.linalg.norm(s[:, 7:10], axis=1)) > 4 + 1e-9:
            errors.append('speed_norm')
        if np.max(np.linalg.norm(s[:, 10:13], axis=1)) > 8 + 1e-9:
            errors.append('body_rate_norm')
    robot = case.split('_')[0]
    if O.dynobench_distance(robot, s[-1], goal) > O.GOAL_EPSILON[robot] + 1e-6:
        errors.append('goal_region')
    return errors


def run(args):
    from flask_repro.dynobench import load
    args.output.mkdir(parents=True, exist_ok=False)
    build_manifest = json.loads((OUT / 'build_manifest.json').read_text())
    (args.output / 'manifest.json').write_text(json.dumps(dict(build=build_manifest, seeds=args.seeds, cases=args.cases,
        timelimit_s=args.timelimit, cpu=args.cpu), indent=2) + '\n')
    with (args.output / 'attempts.jsonl').open('w') as stream:
        for case in args.cases:
            model = CASES[case]
            cfg = args.output / f'{model}.cfg.yaml'
            cfg.write_text(yaml.safe_dump(SST_OPTIONS[model] | dict(timelimit=args.timelimit), sort_keys=False))
            env_file = scene(case, args.output)
            _, _, original, _, goal, _ = load(case)
            for seed in args.seeds:
                result = args.output / f'{case}_seed_{seed}.yaml'
                env = dict(os.environ, FLASK_BENCHMARK_SEED=str(seed), LD_LIBRARY_PATH=LIBS, OMP_NUM_THREADS='1')
                t0 = time.perf_counter()
                proc = subprocess.run(['taskset', '-c', str(args.cpu), str(OUT / 'main_sst'), '--env_file', str(env_file),
                                       '--models_base_path', str(OUT / 'models') + '/', '--cfg_file', str(cfg),
                                       '--results_file', str(result)], capture_output=True, text=True, env=env)
                wall = time.perf_counter() - t0
                (args.output / f'{case}_seed_{seed}.log').write_text(proc.stdout[-20000:] + proc.stderr[-5000:])
                info = yaml.safe_load(result.read_text()) if result.exists() else {}
                raw = (info or {}).get('trajs_raw') or []
                row = dict(case=case, seed=seed, returncode=proc.returncode, wall_s=wall, solved=bool((info or {}).get('solved')),
                           improvements=[dict(time_ms=t.get('time_stamp'), cost=t.get('cost')) for t in raw])
                if raw:
                    first, final = raw[0], raw[-1]
                    errors = independent_check(case, final, original, goal)
                    row.update(first_ms=first.get('time_stamp'), first_length=length(first['states'], case),
                               final_cost=final.get('cost'), final_length=length(final['states'], case),
                               upstream_feasible=final.get('feasible'), independent_errors=errors, valid=not errors)
                else:
                    row.update(valid=False, independent_errors=['no_solution'])
                stream.write(json.dumps(row) + '\n'); stream.flush()
                print(json.dumps({k: row.get(k) for k in ['case', 'seed', 'solved', 'valid', 'first_ms', 'final_length']}), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('build')
    r = sub.add_parser('run')
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--seeds', type=int, nargs='+', required=True)
    r.add_argument('--cases', nargs='+', default=list(CASES))
    r.add_argument('--cpu', type=int, default=default_cpu())
    r.add_argument('--timelimit', type=float, default=180.)
    a = p.parse_args()
    build() if a.command == 'build' else run(a)
