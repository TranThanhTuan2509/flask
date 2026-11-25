"""Upstream Dynoplan iDb-A* baseline for Table I with models matched to the paper (supplementary to FLASK).

The paper cites iDb-A* from DynoBench. Published DynoBench primitive banks exist only for the STOCK models
(e.g. unicycle1_v0 with |v| <= 0.5), so banks are generated here for the matched models of
benchmarks/upstream_sst_campaign.py (MODELS) with Dynoplan's own three-step procedure (README, "How to generate
motion primitives"): mode 0 random start/goal trajectory optimization -> mode 1 cost improvement -> mode 2
random cuts. Bank sizes are smaller than the full DynoBench banks (disclosed; see the manifest).

The search binary is the RNG-seeded Dynoplan main_idbastar (build/seeded_baselines, seeding only) relinked
against the geometry-fixed libdynobench copy built by upstream_sst_campaign.build. Options are Dynoplan's
benchmark/config/algs/idbastar_v0.yaml entries of the closest stock model, with motionsFile replaced.
Paper protocol: 180 s budget, first-solution time and final-solution length; every solution re-checked
independently (original boxes, norm limits, goal region).

Usage: python -m benchmarks.idbastar_campaign build
       python -m benchmarks.idbastar_campaign primitives --model quad2d_flask --count 2000 --cpu 3 [--generator rand]
       python -m benchmarks.idbastar_campaign run --output DIR --seeds 4001 ... --cpu 3 [--timelimit 180]
"""
import argparse
import json
import os
import shlex
import subprocess
import time
from pathlib import Path

import yaml

from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json
from benchmarks.upstream_sst_campaign import (BASE, CASES, LIBS, OUT as SST_OUT, SEEDED, independent_check, length,
                                              scene, sha)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/idbastar'
PRIMITIVES = OUT / 'primitives'
# benchmark/config/algs/idbastar_v0.yaml of the closest stock model (unicycle1_v0 / quad2d_v0 / quad3d_v0)
OPTIONS = {
    'unicycle1_flask': dict(solver_id=1, use_nigh_nn=True, cost_delta_factor=1, smooth_traj=True, search_timelimit=15000,
                            num_primitives_0=100),
    'quad2d_flask': dict(solver_id=1, use_nigh_nn=True, cost_delta_factor=1, smooth_traj=True, search_timelimit=15000,
                         num_primitives_0=400, delta_0=.5, soft_control_bounds=True, max_motions_primitives=50000,
                         new_invariance=True, use_collision_shape=False),
    'quad3d_flask': dict(solver_id=1, use_nigh_nn=True, cost_delta_factor=1, smooth_traj=True, search_timelimit=60000,
                         max_iter=100, weight_goal=100, soft_control_bounds=True, delta_0=.7, num_primitives_0=2000,
                         max_motions_primitives=50000, new_invariance=True, use_collision_shape=False,
                         limit_branching_factor=30),
}


def env():
    return dict(os.environ, LD_LIBRARY_PATH=LIBS, OMP_NUM_THREADS='1')


def build():
    """Relink the seeded main_idbastar object against the geometry-fixed libdynobench copy."""
    OUT.mkdir(parents=True, exist_ok=True)
    lib = SST_OUT / 'libdynobench_matched.a'
    if not lib.exists():
        raise SystemExit('run `python -m benchmarks.upstream_sst_campaign build` first')
    words = shlex.split((BASE / 'CMakeFiles/main_idbastar.dir/link.txt').read_text())
    exe = str(OUT / 'main_idbastar')
    command = []
    for w in words:
        if w.endswith('main_idbastar.cpp.o'):
            w = str(SEEDED / 'main_idbastar.cpp.o')
        elif w == 'dynobench/libdynobench.a':
            w = str(lib)
        elif w == 'libdbastar.a':
            w = str(SEEDED / 'libdbastar.a')  # dbastar with registry-seeded RNG (scripts/build_seeded_baselines.py)
        elif w.endswith('.a') and not w.startswith('/'):
            w = str(BASE / w)
        elif w == 'main_idbastar':
            w = exe
        command.append(w)
    subprocess.run(command, check=True, cwd=BASE)
    manifest = dict(executable_sha256=sha(exe), library_sha256=sha(lib), seeded_main_object_sha256=sha(SEEDED / 'main_idbastar.cpp.o'),
                    seeded_dbastar_archive_sha256=sha(SEEDED / 'libdbastar.a'),
                    primitive_generator_sha256=sha(BASE / 'main_primitives'), link_command=command, options=OPTIONS,
                    upstream_unmodified=True)
    (OUT / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({k: manifest[k] for k in ['executable_sha256', 'library_sha256']}))


def _generator(args, log):
    # dynobench::robot_type_to_path resolves "../models/<type>.yaml" relative to the working directory
    cwd = SST_OUT / 'cwd'
    cwd.mkdir(exist_ok=True)
    # v10: the registry-seeded generator (scripts/build_seeded_baselines.py); upstream seeds from time(0)/random_device
    command = ['taskset', '-c', str(log['cpu']), str(SEEDED / 'main_primitives'), '--models_base_path', '../models/'] + args
    t0 = time.perf_counter()
    # the generator prints every optimization problem (GBs); keep only the tail
    proc = subprocess.run(command, cwd=cwd, env=env() | dict(FLASK_BENCHMARK_SEED=str(log['seed'])), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    tail = proc.stdout[-20000:].decode(errors='replace')
    log['steps'].append(dict(args=args, returncode=proc.returncode, seconds=time.perf_counter() - t0, tail=tail[-3000:]))
    if proc.returncode:
        raise SystemExit(f'main_primitives failed: {args}\n{tail[-2000:]}')


def primitives(a):
    PRIMITIVES.mkdir(parents=True, exist_ok=True)
    base = PRIMITIVES / f'{a.model}.bin'
    log = dict(model=a.model, count=a.count, cpu=a.cpu, generator=a.generator, seed=a.seed,
               generator_sha256=sha(SEEDED / 'main_primitives'), steps=[])
    # 'optimize' (mode 0, README step two) solves random start/goal problems; 'rand' (mode 9,
    # generate_rand) rolls out random controls -- the Dynoplan fallback used when mode 0 yields no primitive
    # for a model (feasibility: quad3d_flask 0/6772 successes in 1000 s).
    mode = '0' if a.generator == 'optimize' else '9'
    _generator(['--mode_gen_id', mode, '--dynamics', a.model, '--max_num_primitives', str(a.count), '--out_file', str(base)], log)
    _generator(['--mode_gen_id', '1', '--dynamics', a.model, '--max_num_primitives', str(a.count), '--in_file', str(base),
                '--solver_id', '1'], log)
    _generator(['--mode_gen_id', '2', '--dynamics', a.model, '--in_file', f'{base}.im.bin', '--max_num_primitives', '-1',
                '--max_splits', '1', '--max_length_cut', '50', '--min_length_cut', '5'], log)
    final = Path(f'{base}.im.bin.sp.bin')
    log.update(bank=str(final.relative_to(ROOT)), bank_sha256=sha(final))
    (PRIMITIVES / f'{a.model}.json').write_text(json.dumps(log, indent=2) + '\n')
    print(json.dumps({k: log[k] for k in ['model', 'count', 'bank']} | dict(seconds=[s['seconds'] for s in log['steps']])))


def run(a):
    from flask_repro.dynobench import load
    from benchmarks.measurement_env import preflight
    from flask_repro.provenance import manifest as source_manifest
    a.output.mkdir(parents=True, exist_ok=False)
    # v10: first-solution times are timing measurements: same preflight and source record as the FLASK stages
    snap, problems = preflight(a.cpu)
    (a.output / 'preflight.json').write_text(json.dumps(dict(snapshot=snap, problems=problems), indent=2) + '\n')
    if problems and not a.allow_uncontrolled:
        raise SystemExit('Preflight failed: ' + '; '.join(problems))
    (a.output / 'source_manifest.json').write_text(json.dumps(source_manifest(dict(purpose='idbastar_run')), indent=2) + '\n')
    build_manifest = json.loads((OUT / 'build_manifest.json').read_text())
    banks = {m: json.loads((PRIMITIVES / f'{m}.json').read_text()) for m in set(CASES.values())}
    (a.output / 'manifest.json').write_text(json.dumps(dict(build=build_manifest, banks={m: {k: v for k, v in b.items() if k != 'steps'}
                                                                                          for m, b in banks.items()},
                                                            seeds=a.seeds, cases=a.cases, timelimit_s=a.timelimit, cpu=a.cpu), indent=2) + '\n')
    with (a.output / 'attempts.jsonl').open('w') as stream:
        for case in a.cases:
            model = CASES[case]
            cfg = a.output / f'{model}.cfg.yaml'
            cfg.write_text(yaml.safe_dump(OPTIONS[model] | dict(timelimit=a.timelimit, motionsFile=str(ROOT / banks[model]['bank'])),
                                          sort_keys=False))
            env_file = scene(case, a.output)
            _, _, original, _, goal, _ = load(case)
            for seed in a.seeds:
                result = a.output / f'{case}_seed_{seed}.yaml'
                t0 = time.perf_counter()
                proc = subprocess.run(['taskset', '-c', str(a.cpu), str(OUT / 'main_idbastar'), '--env_file', str(env_file),
                                       '--models_base_path', str(SST_OUT / 'models') + '/', '--cfg_file', str(cfg),
                                       '--results_file', str(result)], capture_output=True, text=True,
                                      env=env() | dict(FLASK_BENCHMARK_SEED=str(seed)))
                wall = time.perf_counter() - t0
                (a.output / f'{case}_seed_{seed}.log').write_text(proc.stdout[-20000:] + proc.stderr[-5000:])
                info = (yaml.safe_load(result.read_text()) if result.exists() else {}) or {}
                raw = info.get('trajs_opt') or info.get('trajs_raw') or []
                raw = [t for t in raw if t.get('feasible', True) and t.get('states')]
                row = dict(case=case, seed=seed, returncode=proc.returncode, wall_s=wall, solved=bool(info.get('solved')),
                           improvements=[dict(time_ms=t.get('time_stamp'), cost=t.get('cost')) for t in raw])
                if raw:
                    first, final = raw[0], min(raw, key=lambda t: t.get('cost', float('inf')))
                    errors = independent_check(case, final, original, goal)
                    row.update(first_ms=first.get('time_stamp'), first_length=length(first['states'], case),
                               final_cost=final.get('cost'), final_length=length(final['states'], case),
                               independent_errors=errors, valid=not errors)
                else:
                    row.update(valid=False, independent_errors=['no_solution'])
                stream.write(json.dumps(row) + '\n'); stream.flush()
                print(json.dumps({k: row.get(k) for k in ['case', 'seed', 'solved', 'valid', 'first_ms', 'final_length']}), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('build')
    g = sub.add_parser('primitives')
    g.add_argument('--model', choices=sorted(OPTIONS), required=True)
    g.add_argument('--count', type=int, required=True)
    g.add_argument('--cpu', type=int, default=default_cpu())
    g.add_argument('--generator', choices=['optimize', 'rand'], default='optimize')
    g.add_argument('--seed', type=int, default=1, help='FLASK_BENCHMARK_SEED for the generator (recorded)')
    r = sub.add_parser('run')
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--seeds', type=int, nargs='+', required=True)
    r.add_argument('--cases', nargs='+', default=list(CASES))
    r.add_argument('--cpu', type=int, default=default_cpu())
    r.add_argument('--timelimit', type=float, default=180.)
    r.add_argument('--allow-uncontrolled', action='store_true')
    a = p.parse_args()
    {'build': lambda: build(), 'primitives': lambda: primitives(a), 'run': lambda: run(a)}[a.command]()
