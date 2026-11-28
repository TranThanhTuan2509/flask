"""v7-v10 final measurement campaigns: one frozen source, one logical CPU, controlled power policy.

Stages run sequentially. Before the first stage the measurement environment must pass
`benchmarks.measurement_env.preflight` (energy preference 'performance', AC online, idle SMT sibling).
A fixed-work probe runs before and after every timing stage; a stage whose probe drifts by more
than 10% is marked `uncontrolled` (its data are kept, never silently accepted). Source hashes are
checked before and after every stage; any change aborts the campaign.

Usage: python -m benchmarks.run_final_campaign --output results/paper_campaigns/2026-11-24_final_v7 --cpu 3
       python -m benchmarks.run_final_campaign --plan v8 --output results/paper_campaigns/<date>_final_v8 --cpu 3
"""
import argparse
import hashlib
import json
import subprocess
import sys
import tarfile
from datetime import datetime, timezone
from pathlib import Path

from benchmarks.measurement_env import check, fixed_work_probe, preflight, snapshot
from flask_repro.provenance import manifest
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

ROOT = Path(__file__).resolve().parents[1]
PY = sys.executable


def stages_v8(out, cpu):
    """v8: stop-turn-go unicycle and pure-sphere shell covers on new held-out seeds 3001+, with repeated
    stages to measure run-to-run reproducibility of the timings."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    panda = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.manipulator_campaign', '--output', str(out / name),
                                                  '--profile', profile, '--cpu', c, *extra])
    aerial = ['--cases', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
    return [
        native('table_i_v8', 'config/low_dof_final_v8_order2.json'),
        native('table_i_v8_scalar_rrtc', 'config/low_dof_final_v8_order2_scalar.json', '--algorithms', 'rrt_connect'),
        panda('table_ii_v8', 'config/manipulator_final_v8.json', '--methods', 'flask_rrt_connect'),
        native('table_i_v8_repeat_rrtc', 'config/low_dof_final_v8_order2.json', '--algorithms', 'rrt_connect'),
        panda('table_ii_v8_repeat', 'config/manipulator_final_v8.json', '--methods', 'flask_rrt_connect'),
        native('table_i_v8_order4_propagation', 'config/low_dof_final_v8_order4_propagation.json', *aerial,
               '--algorithms', 'sst_bvp', 'sst_simd', 'rrt_dp'),
    ]


def stages_v9(out, cpu):
    """v9: stop-turn shortcut fix, 5x identical-work timing repeats, Panda range 1.0 and lazy path certification,
    new held-out seeds 4001+, repeated stages for reproducibility."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    panda = lambda name, *extra: (name, [PY, '-m', 'benchmarks.manipulator_campaign', '--output', str(out / name),
                                         '--profile', 'config/manipulator_final_v9.json', '--cpu', c, *extra])
    author = lambda name, extra: (name, [PY, '-m', 'benchmarks.author_reference', 'run', '--output', str(out / name),
                                         '--seeds', *map(str, range(4001, 4011)), '--budget', '1.0', '--accel', '0', '--cpu', c,
                                         '--driver', str(ROOT / 'build/author_flask_fixed/mbm_flat'), *extra])
    return [
        native('table_i_v9', 'config/low_dof_final_v9_order2.json'),
        native('table_i_v9_scalar_rrtc', 'config/low_dof_final_v9_order2_scalar.json', '--algorithms', 'rrt_connect'),
        panda('table_ii_v9', '--methods', 'flask_rrt_connect'),
        panda('table_ii_v9_lazy_certified', '--methods', 'flask_rrt_connect_lazy_certified'),
        native('table_i_v9_repeat_rrtc', 'config/low_dof_final_v9_order2.json', '--algorithms', 'rrt_connect'),
        panda('table_ii_v9_repeat', '--methods', 'flask_rrt_connect'),
        # Authors' own library (KavrakiLab/ompl-flask `flat` + their fix-rrt-connect-loop commit), public defaults and
        # with zero-velocity proposals installed through the derivative-sampler extension point.
        author('table_ii_v9_author_fix', []),
        author('table_ii_v9_author_fix_rest', ['--rest']),
    ]


def stages_v10(out, cpu):
    """v10 (PC): faithful flat unicycle (stop-turn-go only as a labelled variant), Quad3D yaw winding, aerial
    order 2 (reduced protocol) and order 4 (full model) with interval validation, Panda rest AND full-state
    samplers with the authors' peak-speed edge sample count, lazy certification, same-session VAMP+TOPP-RA
    baseline and hardware calibration, authors' library, repeated stages. Held-out seeds 5001+."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    panda = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.manipulator_campaign', '--output', str(out / name),
                                                  '--profile', profile, '--cpu', c, *extra])
    author = lambda name, extra: (name, [PY, '-m', 'benchmarks.author_reference', 'run', '--output', str(out / name),
                                         '--seeds', *map(str, range(5001, 5011)), '--budget', '1.0', '--accel', '0', '--cpu', c,
                                         '--driver', str(ROOT / 'build/author_flask_fixed/mbm_flat'), *extra])
    aerial = ['--cases', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
    unicycle = ['--cases', 'unicycle_bugtrap', 'unicycle_wall']
    return [
        ('hardware_calibration', [PY, '-m', 'benchmarks.hardware_calibration', '--output', str(out / 'hardware_calibration.json'), '--cpu', c]),
        native('table_i_v10', 'config/low_dof_v10_order2.json'),
        native('table_i_v10_scalar_rrtc', 'config/low_dof_v10_order2_scalar.json', '--algorithms', 'rrt_connect'),
        native('table_i_v10_unicycle_stop_turn_variant', 'config/low_dof_v10_unicycle_stop_turn.json', *unicycle),
        panda('table_ii_v10_rest', 'config/manipulator_v10_rest.json', '--methods', 'flask_rrt_connect', 'vamp_toppra'),
        panda('table_ii_v10_fullstate', 'config/manipulator_v10_fullstate.json', '--methods', 'flask_rrt_connect'),
        panda('table_ii_v10_rest_lazy_certified', 'config/manipulator_v10_rest.json', '--methods', 'flask_rrt_connect_lazy_certified'),
        author('table_ii_v10_author_fix', []),
        author('table_ii_v10_author_fix_rest', ['--rest']),
        native('table_i_v10_repeat_rrtc', 'config/low_dof_v10_order2.json', '--algorithms', 'rrt_connect'),
        panda('table_ii_v10_rest_repeat', 'config/manipulator_v10_rest.json', '--methods', 'flask_rrt_connect'),
        native('table_i_v10_order4', 'config/low_dof_v10_order4.json', *aerial),
        ('table_i_v10_order4_interval', [PY, '-m', 'benchmarks.validate_continuous_aerial', '--sources', str(out / 'table_i_v10_order4'),
                                         '--output', str(out / 'table_i_v10_order4_interval'), '--cpu', c]),
    ]


def stages_v10b(out, cpu):
    """v10b: identical settings to v10 on fresh held-out seeds 6001+ (config/*_v10b.json differ only in seed_set):
    reruns the UNCONTROLLED v10 Table I stage, the SIMD/scalar pair, the stability pairs (Table I and II) and
    order 4 with the interval oracle (which in v10 rejected the Quad2D |v|/|omega| model overrides)."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    panda = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.manipulator_campaign', '--output', str(out / name),
                                                  '--profile', profile, '--cpu', c, *extra])
    aerial = ['--cases', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
    return [
        ('hardware_calibration', [PY, '-m', 'benchmarks.hardware_calibration', '--output', str(out / 'hardware_calibration.json'), '--cpu', c]),
        native('table_i_v10b', 'config/low_dof_v10_order2_v10b.json'),
        native('table_i_v10b_scalar_rrtc', 'config/low_dof_v10_order2_scalar_v10b.json', '--algorithms', 'rrt_connect'),
        native('table_i_v10b_repeat_rrtc', 'config/low_dof_v10_order2_v10b.json', '--algorithms', 'rrt_connect'),
        panda('table_ii_v10b_rest', 'config/manipulator_v10_rest_v10b.json', '--methods', 'flask_rrt_connect'),
        panda('table_ii_v10b_rest_repeat', 'config/manipulator_v10_rest_v10b.json', '--methods', 'flask_rrt_connect'),
        native('table_i_v10b_order4', 'config/low_dof_v10_order4_v10b.json', *aerial),
        ('table_i_v10b_order4_interval', [PY, '-m', 'benchmarks.validate_continuous_aerial', '--sources', str(out / 'table_i_v10b_order4'),
                                          '--output', str(out / 'table_i_v10b_order4_interval'), '--cpu', c]),
    ]


def stages_v2(out, cpu):
    """flask_v2 held-out campaign (seeds 7001-7030, config/seed_registry.json low_dof_final_v2).
    v2 profiles hold the levers frozen by results/diagnostics/v2_factorial/selection.json; the v1 literal profile is
    the v10b order-2 profile on the same seeds (unicycle only: the order-2 quadrotor settings are unchanged in v2),
    so the v1/v2 comparison is paired. The manipulator planner is unchanged in v2 (Table II stays v10b)."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    aerial = ['--cases', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
    return [
        ('hardware_calibration', [PY, '-m', 'benchmarks.hardware_calibration', '--output', str(out / 'hardware_calibration.json'), '--cpu', c]),
        native('table_i_v2', 'config/low_dof_v2_order2.json'),
        native('table_i_v1_literal', 'config/low_dof_v1_literal_v2seeds.json', '--cases', 'unicycle_bugtrap', 'unicycle_wall'),
        native('table_i_v2_repeat_rrtc', 'config/low_dof_v2_order2.json', '--algorithms', 'rrt_connect'),
        native('table_i_v2_order4', 'config/low_dof_v2_order4.json', *aerial),
        ('table_i_v2_order4_interval', [PY, '-m', 'benchmarks.validate_continuous_aerial', '--sources', str(out / 'table_i_v2_order4'),
                                        '--output', str(out / 'table_i_v2_order4_interval'), '--cpu', c]),
    ]


def stages_v2b(out, cpu):
    """flask_v2b: unicycle Table I on fresh seeds 8001-8030 (config/seed_registry.json low_dof_final_v2b). Only change from
    v2: unicycle_wall uses the forward flat chart (kappa = 0, Eq. 10), read from the paper's Fig. 5a; levers unchanged.
    v1 literal runs on the same seeds with the same chart, so the comparison stays paired."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    unicycle = ['--cases', 'unicycle_bugtrap', 'unicycle_wall']
    return [
        ('hardware_calibration', [PY, '-m', 'benchmarks.hardware_calibration', '--output', str(out / 'hardware_calibration.json'), '--cpu', c]),
        native('table_i_v2b', 'config/low_dof_v2b_order2.json', *unicycle),
        native('table_i_v1_literal_v2b', 'config/low_dof_v1_literal_v2b.json', *unicycle),
    ]


def stages_v2c(out, cpu):
    """flask_v2 final campaign (release code): the v2/v2b profiles and held-out seeds unchanged (unicycle 8001-8030,
    quadrotors 7001-7030); the only difference from 2026-11-28_final_v2/_v2b is the binary, which now checks
    workspace bounds exactly (PlannerSettings.exact_bounds, default; tests/test_exact_bounds.py). Re-run so that every
    published number comes from the released code. No setting was changed."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    unicycle = ['--cases', 'unicycle_bugtrap', 'unicycle_wall']
    aerial = ['--cases', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
    return [
        ('hardware_calibration', [PY, '-m', 'benchmarks.hardware_calibration', '--output', str(out / 'hardware_calibration.json'), '--cpu', c]),
        native('table_i_unicycle', 'config/low_dof_v2b_order2.json', *unicycle),
        native('table_i_unicycle_v1_literal', 'config/low_dof_v1_literal_v2b.json', *unicycle),
        native('table_i_quad_order2', 'config/low_dof_v2_order2.json', *aerial),
        native('table_i_repeat_rrtc_unicycle', 'config/low_dof_v2b_order2.json', *unicycle, '--algorithms', 'rrt_connect'),
        native('table_i_repeat_rrtc_quad_order2', 'config/low_dof_v2_order2.json', *aerial, '--algorithms', 'rrt_connect'),
        native('table_i_quad_order4', 'config/low_dof_v2_order4.json', *aerial),
        ('table_i_quad_order4_interval', [PY, '-m', 'benchmarks.validate_continuous_aerial', '--sources', str(out / 'table_i_quad_order4'),
                                          '--output', str(out / 'table_i_quad_order4_interval'), '--cpu', c]),
    ]



def stages_v2d(out, cpu):
    """flask_v2 v2d campaign: lever L7 (Alg. 4 over subdivided edges) as frozen by config/v2_shortcut_rule.json, on
    held-out seeds 8101-8130 never used before, for every Table I row. Unicycle v2 + L7 and the v1 literal profile,
    quadrotors r = 2 (paper's reduced-order protocol, unchanged) and r = 4 (+ L7 for RRTConnect and BVP-SST*) with
    the interval certificate; repeated RRTConnect stages for determinism."""
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    unicycle = ['--cases', 'unicycle_bugtrap', 'unicycle_wall']
    aerial = ['--cases', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
    return [
        ('hardware_calibration', [PY, '-m', 'benchmarks.hardware_calibration', '--output', str(out / 'hardware_calibration.json'), '--cpu', c]),
        native('table_i_unicycle', 'config/low_dof_v2d_order2.json', *unicycle),
        native('table_i_unicycle_v1_literal', 'config/low_dof_v1_literal_v2d.json', *unicycle),
        native('table_i_quad_order2', 'config/low_dof_v2d_quad_order2.json', *aerial),
        native('table_i_repeat_rrtc_unicycle', 'config/low_dof_v2d_order2.json', *unicycle, '--algorithms', 'rrt_connect'),
        native('table_i_repeat_rrtc_quad_order4', 'config/low_dof_v2d_order4.json', *aerial, '--algorithms', 'rrt_connect'),
        native('table_i_quad_order4', 'config/low_dof_v2d_order4.json', *aerial),
        ('table_i_quad_order4_interval', [PY, '-m', 'benchmarks.validate_continuous_aerial', '--sources', str(out / 'table_i_quad_order4'),
                                          '--output', str(out / 'table_i_quad_order4_interval'), '--cpu', c]),
    ]


def stages(out, cpu):
    c = str(cpu)
    native = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.native_campaign', '--output', str(out / name),
                                                   '--profile', profile, '--cpu', c, '--final', *extra])
    panda = lambda name, profile, *extra: (name, [PY, '-m', 'benchmarks.manipulator_campaign', '--output', str(out / name),
                                                  '--profile', profile, '--cpu', c, *extra])
    return [
        native('table_i_order2', 'config/low_dof_final_v7_order2.json'),
        native('table_i_order4', 'config/low_dof_final_v7_order4.json', '--cases', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window'),
        native('table_i_order2_scalar_rrtc', 'config/low_dof_final_v7_order2_scalar.json', '--algorithms', 'rrt_connect'),
        panda('table_ii_paper_resolution', 'config/manipulator_final_v7.json'),
        panda('table_ii_certified', 'config/manipulator_final_v7_certified.json', '--methods', 'flask_rrt_connect'),
        panda('table_ii_baseline_fixed_grid', 'config/manipulator_final_v7_fixed_grid.json', '--methods', 'vamp_toppra'),
        ('table_ii_author', [PY, '-m', 'benchmarks.author_reference', 'run', '--output', str(out / 'table_ii_author'),
                             '--seeds', *map(str, range(2001, 2011)), '--budget', '1.0', '--accel', '0', '--cpu', c,
                             '--driver', str(ROOT / 'build/author_flask_budgeted/mbm_flat')]),
    ]


def main(args):
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    snap, problems = preflight(args.cpu, set_performance=True)
    (out / 'preflight.json').write_text(json.dumps(dict(snapshot=snap, problems=problems), indent=2) + '\n')
    if problems and not args.allow_uncontrolled:
        raise SystemExit('Preflight failed: ' + '; '.join(problems))
    before = manifest(dict(purpose=f'final_{args.plan}_measurement', cpu=args.cpu, preflight_problems=problems))
    archive = out / 'runtime_source.tar.gz'
    with tarfile.open(archive, 'x:gz') as bundle:
        for name in sorted(before['source_sha256']):
            bundle.add(ROOT / name, arcname=name, recursive=False)
    before['runtime_archive_sha256'] = hashlib.sha256(archive.read_bytes()).hexdigest()
    (out / 'manifest.json').write_text(json.dumps(before, indent=2) + '\n')
    records = []
    # Warm-up probe (discarded): the first probe of v7 ran cold (FLASKCC 34.9 ms vs ~23 ms afterwards).
    (out / 'warmup_probe.json').write_text(json.dumps(fixed_work_probe(args.cpu), indent=2) + '\n')
    plan = dict(v7=stages, v8=stages_v8, v9=stages_v9, v10=stages_v10, v10b=stages_v10b, v2=stages_v2, v2b=stages_v2b, v2c=stages_v2c, v2d=stages_v2d)[args.plan]
    selected = [s for s in plan(out, args.cpu) if not args.only or s[0] in args.only]
    for name, command in selected:
        assert manifest({})['source_sha256'] == before['source_sha256'], 'Source changed before ' + name
        row = dict(name=name, command=command, started_at=datetime.now(timezone.utc).isoformat(),
                   environment_before=snapshot(args.cpu), probe_before=fixed_work_probe(args.cpu))
        records.append(row)
        (out / 'jobs.json').write_text(json.dumps(records, indent=2) + '\n')
        with (out / (name + '.log')).open('x') as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        row.update(exit_code=result.returncode, ended_at=datetime.now(timezone.utc).isoformat(),
                   environment_after=snapshot(args.cpu), probe_after=fixed_work_probe(args.cpu))
        drift = max(abs(row['probe_after'][k] / row['probe_before'][k] - 1) for k in ['vamp_rrtc_median_ns', 'flask_cc_median_ns'])
        row['probe_drift'] = drift
        row['controlled'] = bool(drift <= .10 and not check(row['environment_before']) and not check(row['environment_after']))
        (out / 'jobs.json').write_text(json.dumps(records, indent=2) + '\n')
        print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'probe_drift', 'controlled']}), flush=True)
        assert manifest({})['source_sha256'] == before['source_sha256'], 'Source changed during ' + name
        if result.returncode:
            raise SystemExit(f'Stage {name} failed with exit code {result.returncode}')
    before.update(all_stages_completed=True, completed_at=datetime.now(timezone.utc).isoformat())
    (out / 'manifest.json').write_text(json.dumps(before, indent=2) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cpu', type=int, default=default_cpu())
    p.add_argument('--only', nargs='+')
    p.add_argument('--plan', choices=['v7', 'v8', 'v9', 'v10', 'v10b', 'v2', 'v2b', 'v2c', 'v2d'], default='v7')
    p.add_argument('--allow-uncontrolled', action='store_true')
    main(p.parse_args())
