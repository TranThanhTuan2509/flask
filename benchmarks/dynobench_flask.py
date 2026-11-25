"""Legacy Python search diagnostics; use native_campaign for native experiments.

Python search timings and approximate SST variants in this script are not
measurements of the paper's optimized planners.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from time import perf_counter_ns
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flask_repro import dynobench
from flask_repro.flat_planner import Settings, solve, valid_edge
from flask_repro.provenance import save_manifest, summarize

def run(output, cases, variants, seeds, budget, iterations, simd, radius, spacing, lift_speed=0.1, extension_range=1.0, derivative_bound=0.3):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    cfg = Settings(budget_s=budget, max_iterations=iterations, range=extension_range, sample_derivative_bound=derivative_bound)
    rows = []
    evidence = {}
    inputs = []
    for case in cases:
        (model, world, original, start, goal, provenance) = dynobench.load(case, radius, spacing, simd, lift_speed)
        evidence[case] = provenance
        inputs.append(provenance['input'])
        for variant in variants:
            for seed in seeds:
                t0 = perf_counter_ns()
                result = solve(model, world, start, goal, cfg, seed, variant)
                wall = (perf_counter_ns() - t0) / 1000000.0
                checked = result.solved and all((valid_edge(e, model, original, cfg, independent=True, refinement=4) for e in result.edges))
                row = dict(case=case, variant=variant, seed=seed, solved=result.solved, validated=checked, reason=result.reason, wall_ms=wall, total_ms=(result.plan_ns + result.postprocess_ns) / 1000000.0, plan_ms=result.plan_ns / 1000000.0, postprocess_ms=result.postprocess_ns / 1000000.0, first_solution_ms=None if result.first_solution_ns is None else result.first_solution_ns / 1000000.0, iterations=result.iterations, nodes=result.nodes, terminal_error=result.terminal_error, duration_s=sum((e.duration for e in result.edges)) if result.solved else None, length=sum((e.length(slice(0, len(world.lower))) for e in result.edges)) if result.solved else None, collision=not checked if result.solved else None, edges=[dict(T=e.duration, order=e.order, coefficients=e.coefficients.tolist()) for e in result.edges])
                if result.solved:
                    from flask_repro.flat_planner import edge_times
                    row['collision'] = any((not original.positions_valid(e.derivative(edge_times(e, cfg, 4))[:, :len(world.lower)], independent=True) for e in result.edges))
                rows.append(row)
                output.write_text(json.dumps(dict(rows=rows, evidence=evidence, summary=summarize(rows), settings=asdict(cfg)), indent=2))
                print(f'{case}/{variant}/{seed}: solved={result.solved}, validated={checked}, {wall:.1f} ms', flush=True)
    save_manifest(output, dict(settings=asdict(cfg), cases=cases, variants=variants, seeds=seeds, simd_geometry=simd, evidence=evidence), inputs)
if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--cases', nargs='+', choices=list(dynobench.CASES), default=list(dynobench.CASES))
    p.add_argument('--variants', nargs='+', default=['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp'], choices=['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp'])
    p.add_argument('--seeds', nargs='+', type=int, default=[0])
    p.add_argument('--budget', type=float, default=1.0)
    p.add_argument('--iterations', type=int, default=10000)
    p.add_argument('--no-simd', action='store_true')
    p.add_argument('--radius', type=float, default=0.1)
    p.add_argument('--spacing', type=float, default=0.15)
    p.add_argument('--lift-speed', type=float, default=0.1, help='unpublished unicycle boundary speed, m/s')
    p.add_argument('--range', dest='extension_range', type=float, default=1.0)
    p.add_argument('--derivative-bound', type=float, default=0.3)
    a = p.parse_args()
    run(a.output, a.cases, a.variants, a.seeds, a.budget, a.iterations, not a.no_simd, a.radius, a.spacing, a.lift_speed, a.extension_range, a.derivative_bound)
