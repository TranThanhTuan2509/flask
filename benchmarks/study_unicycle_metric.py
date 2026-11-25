"""Development-only sensitivity study of the reconstructed flat-state metric.

This is not a paper timing campaign and does not recover author parameters.
Settings other than the metric weight are held fixed. Every returned trajectory
is independently checked in the original box geometry and physical model.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from flask_repro.dynobench import load
from flask_repro.planning import PlannerSettings, plan, validate
from flask_repro.provenance import save_manifest


def run(output, weights, seeds, budget, iterations):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for case in ('unicycle_bugtrap', 'unicycle_wall'):
        model, planning, original, start, goal, provenance = load(case, lift_speed=.3)
        for weight in weights:
            for seed in seeds:
                settings = PlannerSettings(
                    budget_s=budget, max_iterations=iterations,
                    derivative_metric_weight=weight, trace=True)
                begin = perf_counter_ns()
                result = plan(model, planning, start, goal, settings, seed)
                wrapper_ns = perf_counter_ns() - begin
                validation = validate(result, model, original, settings)
                trace_statistics = {}
                for tree in ('start', 'goal'):
                    states = np.asarray(result.diagnostics[tree+'_states']).reshape(-1, 4)
                    parents = np.asarray(result.diagnostics[tree+'_parents'])
                    counts = np.bincount(parents)
                    trace_statistics[tree] = dict(
                        allocated_nodes=len(states), distinct_states=len(np.unique(states, axis=0)),
                        largest_parent_child_count=int(counts.max()),
                        position_min=states[:, :2].min(0).tolist(),
                        position_max=states[:, :2].max(0).tolist())
                record = dict(
                    purpose='reconstructed_metric_sensitivity_not_final_timing',
                    case=case, seed=seed, settings=asdict(settings), provenance=provenance,
                    solved=result.solved, validation=validation, iterations=result.iterations,
                    nodes=result.nodes, plan_ns=result.plan_ns,
                    postprocess_ns=result.postprocess_ns, wrapper_ns=wrapper_ns,
                    rejections=result.rejections, trace_statistics=trace_statistics,
                    edges=result.trajectory.to_records())
                records.append(record)
                (output/'records.json').write_text(json.dumps(records, indent=2)+'\n')
                print(json.dumps(dict(case=case, weight=weight, seed=seed,
                                      solved=result.solved, valid=validation['valid'],
                                      iterations=result.iterations, nodes=result.nodes)), flush=True)
    save_manifest(output/'records.json', dict(
        purpose='reconstructed_metric_sensitivity_not_final_timing', weights=weights,
        seeds=seeds, budget_s=budget, max_iterations=iterations),
        [Path('docs/sources/flask_v4.pdf')])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--weights', nargs='+', type=float, default=[.2, 1., 4., 16., 64.])
    parser.add_argument('--seeds', nargs='+', type=int, default=[0, 1])
    parser.add_argument('--budget', type=float, default=2.)
    parser.add_argument('--iterations', type=int, default=10000)
    args = parser.parse_args()
    run(args.output, args.weights, args.seeds, args.budget, args.iterations)
