"""Numerical BVP homogeneity audit; this is not a planner timing campaign.

For order r, z[d] = sqrt(rho)*s**(r-d)*Z[d] transforms the
unit-weight reference problem into one with T=s*T_ref and J=rho*s*J_ref.
The reference uses a NumPy Hermite solve, companion-matrix roots, and
orthogonal-polynomial energy, independently of the native root isolation.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from flask_repro import _core
from flask_repro.flat_bvp import FlatEdge, fixed_time, optimal_time
from flask_repro.provenance import file_hash, manifest


def run(output, seeds):
    output.mkdir(parents=True, exist_ok=False)
    settings = dict(
        purpose="numerical_homogeneity_not_planner_performance",
        seeds=seeds,
        time_scales=[1e-6, 1e-3, 1., 1e3, 1e6],
        time_weights=[1e-6, 1., 1e6],
        cases=[[2, 2], [4, 2], [4, 4]],
        duration_relative_tolerance=2e-7,
        cost_relative_tolerance=2e-7,
        boundary_scaled_absolute_tolerance=2e-7,
    )
    provenance = manifest(settings)
    rows = []
    for order, outputs in settings['cases']:
        for seed in seeds:
            rng = np.random.default_rng(seed + 100 * order + outputs)
            start, goal = rng.normal(size=(2, order, outputs))
            reference_duration = optimal_time(start, goal)
            reference_cost = fixed_time(start, goal, reference_duration).cost()
            for scale in settings['time_scales']:
                for rho in settings['time_weights']:
                    factors = np.sqrt(rho) * scale ** (order - np.arange(order))
                    initial, terminal = start * factors[:, None], goal * factors[:, None]
                    raw = _core.flat_bvp(initial.tolist(), terminal.tolist(), rho)
                    edge = FlatEdge(np.asarray(raw['coefficients']), raw['duration'], order)
                    actual_boundaries = edge.state(np.array([0., edge.duration])) / factors[None, :, None]
                    oracle_cost = edge.cost(rho)
                    expected_duration = scale * reference_duration
                    expected_cost = rho * scale * reference_cost
                    duration_error = abs(edge.duration / expected_duration - 1.)
                    cost_error = abs((raw['effort'] + rho * edge.duration) / expected_cost - 1.)
                    energy_error = abs(oracle_cost / expected_cost - 1.)
                    boundary_error = float(np.max(np.abs(actual_boundaries - np.array([start, goal]))))
                    valid = bool(
                        duration_error <= settings['duration_relative_tolerance']
                        and cost_error <= settings['cost_relative_tolerance']
                        and energy_error <= settings['cost_relative_tolerance']
                        and boundary_error <= settings['boundary_scaled_absolute_tolerance']
                    )
                    rows.append(dict(
                        order=order, outputs=outputs, seed=seed, time_scale=scale, rho=rho,
                        start=initial.tolist(), goal=terminal.tolist(),
                        duration_s=edge.duration, expected_duration_s=expected_duration,
                        native_cost=raw['effort'] + rho * edge.duration,
                        independent_energy_cost=oracle_cost, expected_cost=expected_cost,
                        duration_relative_error=duration_error, cost_relative_error=cost_error,
                        independent_energy_relative_error=energy_error,
                        scaled_boundary_absolute_error=boundary_error, valid=valid,
                    ))
    (output / 'measurements.json').write_text(json.dumps(rows, indent=2) + '\n')
    summary = dict(
        attempts=len(rows), passed=sum(r['valid'] for r in rows),
        maximum_errors={key: max(r[key] for r in rows) for key in [
            'duration_relative_error', 'cost_relative_error',
            'independent_energy_relative_error', 'scaled_boundary_absolute_error']},
        limitations=[
            'Generic randomized boundary shapes with prescribed scale transformations.',
            'Near-drift and repeated-root cases require separate constructive regression tests.',
            'Finite floating-point tests do not prove correctness for every possible boundary.',
            'No feasibility, collision, planning speed, or benchmark reproduction conclusion follows.',
        ],
    )
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    after = manifest(settings)
    provenance['source_changed_during_measurement'] = [
        name for name in set(provenance['source_sha256']) | set(after['source_sha256'])
        if provenance['source_sha256'].get(name) != after['source_sha256'].get(name)
    ]
    provenance['artifact_sha256'] = {
        name: file_hash(output / name) for name in ['measurements.json', 'summary.json']
    }
    (output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    if summary['passed'] != summary['attempts'] or provenance['source_changed_during_measurement']:
        raise AssertionError('Numerical audit failed; inspect the retained raw records')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seeds', nargs='+', type=int, default=list(range(1001, 1011)))
    args = parser.parse_args()
    if not args.seeds:
        raise ValueError('At least one development seed is required')
    run(args.output, args.seeds)
