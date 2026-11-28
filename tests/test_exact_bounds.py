"""Workspace bounds are checked exactly on the polynomial edge (flask_v2).

Regression: the v2 generalization check (results/diagnostics/v2_generalization) found two v1-settings paths whose
robot centre left the workspace by 6 and 15 micrometres between edge samples (the sampled check had a 1e-5 m slack).
With exact_bounds (default) the same queries return paths the independent validator accepts; with the historical
sampled check they do not (negative control).
"""
from dataclasses import replace

import pytest

from benchmarks.v2_generalization import problem
from flask_repro.planning import PlannerSettings, plan, validate

CASES = [('map', 'unicycle_kink', 'forward', 'rrt_dp', 9001), ('random', 9153, 'backward', 'sst_simd', 9153)]


def test_default_is_exact():
    assert PlannerSettings().exact_bounds is True


@pytest.mark.parametrize('kind,key,chart,algorithm,seed', CASES)
def test_grazing_paths_are_rejected_exactly(kind, key, chart, algorithm, seed):
    model, planning, original, start, goal, cfg = problem(kind, key, chart, 'v1', algorithm)
    old = plan(model, planning, start, goal, replace(cfg, exact_bounds=False), seed)
    assert old.solved and validate(old, model, original, cfg)['errors'] != []          # the historical defect
    new = plan(model, planning, start, goal, replace(cfg, exact_bounds=True), seed)
    assert new.solved and validate(new, model, original, cfg)['valid']
