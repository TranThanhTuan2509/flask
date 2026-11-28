"""flask_v2 levers (results/diagnostics/v2_mechanism/RESULT.md).

L1 parent_candidates: cost-ranked parent among the k metric-nearest nodes (RRTConnect).
L3 duration_multipliers: fixed-time fallbacks at c * T* after a dynamic-limit rejection.
Defaults reproduce v1; every returned path is still judged by the independent validator on the original model.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from flask_repro.dynobench import load
from flask_repro.planning import PlannerSettings, plan, validate

ROOT = Path(__file__).resolve().parents[1]
V1 = ROOT.parent / 'flask_repro'          # frozen reproduction (v1) next to this worktree
UNICYCLE = dict(algorithm='rrt_connect', extension_range=1.5, bound1=1.0, goal_tolerance=.2, metric_mode=2, rrt_mode=2,
                goal_mode=1, speed_min=.01, indexed_search=True, spatial_only_validation=True, budget_s=5.,
                max_nodes=200000)
LEVERS = dict(parent_candidates=10, duration_multipliers=(1.5, 2., 3.))


def _unicycle(case):
    model, planning, original, start, goal, _ = load(case, radius=.1, sphere_spacing=.15, shell_only=True,
                                                     shell_keep_boxes=False, lift_speed=.6)
    model.reverse = True
    return model, planning, original, start, goal


def _signature(record):
    # 'moving' (prioritized space-time planning, showcases) is newer than the frozen v1 binary; it must stay 0 here
    assert record.rejections.get('moving', 0) == 0
    rejections = {k: v for k, v in record.rejections.items() if k != 'moving'}
    return dict(solved=record.solved, iterations=record.iterations, rejections=rejections,
                coefficients=[np.asarray(e.coefficients).round(12).tolist() for e in record.trajectory.edges])


def test_defaults_are_v1():
    s = PlannerSettings()
    assert s.parent_candidates == 1 and s.duration_multipliers == ()


@pytest.mark.parametrize('bad', [dict(parent_candidates=0), dict(duration_multipliers=(1.,)),
                                 dict(duration_multipliers=(np.inf,))])
def test_invalid_lever_settings_are_rejected(bad):
    with pytest.raises(ValueError):
        PlannerSettings(**bad).native()


_V1_SCRIPT = r'''
import json, sys
import numpy as np
from flask_repro.dynobench import load
from flask_repro.planning import PlannerSettings, plan
cfg = json.loads(sys.argv[1]); out = []
for case, algorithm, seed in json.loads(sys.argv[2]):
    model, planning, original, start, goal, _ = load(case, radius=.1, sphere_spacing=.15, shell_only=True,
                                                     shell_keep_boxes=False, lift_speed=.6)
    model.reverse = True
    r = plan(model, planning, start, goal, PlannerSettings(**(cfg | dict(algorithm=algorithm))), seed)
    out.append(dict(solved=r.solved, iterations=r.iterations, rejections=r.rejections,
                    coefficients=[np.asarray(e.coefficients).round(12).tolist() for e in r.trajectory.edges]))
print(json.dumps(out))
'''
RUNS = [('unicycle_wall', 'rrt_connect', 1001), ('unicycle_bugtrap', 'rrt_connect', 1002),
        ('unicycle_wall', 'sst_bvp', 1003)]


@pytest.mark.skipif(not (V1 / 'flask_repro').is_dir() or not list((V1 / 'flask_repro').glob('_core*.so')),
                    reason='frozen v1 checkout not present next to this worktree')
def test_levers_off_reproduce_the_frozen_v1_binary_exactly():
    cfg = {k: v for k, v in UNICYCLE.items() if k != 'algorithm'} | dict(max_iterations=20000)
    env = dict(os.environ, PYTHONPATH=str(V1), FLASK_REPRO_ROOT=str(V1))
    v1 = json.loads(subprocess.run([sys.executable, '-c', _V1_SCRIPT, json.dumps(cfg), json.dumps(RUNS)], cwd=V1,
                                   env=env, capture_output=True, text=True, check=True).stdout)
    for (case, algorithm, seed), ref in zip(RUNS, v1):
        model, planning, original, start, goal = _unicycle(case)
        # exact_bounds is a v2 correctness fix (tests/test_exact_bounds.py); off, v2 equals v1 bit for bit
        r = plan(model, planning, start, goal, PlannerSettings(**(cfg | dict(algorithm=algorithm, exact_bounds=False))), seed)
        assert _signature(r) == ref, (case, algorithm, seed)


@pytest.mark.parametrize('case', ['unicycle_wall', 'unicycle_bugtrap'])
def test_levers_return_paths_valid_on_the_original_model(case):
    model, planning, original, start, goal = _unicycle(case)
    cfg = PlannerSettings(**UNICYCLE, **LEVERS)
    for seed in (1001, 1002, 1003):
        r = plan(model, planning, start, goal, cfg, seed)
        assert r.solved
        v = validate(r, model, original, cfg)
        assert v['valid'], v['errors']
        assert r.diagnostics['parent_reranks'] > 0
        assert r.diagnostics['duration_fallback_accepts'] > 0     # L3 is exercised, not dormant


def test_levers_reduce_dynamic_limit_rejections():
    """Negative control: the same seeds without levers reject far more edges on |omega|."""
    model, planning, original, start, goal = _unicycle('unicycle_wall')
    plain = [plan(model, planning, start, goal, PlannerSettings(**UNICYCLE), s) for s in (1001, 1002, 1003)]
    lever = [plan(model, planning, start, goal, PlannerSettings(**UNICYCLE, **LEVERS), s) for s in (1001, 1002, 1003)]
    angular = lambda rs: sum(r.rejections['angular'] for r in rs)
    assert all(r.solved for r in plain + lever)
    assert angular(lever) * 5 < angular(plain)


def test_k1_equals_default_and_multipliers_only_change_rejected_edges():
    model, planning, original, start, goal = _unicycle('unicycle_wall')
    a = plan(model, planning, start, goal, PlannerSettings(**UNICYCLE), 1004)
    b = plan(model, planning, start, goal, PlannerSettings(**UNICYCLE, parent_candidates=1), 1004)
    assert _signature(a) == _signature(b)
    c = plan(model, planning, start, goal, PlannerSettings(**UNICYCLE, duration_multipliers=(2.,)), 1004)
    assert c.diagnostics['duration_fallbacks'] >= c.diagnostics['duration_fallback_accepts'] > 0
    assert a.diagnostics['duration_fallbacks'] == 0
