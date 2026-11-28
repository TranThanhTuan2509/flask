"""Ready-to-use DynoBench problems with the planner settings behind the README tables.

    from flask_repro.profiles import load_profile
    from flask_repro.planning import plan, validate

    s = load_profile('unicycle_wall', 'rrt_connect')            # v2 settings (README headline)
    r = plan(s.model, s.planning, s.start, s.goal, s.settings, seed=1)
    print(r.solved, validate(r, s.model, s.original, s.settings)['valid'])

The problem is built exactly as benchmarks/native_campaign.py builds it (same geometry, boundary lift, flat chart,
model overrides and per-planner settings), so a seed run here repeats the corresponding campaign attempt.

Profiles:
  'v2'      unicycle: config/low_dof_v2d_order2.json (v2 options L1/L3/L5/L6 and the L7 shortcut rounds, Bugtrap
            backward / Wall forward chart);
            quadrotors: config/low_dof_v2d_order4.json (full model, r = 4; L7 for RRTConnect and BVP-SST*). [default]
  'v1'      unicycle: config/low_dof_v1_literal_v2d.json (the paper read literally, same charts);
            quadrotors: config/low_dof_v2_order4.json (full model, the paper's single Alg. 4 pass).
  'order2'  quadrotors at the paper's reduced order r = 2 (config/low_dof_v2d_quad_order2.json); NOT feasible for
            the full quadrotor model, kept to compare with the paper's numbers.

The v2 settings without L7 (the paper's single Alg. 4 pass, release campaign final_v2c) are
load_profile(..., shortcut_subdivisions=1, shortcut_rounds=1).
"""
import json
from dataclasses import dataclass, replace
from pathlib import Path

from .dynobench import load
from .flat_models import Unicycle
from .planning import PlannerSettings

ROOT = Path(__file__).resolve().parents[1]
CASES = ('unicycle_bugtrap', 'unicycle_wall', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window')
ALGORITHMS = ('rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp')
_FILES = {('v2', 'unicycle'): 'low_dof_v2d_order2.json', ('v2', 'quad'): 'low_dof_v2d_order4.json',
          ('v1', 'unicycle'): 'low_dof_v1_literal_v2d.json', ('v1', 'quad'): 'low_dof_v2_order4.json',
          ('order2', 'quad'): 'low_dof_v2d_quad_order2.json'}


@dataclass(frozen=True)
class Setup:
    case: str
    algorithm: str
    profile_file: str
    model: object
    planning: object          # collision world the planner uses (sphere cover)
    original: object          # original world for independent validation
    start: object
    goal: object
    settings: PlannerSettings


def load_profile(case, algorithm='rrt_connect', profile='v2', **overrides):
    """Returns a Setup for `case` and `algorithm`; keyword overrides replace individual PlannerSettings fields."""
    if case not in CASES:
        raise ValueError(f'unknown case {case!r}; choose one of {CASES}')
    if algorithm not in ALGORITHMS:
        raise ValueError(f'unknown algorithm {algorithm!r}; choose one of {ALGORITHMS}')
    family = 'unicycle' if case.startswith('unicycle') else 'quad'
    if (profile, family) not in _FILES:
        raise ValueError(f'profile {profile!r} is not defined for {family} cases')
    path = ROOT / 'config' / _FILES[profile, family]
    p = json.loads(path.read_text())
    ov = p.get('case_overrides', {}).get(case, {})
    geometry = p['geometry'] | ov.get('geometry', {})
    boundary = p['boundary'] | ov.get('boundary', {})
    model, planning, original, start, goal, _ = load(case, **geometry, lift_speed=boundary['lift_speed'])
    raw = p['settings'] | ov.get('settings', {}) | ov.get('algorithm_settings', {}).get(algorithm, {}) | dict(algorithm=algorithm)
    if geometry.get('shell_only') and not geometry.get('shell_keep_boxes', True):
        thickness = original.shell_thickness(geometry.get('sphere_spacing', .15))
        if not raw.get('validation_spacing', .015) < thickness / 1.01:
            raise ValueError(f'{case}: validation spacing does not resolve the sphere-shell thickness {thickness}')
    if boundary.get('rest_turns'):
        model.rest_turns = True
    if isinstance(model, Unicycle):
        model.reverse = boundary['reverse_chart']
    for key, value in (p.get('model', {}) | ov.get('model', {})).items():
        setattr(model, key, value)
    settings = replace(PlannerSettings(**raw), **overrides) if overrides else PlannerSettings(**raw)
    return Setup(case, algorithm, str(path.relative_to(ROOT)), model, planning, original, start, goal, settings)
