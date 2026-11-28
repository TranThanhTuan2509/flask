"""flask_repro.profiles: the README's unicycle rows can be re-run from a clone.

For every unicycle row of the v2 table (held-out seeds 8101-8130, campaign final_v2d), all 30 attempts must return a path valid on the
ORIGINAL model, and the mean path length must match the tracked campaign summary. The length is compared within
10 % because the native extension is built with -march=native: another CPU may round differently and grow a
different (equally valid) tree. On the measuring machine the runs are identical attempt by attempt (checked
when the raw per-seed records are present).
"""
import json
from pathlib import Path

import numpy as np
import pytest

from flask_repro.planning import plan, validate
from flask_repro.profiles import load_profile

ROOT = Path(__file__).resolve().parents[1]
CAMPAIGN = ROOT / 'results/paper_campaigns/2026-11-28_final_v2d/table_i_unicycle'
SEEDS = range(8101, 8131)
V2C = ROOT / 'results/paper_campaigns/2026-11-28_final_v2c/table_i_unicycle'   # same settings without L7


@pytest.mark.parametrize('case', ['unicycle_bugtrap', 'unicycle_wall'])
@pytest.mark.parametrize('algorithm', ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp'])
def test_unicycle_rows_of_the_readme_rerun(case, algorithm):
    s = load_profile(case, algorithm)
    summary = json.loads((CAMPAIGN / 'summary.json').read_text())[f'{case}/{algorithm}']
    lengths, identical, raw = [], 0, 0
    for seed in SEEDS:
        r = plan(s.model, s.planning, s.start, s.goal, s.settings, seed)
        assert r.solved, seed
        v = validate(r, s.model, s.original, s.settings)
        assert v['valid'], (seed, v['errors'])
        lengths.append(r.trajectory.length(2))
        record = CAMPAIGN / f'{case}_{algorithm}_seed_{seed}.json'
        if record.exists():
            raw += 1
            ref = json.loads(record.read_text())
            identical += r.iterations == ref['iterations'] and abs(lengths[-1] - ref['length_m']) < 1e-9
    assert summary['validated_solutions'] == len(SEEDS)
    assert abs(np.mean(lengths) / summary['valid_solution_length_m']['mean'] - 1) < .10
    assert identical == raw                       # 0 == 0 in a clone (raw records are not distributed)


def test_quadrotor_full_model_profile():
    s = load_profile('quad3d_block', 'rrt_connect')
    assert s.model.order == 4 and s.profile_file.endswith('low_dof_v2d_order4.json')
    assert s.settings.shortcut_subdivisions == 4 and s.settings.shortcut_rounds == 3
    assert load_profile('quad3d_block', 'rrt_connect', 'v1').settings.shortcut_rounds == 1
    for seed in (8001, 8002, 8003):
        r = plan(s.model, s.planning, s.start, s.goal, s.settings, seed)
        assert r.solved and validate(r, s.model, s.original, s.settings)['valid']


def test_profiles_and_overrides():
    v1, v2 = load_profile('unicycle_wall', 'rrt_connect', 'v1'), load_profile('unicycle_wall', 'rrt_connect')
    assert v1.settings.parent_candidates == 1 and v2.settings.parent_candidates == 5
    assert v1.settings.shortcut_rounds == 1 and v2.settings.shortcut_subdivisions == 4 and v2.settings.shortcut_rounds == 3
    assert v1.model.reverse is False and v2.model.reverse is False              # Wall drives forward (Fig. 5a)
    assert load_profile('unicycle_bugtrap').model.reverse is True               # Bugtrap drives backward
    assert load_profile('unicycle_wall', budget_s=1.).settings.budget_s == 1.
    assert load_profile('quad2d_hole', profile='order2').model.order == 2
    for bad in (dict(case='unicycle_maze'), dict(case='unicycle_wall', algorithm='rrt_star'),
                dict(case='unicycle_wall', profile='order2')):
        with pytest.raises(ValueError):
            load_profile(**bad)


@pytest.mark.parametrize('case', ['unicycle_bugtrap', 'unicycle_wall'])
def test_l7_off_is_the_release_campaign_without_l7(case):
    """With L7 switched off, the v2 profile repeats campaign final_v2c (paper Alg. 4 only) attempt by attempt."""
    s = load_profile(case, 'rrt_connect', shortcut_subdivisions=1, shortcut_rounds=1)
    raw = 0
    for seed in range(8001, 8011):
        r = plan(s.model, s.planning, s.start, s.goal, s.settings, seed)
        assert r.solved and validate(r, s.model, s.original, s.settings)['valid'], seed
        record = V2C / f'{case}_rrt_connect_seed_{seed}.json'
        if record.exists():
            raw += 1
            ref = json.loads(record.read_text())
            assert r.iterations == ref['iterations'] and abs(r.trajectory.length(2) - ref['length_m']) < 1e-9, seed
    assert raw in (0, 10)
