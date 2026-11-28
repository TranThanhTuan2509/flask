"""flask_v2 lever L7: Alg. 4 over subdivided edges (Settings::shortcut_subdivisions / shortcut_rounds).

Off by default (the paper's single pass; tests/test_profiles.py checks the campaign attempts stay identical). On, it
must keep every path valid on the original model, never raise the LQMT cost of the path (a bypass is accepted only
if cheaper), keep the path continuous, and leave the search itself unchanged.
"""
import numpy as np
import pytest

from flask_repro.planning import PlannerSettings, plan, validate
from flask_repro.profiles import load_profile


def cost(r, rho):
    return sum(e.cost(rho) for e in r.trajectory.edges)


@pytest.mark.parametrize('case,algorithm', [('unicycle_wall', 'rrt_connect'), ('unicycle_bugtrap', 'sst_bvp'),
                                            ('unicycle_wall', 'rrt_dp'), ('quad2d_hole', 'rrt_connect')])
def test_l7_keeps_paths_valid_and_never_more_expensive(case, algorithm):
    off = load_profile(case, algorithm, shortcut_subdivisions=1, shortcut_rounds=1)
    on = load_profile(case, algorithm, shortcut_subdivisions=4, shortcut_rounds=3)
    for seed in (11, 12, 13, 14, 15):
        a = plan(off.model, off.planning, off.start, off.goal, off.settings, seed)
        b = plan(on.model, on.planning, on.start, on.goal, on.settings, seed)
        assert a.solved == b.solved and a.iterations == b.iterations          # the search is untouched
        if not b.solved:
            continue
        assert validate(b, on.model, on.original, on.settings)['valid'], seed
        assert cost(b, on.settings.rho) <= cost(a, off.settings.rho) + 1e-9
        assert b.trajectory.continuity_error() < 1e-6


def test_l7_shortens_a_detour_on_the_wall():
    """Seeds where the first RRTConnect solution detours through a far node (diagnosed on development seeds)."""
    off = load_profile('unicycle_wall', 'rrt_connect', shortcut_subdivisions=1, shortcut_rounds=1)
    on = load_profile('unicycle_wall', 'rrt_connect', shortcut_subdivisions=4, shortcut_rounds=3)
    before = after = 0.
    for seed in range(1001, 1011):
        before += plan(off.model, off.planning, off.start, off.goal, off.settings, seed).trajectory.length(2)
        after += plan(on.model, on.planning, on.start, on.goal, on.settings, seed).trajectory.length(2)
    assert after < 0.9 * before


@pytest.mark.parametrize('bad', [dict(shortcut_subdivisions=0), dict(shortcut_rounds=0), dict(shortcut_rounds=True),
                                 dict(shortcut_subdivisions=2.5)])
def test_l7_rejects_bad_settings(bad):
    with pytest.raises(ValueError):
        PlannerSettings(**bad).native()


def test_l7_defaults_are_the_paper():
    s = PlannerSettings()
    assert s.shortcut_subdivisions == 1 and s.shortcut_rounds == 1
