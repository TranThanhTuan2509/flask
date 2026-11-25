"""Early necessary checks must retain the full unicycle analytic validator."""
import json
from pathlib import Path
import numpy as np
import pytest
from flask_repro import _core
from flask_repro.flat_bvp import fixed_time,FlatEdge
from flask_repro.flat_models import Unicycle


def test_early_and_full_checks_agree_with_independent_polynomial_oracle():
    rng=np.random.default_rng(8814);accepted=0
    for _ in range(500):
        start=np.array([rng.uniform(-.5,.5,2),[rng.uniform(.1,.7),rng.uniform(-.15,.15)]])
        goal=np.array([start[0]+[rng.uniform(.1,1.),rng.uniform(-.4,.4)],
                       [rng.uniform(.1,.7),rng.uniform(-.15,.15)]])
        edge=fixed_time(start,goal,rng.uniform(.5,5.))
        expected=Unicycle().edge_valid(edge)
        for enabled in [False,True]:
            assert _core.unicycle_edge_valid(edge.coefficients.tolist(),edge.duration,
                early_constraint_screening=enabled)==expected
        accepted+=expected
    assert 0<accepted<500


@pytest.mark.parametrize('duration',[1e-6,1.,1e6])
def test_straight_motion_is_valid_across_time_scales(duration):
    start=np.array([[0.,0.],[.3,0.]])
    goal=np.array([[.3*duration,0.],[.3,0.]])
    edge=fixed_time(start,goal,duration)
    for enabled in [False,True]:
        assert _core.unicycle_edge_valid(edge.coefficients.tolist(),duration,
            early_constraint_screening=enabled)


def test_between_sample_yaw_witness_is_still_rejected():
    data=json.loads((Path(__file__).parent/'fixtures/unicycle_between_sample_yaw_spike.json').read_text())
    for enabled in [False,True]:
        assert not _core.unicycle_edge_valid(data['coefficients'],data['duration'],
            early_constraint_screening=enabled)


@pytest.mark.parametrize('enabled',[False,True])
def test_finite_coefficients_with_overflowing_derivatives_fail_closed(enabled):
    coefficients=[[0.,0.],[1e308,0.],[-1e308,0.],[1e308,0.]]
    assert not _core.unicycle_edge_valid(coefficients,1e-300,early_constraint_screening=enabled)
