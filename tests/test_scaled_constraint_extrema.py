"""Joint-limit checking must be invariant to duration/derivative unit scales."""
import numpy as np
import pytest
from flask_repro import _core,lqmt


@pytest.mark.parametrize('duration',[1e-9,1.,1e9])
def test_interior_position_overshoot_is_detected_at_every_time_scale(duration):
    q=np.zeros(6);v=np.full(6,100./duration)
    coefficients=lqmt.hermite_coefficients(q,v,q,v,duration)
    low,high,_,_=lqmt.cubic_extrema(coefficients,duration)
    expected=100./(6*np.sqrt(3.))
    np.testing.assert_allclose(low,-expected,rtol=1e-12)
    np.testing.assert_allclose(high,expected,rtol=1e-12)
    for bound,accepted in [(3.,False),(10.,True)]:
        limits=_core.ur5.JointLimits([-bound]*6,[bound]*6,[200./duration]*6,[1000./duration**2]*6)
        assert _core.ur5.within_limits(q.tolist(),v.tolist(),q.tolist(),v.tolist(),duration,limits)==accepted
        assert lqmt.within_limits(coefficients,duration,[-bound]*6,[bound]*6,
                                  [200./duration]*6,[1000./duration**2]*6)==accepted


@pytest.mark.parametrize('duration',[1e-9,1.,1e9])
def test_interior_velocity_peak_is_detected_at_every_time_scale(duration):
    q0=np.zeros(6);q1=np.ones(6);v=np.zeros(6)
    coefficients=lqmt.hermite_coefficients(q0,v,q1,v,duration)
    _,_,peak,_=lqmt.cubic_extrema(coefficients,duration)
    np.testing.assert_allclose(peak,1.5/duration,rtol=1e-12)
    for bound,accepted in [(1.,False),(2.,True)]:
        limits=_core.ur5.JointLimits([-1.]*6,[2.]*6,[bound/duration]*6,[10./duration**2]*6)
        assert _core.ur5.within_limits(q0.tolist(),v.tolist(),q1.tolist(),v.tolist(),duration,limits)==accepted


def test_linear_and_constant_paths_keep_valid_degenerate_extrema():
    q0=np.zeros(6);q1=np.ones(6);v=np.ones(6)
    limits=_core.ur5.JointLimits([-1.]*6,[2.]*6,[2.]*6,[10.]*6)
    assert _core.ur5.within_limits(q0.tolist(),v.tolist(),q1.tolist(),v.tolist(),1.,limits)
    assert _core.ur5.within_limits(q0.tolist(),q0.tolist(),q0.tolist(),q0.tolist(),1.,limits)
