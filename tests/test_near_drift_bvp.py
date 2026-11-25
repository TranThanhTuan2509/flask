"""Constructive feasible-cost bounds detect root loss and cancellation independently."""
import numpy as np
import pytest

from flask_repro import _core
from flask_repro.flat_bvp import fixed_time,optimal_time
from flask_repro import lqmt


@pytest.mark.parametrize('order,outputs',[(2,2),(4,2),(4,4)])
@pytest.mark.parametrize('distance',[1e-6,1e-10,1e-18,1e-28])
def test_optimal_bvp_cannot_be_worse_than_known_zero_control_drift(order,outputs,distance):
    start=np.zeros((order,outputs));goal=start.copy()
    start[1,0]=goal[1,0]=1.;goal[0,0]=distance
    # This duration follows the integrator chain exactly with zero control.
    # It is an independent feasible upper bound, not a second root solver.
    known=fixed_time(start,goal,distance)
    assert known.effort()==0.
    raw=_core.flat_bvp(start.tolist(),goal.tolist())
    assert 0.<raw['effort']+raw['duration']<=distance*(1+1e-8)
    assert raw['duration']==pytest.approx(distance,rel=1e-8)
    duration=optimal_time(start,goal)
    assert fixed_time(start,goal,duration).cost()<=distance*(1+1e-8)


@pytest.mark.parametrize('distance',[1e-6,1e-10,1e-18,1e-28])
def test_manipulator_duration_and_reported_cost_preserve_near_drift_scale(distance):
    q0=np.zeros(7);q1=q0.copy();q1[0]=distance
    velocity=np.zeros(7);velocity[0]=1.
    result=_core.panda.lqmt(q0.tolist(),velocity.tolist(),q1.tolist(),velocity.tolist(),1.)
    assert result[0]==pytest.approx(distance,rel=1e-8)
    assert result[-1]>0.
    assert result[-1]<=distance*(1+1e-8)
    oracle=lqmt.lqmt_r2(q0,velocity,q1,velocity)
    assert 0.<oracle[-1]<=distance*(1+1e-8)


@pytest.mark.parametrize('order,outputs,inverse_gramian_00',[(2,2,12.),(4,2,100800.),(4,4,100800.)])
@pytest.mark.parametrize('distance',[1e-100,1e-20,1.,1e100])
def test_stationary_boundary_time_scaling_avoids_squaring_small_displacements(order,outputs,inverse_gramian_00,distance):
    start=np.zeros((order,outputs));goal=start.copy();goal[0,0]=distance
    expected=((2*order-1)*inverse_gramian_00)**(1/(2*order))*distance**(1/order)
    native=_core.flat_bvp(start.tolist(),goal.tolist())
    assert native['duration']==pytest.approx(expected,rel=2e-9)
    assert native['effort']==pytest.approx(expected/(2*order-1),rel=2e-8)
    assert optimal_time(start,goal)==pytest.approx(expected,rel=2e-8)
