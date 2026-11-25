"""Equal moving endpoints require a return motion when BVP duration is positive."""
import numpy as np
import pytest
from flask_repro import _core
from flask_repro.flat_bvp import connect


@pytest.mark.parametrize('order,outputs,derivative,energy_coefficient',[
    (2,2,1,12.),(2,4,1,12.),
    (4,2,1,100800.),(4,2,2,720.),(4,2,3,40.),
    (4,4,1,100800.),(4,4,2,720.),(4,4,3,40.),
])
def test_equal_nonstationary_boundaries_have_the_analytic_positive_duration(
        order,outputs,derivative,energy_coefficient):
    boundary=np.zeros((order,outputs));boundary[0]=.3;boundary[derivative,0]=1.
    rho=1.7;power=2*order-2*derivative-1
    # With one nonzero boundary derivative, E(T)=K/T^power. These exact
    # constants come from direct Gramian quadratic forms, not root finding.
    expected=(power*energy_coefficient/rho)**(1/(power+1))
    expected_cost=rho*expected*(1+1/power)
    oracle=connect(boundary,boundary,rho)
    native=_core.flat_bvp(boundary.tolist(),boundary.tolist(),rho,-1.)
    assert oracle is not None
    assert oracle.duration==pytest.approx(expected,rel=1e-9)
    assert native['duration']==pytest.approx(expected,rel=1e-9)
    assert oracle.cost(rho)==pytest.approx(expected_cost,rel=1e-9)
    assert native['effort']+rho*native['duration']==pytest.approx(expected_cost,rel=1e-9)
    np.testing.assert_allclose(oracle.state(0.),boundary,atol=1e-10)
    np.testing.assert_allclose(oracle.state(oracle.duration),boundary,atol=1e-10)
