"""Positive and deliberately invalid controls for the post-hoc interval oracle."""
from fractions import Fraction
import numpy as np
import pytest
from flask_repro.interval_validation import audit_segment,bernstein_jets,hull,measures,iv
from flask_repro.flat_bvp import fixed_time
from flask_repro.flat_models import Quad2D,Quad3D


@pytest.mark.parametrize('robot,outputs',[('quad2d',2),('quad3d',4)])
def test_hover_has_certified_force_and_zero_rotation(robot,outputs):
    coefficients=np.zeros((8,outputs));coefficients[0,outputs-2]=1.
    assert audit_segment(coefficients,2.,robot)['status']=='clear'


@pytest.mark.parametrize('robot,outputs',[('quad2d',2),('quad3d',4)])
def test_excessive_vertical_acceleration_is_rejected(robot,outputs):
    coefficients=np.zeros((8,outputs));axis=1 if outputs==2 else 2
    coefficients[0,axis]=1.;coefficients[2,axis]=10.
    assert audit_segment(coefficients,1.,robot)['status']=='constraint_violation'


def test_excessive_yaw_rate_is_rejected():
    coefficients=np.zeros((8,4));coefficients[0,2]=1.;coefficients[1,3]=12.
    result=audit_segment(coefficients,1.,'quad3d',max_nodes=500)
    assert result['status']=='constraint_violation'
    assert result['constraint']=='angular_speed_squared'


def test_singular_freefall_fails_closed():
    coefficients=np.zeros((8,4));coefficients[0,2]=1.;coefficients[2,2]=-9.81/2
    assert audit_segment(coefficients,1.,'quad3d',max_depth=6)['status']=='unresolved'


@pytest.mark.parametrize('robot,outputs',[('quad2d',2),('quad3d',4)])
def test_nontrivial_smooth_translation_has_valid_interval_bounds(robot,outputs):
    start=np.zeros((4,outputs));goal=start.copy();goal[0,0]=1.
    edge=fixed_time(start,goal,6.)
    result=audit_segment(edge.coefficients,edge.duration,robot)
    assert result['status']=='clear'
    model=Quad2D() if outputs==2 else Quad3D()
    assert np.all(model.valid(model.recover(edge.jet(np.linspace(0,edge.duration,501),4))))


def test_derivative_hulls_include_an_exact_rational_endpoint():
    coefficients=np.zeros((8,4));coefficients[7,0]=.1
    jets=bernstein_jets(coefficients,.3)
    expected=Fraction(.1)*7*6*5*4/Fraction(.3)**4
    enclosure=hull(jets[4])[0]
    exact=iv.mpf(expected.numerator)/expected.denominator
    assert enclosure.a<=exact.a and enclosure.b>=exact.b


def test_point_recovery_agrees_with_independent_newton_euler_model():
    rng=np.random.default_rng(7721)
    for _ in range(20):
        jet=rng.uniform(-.2,.2,(5,4))
        intervals=[[iv.mpf(float(x)) for x in row] for row in jet]
        result=measures(intervals,'quad3d');reference=Quad3D().recover(jet)
        expected=dict(speed_squared=np.sum(reference['velocity']**2),
                      angular_speed_squared=np.sum(reference['omega']**2),
                      thrust_squared=reference['thrust']**2,torque_squared=np.sum(reference['torque']**2))
        for name,value in expected.items():
            bound=result[name][0]
            assert float(bound.a)-1e-10<=value<=float(bound.b)+1e-10


def test_excessive_yaw_torque_is_rejected_without_a_rate_violation():
    coefficients=np.zeros((8,4));coefficients[0,2]=1.;coefficients[2,3]=.1
    result=audit_segment(coefficients,.1,'quad3d')
    assert result['status']=='constraint_violation' and result['constraint']=='torque_squared'


@pytest.mark.parametrize('arguments',[dict(max_nodes=0),dict(max_depth=-1)])
def test_invalid_subdivision_budget_is_rejected(arguments):
    with pytest.raises(ValueError,match='budget'):
        audit_segment(np.zeros((8,4)),1.,'quad3d',**arguments)


def test_oracle_precision_is_independent_of_global_interval_settings():
    from mpmath import iv as shared
    previous=shared.dps
    try:
        shared.dps=3
        assert iv.dps==30
        assert audit_segment(np.zeros((8,4)),1.,'quad3d')['status']=='clear'
    finally:shared.dps=previous
