"""Independent propagation quadrature and an observed SST root obstruction."""
from dataclasses import replace
import numpy as np
import pytest

from benchmarks.native_campaign import configuration_diagnostics
from flask_repro.flat_bvp import propagate
from flask_repro.flat_models import Quad2D
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings,plan
from flask_repro.scales import rest_propagation_scale


@pytest.mark.parametrize('order,bounds,interval',[
    (2,[1.,3.],(0.,.3)),(4,[2.,2.],(0.,.3)),
    (4,[2.,1.,.5,3.],(.1,.3)),(4,[.2,2.],(.3,.3)),
    (4,[1.,2.],(.3,np.nextafter(.3,1.)))])
def test_scale_matches_independent_propagated_edges_and_quadrature(order,bounds,interval):
    lo,hi=interval;weight=.37
    scale=rest_propagation_scale(order,bounds,lo,hi,weight)
    weights=np.r_[1.,weight/3.**np.arange(order-1)]
    root=np.zeros((order,len(bounds)));root[0]=np.arange(len(bounds))+.73
    def distance(control,duration):
        state=propagate(root,control,duration).state(duration)
        return np.sum(weights[:,None]*(state-root)**2)
    np.testing.assert_allclose(scale.maximum**2,distance(np.asarray(bounds),hi),rtol=2e-14)
    # Gauss-Legendre integrates the independent polynomial evaluator, with
    # two control points per output integrating each uniform control square.
    times,time_weights=np.polynomial.legendre.leggauss(10)
    mean=0.
    for time,quadrature_weight in zip(times,time_weights):
        duration=(hi+lo)/2+(hi-lo)/2*time
        mean+=quadrature_weight/2*distance(np.asarray(bounds)/np.sqrt(3.),duration)
    np.testing.assert_allclose(scale.rms**2,mean,rtol=2e-13)


@pytest.mark.parametrize('kwargs',[
    dict(order=0),dict(order=True),dict(control_bounds=[]),dict(control_bounds=[0.]),
    dict(duration_min=.4),dict(duration_max=float('inf')),
    dict(derivative_metric_weight=float('nan'))])
def test_invalid_scale_inputs_are_rejected(kwargs):
    arguments=dict(order=4,control_bounds=[2.,2.],duration_min=0.,duration_max=.3)
    with pytest.raises(ValueError):rest_propagation_scale(**(arguments|kwargs))


@pytest.mark.parametrize('indexed_search',[False,True])
@pytest.mark.parametrize('algorithm',['sst_bvp','sst_simd'])
def test_reported_root_obstruction_matches_native_tree_and_minimal_fix(indexed_search,algorithm):
    model=Quad2D();start=[0.,0.,0.,0.,0.,0.];goal=[8.,8.,0.,0.,0.,0.]
    world=World([-10.,-10.],[10.,10.])
    cfg=PlannerSettings(algorithm=algorithm,max_iterations=1000,phase_iterations=10000,
        budget_s=30.,stop_first=False,trace=True,indexed_search=indexed_search)
    diagnostic=configuration_diagnostics(model,start,cfg)
    assert diagnostic['initial_phase_blocked_by_radius']
    blocked=plan(model,world,start,goal,cfg,6117)
    assert len(blocked.diagnostics['start_states'])==1
    assert blocked.diagnostics['accepted_propagations']>0
    assert blocked.diagnostics['accepted_propagations']==blocked.diagnostics['witness_rejections']
    corrected=replace(cfg,witness_radius=.25*diagnostic['rms_initial_step_metric'])
    assert not configuration_diagnostics(model,start,corrected)['initial_phase_blocked_by_radius']
    expanded=plan(model,world,start,goal,corrected,6117)
    assert len(expanded.diagnostics['start_states'])>10
    assert expanded.diagnostics['phase_restart_iterations']==[]


def test_diagnostic_does_not_apply_rest_formula_to_moving_boundary_or_other_metric():
    model=Quad2D();start=[0.,0.,0.,.1,0.,0.];cfg=PlannerSettings(algorithm='sst_simd')
    assert not configuration_diagnostics(model,start,cfg)['rest_root_scale_applicable']
    assert not configuration_diagnostics(model,[0.]*6,replace(cfg,metric_mode=1))['rest_root_scale_applicable']
