"""Published SST selection and graph invariants, independent of solution timing."""
import numpy as np
import pytest

from flask_repro import _core
from flask_repro.flat_models import Unicycle
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings,plan


@pytest.mark.parametrize('order,outputs',[(2,2),(4,2),(4,4)])
def test_best_near_uses_cost_inside_ball_and_excludes_inactive_nodes(order,outputs):
    states=np.zeros((4,order,outputs));states[:,0,0]=[.01,.4,.2,3.]
    costs=[10.,2.,.1,0.];target=np.zeros((order,outputs))
    choose=lambda active,alive,radius:_core.flat_best_near(
        states.tolist(),costs,active,alive,target.tolist(),radius,.2)
    # Closest is node zero; node one has the best active cost in the ball.
    assert choose([True,True,False,True],[True]*4,.5)==1
    # A deleted node must not win, even if its stale active flag is true.
    assert choose([True]*4,[True,True,False,True],.5)==1
    # Empty ball falls back to nearest active node, not the global cheapest.
    assert choose([True,True,False,True],[True]*4,.001)==0
    with pytest.raises(RuntimeError,match='no active'):
        choose([False]*4,[True]*4,.5)


@pytest.mark.parametrize('algorithm',['sst_bvp','sst_simd'])
@pytest.mark.parametrize('phase_iterations',[1000000,40])
@pytest.mark.parametrize('indexed_search',[False,True])
def test_witness_replacement_prunes_inactive_leaves_and_keeps_parent_chains(algorithm,phase_iterations,indexed_search):
    cfg=PlannerSettings(algorithm=algorithm,max_iterations=2000,budget_s=3.,
        selection_radius=.3,witness_radius=.04,control_bound=.8,
        propagation_min=0.,propagation_max=.5,phase_iterations=phase_iterations,
        shrink_factor=.95,stop_first=False,trace=True,indexed_search=indexed_search)
    result=plan(Unicycle(),World([-10.,-10.],[10.,10.]),[0.,0.,0.],[9.,9.,0.],cfg,seed=1007)
    d=result.diagnostics
    states=np.asarray(d['start_states']);parents=np.asarray(d['start_parents'])
    alive=np.asarray(d['start_alive'],bool);active=np.asarray(d['start_active'],bool)
    costs=np.asarray(d['start_costs']);children=np.asarray(d['start_children'])
    representatives=np.asarray(d['witness_representatives']);witnesses=np.asarray(d['witness_states'])
    assert active[0] and alive[0] and parents[0]==0 and costs[0]==0.
    assert np.all(alive[representatives]&active[representatives])
    np.testing.assert_array_equal(np.sort(representatives),np.flatnonzero(active&alive))
    delta=(states[representatives]-witnesses).reshape(-1,2,2)
    distances=np.sqrt(np.sum(delta[:,0]**2,axis=1)+cfg.derivative_metric_weight*np.sum(delta[:,1]**2,axis=1))
    assert np.all(distances<=d['pruning_radius']+1e-10)
    actual_children=np.bincount(parents[np.flatnonzero(alive)[1:]],minlength=len(states))
    np.testing.assert_array_equal(children,actual_children)
    assert np.all(children[alive&~active]>0)
    for index in np.flatnonzero(alive)[1:]:
        assert alive[parents[index]] and parents[index]<index
        assert costs[index]>costs[parents[index]]
    assert np.any(~alive),'The fixture must exercise actual inactive-leaf pruning'
    assert bool(d['phase_restart_iterations'])==(phase_iterations<2000)
