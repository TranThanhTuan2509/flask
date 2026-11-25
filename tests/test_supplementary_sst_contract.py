"""Graph and incumbent invariants for the explicitly supplementary SST* code."""
import numpy as np
import pytest
from flask_repro import _core
from flask_repro.flat_planner import World
from flask_repro.original_sst import OrigSettings,dynobench_distance,plan,validate


@pytest.mark.parametrize('phase_iterations',[40,1000000])
def test_active_vertices_are_exactly_witness_representatives(phase_iterations):
    world=World([-3.,-3.],[3.,3.]);world.prepare_simd()
    result=plan('unicycle_wall',world,[0.,0.,0.],[2.8,2.8,0.],
        OrigSettings(budget_s=5.,max_iterations=2000,phase_iterations=phase_iterations,
                     shrink=.95,selection_radius=.3,pruning_radius=.03,stop_first=False,trace=True),1007)
    assert result['iterations']==2000
    d=result['diagnostics'];states=np.asarray(d['node_states']);parents=np.asarray(d['parents'])
    alive=np.asarray(d['alive'],bool);active=np.asarray(d['active'],bool)
    children=np.asarray(d['children']);representatives=np.asarray(d['representatives'])
    witnesses=np.asarray(d['witnesses']);costs=np.asarray(d['costs'])
    np.testing.assert_array_equal(np.sort(representatives),np.flatnonzero(alive&active))
    assert all(dynobench_distance('unicycle',states[r],w)<=d['pruning_radius']+1e-12
               for r,w in zip(representatives,witnesses))
    assert active[0] and alive[0] and costs[0]==0.
    np.testing.assert_array_equal(children,np.bincount(parents[np.flatnonzero(alive)[1:]],minlength=len(states)))
    assert np.all(children[alive&~active]>0)
    for i in np.flatnonzero(alive)[1:]:
        assert alive[parents[i]] and parents[i]<i and costs[parents[i]]<costs[i]
    if phase_iterations==1000000:
        assert np.any(~alive),'The uninterrupted fixture must exercise pruning'
    assert bool(d['phase_restarts'])==(phase_iterations==40)


def test_incumbent_survives_fresh_phase_without_pinning_old_representatives():
    world=World([-3.,-3.],[3.,3.]);world.prepare_simd()
    start=[0.,0.,0.];goal=[.3,0.,0.]
    result=plan('unicycle_wall',world,start,goal,
        OrigSettings(budget_s=5.,max_iterations=501,phase_iterations=500,goal_bias=1.,
            selection_radius=.5,pruning_radius=.01,stop_first=False,trace=True),1007)
    assert result['solved'] and result['diagnostics']['phase_restarts']==[500]
    assert validate('unicycle_wall',result,world,start,goal)['valid']
    assert len(result['controls'])==len(result['states'])-1


def test_invalid_initial_state_cannot_escape_into_free_space():
    world=World([-1.,-1.],[1.,1.],boxes=[([0.,0.],[.02,.02])]);world.prepare_simd()
    result=plan('unicycle_wall',world,[0.,0.,0.],[.8,0.,0.],OrigSettings(max_iterations=100),1007)
    assert not result['solved'] and result['propagation_steps']==0


def test_satisfied_initial_goal_returns_zero_cost_identity():
    world=World([-1.,-1.],[1.,1.]);world.prepare_simd()
    result=plan('unicycle_wall',world,[0.,0.,0.],[.05,0.,0.],OrigSettings(trace=True),1007)
    assert result['solved'] and result['best_cost']==0. and result['iterations']==0
    assert result['states']==[[0.,0.,0.]] and result['controls']==[]


def test_supplementary_binding_rejects_dimension_and_schedule_errors():
    world=World([-1.,-1.],[1.,1.]);world.prepare_simd();cfg=OrigSettings().native('unicycle')
    with pytest.raises(ValueError):
        _core.orig_sst_plan('unicycle',world.cpp_environment,[0.,0.,0.],[.8,0.,0.],[-1.]*3,[1.]*3,.1,cfg,1)
    cfg.phase_iterations=0
    with pytest.raises(ValueError):
        _core.orig_sst_plan('unicycle',world.cpp_environment,[0.,0.,0.],[.8,0.,0.],[-1.]*2,[1.]*2,.1,cfg,1)
