"""Resource, ablation and proposal contracts for the native FLASK planners."""
from dataclasses import replace
import numpy as np
import pytest

from flask_repro import _core
from flask_repro.dynobench import load
from flask_repro.flat_models import Quad3D,Unicycle
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings,plan


def aerial_problem():
    return Quad3D(),World([-10.]*3,[10.]*3),[-5.,-5.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.],[5.,5.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.]


@pytest.mark.parametrize('mode',[0,1,2])
def test_low_dof_connect_never_exceeds_shared_node_capacity(mode):
    model,world,_,start,goal,_=load('quad3d_window')
    for seed in range(20):
        cfg=PlannerSettings(rrt_mode=mode,max_nodes=4,budget_s=1.,max_iterations=20,extension_range=.4)
        result=plan(model,world,start,goal,cfg,seed)
        assert result.nodes<=cfg.max_nodes


@pytest.mark.parametrize('algorithm',['sst_simd','rrt_dp'])
def test_propagation_only_planning_never_calls_bvp(algorithm):
    model,world,start,goal=aerial_problem()
    cfg=PlannerSettings(algorithm=algorithm,max_iterations=50,budget_s=10.)
    result=plan(model,world,start,goal,cfg,41)
    assert not result.solved
    assert result.reason=='iteration_limit'
    assert result.diagnostics['planning_bvp_calls']==0
    assert result.diagnostics['accepted_propagations']>0


def test_per_output_control_bounds_preserve_exact_propagation_units():
    model,world,start,goal=aerial_problem()
    cfg=PlannerSettings(algorithm='rrt_dp',control_bounds=(.1,.2,.3,.4),
        propagation_min=.1,propagation_max=.1,max_iterations=100,budget_s=10.,trace=True)
    result=plan(model,world,start,goal,cfg,42)
    states=np.asarray(result.diagnostics['start_states']).reshape(-1,4,4)
    parents=np.asarray(result.diagnostics['start_parents'])
    assert len(states)>10
    # The independent identity jerk(t+h)-jerk(t)=snap*h applies per output.
    controls=(states[1:,3]-states[parents[1:],3])/.1
    assert np.all(np.abs(controls)<=np.asarray(cfg.control_bounds)+1e-12)
    assert np.all(np.max(np.abs(controls),axis=0)>.4*np.asarray(cfg.control_bounds))


def test_equal_vector_and_scalar_bounds_keep_the_same_rng_and_tree():
    model,world,start,goal=aerial_problem()
    cfg=PlannerSettings(algorithm='rrt_dp',max_iterations=100,budget_s=10.,trace=True)
    a=plan(model,world,start,goal,cfg,42)
    b=plan(model,world,start,goal,replace(cfg,control_bounds=(2.,)*4),42)
    np.testing.assert_array_equal(a.diagnostics['start_states'],b.diagnostics['start_states'])
    np.testing.assert_array_equal(a.diagnostics['start_parents'],b.diagnostics['start_parents'])


def test_rest_mixture_retains_uniform_samples_and_is_not_the_default():
    model,world,start,goal=aerial_problem()
    cfg=PlannerSettings(algorithm='rrt_dp',rest_sample_probability=.5,
        max_iterations=100,budget_s=10.)
    result=plan(model,world,start,goal,cfg,42)
    counters=result.diagnostics
    assert 0<counters['rest_samples']<counters['samples']
    assert PlannerSettings().rest_sample_probability==0.
    with pytest.raises(ValueError,match='singular'):
        plan(Unicycle(),World([-5.]*2,[5.]*2),[0.,0.,0.],[1.,1.,0.],cfg)
    with pytest.raises(ValueError,match='support'):
        replace(cfg,rest_sample_probability=1.).native()


@pytest.mark.parametrize('overrides',[
    dict(rho=float('nan')),dict(range=float('inf')),dict(goal_bias=float('nan')),
    dict(shrink=1.),dict(control_bounds=[1.,2.,3.]),dict(control_bounds=[1.,2.,3.,float('inf')]),
    dict(rest_sample_probability=1.),dict(metric_mode=99),dict(bound1=float('nan')),
])
def test_direct_native_entry_rejects_invalid_configuration(overrides):
    model,world,start,goal=aerial_problem();world.prepare_simd();cfg=PlannerSettings().native()
    for name,value in overrides.items():setattr(cfg,name,value)
    with pytest.raises(ValueError):
        _core.flat_plan('quad3d',world.cpp_environment,model.lift(start).tolist(),model.lift(goal).tolist(),
            world.lower.tolist(),world.upper.tolist(),world.radius,cfg,42,0)


def test_exhaustion_reasons_distinguish_time_iteration_and_node_limits():
    model,world,start,goal=aerial_problem()
    base=PlannerSettings(algorithm='rrt_dp',budget_s=10.)
    assert plan(model,world,start,goal,replace(base,max_iterations=1),42).reason=='iteration_limit'
    assert plan(model,world,start,goal,replace(base,max_nodes=4),42).reason=='node_limit'
    timed=plan(model,world,start,goal,replace(base,budget_s=1e-9),42)
    assert timed.reason=='time_limit' and timed.iterations==0


def test_heading_embedding_retains_opposite_headings_at_small_speeds():
    for speed in [1e-8,.1,1.]:
        a=np.array([[0.,0.],[speed,0.]]);b=-a
        actual=_core.unicycle_search_distance(a.tolist(),b.tolist())
        assert actual==pytest.approx(np.hypot(np.sqrt(.2)*2*speed,4./3.))
    a=np.array([[1.,2.],[.1,-1e-10]])
    b=np.array([[1.,2.],[.1,1e-10]])
    assert _core.unicycle_search_distance(a.tolist(),b.tolist())<1e-7
    assert _core.unicycle_search_distance(a.tolist(),a.tolist())==0.


def test_heading_embedding_matches_independent_euclidean_features():
    rng=np.random.default_rng(5041)
    def features(state):
        return np.r_[state[0],np.sqrt(.2)*state[1],(2./3.)*state[1]/np.linalg.norm(state[1])]
    for _ in range(50):
        a,b=rng.normal(size=(2,2,2))
        assert _core.unicycle_search_distance(a.tolist(),b.tolist())==pytest.approx(np.linalg.norm(features(a)-features(b)))
