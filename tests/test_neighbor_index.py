"""Indexed search must preserve the scan's metric, filtering and tie rules."""
from dataclasses import replace
import numpy as np
import pytest
from flask_repro import _core
from flask_repro.flat_models import Unicycle,Quad2D,Quad3D
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings,plan


@pytest.mark.parametrize('order,outputs,heading',[(2,2,False),(2,2,True),(4,2,False),(4,4,False)])
def test_best_near_matches_scan_across_tree_splits_and_inactive_nodes(order,outputs,heading):
    rng=np.random.default_rng(2217+order+outputs)
    states=rng.normal(size=(700,order,outputs));costs=rng.integers(0,20,700).astype(float)
    active=rng.uniform(size=700)>.7;alive=rng.uniform(size=700)>.2;active[0]=alive[0]=True
    # Identical states/costs must select the first eligible inserted node.
    states[17]=states[14];costs[17]=costs[14]=0.;active[[14,17]]=True;alive[[14,17]]=True
    queries=[states[14],*rng.normal(size=(15,order,outputs))]
    for target in queries:
        for radius in [1e-12,.5,2.,20.]:
            arguments=[states.tolist(),costs.tolist(),active.tolist(),alive.tolist(),target.tolist(),radius,.2]
            scan=_core.flat_best_near(*arguments,indexed=False,heading=heading)
            indexed=_core.flat_best_near(*arguments,indexed=True,heading=heading)
            assert indexed==scan


@pytest.mark.parametrize('offset',[0.,1e9])
def test_refinement_preserves_near_ties_and_radius_boundary(offset):
    rng=np.random.default_rng(2281);states=rng.normal(size=(400,2,2))
    states[:,0]+=offset;costs=np.ones(400);active=np.ones(400,bool);alive=active.copy()
    target=np.array([[offset+.5,offset+.25],[.25,.3]])
    states[12]=target;states[12,0,0]+=1e-10
    states[13]=states[12];costs[[12,13]]=0.
    for radius in [1e-11,1e-10,1e-9,.1,1.]:
        a=[states.tolist(),costs.tolist(),active.tolist(),alive.tolist(),target.tolist(),radius,.2]
        assert _core.flat_best_near(*a,indexed=True)==_core.flat_best_near(*a,indexed=False)


def test_heading_index_does_not_merge_opposite_slow_headings():
    for speed in [1e-8,.1,1.]:
        states=[[[0.,0.],[-speed,0.]],[[.1,0.],[speed,0.]]]
        assert _core.flat_best_near(states,[0.,10.],[True]*2,[True]*2,
            [[0.,0.],[speed,0.]],.5,indexed=True,heading=True)==1


def test_index_reports_no_active_representative():
    with pytest.raises(RuntimeError,match='no active'):
        _core.flat_best_near([[[0.,0.],[.3,0.]]],[0.],[False],[True],
            [[1.,1.],[.3,0.]],.5,indexed=True)


def test_keys_remain_stable_across_storage_block_reallocation():
    rng=np.random.default_rng(8842);states=rng.normal(size=(5000,4,4));costs=rng.uniform(size=5000)
    active=rng.uniform(size=5000)>.9;alive=np.ones(5000,bool)
    for target in [states[0],states[2500],rng.normal(size=(4,4))]:
        arguments=[states.tolist(),costs.tolist(),active.tolist(),alive.tolist(),target.tolist(),.2,.2]
        assert _core.flat_best_near(*arguments,indexed=True)==_core.flat_best_near(*arguments,indexed=False)


@pytest.mark.parametrize('robot,algorithm',[
    ('unicycle','rrt_connect'),('unicycle','rrt_dp'),('unicycle','sst_bvp'),('unicycle','sst_simd'),
    ('quad2d','sst_bvp'),('quad2d','sst_simd'),('quad3d','sst_bvp'),('quad3d','sst_simd')])
def test_complete_fixed_work_search_and_trajectory_match_scan(robot,algorithm):
    if robot=='unicycle':
        model=Unicycle();start=[0.,0.,0.];goal=[8.,8.,0.];dimension=2;mode=2
    elif robot=='quad2d':
        model=Quad2D();start=[0.,0.,0.,0.,0.,0.];goal=[8.,8.,0.,0.,0.,0.];dimension=2;mode=0
    else:
        model=Quad3D();start=[0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.]
        goal=[8.,8.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.];dimension=3;mode=0
    world=World([-10.]*dimension,[10.]*dimension)
    cfg=PlannerSettings(algorithm=algorithm,metric_mode=mode,max_iterations=3000,
        budget_s=30.,trace=True,stop_first=False,phase_iterations=500)
    scan=plan(model,world,start,goal,cfg,6221)
    indexed=plan(model,world,start,goal,replace(cfg,indexed_search=True),6221)
    assert scan.solved==indexed.solved and scan.iterations==indexed.iterations
    assert scan.trajectory.to_records()==indexed.trajectory.to_records()
    assert scan.rejections==indexed.rejections
    for name in ['start_states','start_parents','start_active','start_alive',
                 'goal_states','goal_parents','witness_states','witness_representatives','phase_restart_iterations']:
        np.testing.assert_array_equal(scan.diagnostics[name],indexed.diagnostics[name])
