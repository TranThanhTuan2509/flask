"""RunRecord.validated must mean original-model validity, including after rechecking."""
from dataclasses import asdict
import numpy as np
import pytest
from flask_repro.flat_models import Quad2D,Quad3D
from flask_repro.flat_bvp import fixed_time
from flask_repro.flat_planner import World
from flask_repro.planning import RunRecord,Trajectory,PlannerSettings,validate


def example(kind,order):
    if kind=='planar':
        model=Quad2D(order=order);start=np.zeros(6);goal=np.array([1.,0.,0.,0.,0.,0.]);dims=2
    else:
        model=Quad3D(order=order);start=np.array([0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.]);goal=start.copy();goal[0]=1.;dims=3
    edge=fixed_time(model.lift(start),model.lift(goal),8.)
    cfg=PlannerSettings()
    record=RunRecord(solved=True,trajectory=Trajectory([edge]),plan_ns=0,postprocess_ns=0,
        first_solution_ns=0,iterations=0,nodes=2,terminal_error=0.,settings=asdict(cfg),
        start_state=start.tolist(),goal_state=goal.tolist())
    return record,model,World([-5.]*dims,[5.]*dims),cfg


@pytest.mark.parametrize('kind',['planar','spatial'])
def test_reduced_diagnostic_cannot_mark_record_original_model_valid(kind):
    record,model,world,cfg=example(kind,2)
    diagnostic=validate(record,model,world,cfg,allow_reduced_model=True)
    assert diagnostic['valid'] and not diagnostic['original_model_valid']
    assert not record.validated
    strict=validate(record,model,world,cfg)
    assert not strict['valid'] and not record.validated


@pytest.mark.parametrize('kind',['planar','spatial'])
def test_original_model_flag_is_set_and_cleared_on_recheck(kind):
    record,model,world,cfg=example(kind,4)
    result=validate(record,model,world,cfg)
    assert result['valid'] and result['original_model_valid'] and record.validated
    record.trajectory.edges[0].coefficients[0,0]+=.1
    invalid=validate(record,model,world,cfg)
    assert not invalid['valid'] and not invalid['original_model_valid'] and not record.validated
