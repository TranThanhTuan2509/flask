"""Preserved counterexamples protect against accepting known interior collisions."""
from pathlib import Path
import numpy as np
import pytest
from flask_repro import native,mbm
from flask_repro.robot import get_robot

FIXTURES=Path(__file__).resolve().parent/'fixtures/mbm_collision_witnesses'

@pytest.mark.parametrize('name',['bookshelf_tall_27','bookshelf_thin_34','box_19','table_under_pick_43'])
def test_strict_validator_rejects_recorded_interior_collision(name):
    robot=get_robot('panda');module=native.module(robot);scene,index=name.rsplit('_',1)
    world=mbm.native_environment(mbm.load_dataset('panda')[scene][int(index)])
    with np.load(FIXTURES/(name+'.npz')) as data:
        assert all(not module.validate(world,q.tolist()) for q in data['collision_configurations'])
        strict=native.joint_limits(robot,'strict_development')
        edges=[module.flask_cc(world,*[a.tolist() for a in (data['Q'][i],data['V'][i],data['Q'][i+1],data['V'][i+1])],float(t),strict) for i,t in enumerate(data['T'])]
        assert not all(edges)
