"""A geometry predicate must not accept NaN FK as collision-free."""
import numpy as np
import pytest
from flask_repro import _core


@pytest.mark.parametrize('name,q',[
    ('panda',[0.,-.785,0.,-2.356,0.,1.571,.785]),('ur5',[0.]*6),
])
@pytest.mark.parametrize('value',[np.nan,np.inf,-np.inf,1e300])
def test_nonfinite_or_unrepresentable_joint_configuration_is_rejected(name,q,value):
    world=_core.Environment();world.prepare();module=getattr(_core,name)
    assert module.validate(world,q),'Exercise a valid positive control before corrupting it'
    malformed=q.copy();malformed[0]=value
    assert not module.validate(world,malformed)
    with pytest.raises(ValueError,match='nonfinite'):
        module.count_collisions(world,[q,malformed])
