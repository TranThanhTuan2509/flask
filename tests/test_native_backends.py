"""Matched scalar/SIMD validation with independent trajectory and model oracles."""
import json
from pathlib import Path
import numpy as np
import pytest

from flask_repro import _core
from flask_repro.flat_bvp import fixed_time
from flask_repro.flat_models import Unicycle,Quad2D,Quad3D
from flask_repro.flat_planner import World,Settings,valid_edge
from flask_repro.planning import PlannerSettings,plan,validate


def native_settings(model,backend):
    settings=PlannerSettings(validation_backend=backend,validation_dt=.03,validation_spacing=.04).native()
    settings.max_speed=model.velocity_max;settings.max_yaw_rate=model.angular_velocity_max
    if isinstance(model,Quad3D):settings.max_torque=model.torque_max
    return settings


@pytest.mark.parametrize('name,model',[('unicycle',Unicycle()),('quad2d',Quad2D()),('quad3d',Quad3D())])
def test_scalar_and_simd_agree_on_heterogeneous_edges(name,model):
    rng=np.random.default_rng(1031+model.outputs);edges=[]
    dimensions=3 if model.outputs==4 else 2
    world=World([-3.]*dimensions,[3.]*dimensions,
        spheres=[([.3]*dimensions,.18)],boxes=[([-.7]*dimensions,[.3]*dimensions)])
    world.prepare_simd()
    for _ in range(80):
        if isinstance(model,Unicycle):
            start=np.array([rng.uniform(-1.,1.,2),[rng.uniform(.2,.6),rng.uniform(-.1,.1)]])
            goal=np.array([start[0]+[rng.uniform(.2,1.),rng.uniform(-.2,.2)],
                           [rng.uniform(.2,.6),rng.uniform(-.1,.1)]])
        else:
            start=rng.uniform(-.1,.1,(4,model.outputs));goal=rng.uniform(-.1,.1,(4,model.outputs))
            start[0]=rng.uniform(-1.,1.,model.outputs);goal[0]=start[0]+rng.uniform(-.7,.7,model.outputs)
            if dimensions==3:
                start[0,3]=rng.uniform(-np.pi,np.pi);goal[0,3]=start[0,3]+rng.uniform(-.5,.5)
        edges.append(fixed_time(start,goal,rng.uniform(2.,6.)))
    coefficients=[edge.coefficients.tolist() for edge in edges];durations=[edge.duration for edge in edges]
    results={}
    for backend in ['scalar','simd']:
        results[backend]=_core.flat_check_edges(name,world.cpp_environment,coefficients,durations,
            world.lower.tolist(),world.upper.tolist(),world.radius,native_settings(model,backend))
    np.testing.assert_array_equal(results['scalar']['valid'],results['simd']['valid'])
    assert 0<sum(results['simd']['valid'])<len(edges),'Exercise acceptance and rejection, not only invalid edges'
    independent=Settings(validation_dt=.03,validation_spacing=.04)
    for edge,accepted in zip(edges,results['simd']['valid']):
        if accepted:assert valid_edge(edge,model,world,independent,independent=True,refinement=4)


@pytest.mark.parametrize('backend',['scalar','simd'])
def test_backend_rejects_recorded_near_contact_witness(backend):
    record=json.loads((Path(__file__).parent/'fixtures/sphere_broadphase_witness.json').read_text())
    environment=_core.Environment();environment.add_sphere(record['center'],record['obstacle_radius']);environment.prepare()
    start=np.array([record['position'][:2],[.2,0.]])
    goal=start.copy();goal[0,0]+=.0002
    edge=fixed_time(start,goal,.001)
    result=_core.flat_check_edges('unicycle',environment,[edge.coefficients.tolist()],[edge.duration],
        [-2.,-2.],[2.,2.],record['robot_radius'],native_settings(Unicycle(),backend))
    assert result['valid']==[False]


@pytest.mark.parametrize('backend',['scalar','simd'])
def test_native_kernel_rejects_sample_count_overflow(backend):
    start=np.zeros((4,4));goal=start.copy();start[0,2]=goal[0,2]=1.;goal[0,0]=.2
    edge=fixed_time(start,goal,4.)
    environment=_core.Environment();environment.prepare()
    result=_core.flat_check_edges('quad3d',environment,[edge.coefficients.tolist()],[1e300],
        [-2.]*3,[2.]*3,.1,native_settings(Quad3D(),backend))
    assert result['valid']==[False] and result['nonfinite_rejections']==1


def test_scalar_traversal_has_identical_sample_count_and_time_order():
    # Every point is valid, so both backends must traverse the complete grid.
    model=Quad3D();a=np.zeros((4,4));b=a.copy();a[0,2]=b[0,2]=1.;b[0,0]=.2
    edge=fixed_time(a,b,4.);environment=_core.Environment();environment.prepare()
    scalar=_core.flat_check_edges('quad3d',environment,[edge.coefficients.tolist()],[4.],
        [-3.]*3,[3.]*3,.1,native_settings(model,'scalar'))
    simd=_core.flat_check_edges('quad3d',environment,[edge.coefficients.tolist()],[4.],
        [-3.]*3,[3.]*3,.1,native_settings(model,'simd'))
    assert scalar['valid']==simd['valid']==[True]
    assert scalar['batches']==8*simd['batches']


def test_backends_agree_on_rotated_capsules_and_cuboids():
    environment=_core.Environment()
    environment.add_capsule([-.3,0.,0.],[.2,.8,.4],.12,.7)
    environment.add_capsule([.5,-.5,0.],[0.,0.,0.],.1,.6)
    environment.add_cuboid([.4,.3,0.],[.1,.3,.7],[.2,.3,.4]);environment.prepare()
    rng=np.random.default_rng(1037);edges=[]
    for _ in range(80):
        start=np.array([rng.uniform(-1.,1.,2),[.2,0.]])
        goal=start.copy();goal[0,0]+=.2
        edges.append(fixed_time(start,goal,1.))
    results=[]
    for backend in ['scalar','simd']:
        results.append(_core.flat_check_edges('unicycle',environment,
            [e.coefficients.tolist() for e in edges],[1.]*len(edges),
            [-2.,-2.],[2.,2.],.1,native_settings(Unicycle(),backend)))
    assert results[0]['valid']==results[1]['valid']
    assert 0<sum(results[0]['valid'])<len(edges)
