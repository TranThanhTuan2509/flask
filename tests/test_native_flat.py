"""Cross-check native steering against independent Hermite/Gramian solutions."""
import numpy as np
import pytest
from flask_repro import _core
from flask_repro.flat_bvp import fixed_time,optimal_time,FlatEdge
from flask_repro.flat_models import Unicycle,Quad2D,Quad3D
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings,plan,validate

@pytest.mark.parametrize('r,n',[(2,2),(4,2),(4,4),(2,4)])
def test_native_boundary_cost_and_optimal_duration(r,n):
    rng=np.random.default_rng(234+r+n)
    for _ in range(15):
        a,b=rng.normal(size=(2,r,n))
        for T in [-1.,.7,4.]:
            d=_core.flat_bvp(a.tolist(),b.tolist(),1.,T)
            assert d['duration']>0
            e=FlatEdge(np.asarray(d['coefficients']),d['duration'],r)
            np.testing.assert_allclose(e.state(0),a,atol=1e-7)
            np.testing.assert_allclose(e.state(e.duration),b,atol=2e-6)
            assert e.effort()==pytest.approx(d['effort'],rel=1e-7)
            if T>0:assert e.duration==T
            else:
                ref=optimal_time(a,b)
                assert e.cost() == pytest.approx(fixed_time(a,b,ref).cost(),rel=2e-6)

@pytest.mark.parametrize('model,start,goal',[
 (Unicycle(),[0,0,0],[1,0,0]),
 (Quad2D(),[0,0,0,0,0,0],[1,0,0,0,0,0]),
 (Quad3D(),[0,0,1,0,0,0,1,0,0,0,0,0,0],[1,0,1,0,0,0,1,0,0,0,0,0,0]),
])
def test_native_direct_connections(model,start,goal):
    d=3 if isinstance(model,Quad3D) else 2
    world=World(-5*np.ones(d),5*np.ones(d))
    cfg=PlannerSettings(budget_s=.2)
    result=plan(model,world,start,goal,cfg,seed=4)
    assert result.solved
    assert validate(result,model,world,cfg)['valid']
    assert result.terminal_error<1e-6

def test_native_rejects_between_sample_unicycle_yaw_spike():
    import json
    from pathlib import Path
    data=json.loads((Path(__file__).parent/'fixtures/unicycle_between_sample_yaw_spike.json').read_text())
    edge=FlatEdge(np.asarray(data['coefficients']),data['duration'],data['order'])
    assert not Unicycle().edge_valid(edge)
    assert not _core.unicycle_edge_valid(data['coefficients'],data['duration'])


def test_native_unicycle_extrema_agree_with_independent_polynomial_oracle():
    from flask_repro.flat_bvp import fixed_time
    rng=np.random.default_rng(798)
    for _ in range(200):
        start=rng.uniform(-.8,.8,(2,2));goal=rng.uniform(-.8,.8,(2,2))
        edge=fixed_time(start,goal,rng.uniform(.5,8))
        assert _core.unicycle_edge_valid(edge.coefficients.tolist(),edge.duration)==Unicycle().edge_valid(edge)


@pytest.mark.parametrize('r,n',[(2,2),(4,2),(4,4)])
def test_native_effort_agrees_with_independent_positive_quadrature(r,n):
    """Check effort without the oracle's signed monomial or Gramian sums."""
    rng=np.random.default_rng(392+r+n)
    points,weights=np.polynomial.legendre.leggauss(12)
    for duration in [1e-4,.3,100.]:
        for magnitude in [1e-9,1.,1e9]:
            start,goal=rng.normal(size=(2,r,n))*magnitude
            raw=_core.flat_bvp(start.tolist(),goal.tolist(),1.,duration)
            # Solve the Hermite problem independently of the native inverse.
            oracle=fixed_time(start,goal,duration)
            control=oracle.derivative(.5*duration*(points+1),r)
            expected=.5*duration*np.sum(weights*np.sum(control*control,axis=-1))
            assert raw['effort']>=0.
            assert raw['effort']==pytest.approx(expected,rel=2e-9)


def test_native_time_optimization_preserves_tiny_nonzero_bvps():
    start=np.zeros((2,2))
    for displacement in [1e-12,1e-20,1e-28]:
        goal=start.copy();goal[0,0]=displacement
        raw=_core.flat_bvp(start.tolist(),goal.tolist(),1.,-1.)
        # For zero endpoint velocity, J(T)=12*d^2/T^3+T.
        expected=(36*displacement**2)**.25
        assert raw['duration']==pytest.approx(expected,rel=2e-10)
        assert raw['effort']==pytest.approx(expected/3.,rel=2e-10)


@pytest.mark.parametrize('speed',[1e-8,1e-5,.3])
def test_unicycle_extrema_are_consistent_under_small_velocity_scales(speed):
    start=np.array([[0.,0.],[speed,0.]])
    goal=np.array([[speed,0.],[speed,0.]])
    edge=fixed_time(start,goal,1.)
    assert Unicycle().edge_valid(edge)
    assert _core.unicycle_edge_valid(edge.coefficients.tolist(),edge.duration)


@pytest.mark.parametrize('model',[Unicycle(),Quad2D(),Quad3D()])
def test_native_original_state_distance_matches_independent_recovery(model):
    from flask_repro.planning import _physical_distance
    rng=np.random.default_rng(831+model.outputs)
    for _ in range(60):
        a,b=rng.normal(size=(2,model.order,model.outputs))
        if isinstance(model,Unicycle):
            # A large heading error can have an arbitrarily small flat metric.
            a[1]*=1e-7;b[1]*=1e-7
        native=_core.flat_state_distance(a.tolist(),b.tolist())
        def recover(z):return model.recover(np.r_[z,np.zeros((5-len(z),model.outputs))])
        expected=_physical_distance(model,recover(a),recover(b))
        assert native==pytest.approx(expected,rel=2e-10,abs=1e-11)


def test_sst_star_restarts_with_the_published_radius_schedule():
    cfg=PlannerSettings(algorithm='sst_simd',budget_s=2.,max_iterations=22,
                        phase_iterations=4,shrink_factor=.95,control_bound=1e-5,
                        propagation_min=.02,propagation_max=.02,trace=True)
    result=plan(Unicycle(),World([-200.,-200.],[200.,200.]),
                [0.,0.,0.],[100.,100.,0.],cfg,seed=9)
    # Original SST paper Alg.9: ceil(N0*(1+log j)*xi^(-(d+l+1)*j)).
    boundaries=[];end=cfg.phase_iterations
    for phase in range(1,4):
        boundaries.append(end)
        end+=int(np.ceil(cfg.phase_iterations*(1+np.log(phase))*cfg.shrink_factor**(-7*phase)))
    assert result.diagnostics['phase_restart_iterations']==[i for i in boundaries if i<22]
    assert not result.solved


@pytest.mark.parametrize('changes',[
    dict(phase_iterations=0),dict(max_iterations=-1),dict(max_nodes=2),
    dict(propagation_min=.4,propagation_max=.2),dict(derivative_metric_weight=0.),
    dict(control_bound=float('nan')),
    dict(bound1=float('nan')),dict(bound2=float('inf')),dict(bound3=-1.),
    dict(speed_min=float('nan')),dict(speed_min=-.1),dict(speed_min=2.,bound1=1.),
])
def test_invalid_native_settings_fail_before_search(changes):
    with pytest.raises(ValueError):PlannerSettings(**changes).native()


@pytest.mark.parametrize('model,start,goal',[
    (Quad2D(mass=.05),[0.]*6,[1.,0.,0.,0.,0.,0.]),
    (Quad3D(inertia=np.diag([.2,.2,.3])),
     [0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.],
     [1.,0.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.]),
])
def test_native_rejects_unsupported_inertial_models(model,start,goal):
    dimensions=3 if isinstance(model,Quad3D) else 2
    with pytest.raises(ValueError,match='published mass'):
        plan(model,World(-5*np.ones(dimensions),5*np.ones(dimensions)),start,goal)


@pytest.mark.parametrize('rho,duration',[(np.inf,-1.),(1.,np.inf),(np.nan,1.)])
def test_native_bvp_rejects_nonfinite_cost_or_duration(rho,duration):
    with pytest.raises(ValueError):_core.flat_bvp([[0.,0.],[0.,0.]],[[1.,0.],[0.,0.]],rho,duration)


@pytest.mark.parametrize('model',[
    Quad2D(velocity_max=-1.),Quad3D(torque_max=-1.),Unicycle(angular_velocity_max=np.nan),
])
def test_native_rejects_invalid_original_model_limits(model):
    with pytest.raises(ValueError,match='must be positive'):
        plan(model,World([-5.,-5.],[5.,5.]),[0.,0.,0.],[1.,0.,0.])


@pytest.mark.parametrize('r,n',[(2,2),(4,2),(4,4)])
@pytest.mark.parametrize('weight',[.2,16.])
def test_native_nearest_index_matches_brute_force_metric(r,n,weight):
    rng=np.random.default_rng(214+r+n)
    states=rng.normal(size=(600,r,n));queries=rng.normal(size=(80,r,n))
    metric_weights=np.r_[1.,weight/3.**np.arange(r-1)]
    expected=np.argmin(np.sum((queries[:,None]-states[None])**2*metric_weights[None,None,:,None],axis=(2,3)),axis=1)
    actual=_core.flat_nearest_indices(states.tolist(),queries.tolist(),weight)
    np.testing.assert_array_equal(actual,expected)


@pytest.mark.parametrize('name,model',[
    ('unicycle',Unicycle()),('unicycle',Unicycle(reverse=True)),
    ('quad2d',Quad2D()),('quad3d',Quad3D()),
])
def test_native_recovery_matches_independent_original_state_and_control(name,model):
    """Audit production planar/spatial SIMD recovery, including partial batches."""
    rng=np.random.default_rng(684+model.outputs)
    jets=rng.normal(size=(237,5,model.outputs))
    if isinstance(model,Unicycle):jets[:,1,0]+=2.
    raw=_core.flat_recover(name,jets.tolist(),bool(getattr(model,'reverse',False)))
    expected=model.recover(jets)
    for key in ['position','velocity','control']:
        np.testing.assert_allclose(raw[key],expected[key],rtol=2e-5,atol=2e-6)
    if isinstance(model,Quad3D):
        for key in ['rotation','omega','torque','thrust']:
            np.testing.assert_allclose(raw[key],expected[key],rtol=2e-5,atol=2e-6)
    else:
        actual=np.asarray(raw['state']);reference=expected['state'].copy()
        angle_error=np.arctan2(np.sin(actual[:,2]-reference[:,2]),np.cos(actual[:,2]-reference[:,2]))
        np.testing.assert_allclose(angle_error,0.,atol=2e-6)
        actual[:,2]=reference[:,2]
        np.testing.assert_allclose(actual,reference,rtol=2e-5,atol=2e-6)


@pytest.mark.parametrize('name,n',[('unicycle',2),('quad2d',2),('quad3d',4)])
def test_native_recovery_rejects_singular_and_nonfinite_jets(name,n):
    jets=np.zeros((1,5,n))
    if name!='unicycle':jets[0,2,1 if n==2 else 2]=-9.81
    with pytest.raises(ValueError):_core.flat_recover(name,jets.tolist())
    for value in [np.nan,np.inf,1e100]:
        jets=np.zeros((1,5,n));jets[0,0,0]=value
        with pytest.raises(ValueError):_core.flat_recover(name,jets.tolist())


def test_native_broadphase_rejects_recorded_sphere_penetration():
    import json
    from pathlib import Path
    record=json.loads((Path(__file__).parent/'fixtures/sphere_broadphase_witness.json').read_text())
    environment=_core.Environment()
    environment.add_sphere(record['center'],record['obstacle_radius']);environment.prepare()
    position=np.asarray(record['position'])
    # Quantize the geometry independently to the actual native precision.
    distance=np.linalg.norm(position.astype(np.float32).astype(float)-np.asarray(record['center'],np.float32).astype(float))
    combined=float(np.float32(record['obstacle_radius']))+float(np.float32(record['robot_radius']))
    assert distance<combined-5e-6
    assert not _core.sphere_positions_valid(environment,position[None],record['robot_radius'])


def test_reduced_order_aerial_join_fails_original_state_continuity():
    from flask_repro.flat_bvp import fixed_time
    from flask_repro.planning import Trajectory,RunRecord
    a=np.zeros((2,4));a[0,2]=1
    b=a.copy();b[0,0]=1
    c=b.copy();c[0,1]=1
    trajectory=Trajectory([fixed_time(a,b,4.),fixed_time(b,c,4.)])
    assert trajectory.continuity_error()<1e-10
    assert trajectory.physical_continuity_error(Quad3D())>.01
    record=RunRecord(True,trajectory,0,0,0,0,0,0)
    report=validate(record,Quad3D(),World([-5,-5,0],[5,5,5]))
    assert 'original_state_discontinuity' in report['errors']
