"""Test flat systems: FLASK implementation and evaluation utilities."""
import numpy as np
import pytest
from scipy.integrate import quad
from scipy.linalg import expm
from flask_repro.flat_bvp import fixed_time, connect, gramian, propagate, optimal_time
from flask_repro.flat_models import Manipulator, Unicycle, Quad2D, Quad3D, SingularFlatState


@pytest.mark.parametrize('order,outputs',[(1,2),(2,2),(4,2),(4,4)])
def test_chain_controllability_and_gramian_volume_exponent(order,outputs):
    dimension=order*outputs
    A=np.kron(np.diag(np.ones(order-1),1),np.eye(outputs))
    B=np.zeros((dimension,outputs));B[-outputs:]=np.eye(outputs)
    controllability=np.concatenate([np.linalg.matrix_power(A,k)@B for k in range(order)],axis=1)
    assert np.linalg.matrix_rank(controllability)==dimension
    np.testing.assert_array_equal(np.linalg.matrix_power(A,order),np.zeros_like(A))
    rng=np.random.default_rng(702+order+outputs);matrix=rng.normal(size=(outputs,outputs))
    weight=matrix.T@matrix+np.eye(outputs)
    base_sign,base_logdet=np.linalg.slogdet(gramian(1.,order,outputs,weight))
    assert base_sign>0
    for duration in [.2,.7,2.]:
        sign,logdet=np.linalg.slogdet(gramian(duration,order,outputs,weight))
        assert sign>0
        assert logdet-base_logdet==pytest.approx(order**2*outputs*np.log(duration),abs=2e-9)

@pytest.mark.parametrize('r', [1, 2, 3, 4])
def test_hermite_against_gramian_and_boundary(r):
    rng = np.random.default_rng(230 + r)
    n = 2
    for T in [0.2, 1.0, 7.0]:
        (z0, z1) = rng.normal(size=(2, r, n))
        edge = fixed_time(z0, z1, T)
        np.testing.assert_allclose(edge.state(0), z0, atol=2e-08)
        np.testing.assert_allclose(edge.state(T), z1, atol=2e-08)
        A = np.kron(np.diag(np.ones(r - 1), 1), np.eye(n))
        B = np.zeros((r * n, n))
        B[-n:] = np.eye(n)
        R = np.array([[2.0, 0.2], [0.2, 1.0]])
        G = gramian(T, r, n, R)
        d = z1.ravel() - expm(A * T) @ z0.ravel()
        energy = d @ np.linalg.solve(G, d)
        assert edge.effort(R) == pytest.approx(energy, rel=3e-09)
        numeric = quad(lambda t: edge.derivative(t, r) @ R @ edge.derivative(t, r), 0, T)[0]
        assert edge.effort(R) == pytest.approx(numeric, rel=3e-09)
        t = 0.37 * T
        w = np.linalg.solve(R, B.T @ expm(A.T * (T - t)) @ np.linalg.solve(G, d))
        np.testing.assert_allclose(edge.derivative(t, r), w, rtol=1e-08, atol=1e-07)

@pytest.mark.parametrize('r', [1, 2, 4])
def test_optimal_time_global_and_stationary(r):
    rng = np.random.default_rng(14 + r)
    for _ in range(12):
        (z0, z1) = rng.normal(size=(2, r, 2))
        rho = 10 ** rng.uniform(-2, 2)
        T = optimal_time(z0, z1, rho)
        cost = lambda s: fixed_time(z0, z1, s).cost(rho)
        assert cost(T) <= min((cost(t) for t in T * np.geomspace(0.08, 15.0, 80))) * (1 + 1e-08)
        assert abs((cost(T * 1.00001) - cost(T * 0.99999)) / (cost(T) * 2e-05)) < 2e-05

def test_r2_matches_existing_quartic_independently():
    from flask_repro import lqmt
    rng = np.random.default_rng(67)
    for _ in range(100):
        (z0, z1) = rng.normal(size=(2, 2, 3))
        T = optimal_time(z0, z1)
        assert T == pytest.approx(lqmt.optimal_duration(*z0, *z1), rel=1e-08)

def test_degenerate_and_invalid_bvp():
    z = np.array([[1.,1.],[0.,0.]])
    assert connect(z, z) is None
    for rho in [0, -1, np.nan]:
        with pytest.raises(ValueError):
            optimal_time(z, z + 1, rho)
    with pytest.raises(ValueError):
        fixed_time(z, z, 0)
    with pytest.raises(ValueError):
        fixed_time(z, np.zeros((3, 2)), 1)
    with pytest.raises(ValueError):
        connect(z, z + 1, R=np.array([[1, 3], [3, 1]]))

def test_propagation_against_matrix_exponential():
    from scipy.linalg import expm
    rng = np.random.default_rng(45)
    for r in [2, 4]:
        z = rng.normal(size=(r, 2))
        u = rng.normal(size=2)
        e = propagate(z, u, 0.7)
        A = np.kron(np.diag(np.ones(r - 1), 1), np.eye(2))
        Aug = np.zeros((2 * r + 2, 2 * r + 2))
        Aug[:2 * r, :2 * r] = A
        Aug[2 * r - 2:2 * r, 2 * r:] = np.eye(2)
        expected = (expm(Aug * 0.7) @ np.r_[z.ravel(), u])[:2 * r].reshape(r, 2)
        np.testing.assert_allclose(e.state(0.7), expected, atol=1e-12)
        np.testing.assert_allclose(e.derivative([0, 0.7], r), [u, u])

def test_unicycle_kinematic_equations_forward_and_reverse():
    edge = fixed_time([[0, 0], [0.3, 0.2]], [[1, 1], [0.2, 0.4]], 4.0)
    times = np.linspace(0.01, 3.99, 99)
    for reverse in [False, True]:
        model = Unicycle(reverse=reverse)
        val = model.recover(edge.jet(times))
        x = val['state']
        u = val['control']
        np.testing.assert_allclose(u[:, :1] * np.c_[np.cos(x[:, 2]), np.sin(x[:, 2])], edge.derivative(times, 1), atol=1e-12)
        h = 1e-05
        xp = model.recover(edge.jet(times + h))['state']
        xm = model.recover(edge.jet(times - h))['state']
        np.testing.assert_allclose((xp[:, 2] - xm[:, 2]) / (2 * h), u[:, 1], rtol=2e-07)
    with pytest.raises(SingularFlatState):
        Unicycle().recover(np.zeros((3, 2)))

def test_unicycle_rejects_unsampled_stop_and_yaw_rate_spike():
    from flask_repro.flat_bvp import FlatEdge
    from flask_repro.flat_planner import World, Settings, valid_edge
    model = Unicycle()
    stopped = FlatEdge(np.array([[0.0, 0.0], [-0.513125, 0.0], [0.5, 0.0]]), 1.0, 2)
    assert not model.edge_valid(stopped)
    spike = FlatEdge(np.array([[0.0, 0.0], [-0.5001, 0.001], [0.5, 0.0]]), 1.0, 2)
    grid = np.linspace(0, 1, 6)
    assert np.all(model.valid(model.recover(spike.jet(grid))))
    assert not model.edge_valid(spike)
    assert not valid_edge(spike, model, World([-2, -2], [2, 2]), Settings(validation_dt=0.2, validation_spacing=10.0))

def test_quad2d_full_dynamics_and_boundary_lift():
    model = Quad2D()
    x = np.array([1.0, 2.0, 0.2, 0.3, -0.1, 0.4])
    z = model.lift(x)
    val = model.recover(np.r_[z, np.zeros((1, 2))])
    np.testing.assert_allclose(val['state'], x, atol=1e-12)
    edge = fixed_time(model.lift([0, 1, 0, 0, 0, 0]), model.lift([2, 2, 0, 0, 0, 0]), 5.0)
    t = np.linspace(0.1, 4.9, 31)
    v = model.recover(edge.jet(t))
    h = 1e-05
    dp = (model.recover(edge.jet(t + h))['state'] - model.recover(edge.jet(t - h))['state']) / (2 * h)
    (f1, f2) = v['control'].T
    acc = np.c_[-(f1 + f2) * np.sin(v['theta']) / model.mass, (f1 + f2) * np.cos(v['theta']) / model.mass - model.gravity]
    expected = np.c_[v['velocity'], v['omega'], acc, model.arm * (f1 - f2) / model.inertia]
    np.testing.assert_allclose(dp, expected, atol=2e-08, rtol=1e-06)
    bad = np.zeros((5, 2))
    bad[2, 1] = -model.gravity
    with pytest.raises(SingularFlatState):
        model.recover(bad)

def test_quad3d_rotation_and_newton_euler_residuals():
    model = Quad3D()
    z0 = model.lift([0, 0, 1, 0, 0, 0, 1, 0.1, 0.2, 0.3, 0.1, 0.2, 0.3])
    recovered = model.recover(np.r_[z0, np.zeros((1, 4))])
    np.testing.assert_allclose(recovered['omega'], [0.1, 0.2, 0.3], atol=1e-12)
    z1 = model.lift([2, 1, 3, 0, 0, np.sin(0.3), np.cos(0.3), 0, 0, 0, 0, 0, 0])
    e = fixed_time(z0, z1, 7.0)
    t = np.linspace(0.1, 6.9, 37)
    h = 1e-05
    v = model.recover(e.jet(t))
    R = v['rotation']
    np.testing.assert_allclose(np.swapaxes(R, -1, -2) @ R, np.broadcast_to(np.eye(3), R.shape), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)
    np.testing.assert_allclose(v['thrust'][:, None] * R[:, :, 2] / model.mass - [0, 0, model.gravity], e.derivative(t, 2)[:, :3], atol=1e-12)
    vp = model.recover(e.jet(t + h))
    vm = model.recover(e.jet(t - h))
    omega_dot = (vp['omega'] - vm['omega']) / (2 * h)
    rhs = omega_dot @ model.inertia.T + np.cross(v['omega'], v['omega'] @ model.inertia.T)
    np.testing.assert_allclose(v['torque'], rhs, atol=3e-09, rtol=2e-06)
    np.testing.assert_allclose(v['rotation_dot'], (vp['rotation'] - vm['rotation']) / (2 * h), atol=1e-09)
    bad = np.zeros((5, 4))
    bad[2, 2] = -model.gravity
    with pytest.raises(SingularFlatState):
        model.recover(bad)

def test_manipulator_does_not_claim_torque_without_inverse_dynamics():
    m = Manipulator(2)
    jet = np.array([[1.0, 2.0], [0.1, 0.2], [0.3, 0.4]])
    assert 'control' not in m.recover(jet)
    m.inverse_dynamics = lambda q, v, a: 2 * a + v + np.sin(q)
    np.testing.assert_allclose(m.recover(jet)['control'], 2 * jet[2] + jet[1] + np.sin(jet[0]))
