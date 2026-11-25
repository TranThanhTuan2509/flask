"""Test lqmt: FLASK implementation and evaluation utilities."""
import numpy as np
import pytest
from scipy.optimize import minimize_scalar
from flask_repro import lqmt as L
RNG = np.random.default_rng(0)

def _random_states(n=6, v=1.0):
    return (RNG.uniform(-np.pi, np.pi, n), RNG.uniform(-v, v, n), RNG.uniform(-np.pi, np.pi, n), RNG.uniform(-v, v, n))

@pytest.mark.parametrize('k', range(50))
def test_condition_condition_boundary_and_index_phi(k):
    (q0, v0, q1, v1) = _random_states()
    rho = [1.0, 0.3, 3.0][k % 3]
    (T, coef, C) = L.lqmt_r2(q0, v0, q1, v1, rho)
    (q, trajectory, qdd, _) = L.evaluate(coef, np.array([0.0, T]))
    assert np.allclose(q[0], q0, atol=1e-09) and np.allclose(q[1], q1, atol=1e-09)
    assert np.allclose(trajectory[0], v0, atol=1e-09) and np.allclose(trajectory[1], v1, atol=1e-09)
    t = np.linspace(0.0, T, 20001)
    (_, _, w, _) = L.evaluate(coef, t)
    integral = np.trapz((w * w).sum(1), t)
    assert integral == pytest.approx(C - rho * T, rel=1e-06)
    (a, b, c) = L.cost_coefficients(q0, v0, q1, v1)
    h = 1e-06 * T
    dC = (L.cost(T + h, a, b, c, rho) - L.cost(T - h, a, b, c, rho)) / (2 * h)
    assert abs(dC) < 1e-05 * max(1.0, C)

@pytest.mark.parametrize('k', range(30))
def test_T_star_is_extremum_minimum_global_extremum(k):
    (q0, v0, q1, v1) = _random_states(v=3.0)
    (a, b, c) = L.cost_coefficients(q0, v0, q1, v1)
    T = L.optimal_duration(q0, v0, q1, v1)
    grid = np.geomspace(0.001, 100.0, 20000)
    Cg = L.cost(grid, a, b, c)
    Tg = grid[np.argmin(Cg)]
    r = minimize_scalar(lambda x: L.cost(x, a, b, c), bounds=(Tg / 1.05, Tg * 1.05), method='bounded')
    assert T == pytest.approx(r.x, rel=0.0001)

def test_state_state_identical_same():
    q = np.zeros(6)
    assert L.lqmt_r2(q, q, q, q) is None

def test_same_position_position_different_velocity_velocity():
    q = np.zeros(3)
    v0 = np.array([1.0, 0, 0])
    v1 = np.array([-1.0, 0, 0])
    (T, coef, _) = L.lqmt_r2(q, v0, q, v1)
    (a, b, c) = L.cost_coefficients(q, v0, q, v1)
    assert T == pytest.approx(np.sqrt(4 * c), rel=1e-09)

@pytest.mark.parametrize('k', range(40))
def test_extremum_position_solution_analytic_match_scan_dense(k):
    (q0, v0, q1, v1) = _random_states(v=2.0)
    (T, coef, _) = L.lqmt_r2(q0, v0, q1, v1)
    (q_min, q_max, v_abs, a_abs) = L.cubic_extrema(coef, T)
    t = np.linspace(0, T, 200001)
    (q, trajectory, qdd, _) = L.evaluate(coef, t)
    assert np.allclose(q_min, q.min(0), atol=1e-08) and np.allclose(q_max, q.max(0), atol=1e-08)
    assert np.allclose(v_abs, np.abs(trajectory).max(0), atol=1e-07)
    assert np.allclose(a_abs, np.abs(qdd).max(0), atol=1e-09)

def test_negative_control_negative_wrong_sign_quartic():
    """Test negative control negative wrong sign quartic."""
    (q0, v0, q1, v1) = _random_states(v=2.0)
    (a, b, c) = L.cost_coefficients(q0, v0, q1, v1)
    r = np.roots([1.0, 0.0, -4.0 * c, -24.0 * b, -36.0 * a])
    Ts = [z.real for z in r if abs(z.imag) < 1e-09 and z.real > 0]
    correct_duration = L.optimal_duration(q0, v0, q1, v1)
    if abs(b) > 0.001:
        assert all((abs(T - correct_duration) > 1e-06 for T in Ts))
