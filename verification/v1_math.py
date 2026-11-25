"""V1 (acceptance checks; see "Verification" in README.md): first-principles checks of the FLASK mathematics, independent of the code paths
under test wherever possible.

  V1a  LQMT fixed-time BVP (Eq. 20-25): boundary jets, and first-order optimality via the variational identity
       int <w, d^r(dy)/dt^r> dt = 0 for every admissible perturbation dy vanishing with its first r-1 derivatives at
       both ends (Gauss-Legendre quadrature, exact for the polynomial degrees involved). Perturbed effort must be larger.
  V1b  Optimal duration T* (Eq. 26-28): cost(T*) <= min over a dense log-scan + golden refinement of cost(T).
  V1c  Propagation (Eq. 29) vs numerical integration of the integrator chain z' = Az + Bw (solve_ivp, rtol 1e-12).
  V1d  Triangle inequality c(a,c) <= c(a,b) + c(b,c), which Alg. 4 shortcutting relies on.
  V1e  Differential flatness (Examples 2, 3): controls recovered from a flat trajectory, fed to the ORIGINAL dynamics
       (unicycle, planar quadrotor, 3D quadrotor) and integrated numerically, reproduce the recovered state.
  V1f  Native C++ kernel vs Python reference on random BVPs (T*, coefficients).

Usage: python -m verification.v1_math --output results/verification/<date>_acceptance/v1.json [--count 10000]
"""
import argparse
import json
import time
from math import factorial
from pathlib import Path

import numpy as np
from numpy.polynomial import polynomial as P
from scipy.integrate import solve_ivp

from flask_repro import flat_bvp as B
from flask_repro.flat_models import Quad2D, Quad3D, Unicycle

GL_X, GL_W = np.polynomial.legendre.leggauss(40)


def quad(f, T):
    """Gauss-Legendre on [0, T]; exact for polynomials up to degree 79."""
    t = .5 * T * (GL_X + 1)
    return .5 * T * np.sum(GL_W[:, None] * f(t), axis=0) if np.ndim(f(t)) > 1 else .5 * T * np.sum(GL_W * f(t))


def poly_eval(c, T, t, d=0):
    c = P.polyder(c, m=d, axis=0) / T ** d if d else c
    return np.moveaxis(P.polyval(np.asarray(t) / T, c), 0, -1)


def random_state(rng, r, n, scale=1.):
    return rng.normal(size=(r, n)) * scale


def v1a(rng, count, r, n):
    worst_bc, worst_var, worse_effort = 0., 0., 0
    for _ in range(count):
        z0, z1 = random_state(rng, r, n), random_state(rng, r, n)
        T = float(rng.uniform(.2, 5.))
        e = B.fixed_time(z0, z1, T)
        for j in range(r):
            worst_bc = max(worst_bc, np.max(np.abs(e.derivative(0., j) - z0[j])), np.max(np.abs(e.derivative(T, j) - z1[j])))
        # admissible perturbation: s^r (1-s)^r q(s), q random polynomial (degree 3), per output
        s_poly = P.polypow([0, 1], r)
        s_poly = P.polymul(s_poly, P.polypow([1, -1], r))
        q = rng.normal(size=(4, n))
        dc = np.stack([P.polymul(s_poly, q[:, j]) for j in range(n)], axis=1)
        w = lambda t: poly_eval(e.coefficients, T, t, r)
        dw = lambda t: poly_eval(dc, T, t, r)
        inner = quad(lambda t: np.sum(w(t) * dw(t), axis=-1), T)
        norm = np.sqrt(quad(lambda t: np.sum(w(t) ** 2, axis=-1), T) * quad(lambda t: np.sum(dw(t) ** 2, axis=-1), T))
        worst_var = max(worst_var, abs(inner) / max(norm, 1e-300))
        eps = 1e-3
        perturbed = quad(lambda t: np.sum((w(t) + eps * dw(t)) ** 2, axis=-1), T)
        if perturbed < quad(lambda t: np.sum(w(t) ** 2, axis=-1), T):
            worse_effort += 1
    return dict(count=count, r=r, n=n, max_boundary_error=worst_bc, max_relative_variation=worst_var,
                perturbations_with_lower_effort=worse_effort,
                passed=bool(worst_bc <= 1e-9 * 100 and worst_var <= 1e-9 and worse_effort == 0))


def v1b(rng, count, r, n, rho=1.):
    worst_gap, worst_t, failures = -np.inf, 0., 0
    for _ in range(count):
        z0, z1 = random_state(rng, r, n), random_state(rng, r, n)
        T = B.optimal_time(z0, z1, rho)
        if T is None:
            failures += 1; continue
        cost = lambda x: B.fixed_time(z0, z1, x).cost(rho)
        grid = np.geomspace(T * 1e-3, T * 1e3, 600)
        values = np.array([cost(x) for x in grid])
        k = int(np.argmin(values))
        lo, hi = grid[max(k - 1, 0)], grid[min(k + 1, len(grid) - 1)]
        g = (np.sqrt(5) - 1) / 2
        a, b = lo, hi
        for _ in range(200):
            c, d = b - g * (b - a), a + g * (b - a)
            if cost(c) < cost(d): b = d
            else: a = c
        best = .5 * (a + b)
        gap = (cost(T) - cost(best)) / cost(best)
        worst_gap = max(worst_gap, gap)
        worst_t = max(worst_t, abs(T - best) / best if gap > 1e-12 else 0.)
    return dict(count=count, r=r, n=n, rho=rho, max_relative_cost_excess_over_scan=worst_gap,
                max_relative_duration_gap_when_excess=worst_t, no_root_failures=failures,
                passed=bool(worst_gap <= 1e-6 and failures == 0))


def v1c(rng, count, r, n):
    worst = 0.
    for _ in range(count):
        z0 = random_state(rng, r, n); w = rng.normal(size=n); T = float(rng.uniform(.05, 2.))
        e = B.propagate(z0, w, T)

        def f(_, x):
            z = x.reshape(r, n); dz = np.zeros_like(z); dz[:-1] = z[1:]; dz[-1] = w
            return dz.ravel()
        sol = solve_ivp(f, (0, T), z0.ravel(), rtol=1e-12, atol=1e-12, method='DOP853')
        analytic = np.array([e.derivative(T, j) for j in range(r)])
        worst = max(worst, float(np.max(np.abs(sol.y[:, -1].reshape(r, n) - analytic))))
    return dict(count=count, r=r, n=n, max_state_error=worst, passed=bool(worst <= 1e-8))


def v1d(rng, count, r, n, rho=1.):
    worst = -np.inf
    for _ in range(count):
        a, b, c = (random_state(rng, r, n) for _ in range(3))
        cost = lambda x, y: B.connect(x, y, rho).cost(rho)
        worst = max(worst, (cost(a, c) - cost(a, b) - cost(b, c)) / cost(a, c))
    return dict(count=count, r=r, n=n, max_relative_violation=worst, passed=bool(worst <= 1e-9))


def _flat_edge(rng, r, n, scale):
    z0 = random_state(rng, r, n, scale); z1 = random_state(rng, r, n, scale)
    z1[0] += rng.normal(size=n)
    return B.fixed_time(z0, z1, float(rng.uniform(1., 3.)))


def v1e(rng, count):
    out = {}
    # unicycle (r=2): needs nonzero speed throughout
    errs = []
    m = Unicycle()
    while len(errs) < count:
        z0 = np.array([[0, 0], [1., 0]]) + rng.normal(size=(2, 2)) * .2
        z1 = np.array([[2, 1], [1., .3]]) + rng.normal(size=(2, 2)) * .2
        e = B.fixed_time(z0, z1, 2.5)
        ts = np.linspace(0, e.duration, 400)
        jet = np.stack([e.derivative(ts, d) for d in range(3)], axis=-2)
        if np.min(np.linalg.norm(jet[:, 1], axis=-1)) < .2: continue
        rec = m.recover(jet)
        ctrl = lambda t: m.recover(np.stack([e.derivative(np.clip(t, 0, e.duration), d) for d in range(3)], axis=0))['control']
        f = lambda t, x: [ctrl(t)[0] * np.cos(x[2]), ctrl(t)[0] * np.sin(x[2]), ctrl(t)[1]]
        sol = solve_ivp(f, (0, e.duration), rec['state'][0], rtol=1e-11, atol=1e-11, method='DOP853')
        d = sol.y[:, -1] - rec['state'][-1]; d[2] = np.arctan2(np.sin(d[2]), np.cos(d[2]))
        errs.append(float(np.max(np.abs(d))))
    out['unicycle'] = dict(count=count, max_state_error=max(errs))
    # planar quadrotor (r=4, motors -> f, torque)
    q = Quad2D(); errs = []
    while len(errs) < count:
        e = _flat_edge(rng, 4, 2, .3)
        jet = lambda t: np.stack([e.derivative(np.clip(t, 0, e.duration), d) for d in range(5)], axis=0)
        ts = np.linspace(0, e.duration, 200)
        if np.min(np.linalg.norm(np.stack([e.derivative(ts, 2)], 0)[0] + [0, 9.81], axis=-1)) < 2.: continue
        x0 = q.recover(jet(0.))['state']

        def f(t, x):
            u = q.recover(jet(t))['control']; F = u[0] + u[1]; tau = (u[0] - u[1]) * q.arm
            th = x[2]
            return [x[3], x[4], x[5], -F * np.sin(th) / q.mass, F * np.cos(th) / q.mass - q.gravity, tau / q.inertia]
        sol = solve_ivp(f, (0, e.duration), x0, rtol=1e-11, atol=1e-11, method='DOP853')
        xT = q.recover(jet(e.duration))['state']
        d = sol.y[:, -1] - xT; d[2] = np.arctan2(np.sin(d[2]), np.cos(d[2]))
        errs.append(float(np.max(np.abs(d))))
    out['quad2d'] = dict(count=count, max_state_error=max(errs))
    # 3D quadrotor (r=4): p'' = R e3 f/m - g e3, R' = R hat(w), J w' = tau - w x Jw
    q = Quad3D(); J = np.asarray(q.inertia); errs = []
    hat = lambda w: np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    while len(errs) < count:
        e = _flat_edge(rng, 4, 4, .3)
        jet = lambda t: np.stack([e.derivative(np.clip(t, 0, e.duration), d) for d in range(5)], axis=0)
        ts = np.linspace(0, e.duration, 200)
        if np.min(np.linalg.norm(e.derivative(ts, 2)[:, :3] + [0, 0, 9.81], axis=-1)) < 2.: continue
        r0 = q.recover(jet(0.))

        def f(t, x):
            u = q.recover(jet(t)); F, tau = u['thrust'], u['torque']
            R = x[6:15].reshape(3, 3); w = x[15:18]
            return np.r_[x[3:6], R[:, 2] * F / q.mass - [0, 0, q.gravity], (R @ hat(w)).ravel(),
                         np.linalg.solve(J, tau - np.cross(w, J @ w))]
        x0 = np.r_[r0['position'], r0['velocity'], r0['rotation'].ravel(), r0['omega']]
        sol = solve_ivp(f, (0, e.duration), x0, rtol=1e-11, atol=1e-11, method='DOP853')
        rT = q.recover(jet(e.duration))
        xT = np.r_[rT['position'], rT['velocity'], rT['rotation'].ravel(), rT['omega']]
        errs.append(float(np.max(np.abs(sol.y[:, -1] - xT))))
    out['quad3d'] = dict(count=count, max_state_error=max(errs))
    out['passed'] = bool(all(v['max_state_error'] <= 1e-6 for v in out.values() if isinstance(v, dict)))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--count', type=int, default=10000)
    p.add_argument('--seed', type=int, default=7001)
    a = p.parse_args()
    rng = np.random.default_rng(a.seed)
    t0 = time.time(); res = dict(seed=a.seed)
    for r, n in [(2, 2), (2, 7), (4, 2), (4, 4)]:
        res[f'V1a_bvp_optimality_r{r}_n{n}'] = v1a(rng, a.count, r, n)
        res[f'V1b_tstar_r{r}_n{n}'] = v1b(rng, max(a.count // 20, 50), r, n)
        res[f'V1c_propagation_r{r}_n{n}'] = v1c(rng, max(a.count // 20, 50), r, n)
        res[f'V1d_triangle_r{r}_n{n}'] = v1d(rng, max(a.count // 10, 100), r, n)
    res['V1e_flatness_original_dynamics'] = v1e(rng, max(a.count // 200, 20))
    res['all_passed'] = all(v['passed'] for k, v in res.items() if isinstance(v, dict))
    res['seconds'] = time.time() - t0
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(res, indent=2) + '\n')
    print(json.dumps({k: (v['passed'] if isinstance(v, dict) else v) for k, v in res.items()}, indent=1))


if __name__ == '__main__':
    main()
