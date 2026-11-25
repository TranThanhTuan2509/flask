"""Flat bvp: FLASK implementation and evaluation utilities."""
from dataclasses import dataclass
from functools import lru_cache
from math import factorial,comb
import numpy as np
from numpy.polynomial import polynomial as poly

@lru_cache(None)
def _hermite_matrix(r):
    return np.array([[factorial(k) / factorial(k - j) for k in range(r, 2 * r)] for j in range(r)], dtype=float)

@lru_cache(None)
def _shifted_legendre_transform(size):
    from numpy.polynomial.legendre import poly2leg
    transform=np.zeros((size,size))
    for degree in range(size):
        coefficients=np.array([comb(degree,k)/2.**degree for k in range(degree+1)])
        column=poly2leg(coefficients)
        transform[:len(column),degree]=column
    return transform

def _states(z0, z1):
    (a, b) = (np.asarray(z0, float), np.asarray(z1, float))
    if a.ndim != 2 or a.shape != b.shape or min(a.shape) < 1:
        raise ValueError('Boundary states must have matching (order, outputs) shapes')
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Boundary states must be finite')
    return (a, b)

def _weight(R, n):
    R = np.eye(n) if R is None else np.asarray(R, float)
    if R.shape != (n, n) or not np.isfinite(R).all() or (not np.allclose(R, R.T)):
        raise ValueError('Control weight must be finite, square, and symmetric')
    np.linalg.cholesky(R)
    return R

@dataclass
class FlatEdge:
    coefficients: np.ndarray
    duration: float
    order: int

    @property
    def n(self):
        return self.coefficients.shape[1]

    def derivative(self, times, degree=0):
        t = np.asarray(times, float)
        if not np.isfinite(t).all() or np.any(t < -1e-12) or np.any(t > self.duration + 1e-12):
            raise ValueError('Evaluation time is outside the trajectory interval')
        c = poly.polyder(self.coefficients, m=degree, axis=0)
        for _ in range(degree):c=c/self.duration
        return np.moveaxis(poly.polyval(np.clip(t / self.duration, 0, 1), c), 0, -1)

    def jet(self, times, through=None):
        """Jet."""
        through = self.order if through is None else through
        return np.stack([self.derivative(times, j) for j in range(through + 1)], axis=-2)

    def state(self, times):
        return self.jet(times, self.order - 1)

    def effort(self, R=None):
        R = _weight(R, self.n)
        c = poly.polyder(self.coefficients, m=self.order, axis=0)
        for _ in range(self.order):c=c/self.duration
        # Orthogonal polynomial energy: independent of native Gauss quadrature
        # and positive even when signed monomial cross terms would cancel.
        coefficients=_shifted_legendre_transform(len(c))@c
        weighted=coefficients@np.linalg.cholesky(R)
        weighted*=np.sqrt(self.duration/(2*np.arange(len(c))+1))[:,None]
        return float(np.sum(weighted*weighted))

    def cost(self, rho=1.0, R=None):
        return self.effort(R) + rho * self.duration

    def length(self, dimensions=None):
        from scipy.integrate import quad
        dims = slice(None) if dimensions is None else dimensions
        return quad(lambda t: np.linalg.norm(self.derivative(t, 1)[dims]), 0.0, self.duration, epsabs=1e-08, limit=100)[0]

def fixed_time(z0, z1, duration):
    """Fixed time."""
    (z0, z1) = _states(z0, z1)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError('Duration must be finite and positive')
    (r, n) = z0.shape
    c = np.zeros((2 * r, n))
    for j in range(r):
        c[j] = z0[j] * duration ** j / factorial(j)
    rhs = np.array([z1[j] * duration ** j - sum((c[k] * factorial(k) / factorial(k - j) for k in range(j, r))) for j in range(r)])
    c[r:] = np.linalg.solve(_hermite_matrix(r), rhs)
    return FlatEdge(c, float(duration), r)

def gramian(duration, order, outputs=1, R=None):
    """Gramian."""
    if duration <= 0 or order < 1:
        raise ValueError('Gramian requires positive duration and order')
    Ri = np.linalg.inv(_weight(R, outputs))
    r = order
    G = np.array([[duration ** (2 * r - 1 - i - j) / (factorial(r - 1 - i) * factorial(r - 1 - j) * (2 * r - 1 - i - j)) for j in range(r)] for i in range(r)])
    return np.kron(G, Ri)

def effort_coefficients(z0, z1, R=None):
    """Effort coefficients."""
    (z0, z1) = _states(z0, z1)
    (r, n) = z0.shape
    R = _weight(R, n)
    Gi = np.linalg.inv(gramian(1.0, r))
    d = []
    for i in range(r):
        di = -np.array([z0[j] / factorial(j - i) for j in range(i, r)])
        di[0] += z1[i]
        d.append(di)
    E = np.zeros(2 * r)
    for i in range(r):
        for j in range(r):
            for (a, already) in enumerate(d[i]):
                for (b, db) in enumerate(d[j]):
                    E[2 * r - 1 - i - j - a - b] += Gi[i, j] * (already @ R @ db)
    return E

def optimal_time(z0, z1, rho=1.0, R=None):
    """Optimal time."""
    (z0, z1) = _states(z0, z1)
    if not np.isfinite(rho) or rho <= 0:
        raise ValueError('Time weight rho must be finite and positive')
    R = _weight(R, z0.shape[1])
    # A resting identity has only the T -> 0 infimum for rho > 0. Equal
    # nonstationary states still have a positive-duration return BVP: closing
    # position while preserving nonzero derivatives requires control effort.
    # A planner's separate zero-duration goal-membership shortcut is different.
    if np.array_equal(z0, z1) and not np.any(z0[1:]):
        return None
    r = len(z0)
    root_rho=np.sqrt(rho)
    magnitude=np.r_[np.max(np.abs(z1[0]-z0[0])),
                    [np.max(np.abs([z0[d],z1[d]])) for d in range(1,r)]]
    time_scale=max((value/root_rho)**(1/(r-degree)) for degree,value in enumerate(magnitude))
    if not np.isfinite(time_scale) or time_scale<=0:return None
    normalized_start=np.zeros_like(z0);normalized_goal=np.empty_like(z1)
    for degree in range(r):
        normalized_start[degree]=0. if degree==0 else z0[degree]/root_rho
        normalized_goal[degree]=((z1[0]-z0[0]) if degree==0 else z1[degree])/root_rho
        for _ in range(r-degree):
            normalized_start[degree]/=time_scale;normalized_goal[degree]/=time_scale
    E = effort_coefficients(normalized_start, normalized_goal, R)
    scale = max((abs(k * E[k])) ** (1 / (k + 1)) for k in range(1, 2 * r))
    if scale == 0:
        return None
    p = np.zeros(2 * r + 1)
    p[-1] = 1.0
    for k in range(1, 2 * r):
        p[2 * r - k - 1] = -k * E[k] / scale ** (k + 1)
    roots = poly.polyroots(p)
    Ts = [float(z.real * scale*time_scale) for z in roots if z.real > 0 and abs(z.imag) < 1e-07 * abs(z.real)]
    for degree in range(r-1):
        mean=.5*(z0[degree+1]+z1[degree+1])
        for delta,velocity in zip(z1[degree]-z0[degree],mean):
            if velocity!=0:
                candidate=delta/velocity
                if candidate>0 and np.isfinite(candidate):Ts.append(float(candidate))
    if not Ts:
        raise ArithmeticError('No positive real stationary duration was found')
    return min(Ts, key=lambda T: fixed_time(z0, z1, T).cost(rho, R))

def connect(z0, z1, rho=1.0, R=None):
    T = optimal_time(z0, z1, rho, R)
    return None if T is None else fixed_time(z0, z1, T)

def propagate(z0, control, duration):
    """Propagate."""
    z0 = np.asarray(z0, float)
    control = np.asarray(control, float)
    _states(z0, z0)
    if duration <= 0 or not np.isfinite(duration) or control.shape != z0.shape[1:] or (not np.isfinite(control).all()):
        raise ValueError('Propagation requires a finite control vector and positive duration')
    r = len(z0)
    c = np.array([z0[j] * duration ** j / factorial(j) for j in range(r)] + [control * duration ** r / factorial(r)])
    return FlatEdge(c, float(duration), r)
