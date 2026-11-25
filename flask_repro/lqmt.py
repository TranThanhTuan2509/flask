"""Lqmt: FLASK implementation and evaluation utilities."""
import numpy as np

def cost_coefficients(q0, v0, q1, v1):
    dq = np.asarray(q1, float) - np.asarray(q0, float)
    v0 = np.asarray(v0, float)
    v1 = np.asarray(v1, float)
    a = float(dq @ dq)
    b = float((v1 + v0) @ dq)
    c = float(v0 @ v0 + v0 @ v1 + v1 @ v1)
    return (a, b, c)

def cost(T, a, b, c, rho=1.0):
    """Cost."""
    return 12.0 * a / T ** 3 - 12.0 * b / T ** 2 + 4.0 * c / T + rho * T

def boundary_cost(q0,v0,q1,v1,T,rho=1.):
    difference=np.asarray(q1,float)-np.asarray(q0,float)
    initial=np.asarray(v0,float);terminal=np.asarray(v1,float)
    residual=(difference-.5*(initial+terminal)*T)/T
    return float(np.sum((12*residual*residual+(terminal-initial)**2)/T)+rho*T)

def duration_roots(a, b, c, rho=1.0):
    """Duration roots."""
    if not np.isfinite([a, b, c, rho]).all() or rho <= 0:
        raise ValueError('Cost coefficients and positive time weight must be finite')
    scale = max((36 * a / rho) ** 0.25, np.sqrt(4 * c / rho), np.cbrt(24 * abs(b) / rho))
    if scale == 0:
        return []
    r = np.roots([1.0, 0.0, -4 * c / rho / scale ** 2, 24 * b / rho / scale ** 3, -36 * a / rho / scale ** 4])
    return sorted((float(z.real * scale) for z in r if abs(z.imag) < 1e-07 * max(1.0, abs(z)) and z.real > 0))

def optimal_duration(q0, v0, q1, v1, rho=1.0):
    """Optimal duration."""
    (a, b, c) = cost_coefficients(q0, v0, q1, v1)
    Ts = duration_roots(a, b, c, rho)
    for delta,mean in zip(np.asarray(q1)-np.asarray(q0),.5*(np.asarray(v0)+np.asarray(v1))):
        if mean!=0:
            candidate=delta/mean
            if candidate>0 and np.isfinite(candidate):Ts.append(float(candidate))
    if not Ts:
        return None
    return min(Ts, key=lambda T: boundary_cost(q0,v0,q1,v1,T,rho))

def hermite_coefficients(q0, v0, q1, v1, T):
    """Hermite coefficients."""
    q0 = np.asarray(q0, float)
    v0 = np.asarray(v0, float)
    dp = np.asarray(q1, float) - q0 - T * v0
    dv = np.asarray(v1, float) - v0
    c3 = -2.0 * dp / T ** 3 + dv / T ** 2
    c2 = 3.0 * dp / T ** 2 - dv / T
    return (c3, c2, v0.copy(), q0.copy())

def evaluate(coef, t):
    """Evaluate."""
    (c3, c2, c1, c0) = coef
    t = np.asarray(t, float)[:, None]
    q = ((c3 * t + c2) * t + c1) * t + c0
    trajectory = (3.0 * c3 * t + 2.0 * c2) * t + c1
    qdd = 6.0 * c3 * t + 2.0 * c2
    qddd = np.broadcast_to(6.0 * c3, q.shape)
    return (q, trajectory, qdd, qddd)

def lqmt_r2(q0, v0, q1, v1, rho=1.0):
    """Lqmt r2."""
    T = optimal_duration(q0, v0, q1, v1, rho)
    if T is None:
        return None
    (a, b, c) = cost_coefficients(q0, v0, q1, v1)
    return (T, hermite_coefficients(q0, v0, q1, v1, T), boundary_cost(q0,v0,q1,v1,T,rho))

def cubic_extrema(coef, T):
    """Cubic extrema."""
    (c3, c2, c1, c0) = (np.asarray(x, float) for x in coef)
    n = c0.shape[0]
    q_min = np.empty(n)
    q_max = np.empty(n)
    v_abs = np.empty(n)
    a_abs = np.empty(n)
    for j in range(n):
        ts = [0.0, T]
        normalized=np.array([c1[j]*T,(c2[j]*T)*T,((c3[j]*T)*T)*T])
        if not np.isfinite(normalized).all():
            q_min[j],q_max[j],v_abs[j],a_abs[j]=-np.inf,np.inf,np.inf,np.inf
            continue
        h1,h2,h3=normalized
        if h3 != 0.:
            s = -h2 / (3.0 * h3)
            if 0.0 < s < 1.:
                ts_v = ts + [s*T]
            else:
                ts_v = ts
        else:
            ts_v = ts
        ts_q = list(ts)
        scale=np.max(np.abs(normalized))
        if scale>0.:
            # Companion-matrix polynomial roots are independent of the native
            # stable quadratic formula; both operate on dimensionless time.
            derivative=np.array([h1/scale,2*(h2/scale),3*(h3/scale)])
            for root in np.polynomial.polynomial.polyroots(derivative):
                if abs(root.imag)<1e-12 and 0.<root.real<1.:
                    ts_q.append(float(root.real)*T)
        tq = np.array(ts_q)
        qv = ((c3[j] * tq + c2[j]) * tq + c1[j]) * tq + c0[j]
        tv_ = np.array(ts_v)
        vv = (3 * c3[j] * tv_ + 2 * c2[j]) * tv_ + c1[j]
        av = 6 * c3[j] * np.array(ts) + 2 * c2[j]
        (q_min[j], q_max[j]) = (qv.min(), qv.max())
        v_abs[j] = np.abs(vv).max()
        a_abs[j] = np.abs(av).max()
    return (q_min, q_max, v_abs, a_abs)

def within_limits(coef, T, q_lo, q_hi, v_max, a_max):
    (q_min, q_max, v_abs, a_abs) = cubic_extrema(coef, T)
    return bool(np.all(q_min >= q_lo) and np.all(q_max <= q_hi) and np.all(v_abs <= v_max) and np.all(a_abs <= a_max))
