"""Why are 96-98% of faithful-unicycle BVP edges rejected by |omega| <= 1.5? Edge-level study (no planner).
Random flat states as the selected v10 planner proposes them (positions in the world, |v| in [0.01, 1] isotropic,
range clipping 1.5 m), optimal-T BVP with rho = 1; exact speed/omega along each edge (dense evaluation)."""
import json, sys
import numpy as np
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from flask_repro import _core
rng = np.random.default_rng(0)
def sample_state(lo, hi, vmin=.01, vmax=1.):
    p = rng.uniform(lo, hi); r = np.sqrt(rng.uniform(vmin**2, vmax**2)); a = rng.uniform(0, 2*np.pi)
    return np.array([p, [r*np.cos(a), r*np.sin(a)]])
def edge_stats(e):
    c = np.asarray(e['coefficients']); T = e['duration']; s = np.linspace(0, 1, 801)
    v = sum(np.outer(k*s**(k-1), c[k]) for k in range(1, 4)) / T
    a = sum(np.outer(k*(k-1)*s**(k-2), c[k]) for k in range(2, 4)) / T**2
    sp = np.linalg.norm(v, axis=1); om = (v[:,0]*a[:,1]-v[:,1]*a[:,0]) / np.maximum(sp**2, 1e-12)
    i = int(np.argmax(np.abs(om)))
    return dict(T=T, vmax=float(sp.max()), vmin=float(sp.min()), wmax=float(np.abs(om).max()), speed_at_wmax=float(sp[i]),
                curvature_at_wmax=float(abs(om[i])/max(sp[i],1e-12)))
rows = []
for _ in range(20000):
    a = sample_state([0, 0], [6, 6]); b = sample_state([0, 0], [6, 6])
    d = b[0]-a[0]; L = np.linalg.norm(d)
    if L > 1.5: b[0] = a[0] + d*1.5/L
    e = _core.flat_bvp(a.tolist(), b.tolist(), 1.)
    if e['duration'] > 0: rows.append(edge_stats(e) | dict(dist=float(min(L,1.5)), turn=float(abs(np.angle(complex(*b[1])/complex(*a[1]))))))
R = {k: np.array([r[k] for r in rows]) for k in rows[0]}
bad = R['wmax'] > 1.5; spd = R['vmax'] > 1.
out = dict(n=len(rows), omega_violation=float(bad.mean()), speed_violation=float(spd.mean()),
           omega_violation_low_speed_lt_0p2=float((bad & (R['speed_at_wmax'] < .2)).mean()),
           omega_violation_at_speed_ge_0p5=float((bad & (R['speed_at_wmax'] >= .5)).mean()),
           valid=float((~bad & ~spd).mean()),
           valid_by_turn_deg={f'{lo}-{lo+30}': float((~bad & ~spd)[(np.degrees(R['turn'])>=lo)&(np.degrees(R['turn'])<lo+30)].mean()) for lo in range(0,180,30)},
           median_curvature_at_wmax=float(np.median(R['curvature_at_wmax'][bad])))
print(json.dumps(out, indent=1))
json.dump(out, open(ROOT / 'results/diagnostics/unicycle_omega_v10/edge_study.json','w'), indent=1)
