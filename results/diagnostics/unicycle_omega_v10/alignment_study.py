"""Validity of optimal-T unicycle BVP edges vs (i) alignment of the endpoint velocities with the displacement and (ii) rho.
Diagnostic only: the paper fixes rho = 1."""
import json, sys
import numpy as np
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from flask_repro import _core
rng = np.random.default_rng(1)
def valid(e):
    c = np.asarray(e['coefficients']); T = e['duration']; s = np.linspace(0, 1, 801)
    v = sum(np.outer(k*s**(k-1), c[k]) for k in range(1, 4)) / T
    a = sum(np.outer(k*(k-1)*s**(k-2), c[k]) for k in range(2, 4)) / T**2
    sp = np.linalg.norm(v, axis=1); om = (v[:,0]*a[:,1]-v[:,1]*a[:,0]) / np.maximum(sp**2, 1e-12)
    return sp.max() <= 1 and np.abs(om).max() <= 1.5 and sp.min() > 1e-6
out = {}
for rho in (.1, 1., 10.):
    for spread in (0, 30, 90, 180):     # max deviation (deg) of both endpoint velocity directions from the displacement direction
        ok = 0; n = 2000
        for _ in range(n):
            L = rng.uniform(.2, 1.5); bearing = rng.uniform(0, 2*np.pi)
            def vel():
                ang = bearing + np.radians(rng.uniform(-spread, spread)); r = rng.uniform(.3, 1.)
                return [r*np.cos(ang), r*np.sin(ang)]
            a = [[0., 0.], vel()]; b = [[L*np.cos(bearing), L*np.sin(bearing)], vel()]
            e = _core.flat_bvp(a, b, rho)
            ok += e['duration'] > 0 and valid(e)
        out[f'rho={rho} spread={spread}deg'] = ok / n
print(json.dumps(out, indent=1))
json.dump(out, open(ROOT / 'results/diagnostics/unicycle_omega_v10/alignment_study.json', 'w'), indent=1)
