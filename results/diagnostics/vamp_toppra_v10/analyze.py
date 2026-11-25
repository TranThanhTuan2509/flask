"""Post-hoc breakdown of VAMP RRTConnect + TOPP-RA invalidity in table_ii_v10_rest (no re-planning, no timing).

The v10 gate (declared beforehand) counts an attempt valid only with no dense/URDF collision and sampled limit
excess <= 1e-5. This script only explains the failures; it does not change any verdict.
Usage: python results/diagnostics/vamp_toppra_v10/analyze.py
"""
import collections, json, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[3]; sys.path.insert(0, str(ROOT))
from flask_repro import robot as R
from benchmarks.mbm_baseline import topp_ra

S = ROOT / 'results/paper_campaigns/2026-11-25_final_v10/table_ii_v10_rest'
rb = R.get_robot('panda')
rows = [json.loads(l) for l in open(S / 'attempts.jsonl')]
rows = [r for r in rows if r.get('method') == 'vamp_toppra']
out = collections.Counter(); per_env = collections.defaultdict(collections.Counter); rv = []; ra = []
for r in rows:
    if not r['solved']:
        out['unsolved'] += 1; continue
    val = r['validation']
    col = bool(val['dense']['collision'] or val['independent_environment']['environment_collision'])
    lim = val['sampled_limit_violation'] > 1e-5
    key = 'collision+limit' if col and lim else 'collision_only' if col else 'limit_only' if lim else 'valid'
    out[key] += 1; per_env[r['environment']][key] += 1
    d = np.load(S / r['trajectory_artifact'])
    t = topp_ra(d['simplified_waypoints'], rb, 'chord_length', None)  # deterministic reconstruction
    ts = np.linspace(0, t.duration, 2000)
    rv.append(float(np.max(np.abs(t(ts, 1)) / rb.v_max))); ra.append(float(np.max(np.abs(t(ts, 2)) / rb.a_max)))
pct = lambda x: dict(zip(['p50', 'p90', 'p99', 'max'], np.round(np.r_[np.percentile(x, [50, 90, 99]), np.max(x)], 4).tolist()))
summary = dict(attempts=len(rows), outcomes=dict(out), per_environment={k: dict(v) for k, v in per_env.items()},
               max_velocity_over_limit=pct(rv), max_acceleration_over_limit=pct(ra),
               within_1pct_limits=int(sum(v <= 1.01 and a <= 1.01 for v, a in zip(rv, ra))),
               collision_free=int(out['valid'] + out['limit_only']),
               explanation=['TOPP-RA enforces velocity/acceleration only at its gridpoints; between them the spline '
                            'curvature term q\'\'(s) sdot^2 exceeds the limits slightly (limit_only).',
                            'SplineInterpolator through the shortcut waypoints leaves the piecewise-linear path VAMP '
                            'certified; collisions are spline deviation (collision_*).'])
(Path(__file__).parent / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
