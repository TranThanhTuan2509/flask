"""Diagnostic: unicycle flat chart kappa (Eq. 10) forward vs reverse under the frozen v2 settings, dev seeds."""
import json, os, sys
import numpy as np
sys.path.insert(0, '.')
from benchmarks.v2_factorial import case_setup
from flask_repro.planning import PlannerSettings, plan, validate
os.sched_setaffinity(0, {10})
prof = json.load(open('config/low_dof_v2_order2.json'))
out = []
for case in ['unicycle_wall', 'unicycle_bugtrap']:
    for alg in ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']:
        for reverse in (True, False):
            model, planning, original, start, goal, base = case_setup(case)
            model.reverse = reverse
            extra = prof['case_overrides'][case]['algorithm_settings'][alg]
            cfg = PlannerSettings(**(base | dict(algorithm=alg) | {k: (tuple(v) if isinstance(v, list) else v) for k, v in extra.items()}))
            rows = []
            for seed in range(1001, 1011):
                r = plan(model, planning, start, goal, cfg, seed); v = validate(r, model, original, cfg)
                rows.append(dict(valid=bool(r.solved and v['valid']), ms=(r.plan_ns + r.postprocess_ns) / 1e6,
                                 length=r.trajectory.length(2) if r.solved else np.nan))
            s = dict(case=case, alg=alg, chart='reverse' if reverse else 'forward', valid=sum(x['valid'] for x in rows),
                     mean_ms=float(np.mean([x['ms'] for x in rows])), length=float(np.nanmean([x['length'] for x in rows])))
            out.append(s); print(json.dumps(s), flush=True)
json.dump(out, open('.tmp/chart_study.json', 'w'), indent=1)
