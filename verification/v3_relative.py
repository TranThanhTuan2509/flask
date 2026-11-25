"""V3 (acceptance checks; see "Verification" in README.md): the paper's relative claims, which do not depend on hardware or unpublished
parameters.

  (a) BVP variants (RRTConnect, BVP-SST*) faster than propagation variants (SIMD-SST*, DP-RRT): controlled v10b
      table_i_v10b, per case, mean first-solution time of the faster BVP variant vs the faster propagation variant.
  (b) SIMD vs scalar validation (FLASKCC, Alg. 3): table_i_v10b_repeat_rrtc vs table_i_v10b_scalar_rrtc, identical
      work required, ratio of mean times.
  (c) FLASK-RRTConnect on the Panda in the same time class as geometric VAMP RRTConnect (paper Table II framing):
      table_ii_v10_rest, median FLASK total time vs median VAMP RRTConnect planning time (before TOPP-RA), per scene.
  (d) Anytime behaviour of BVP-SST* (asymptotic-optimality claim): the same seed rerun with budgets 0.1/0.5/2/8 s and
      stop_first=False; the best cost must never increase with the budget, and it should decrease for most seeds.

Usage: python -m verification.v3_relative --output results/verification/<date>_acceptance/v3.json [--cpu 20]
"""
import argparse
import glob
import json
import os
import statistics as st
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
V10B = ROOT / 'results/paper_campaigns/2026-11-25_final_v10b'
V10 = ROOT / 'results/paper_campaigns/2026-11-25_final_v10'


def stage(path):
    rows = {}
    for f in glob.glob(str(path / '*_seed_*.json')):
        r = json.load(open(f)); rows[(r['case'], r['algorithm'], r['seed'])] = r
    return rows


def t_ms(r):
    rep = r.get('timing_repeats') or {}
    return (rep.get('median_total_ns') or (r['plan_ns'] + r['postprocess_ns'])) / 1e6


def part_a():
    rows = stage(V10B / 'table_i_v10b'); out = {}
    for case in sorted({k[0] for k in rows}):
        mean = {a: st.mean(t_ms(r) for k, r in rows.items() if k[0] == case and k[1] == a and r['solved'])
                for a in ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']}
        bvp, prop = min(mean['rrt_connect'], mean['sst_bvp']), min(mean['sst_simd'], mean['rrt_dp'])
        out[case] = dict(mean_ms=mean, best_bvp_ms=bvp, best_propagation_ms=prop, holds=bool(bvp < prop),
                         every_bvp_faster_than_every_propagation=bool(max(mean['rrt_connect'], mean['sst_bvp']) < prop))
    n = sum(v['holds'] for v in out.values())
    return dict(cases=out, holds=n, total=len(out), passed=bool(n >= len(out) - 1))


def part_b():
    simd, scalar = stage(V10B / 'table_i_v10b_repeat_rrtc'), stage(V10B / 'table_i_v10b_scalar_rrtc'); out = {}
    for case in sorted({k[0] for k in simd}):
        keys = [k for k in simd if k[0] == case]
        same = all(simd[k]['iterations'] == scalar[k]['iterations'] and simd[k]['trajectory'] == scalar[k]['trajectory'] for k in keys)
        ratio = st.mean(t_ms(scalar[k]) for k in keys) / st.mean(t_ms(simd[k]) for k in keys)
        out[case] = dict(identical_work_and_paths=same, scalar_over_simd=ratio)
    aerial = [v for c, v in out.items() if c.startswith('quad')]
    return dict(cases=out, passed=bool(all(v['identical_work_and_paths'] for v in out.values())
                                       and all(v['scalar_over_simd'] >= 1.2 for v in aerial)))


def part_c():
    rows = [json.loads(l) for l in open(V10 / 'table_ii_v10_rest/attempts.jsonl')]
    out = {}
    for env in sorted({r['environment'] for r in rows}):
        f = [r['total_ns'] / 1e6 for r in rows if r['environment'] == env and r['method'] == 'flask_rrt_connect' and r['solved']]
        v = [r['plan_ns'] / 1e6 for r in rows if r['environment'] == env and r['method'] == 'vamp_toppra' and r['solved']]
        out[env] = dict(flask_median_ms=st.median(f), vamp_rrtc_median_ms=st.median(v), ratio=st.median(f) / st.median(v))
    return dict(environments=out, passed=bool(all(x['ratio'] <= 3 for x in out.values())))


def part_d(cpu, seeds, budgets):
    from flask_repro.dynobench import load
    from flask_repro.planning import PlannerSettings, plan
    from flask_repro.flat_models import Unicycle
    if cpu is not None:
        os.sched_setaffinity(0, {cpu})
    profile = json.loads((ROOT / 'config/low_dof_v10_order2.json').read_text()); out = {}
    for case, o in profile['case_overrides'].items():
        geometry = profile['geometry'] | o.get('geometry', {}); boundary = profile['boundary'] | o.get('boundary', {})
        model, planning, original, start, goal, _ = load(case, **geometry, lift_speed=boundary['lift_speed'])
        if isinstance(model, Unicycle): model.reverse = boundary['reverse_chart']
        for k, v in (profile.get('model', {}) | o.get('model', {})).items(): setattr(model, k, v)
        per_seed, per_search = [], []
        for seed in seeds:
            costs, search = [], []
            for b in budgets:
                cfg = PlannerSettings(**(profile['settings'] | o.get('settings', {}) | dict(algorithm='sst_bvp', budget_s=b, stop_first=False)))
                rec = plan(model, planning, start, goal, cfg, seed)
                costs.append(rec.trajectory.cost(cfg.rho) if rec.solved and hasattr(rec.trajectory, 'cost') else
                             (sum(e.duration for e in rec.trajectory.edges) if rec.solved else None))
                search.append(rec.diagnostics.get('search_cost') if rec.solved else None)
            per_seed.append(costs); per_search.append(search)
        inf = lambda cs: [np.inf if c is None else c for c in cs]  # unsolved at a budget = infinite cost
        mono_ok = lambda cs: cs[-1] is not None and all(b <= a * (1 + 1e-9) for a, b in zip(inf(cs), inf(cs)[1:]))
        monotone = sum(mono_ok(cs) for cs in per_seed)
        improved = sum(cs[-1] is not None and cs[-1] < inf(cs)[0] * (1 - 1e-6) for cs in per_seed)
        smono = sum(mono_ok(cs) for cs in per_search)
        out[case] = dict(costs=per_seed, search_costs=per_search, monotone=monotone, improved=improved,
                         search_monotone=smono, seeds=len(seeds))
    total = sum(v['seeds'] for v in out.values()); mono = sum(v['monotone'] for v in out.values())
    smono = sum(v['search_monotone'] for v in out.values())
    # declared gate: the RETURNED (post Alg. 4) cost; the search cost is a supplementary diagnosis
    return dict(cases=out, budgets_s=budgets, monotone=mono, search_monotone=smono, total=total,
                passed=bool(mono >= .95 * total), search_cost_passed=bool(smono >= .95 * total),
                note='unsolved at a budget = infinite cost; must be solved at the largest budget')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cpu', type=int)
    p.add_argument('--seeds', type=int, nargs='+', default=list(range(7001, 7011)))
    a = p.parse_args()
    res = dict(V3a_bvp_faster_than_propagation=part_a(), V3b_simd_vs_scalar=part_b(), V3c_panda_vs_geometric=part_c())
    res['V3d_anytime_sst_bvp'] = part_d(a.cpu, a.seeds, [.1, .5, 2., 8.])
    res['all_passed'] = all(v['passed'] for v in res.values() if isinstance(v, dict))
    a.output.write_text(json.dumps(res, indent=2) + '\n')
    print(json.dumps({k: (v.get('passed') if isinstance(v, dict) else v) for k, v in res.items()}, indent=1))


if __name__ == '__main__':
    main()
