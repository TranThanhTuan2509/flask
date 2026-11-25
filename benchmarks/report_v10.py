"""Claim evaluation with the pre-declared v10 criteria (config/claim_criteria_v10.json).

Differences from the v7/v8 generators (which stay unchanged as historical records):
  * validity is the ORIGINAL-model independent check, with every attempt in the denominator; Table II uses
    `validation.valid` (dense VAMP + URDF + limits), not "returned";
  * time statistics use validated attempts only; an attempt's time is the median of its timing repeats when present;
  * the time band is two-sided around k_hw * paper: < 1/3 -> DIFFERENT_FASTER, > 3 -> DIFFERENT_SLOWER;
  * order-2 aerial rows are judged on the reduced-order protocol check but labelled REDUCED_ORDER_PROTOCOL
    (never an original-model claim); unicycle stop-turn-go rows are a labelled variant, never REPRODUCED;
  * a stage whose jobs.json entry is not `controlled` yields UNCONTROLLED instead of a verdict;
  * 95% cluster-bootstrap intervals (seeds for Table I, (environment, problem) for Table II) for time ratios.

Usage (re-evaluating the laptop v8 campaign as a check of the gates):
  python -m benchmarks.report_v10 --campaign results/paper_campaigns/2026-11-24_final_v8 \
      --table-i table_i_v8 --table-ii table_ii_v8 --seeds-i 3001-3030 --seeds-ii 3001-3010 \
      --calibration config/hardware_calibration.json --output DIR
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CASES = ['unicycle_bugtrap', 'unicycle_wall', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']
VARIANTS = ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']
ENVS = ['bookshelf_thin', 'bookshelf_tall', 'bookshelf_small', 'cage', 'box', 'table_under_pick', 'table_pick']
LOW, HIGH, LEN_TOL, MIN_VALID = 1 / 3, 3., .20, .95


def wilson(k, n):
    if n == 0:
        return [None, None]
    z = 1.959963984540054; p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [max(0., c - h), min(1., c + h)]


def stats(v):
    a = np.asarray([x for x in v if x is not None and np.isfinite(x)], float)
    if not a.size:
        return dict(n=0, mean=None, p50=None, p75=None, p95=None)
    return dict(n=int(a.size), mean=float(a.mean()), p50=float(np.median(a)), p75=float(np.percentile(a, 75)),
                p95=float(np.percentile(a, 95)))


def cluster_ci(values_by_cluster, stat=np.mean, n=2000, seed=0):
    """95% percentile interval of `stat` over values, resampling whole clusters."""
    clusters = [np.asarray(v, float) for v in values_by_cluster.values() if len(v)]
    if not clusters:
        return [None, None]
    rng = np.random.default_rng(seed)
    out = [stat(np.concatenate([clusters[i] for i in rng.integers(0, len(clusters), len(clusters))])) for _ in range(n)]
    return [float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))]


def seeds(text):
    a, _, b = text.partition('-')
    return list(range(int(a), int(b) + 1)) if b else [int(a)]


def time_verdict(local, paper, k_hw):
    """Two-sided band on local / (k_hw * paper)."""
    if local is None or paper is None:
        return None, 'NO_DATA'
    ratio = local / (k_hw * paper)
    return ratio, 'DIFFERENT_FASTER' if ratio < LOW else 'DIFFERENT_SLOWER' if ratio > HIGH else 'WITHIN_BAND'


def verdict(complete, controlled, valid_frac, time_flags, length_ok, label=None):
    if not complete:
        return 'PENDING'
    if not controlled:
        return 'UNCONTROLLED'
    if label == 'VARIANT':
        return 'VARIANT_NOT_PAPER_PROTOCOL'
    if valid_frac < MIN_VALID:
        return 'DIFFERENT_INVALID'
    for flag in time_flags:
        if flag != 'WITHIN_BAND':
            return flag
    if length_ok is False:
        return 'DIFFERENT_LENGTH'
    return 'REPRODUCED' if label is None else f'{label}:REPRODUCED'


def attempt_ms(r, plan='plan_ns', post='postprocess_ns'):
    rep = r.get('timing_repeats') or {}
    if rep.get('median_total_ns') is not None:
        return rep['median_total_ns'] / 1e6
    return (r[plan] + r.get(post, 0)) / 1e6


def table_i(campaign, stage, seed_list, paper, k_hw, controlled):
    rows = {}
    for p in sorted((campaign / stage).glob('*_seed_*.json')):
        r = json.loads(p.read_text())
        rows[(r['case'], r['algorithm'], r['seed'])] = r
    out = []
    for case in CASES:
        for v in VARIANTS:
            g = [rows.get((case, v, s)) for s in seed_list]
            if all(x is None for x in g):
                continue
            complete = all(x is not None for x in g)
            g = [x for x in g if x is not None]
            settings = g[0].get('settings', {})
            stop_turn = bool(settings.get('unicycle_rest_turns'))
            reduced = any('protocol_validation' in r['validation'] and r['validation'].get('reduced_order') for r in g)
            full_valid = [r for r in g if r['validation']['valid'] and not stop_turn]
            if reduced:
                judged = [r for r in g if r['validation']['protocol_validation']['valid']]
                label = 'REDUCED_ORDER_PROTOCOL'
            elif stop_turn:
                judged = [r for r in g if r['validation']['valid']]
                label = 'VARIANT'
            else:
                judged, label = full_valid, None
            ref = paper.get(case, {}).get(v, {})
            times = {r['seed']: [attempt_ms(r)] for r in judged}
            t = stats([x[0] for x in times.values()])
            length = stats([r.get('length_m') for r in judged])
            ratio, flag = time_verdict(t['mean'], ref.get('planning_ms'), k_hw)
            length_ok = None if length['mean'] is None or not ref.get('length_m') else abs(length['mean'] / ref['length_m'] - 1) <= LEN_TOL
            ci = cluster_ci(times)
            out.append(dict(case=case, variant=v, attempts=len(g), complete=complete,
                            original_model_valid=len(full_valid), judged_valid=len(judged), label=label or 'PAPER_PROTOCOL',
                            valid_ci=wilson(len(judged), len(g)), time_ms=t, length_m=length,
                            paper_ms=ref.get('planning_ms'), paper_length_m=ref.get('length_m'),
                            ratio_mean=ratio, ratio_mean_ci=[x / (k_hw * ref['planning_ms']) if x is not None and ref.get('planning_ms') else None for x in ci],
                            status=verdict(complete, controlled, len(judged) / max(1, len(g)), [flag], length_ok, label)))
    return out


def table_ii(campaign, stage, method, seed_list, paper, k_hw, controlled, eligible):
    path = campaign / stage / 'attempts.jsonl'
    rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []
    rows = [r for r in rows if r.get('method') == method and r['seed'] in seed_list]
    have = {(r['environment'], r['problem'], r['seed']) for r in rows}
    out = []
    for env in ENVS + ['overall']:
        g = rows if env == 'overall' else [r for r in rows if r['environment'] == env]
        exp = {(e, i, s) for e, i in eligible for s in seed_list if env in ('overall', e)}
        valid = [r for r in g if r['solved'] and r['validation']['valid']]
        clusters = {}
        for r in valid:
            clusters.setdefault((r['environment'], r['problem']), []).append(attempt_ms(r, 'plan_ns', 'shortcut_ns') if 'timing_repeats' in r else r['total_ns'] / 1e6)
        t = stats([x for v in clusters.values() for x in v])
        length = stats([r.get('length_rad') for r in valid])
        ref = paper[env]
        r_mean, f_mean = time_verdict(t['mean'], ref['total'][0], k_hw)
        r_med, f_med = time_verdict(t['p50'], ref['total'][3], k_hw)
        length_ok = None if length['mean'] is None else abs(length['mean'] / ref['length'][0] - 1) <= LEN_TOL
        ci = cluster_ci(clusters, np.median)
        out.append(dict(environment=env, attempts=len(g), complete=exp <= have, returned=sum(r['solved'] for r in g),
                        validated=len(valid), returned_invalid=sum(r['solved'] and not r['validation']['valid'] for r in g),
                        valid_ci=wilson(len(valid), len(g)), time_ms=t, length_rad=length,
                        paper_mean_ms=ref['total'][0], paper_median_ms=ref['total'][3], paper_length=ref['length'][0],
                        ratio_mean=r_mean, ratio_median=r_med,
                        ratio_median_ci=[x / (k_hw * ref['total'][3]) if x is not None else None for x in ci],
                        status=verdict(exp <= have, controlled, len(valid) / max(1, len(g)), [f_mean, f_med], length_ok)))
    return out


def eligible_pairs():
    from flask_repro import mbm
    ds = mbm.load_dataset('panda')
    return [(e, i) for e in ENVS for i, p in enumerate(ds[e]) if p['valid']]


def fmt(x, spec='.2f'):
    return '–' if x is None else format(x, spec)


def markdown(res):
    L = [f"# v10 claim evaluation: {res['campaign']}", '',
         f"Criteria: `config/claim_criteria_v10.json`. k_hw = {fmt(res['k_hw'], '.3f')} ({res['calibration']}). "
         'Ratios are local / (k_hw · paper); band [1/3, 3].', '']
    if res.get('table_i'):
        L += ['## Table I', '', '| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |',
              '|---|---|---|---|---|---|---|---|---|---|']
        for r in res['table_i']:
            ci = r['ratio_mean_ci']
            L.append(f"| {r['case']} | {r['variant']} | {r['label']} | {r['judged_valid']}/{r['attempts']} | {r['original_model_valid']}/{r['attempts']} | "
                     f"{fmt(r['time_ms']['mean'])} / {fmt(r['time_ms']['p50'])} | {r['paper_ms']} | {fmt(r['ratio_mean'])} [{fmt(ci[0])}, {fmt(ci[1])}] | "
                     f"{fmt(r['length_m']['mean'])} / {r['paper_length_m']} | **{r['status']}** |")
        L.append('')
    if res.get('table_ii'):
        L += ['## Table II (FLASK-RRTConnect)', '', '| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |',
              '|---|---|---|---|---|---|---|---|']
        for r in res['table_ii']:
            ci = r['ratio_median_ci']
            L.append(f"| {r['environment']} | {r['validated']}/{r['attempts']} | {r['returned_invalid']} | {fmt(r['time_ms']['mean'])} / {fmt(r['time_ms']['p50'])} | "
                     f"{r['paper_mean_ms']} / {r['paper_median_ms']} | {fmt(r['ratio_mean'])} / {fmt(r['ratio_median'])} [{fmt(ci[0])}, {fmt(ci[1])}] | "
                     f"{fmt(r['length_rad']['mean'])} / {r['paper_length']} | **{r['status']}** |")
        L.append('')
    return '\n'.join(L) + '\n'


def main(a):
    campaign = (ROOT / a.campaign).resolve() if not a.campaign.is_absolute() else a.campaign
    jobs = {j['name']: j for j in json.loads((campaign / 'jobs.json').read_text())} if (campaign / 'jobs.json').exists() else {}
    cal = json.loads(a.calibration.read_text())
    k_hw = float(cal['k_hw'])
    if not .25 <= k_hw <= 4:
        raise SystemExit('HARDWARE_UNCALIBRATED: k_hw outside [0.25, 4]')
    paper_i = json.loads((ROOT / 'config/paper_reference_tables.json').read_text())['table_i']
    paper_ii = json.loads((ROOT / 'config/paper_table_ii_components.json').read_text())['flask_rrt_connect']
    res = dict(campaign=str(campaign.relative_to(ROOT)) if campaign.is_relative_to(ROOT) else str(campaign),
               calibration=str(a.calibration), k_hw=k_hw)
    if a.table_i:
        res['table_i'] = table_i(campaign, a.table_i, seeds(a.seeds_i), paper_i, k_hw, bool(jobs.get(a.table_i, {}).get('controlled')))
    if a.table_ii:
        res['table_ii'] = table_ii(campaign, a.table_ii, a.method, seeds(a.seeds_ii), paper_ii, k_hw,
                                   bool(jobs.get(a.table_ii, {}).get('controlled')), eligible_pairs())
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output / 'claims_v10.json').write_text(json.dumps(res, indent=1) + '\n')
    (a.output / 'claims_v10.md').write_text(markdown(res))
    print(markdown(res))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--campaign', type=Path, required=True)
    p.add_argument('--table-i'); p.add_argument('--table-ii')
    p.add_argument('--method', default='flask_rrt_connect')
    p.add_argument('--seeds-i', default='3001-3030'); p.add_argument('--seeds-ii', default='3001-3010')
    p.add_argument('--calibration', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    main(p.parse_args())
