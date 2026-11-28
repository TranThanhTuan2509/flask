"""Figures and LaTeX tables for the README, from the controlled campaigns.

Sources (local raw data, summarized once into results/readme_figures_data.json, which is tracked so the figures can
be rebuilt from a clone):
  * FLASK, Table I (headline, flask_v2): campaign 2026-11-28_final_v2d (controlled, CPU 10, v2 with lever L7): unicycle
    v2 and v1 literal, quadrotors r = 2 and r = 4, all on held-out seeds 8101-8130, interval certificate (falls back to
    the release campaign final_v2c without L7, then to the earlier v2 / v2b campaigns, when those reports are absent)
  * v2 levers: results/diagnostics/v2_mechanism (edge study) and v2_factorial/selection.json (development seeds)
  * FLASK, Table II (Panda): campaign v10b (held-out seeds 6001+, controlled, CPU 10) reports
  * VAMP RRTConnect + TOPP-RA: campaign v10, same machine; Dynoplan SST / iDb-A*: v10 baselines (seeds 5001-5030)
  * UR5 tracking: results/ur5_sim_v10/e1_mesh_ff (MuJoCo)
Times "scaled to the paper's CPU" divide by k_hw (VAMP calibration workload, each campaign's hardware_calibration.json).

    python -m benchmarks.readme_figures            # writes assets/figures/*.png and docs/figures/*.tex
"""
import glob
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'results/readme_figures_data.json'
FIG = ROOT / 'assets/figures'; TEX = ROOT / 'docs/figures'
V10B = ROOT / 'results/paper_campaigns/2026-11-25_final_v10b'
V10 = ROOT / 'results/paper_campaigns/2026-11-25_final_v10'
V2 = ROOT / 'results/paper_campaigns/2026-11-28_final_v2'
V2B = ROOT / 'results/paper_campaigns/2026-11-28_final_v2b'
V2C = ROOT / 'results/paper_campaigns/2026-11-28_final_v2c'
V2D = ROOT / 'results/paper_campaigns/2026-11-28_final_v2d'
V2_DIAG = ROOT / 'results/diagnostics'
plt.rcParams.update({'font.family': 'STIXGeneral', 'mathtext.fontset': 'stix',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 10})
C_PAPER, C_OURS, C_OURS4, C_BASE, C_BASE2 = '#7f7f7f', '#e8771b', '#b4541a', '#2c6fbb', '#58a55c'
SCENES = ['bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'box', 'cage', 'table_pick', 'table_under_pick']
VARIANTS = ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']
NAMES = dict(rrt_connect='RRTConnect', sst_bvp='BVP-SST*', sst_simd='SIMD-SST*', rrt_dp='DP-RRT')
CASES = ['unicycle_bugtrap', 'unicycle_wall', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']


# ------------------------------------------------------------------------------------------------------------ data
def collect():
    k = json.loads((V10B / 'hardware_calibration.json').read_text())
    k_hw = k.get('k_hw') or k.get('ratio') or 0.499
    main = json.loads((V10B / 'report_main/claims_v10.json').read_text())
    o4 = json.loads((V10B / 'report_order4/claims_v10.json').read_text())
    t2 = {r['environment']: r for r in main['table_ii']}
    raw = [json.loads(l) for l in open(V10 / 'table_ii_v10_rest/attempts.jsonl')]
    vtd = {}
    for s in SCENES:                      # baseline time over every returned path; validity reported separately
        rows = [r for r in raw if r['method'] == 'vamp_toppra' and r['environment'] == s]
        sol = [r['total_ns'] / 1e6 for r in rows if r['solved']]
        vtd[s] = dict(med=float(np.median(sol)), valid=sum(r['validation']['valid'] for r in rows if r['solved']) / len(rows))
    panda = {s: dict(paper_med=t2[s]['paper_median_ms'], paper_mean=t2[s]['paper_mean_ms'], ours_med=t2[s]['time_ms']['p50'],
                     ours_mean=t2[s]['time_ms']['mean'], valid=t2[s]['validated'] / t2[s]['attempts'],
                     toppra_med=vtd[s]['med'], toppra_valid=vtd[s]['valid'])
             for s in SCENES}
    t1 = {(r['case'], r['variant']): r for r in main['table_i']}; t4 = {(r['case'], r['variant']): r for r in o4['table_i']}

    def row(r):
        return None if r is None else dict(mean=r['time_ms']['mean'], paper=r['paper_ms'], valid=r['original_model_valid'],
                                           n=r['attempts'], length=r['length_m']['mean'], paper_length=r['paper_length_m'])
    dyno = {c: dict(rrtc2=row(t1.get((c, 'rrt_connect'))), sst2=row(t1.get((c, 'sst_bvp'))),
                    rrtc4=row(t4.get((c, 'rrt_connect'))), sst4=row(t4.get((c, 'sst_bvp')))) for c in CASES}
    for name, folder in [('sst', '2026-11-25_upstream_sst_v10'), ('idbastar', '2026-11-25_idbastar_v10')]:
        rows = [json.loads(l) for l in open(ROOT / 'results/paper_campaigns' / folder / 'attempts.jsonl')]
        for c in CASES:
            ok = [r for r in rows if r['case'] == c and r.get('valid')]
            dyno[c][name] = dict(solved=len(ok), n=sum(r['case'] == c for r in rows),
                                 first_ms=float(np.median([r['first_ms'] for r in ok])) if ok else None)
    return dict(k_hw=k_hw, panda=panda, dyno=dyno, paths=collect_paths(), ur5=collect_ur5())


def collect_paths():
    """One representative run per method on the unicycle cases (FLASK: v10b seed closest to median time)."""
    import yaml
    from flask_repro.flat_bvp import FlatEdge
    out = {}
    for case in ['unicycle_bugtrap', 'unicycle_wall']:
        out[case] = {}
        for alg in ['rrt_connect', 'sst_bvp']:
            runs = [json.load(open(f)) for f in glob.glob(str(V10B / f'table_i_v10b/{case}_{alg}_seed_*.json'))]
            runs = [r for r in runs if r['solved']]
            t = np.array([r['plan_ns'] + r['postprocess_ns'] for r in runs]); r = runs[int(np.argmin(abs(t - np.median(t))))]
            pts = np.vstack([FlatEdge(np.asarray(e['coefficients']), e['duration'], e['order']).derivative(
                np.linspace(0, e['duration'], 30), 0) for e in r['trajectory']])
            out[case][alg] = dict(xy=pts[:, :2].round(4).tolist(), ms=(r['plan_ns'] + r['postprocess_ns']) / 1e6, length=r['length_m'])
        for name, folder in [('sst', '2026-11-25_upstream_sst_v10'), ('idbastar', '2026-11-25_idbastar_v10')]:
            rows = [json.loads(l) for l in open(ROOT / 'results/paper_campaigns' / folder / 'attempts.jsonl')]
            ok = [r for r in rows if r['case'] == case and r.get('valid')]
            if not ok:
                continue
            t = np.array([r['first_ms'] for r in ok]); r = ok[int(np.argmin(abs(t - np.median(t))))]
            y = yaml.safe_load(open(ROOT / 'results/paper_campaigns' / folder / f"{case}_seed_{r['seed']}.yaml"))
            traj = (y.get('trajs_opt') or y.get('trajs_raw'))[0]
            out[case][name] = dict(xy=np.asarray(traj['states'])[:, :2].round(4).tolist(), ms=r['first_ms'], length=r['first_length'])
        from flask_repro.dynobench import load
        _, _, world, start, goal, _ = load(case)
        out[case]['world'] = dict(lower=world.lower.tolist(), upper=world.upper.tolist(),
                                  boxes=[[list(c), list(s)] for c, s in world.boxes], start=start.tolist(), goal=goal.tolist())
    return out


def collect_ur5():
    d = ROOT / 'results/ur5_sim_v10/e1_mesh_ff'
    s = json.loads((d / 'summary.json').read_text())
    f, g = np.load(d / 'trial_00_flask.npz'), np.load(d / 'trial_00_geometric.npz')
    def pack(z):
        k = slice(None, None, 10)
        return dict(t=z['t'][k].round(3).tolist(), q=z['q'][k, 2].round(4).tolist(), qd=z['qd'][k, 2].round(4).tolist(),
                    err=np.linalg.norm(z['q'] - z['qd'], axis=1)[k].round(5).tolist())
    return dict(summary=s, flask=pack(f), geometric=pack(g))


def _row(r, k):
    """One claims row; `scaled` = mean / k_hw of the campaign that measured it (paper-CPU equivalent)."""
    return None if r is None else dict(mean=r['time_ms']['mean'], scaled=r['time_ms']['mean'] / k, k=k, paper=r['paper_ms'], valid=r['original_model_valid'],
                                       n=r['attempts'], length=r['length_m']['mean'], paper_length=r['paper_length_m'],
                                       ratio=r.get('ratio_mean'), ci=r.get('ratio_mean_ci'), status=r['status'])


def _median_run(folder, case, alg):
    from flask_repro.flat_bvp import FlatEdge
    runs = [json.load(open(f)) for f in glob.glob(str(folder / f'{case}_{alg}_seed_*.json'))]
    runs = [r for r in runs if r['solved']]
    t = np.array([r['plan_ns'] + r['postprocess_ns'] for r in runs]); r = runs[int(np.argmin(abs(t - np.median(t))))]
    pts = np.vstack([FlatEdge(np.asarray(e['coefficients']), e['duration'], e['order']).derivative(
        np.linspace(0, e['duration'], 30), 0) for e in r['trajectory']])
    return dict(xy=pts[:, :2].round(4).tolist(), ms=(r['plan_ns'] + r['postprocess_ns']) / 1e6, length=r['length_m'], seed=r['seed'])


def add_v2(d):
    """flask_v2: Table I from the 2026-11-28 held-out campaign (v2 profile + v1 literal on the same seeds)."""
    k = json.loads((V2 / 'hardware_calibration.json').read_text())['k_hw']
    rep = lambda name: {(r['case'], r['variant']): r for r in json.loads((V2 / name / 'claims_v10.json').read_text())['table_i']}
    main, o4, v1 = rep('report_main'), rep('report_order4'), rep('report_v1_literal')
    d['k_hw_v2'] = k
    for c in CASES:
        d['dyno'][c].update(rrtc2=_row(main.get((c, 'rrt_connect')), k), sst2=_row(main.get((c, 'sst_bvp')), k),
                            rrtc4=_row(o4.get((c, 'rrt_connect')), k), sst4=_row(o4.get((c, 'sst_bvp')), k))
    sel = json.loads((V2_DIAG / 'v2_factorial/selection.json').read_text())
    d['ablation'] = dict(frozen=sel['frozen'],
                         times={key: {cfg: s['mean_ms'] for cfg, s in v['stats'].items()} for key, v in sel['per_case'].items()
                                if key.startswith('unicycle')})
    edge = json.loads((V2_DIAG / 'v2_mechanism/edge_study.json').read_text())['results']
    d['edge'] = {key: dict(extend=v['extend_valid'], connect=v['connect_valid']) for key, v in edge.items()}
    if (V2 / 'table_i_v2').is_dir():                      # raw per-seed files: local only
        for case in CASES[:2]:
            for alg in ['rrt_connect', 'sst_bvp']:
                d['paths'][case][alg] = _median_run(V2 / 'table_i_v2', case, alg)
            d['paths'][case]['rrt_connect_v1'] = _median_run(V2 / 'table_i_v1_literal', case, 'rrt_connect')
    if (V2D / 'report_quad_order4/claims_v10.json').exists():
        return add_v2c(d, V2D, uni_seeds='8101-8130', quad_seeds='8101-8130')
    return add_v2c(d) if (V2C / 'report_unicycle/claims_v10.json').exists() else add_v2b(d)


def add_v2c(d, folder=V2C, uni_seeds='8001-8030', quad_seeds='7001-7030'):
    """flask_v2 release campaign 2026-11-28_final_v2c: every Table I row from one controlled campaign, release binary
    (exact workspace bounds). Unicycle seeds 8001-8030 (v2 and v1 literal), quadrotors 7001-7030 (r = 2 and r = 4).
    The same layout serves final_v2d (v2 with lever L7, every row on seeds 8101-8130)."""
    k = json.loads((folder / 'hardware_calibration.json').read_text())['k_hw']
    rep = lambda name: {(r['case'], r['variant']): r for r in json.loads((folder / name / 'claims_v10.json').read_text())['table_i']}
    uni, v1, q2, q4 = rep('report_unicycle'), rep('report_unicycle_v1_literal'), rep('report_quad_order2'), rep('report_quad_order4')
    d['k_hw_v2'] = d['k_hw_v2b'] = k
    d['uni_seeds'], d['quad_seeds'] = uni_seeds, quad_seeds
    for c in CASES:
        main = uni if c.startswith('unicycle') else q2
        d['dyno'][c].update(rrtc2=_row(main[(c, 'rrt_connect')], k), sst2=_row(main[(c, 'sst_bvp')], k),
                            rrtc4=_row(q4.get((c, 'rrt_connect')), k), sst4=_row(q4.get((c, 'sst_bvp')), k))
    d['unicycle'] = {c: {v: dict(v1=_row(v1[(c, v)], k), v2=_row(uni[(c, v)], k)) for v in VARIANTS} for c in CASES[:2]}
    if (folder / 'table_i_unicycle').is_dir():              # raw per-seed files: local only
        for case in CASES[:2]:
            for alg in ['rrt_connect', 'sst_bvp']:
                d['paths'][case][alg] = _median_run(folder / 'table_i_unicycle', case, alg)
            d['paths'][case]['rrt_connect_v1'] = _median_run(folder / 'table_i_unicycle_v1_literal', case, 'rrt_connect')
    d['campaign'] = folder.name
    return d


def add_v2b(d):
    """flask_v2b: unicycle rows from the 2026-11-28 v2b campaign (seeds 8001-8030; Wall on the forward chart, Fig. 5a)."""
    k = json.loads((V2B / 'hardware_calibration.json').read_text())['k_hw']
    rep = lambda name: {(r['case'], r['variant']): r for r in json.loads((V2B / name / 'claims_v10.json').read_text())['table_i']}
    main, v1 = rep('report_main'), rep('report_v1_literal')
    d['k_hw_v2b'] = k
    for c in CASES[:2]:
        d['dyno'][c].update(rrtc2=_row(main[(c, 'rrt_connect')], k), sst2=_row(main[(c, 'sst_bvp')], k))
    d['unicycle'] = {c: {v: dict(v1=_row(v1[(c, v)], k), v2=_row(main[(c, v)], k)) for v in VARIANTS} for c in CASES[:2]}
    if (V2B / 'table_i_v2b').is_dir():
        for case in CASES[:2]:
            for alg in ['rrt_connect', 'sst_bvp']:
                d['paths'][case][alg] = _median_run(V2B / 'table_i_v2b', case, alg)
            d['paths'][case]['rrt_connect_v1'] = _median_run(V2B / 'table_i_v1_literal_v2b', case, 'rrt_connect')
    return d


# ------------------------------------------------------------------------------------------------------------ tables
def booktabs_png(path, header, rows, align, caption, col_w):
    """A booktabs-style table rendered with Computer Modern (matches the LaTeX source written next to it)."""
    n = len(rows) + 1; h = 0.30 * n + 0.75; w = sum(col_w) + 0.3
    fig = plt.figure(figsize=(w, h)); ax = fig.add_axes([0, 0, 1, 1]); ax.axis('off')
    ax.set_xlim(0, w); ax.set_ylim(0, h)
    x = np.r_[0.15, 0.15 + np.cumsum(col_w)]
    def line(y, lw):
        ax.plot([0.12, w - 0.12], [y, y], color='black', lw=lw)
    top = h - 0.18; line(top, 1.2); y = top - 0.24
    for j, t in enumerate(header):
        ax.text(x[j + 1] - 0.08 if align[j] == 'r' else x[j] + 0.05, y, t, ha='right' if align[j] == 'r' else 'left',
                va='center', fontsize=9.5, fontweight='bold')
    line(y - 0.16, 0.7); y -= 0.38
    for i, r in enumerate(rows):
        if r == 'MID':
            line(y + 0.13, 0.5); continue
        for j, t in enumerate(r):
            ax.text(x[j + 1] - 0.08 if align[j] == 'r' else x[j] + 0.05, y, t, ha='right' if align[j] == 'r' else 'left',
                    va='center', fontsize=9.5)
        y -= 0.30
    line(y + 0.13, 1.2)
    ax.text(0.15, y - 0.12, caption, ha='left', va='top', fontsize=8.5, wrap=True, color='#333333')
    fig.savefig(path, dpi=220, bbox_inches='tight', pad_inches=0.05, facecolor='white'); plt.close(fig)


def booktabs_tex(path, header, rows, align, caption):
    esc = lambda s: s.replace('%', r'\%').replace('_', r'\_').replace('µs', r'$\mu$s').replace('±', r'$\pm$').replace('×', r'$\times$').replace('—', '--')
    body = [r'\documentclass[border=4pt]{standalone}', r'\usepackage{booktabs}', r'\begin{document}',
            r'\begin{tabular}{' + ''.join(align) + '}', r'\toprule', ' & '.join(r'\textbf{' + esc(h) + '}' for h in header) + r' \\', r'\midrule']
    body += [r'\midrule' if r == 'MID' else ' & '.join(esc(c) for c in r) + r' \\' for r in rows]
    body += [r'\bottomrule', r'\end{tabular}', r'% ' + caption, r'\end{document}']
    path.write_text('\n'.join(body) + '\n')


def fmt(ms):
    return '—' if ms is None else f'{ms * 1000:.0f} µs' if ms < 1 else f'{ms:.2f}' if ms < 10 else f'{ms:.1f}' if ms < 1000 else f'{ms / 1000:.1f} s'


def fmt_unit(ms):
    return fmt(ms) if ms < 1 or ms >= 1000 else fmt(ms) + ' ms'


def tables(d):
    k = d['k_hw']; P = d['panda']
    header = ['Scene', 'Paper median (mean)', 'Ours median (mean)', 'Ours ÷ k_hw, median', 'Ours valid',
              'VAMP+TOPP-RA median', 'TOPP-RA valid']
    rows = [[s.replace('_', ' '), f'{fmt(p["paper_med"])} ({fmt(p["paper_mean"])})', f'{fmt(p["ours_med"])} ({fmt(p["ours_mean"])})',
             fmt(p['ours_med'] / k), f'{100 * p["valid"]:.1f}%', fmt(p['toppra_med']), f'{100 * p["toppra_valid"]:.0f}%'] for s, p in P.items()]
    cap = (f'Panda, MotionBenchMaker, 1000 problems per scene, times in ms on each CPU (VAMP+TOPP-RA on ours). Ours: held-out seeds, one P-core of an i9-14900 '
           f'(k_hw = {k:.3f}: our CPU is about 2× faster than the paper\'s i7-6900K). Valid = dense (1 kHz) + URDF collision and '
           'limit checks on the original model. TOPP-RA retimes a geometric path; its spline leaves the checked path.')
    booktabs_png(FIG / 'table_panda.png', header, rows, 'lrrrrrr', cap, [1.45, 1.75, 1.75, 1.55, 1.0, 1.75, 1.15])
    booktabs_tex(TEX / 'table_panda.tex', header, rows, 'lrrrrrr', cap)
    D = d['dyno']
    header = ['Case', 'Paper RRTC', 'Ours RRTC r=2', 'Ours RRTC r=4', 'Paper / ours BVP-SST*', 'SST (Dynoplan)', 'iDb-A*']
    rows = []
    for c in CASES:
        x = D[c]; r2, r4, s2 = x['rrtc2'], x['rrtc4'], x['sst2']
        uni = c.startswith('unicycle')
        r2s = f'{fmt(r2["scaled"])}' + (' ‡' if uni else ' †')
        r4s = '—' if r4 is None else f'{fmt(r4["scaled"])} ({r4["valid"]}/{r4["n"]} ✓)'
        rows.append([c.replace('_', ' '), fmt(r2['paper']), r2s, r4s, f'{fmt(s2["paper"])} / {fmt(s2["scaled"])}',
                     f'{x["sst"]["solved"]}/{x["sst"]["n"]}, {fmt(x["sst"]["first_ms"])}',
                     f'{x["idbastar"]["solved"]}/{x["idbastar"]["n"]}, {fmt(x["idbastar"]["first_ms"])}'])
    cap = (f'DynoBench (paper Table I), mean planning time in ms scaled to the paper\'s CPU (ours / k_hw of each campaign), 30 held-out seeds: ' +
           (f'quadrotors {d.get("quad_seeds", "7001-7030")}, unicycle {d.get("uni_seeds", "8001-8030")}, one campaign (k_hw = {d["k_hw_v2"]:.3f}). ' if d['k_hw_v2'] == d['k_hw_v2b'] else
            f'quadrotors 7001-7030 (k_hw = {d["k_hw_v2"]:.3f}), unicycle 8001-8030 (k_hw = {d["k_hw_v2b"]:.3f}). ') +
           '‡ unicycle: flask_v2 profile (see the unicycle table). † r = 2 is the paper\'s reduced-order practice for quadrotors; those paths are not feasible for the full model. '
           'r = 4: full model, every returned path interval-certified (✓ = valid/attempts). Unicycle: curved edges with '
           '|ω| ≤ 1.5 rad/s enforced along every edge. SST / iDb-A*: solved within 180 s, median first-solution time (baseline settings differ from the paper\'s).')
    booktabs_png(FIG / 'table_dynobench.png', header, rows, 'lrrrrrr', cap, [1.45, 1.1, 1.4, 1.75, 1.85, 1.6, 1.4])
    booktabs_tex(TEX / 'table_dynobench.tex', header, rows, 'lrrrrrr', cap)
    if 'unicycle' in d:
        table_unicycle(d)


def _verdict(row):
    s = row['status'].split(':')[-1]
    if s == 'REPRODUCED':                                   # rule: flag a row whose 95% CI leaves the band
        ci = row.get('ci')
        return 'in band, CI crosses edge' if ci and (ci[0] < 1 / 3 or ci[1] > 3) else 'in band'
    if s == 'DIFFERENT_LENGTH':
        x = 100 * (row['length'] / row['paper_length'] - 1)
        return f'length {x:+.0f}%'.replace('-', '−') + (' (shorter)' if x < 0 else '')
    out = {'DIFFERENT_FASTER': 'faster', 'DIFFERENT_SLOWER': 'slower', 'DIFFERENT_INVALID': 'invalid', 'UNSOLVED_IN_BUDGET': 'unsolved in budget'}.get(s, s.lower())
    ci = row.get('ci')                                      # same rule, other side: the CI reaches into the band
    if s in ('DIFFERENT_FASTER', 'DIFFERENT_SLOWER') and ci and ci[1] >= 1 / 3 and ci[0] <= 3:
        out += ', CI crosses edge'
    return out


def table_unicycle(d):
    k = d['k_hw_v2b']
    header = ['Case', 'Planner', 'Paper', 'v1 literal', 'v2', 'v2 ratio', 'Length paper / v1 / v2', 'v1', 'v2']
    rows = []
    for c in CASES[:2]:
        if rows:
            rows.append('MID')
        for v in VARIANTS:
            a, b = d['unicycle'][c][v]['v1'], d['unicycle'][c][v]['v2']
            rows.append([c.replace('unicycle_', ''), NAMES[v], fmt(b['paper']), fmt(a['scaled']), fmt(b['scaled']),
                         f'{b["ratio"]:.2f}', f'{b["paper_length"]:.1f} / {a["length"]:.1f} / {b["length"]:.1f}',
                         _verdict(a), _verdict(b)])
    cap = (f'Unicycle, mean planning time in ms scaled to the paper\'s CPU (k_hw = {k:.3f}), the same 30 held-out seeds ({d.get("uni_seeds", "8001-8030")}) for v1 and v2, '
           'Bugtrap on the backward and Wall on the forward chart (paper Fig. 5a), '
           'all paths valid on the original model (|v| ≤ 1 m/s, |ω| ≤ 1.5 rad/s checked densely). ' +
           ('v2 = the v2 profile with lever L7 (Alg. 4 repeated over subdivided edges, results/diagnostics/v2_shortcut). ' if d.get('campaign', '').endswith('v2d') else '') +
           'Ratio = ours / (k_hw · paper); '
           '"in band" = ratio in [1/3, 3] and length within 20 % (config/claim_criteria_v10.json); "CI crosses edge" = the 95 % '
           'bootstrap interval of the ratio extends outside [1/3, 3] (or, for "faster"/"slower", into it), so another seed set may change the verdict; "faster" = ratio < 1/3.')
    booktabs_png(FIG / 'table_unicycle.png', header, rows, 'llrrrrrll', cap, [0.85, 1.05, 0.75, 0.95, 0.75, 0.8, 1.7, 1.65, 1.85])
    booktabs_tex(TEX / 'table_unicycle.tex', header, rows, 'llrrrrrll', cap)


# ------------------------------------------------------------------------------------------------------------ plots
def bars(d):
    k = d['k_hw']; fig, axes = plt.subplots(1, 2, figsize=(11, 3.4), gridspec_kw=dict(width_ratios=[7, 6]))
    ax = axes[0]; P = d['panda']; x = np.arange(len(P)); w = 0.27
    ax.bar(x - w, [p['paper_med'] for p in P.values()], w, color=C_PAPER, label='FLASK (paper)')
    ax.bar(x, [p['ours_med'] / k for p in P.values()], w, color=C_OURS, label='FLASK (ours)')
    ax.bar(x + w, [p['toppra_med'] / k for p in P.values()], w, color=C_BASE, label='VAMP RRTC + TOPP-RA (ours)')
    ax.set_yscale('log'); ax.set_xticks(x, [s.replace('_', '\n') for s in P], fontsize=8); ax.set_ylabel('median planning time [ms]')
    ax.set_title('(a) Panda, MotionBenchMaker (paper Table II)', fontsize=10); ax.legend(fontsize=8, frameon=False)
    ax = axes[1]; D = d['dyno']; x = np.arange(len(CASES)); w = 0.27
    ax.bar(x - w, [D[c]['rrtc2']['paper'] for c in CASES], w, color=C_PAPER, label='FLASK RRTC (paper)')
    ax.bar(x, [D[c]['rrtc2']['scaled'] for c in CASES], w, color=C_OURS, label='ours, r = 2 (unicycle: exact model, v2)')
    ax.bar(x + w, [(D[c]['rrtc4'] or {}).get('scaled', np.nan) for c in CASES], w, color=C_OURS4, label='ours, r = 4 (full model)')
    ax.set_yscale('log'); ax.set_xticks(x, [c.replace('_', '\n') for c in CASES], fontsize=8); ax.set_ylabel('mean planning time [ms]')
    ax.set_title('(b) DynoBench, FLASK-RRTConnect (paper Table I)', fontsize=10); ax.legend(fontsize=8, frameon=False)
    for a in axes:
        a.grid(axis='y', which='major', alpha=0.25); a.spines[['top', 'right']].set_visible(False)
        a.set_ylim(top=a.get_ylim()[1] * 12)            # head-room for the legend
    fig.text(0.5, -0.02, f'Ours scaled to the paper\'s CPU (divided by the k_hw of each campaign: {k:.3f} Panda, {d["k_hw_v2"]:.3f} DynoBench). Log scale.', ha='center', fontsize=8.5, style='italic')
    fig.tight_layout(); fig.savefig(FIG / 'planning_times.png', dpi=200, bbox_inches='tight', facecolor='white'); plt.close(fig)


def paths(d):
    fig, axes = plt.subplots(1, 2, figsize=(11, 6.0))
    style = dict(rrt_connect_v1=('#9a9a9a', ':', 1.4, 'FLASK RRTConnect, v1 literal'),
                 rrt_connect=(C_OURS, '-', 2.2, 'FLASK RRTConnect, v2'), sst_bvp=(C_OURS4, '--', 1.8, 'FLASK BVP-SST*, v2'),
                 sst=(C_BASE, '-', 1.4, 'SST (Dynoplan)'), idbastar=(C_BASE2, '-', 1.4, 'iDb-A*'))
    for ax, case in zip(axes, ['unicycle_bugtrap', 'unicycle_wall']):
        P = d['paths'][case]; W = P['world']
        for c, s in W['boxes']:
            ax.add_patch(plt.Rectangle((c[0] - s[0] / 2, c[1] - s[1] / 2), s[0], s[1], color='#3a3f47'))
        for name, (col, ls, lw, label) in style.items():
            if name in P:
                xy = np.asarray(P[name]['xy'])
                ax.plot(xy[:, 0], xy[:, 1], ls, color=col, lw=lw, label=f'{label}: {fmt_unit(P[name]["ms"])}, {P[name]["length"]:.1f} m')
        ax.plot(*W['start'][:2], 'o', color='black', ms=6); ax.plot(*W['goal'][:2], '*', color='black', ms=11)
        ax.set_xlim(W['lower'][0], W['upper'][0]); ax.set_ylim(W['lower'][1], W['upper'][1]); ax.set_aspect('equal')
        ax.set_title(case.replace('_', ' ').title(), fontsize=10)
        ax.legend(fontsize=7.5, loc='upper center', bbox_to_anchor=(0.5, -0.02), frameon=False, ncol=1)
        ax.set_xticks([]); ax.set_yticks([])
    fig.text(0.5, 0.01, 'Representative runs (median planning time, held-out seeds). Unicycle |v| ≤ 1 m/s, |ω| ≤ 1.5 rad/s; times on our CPU '
             '(first solution for SST / iDb-A*).', ha='center', fontsize=8.5, color='#333333')
    fig.tight_layout(rect=(0, 0.03, 1, 1)); fig.savefig(FIG / 'trajectories_unicycle.png', dpi=200, bbox_inches='tight', facecolor='white'); plt.close(fig)


def ur5(d):
    U = d['ur5']; S = U['summary']
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.0))
    for ax, name, title in [(axes[0], 'flask', '(a) tracking a FLASK trajectory'), (axes[1], 'geometric', '(b) tracking a time-scaled geometric path')]:
        z = U[name]; ax.plot(z['t'], z['qd'], '-', color=C_BASE, lw=1.4, label='desired $q_3$'); ax.plot(z['t'], z['q'], '--', color=C_OURS, lw=1.3, label='actual $q_3$')
        ax.set_title(title, fontsize=10); ax.set_xlabel('time [s]'); ax.set_ylabel('joint angle [rad]'); ax.legend(fontsize=8, frameon=False)
    ax = axes[2]
    ax.plot(U['flask']['t'], U['flask']['err'], '-', color=C_OURS, lw=1.3, label='FLASK')
    ax.plot(U['geometric']['t'], U['geometric']['err'], '-', color=C_BASE, lw=1.1, label='geometric + time scaling')
    ax.set_yscale('log'); ax.set_title('(c) tracking error $\\|q - q_d\\|$', fontsize=10); ax.set_xlabel('time [s]'); ax.legend(fontsize=8, frameon=False)
    for a in axes:
        a.spines[['top', 'right']].set_visible(False)
    f, g = S['flask'], S['geometric']
    fig.text(0.5, -0.04, f'UR5 in MuJoCo, 20 trials (paper Fig. 7 analogue): max tracking error median {f["max_error_median"]:.4f} vs '
             f'{g["max_error_median"]:.4f} rad; trials with contact {f["collision_trials"]}/20 vs {g["collision_trials"]}/20.',
             ha='center', fontsize=8.5, style='italic')
    fig.tight_layout(); fig.savefig(FIG / 'ur5_tracking.png', dpi=200, bbox_inches='tight', facecolor='white'); plt.close(fig)


def ablation(d):
    """(a) free-space edge validity per lever (Phase 1), (b) development-seed mean time, v1 vs frozen v2 setting."""
    E = d['edge']; fig, axes = plt.subplots(1, 2, figsize=(11, 3.4), gridspec_kw=dict(width_ratios=[5, 7]))
    ax = axes[0]
    bars_ = [('v1', 'speed_min=0.01 k=1 mult=none'), ('L1, k=5', 'speed_min=0.01 k=5 mult=none'),
             ('L1, k=20', 'speed_min=0.01 k=20 mult=none'), ('L2 (dropped)', 'speed_min=0.4 k=1 mult=none'),
             ('L3', 'speed_min=0.01 k=1 mult=1.5/2.0/3.0'), ('L1 k=5 + L3', 'speed_min=0.01 k=5 mult=1.5/2.0/3.0'),
             ('L1 k=20 + L3', 'speed_min=0.01 k=20 mult=1.5/2.0/3.0')]
    vals = [100 * E[key]['extend'] for _, key in bars_]
    cols = [C_PAPER] + [C_OURS if 'dropped' not in n else '#cfcfcf' for n, _ in bars_[1:]]
    ax.barh(range(len(bars_)), vals, color=cols)
    for i, v in enumerate(vals):
        ax.text(v + 1, i, f'{v:.1f}%', va='center', fontsize=8)
    ax.set_yticks(range(len(bars_)), [n for n, _ in bars_], fontsize=8.5); ax.invert_yaxis(); ax.set_xlim(0, 70)
    ax.set_xlabel('EXTEND edges valid (|ω| ≤ 1.5, |v| ≤ 1) [%]')
    ax.set_title('(a) unicycle RRTConnect edges, free space', fontsize=10)
    ax = axes[1]; A = d['ablation']; keys = [f'{c}/{v}' for v in VARIANTS for c in CASES[:2]]; x = np.arange(len(keys)); w = 0.38
    t_v1 = [A['times'][key]['v1'] for key in keys]
    sel = [A['frozen']['unicycle/' + key.split('/')[1]] for key in keys]
    t_v2 = [A['times'][key][s] for key, s in zip(keys, sel)]
    ax.bar(x - w / 2, t_v1, w, color=C_PAPER, label='v1 literal'); ax.bar(x + w / 2, t_v2, w, color=C_OURS, label='v2 (frozen setting)')
    for i, s in enumerate(sel):
        ax.text(x[i] + w / 2, t_v2[i] * 1.25, s.replace('+', '+\n'), ha='center', va='bottom', fontsize=6.5, color='#555555')
    ax.set_yscale('log'); ax.set_ylim(0.05, 8000); ax.set_ylabel('mean planning time [ms], our CPU')
    ax.set_xticks(x, [NAMES[key.split('/')[1]] + '\n' + key.split('/')[0].replace('unicycle_', '') for key in keys], fontsize=7.5)
    ax.set_title('(b) development seeds 1001-1010: v1 vs selected levers', fontsize=10); ax.legend(fontsize=8, frameon=False, loc='upper left')
    for a in axes:
        a.spines[['top', 'right']].set_visible(False)
    ax.grid(axis='y', alpha=0.25)
    fig.text(0.5, -0.03, 'L1 cost-ranked parent among k nearest, L2 speed lower bound, L3 fixed-time fallback at 1.5/2/3 T*, '
             'L5 pseudo-control bound 1.0, L6 propagation durations 0.2-2 s. Selection rule: config/v2_selection_rule.json.',
             ha='center', fontsize=8, style='italic')
    fig.tight_layout(); fig.savefig(FIG / 'v2_ablation.png', dpi=200, bbox_inches='tight', facecolor='white'); plt.close(fig)


def main():
    FIG.mkdir(parents=True, exist_ok=True); TEX.mkdir(parents=True, exist_ok=True)
    try:
        d = collect()
    except FileNotFoundError:                              # raw v10 data is not distributed: start from the tracked summary
        d = json.loads(DATA.read_text())
    if (V2 / 'report_main/claims_v10.json').exists():
        d = add_v2(d)
    DATA.write_text(json.dumps(d) + '\n')
    tables(d); bars(d); paths(d); ur5(d)
    if 'ablation' in d:
        ablation(d)
    print('figures ->', FIG, ' tables ->', TEX)


if __name__ == '__main__':
    main()
