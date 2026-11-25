"""Figures and LaTeX tables for the README, from the controlled campaigns.

Sources (local raw data, summarized once into results/readme_figures_data.json, which is tracked so the figures can
be rebuilt from a clone):
  * FLASK, Tables I/II: campaign v10b (held-out seeds 6001+, controlled, CPU 10) reports
  * VAMP RRTConnect + TOPP-RA: campaign v10, same machine; Dynoplan SST / iDb-A*: v10 baselines (seeds 5001-5030)
  * UR5 tracking: results/ur5_sim_v10/e1_mesh_ff (MuJoCo)
Times "scaled to the paper's CPU" divide by k_hw = 0.499 (VAMP calibration workload, hardware_calibration.json).

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
plt.rcParams.update({'font.family': 'STIXGeneral', 'mathtext.fontset': 'stix',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 10})
C_PAPER, C_OURS, C_OURS4, C_BASE, C_BASE2 = '#7f7f7f', '#e8771b', '#b4541a', '#2c6fbb', '#58a55c'
SCENES = ['bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'box', 'cage', 'table_pick', 'table_under_pick']
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
        r2s = f'{fmt(r2["mean"] / k)}' + ('' if uni else ' †')
        r4s = '—' if r4 is None else f'{fmt(r4["mean"] / k)} ({r4["valid"]}/{r4["n"]} ✓)'
        rows.append([c.replace('_', ' '), fmt(r2['paper']), r2s, r4s, f'{fmt(s2["paper"])} / {fmt(s2["mean"] / k)}',
                     f'{x["sst"]["solved"]}/{x["sst"]["n"]}, {fmt(x["sst"]["first_ms"])}',
                     f'{x["idbastar"]["solved"]}/{x["idbastar"]["n"]}, {fmt(x["idbastar"]["first_ms"])}'])
    cap = ('DynoBench (paper Table I), mean planning time in ms scaled to the paper\'s CPU (ours / k_hw), 30 held-out seeds. '
           '† r = 2 is the paper\'s reduced-order practice for quadrotors; those paths are not feasible for the full model. '
           'r = 4: full model, every returned path interval-certified (✓ = valid/attempts). Unicycle: curved edges with '
           '|ω| ≤ 1.5 rad/s enforced along every edge. SST / iDb-A*: solved within 180 s, median first-solution time (baseline settings differ from the paper\'s).')
    booktabs_png(FIG / 'table_dynobench.png', header, rows, 'lrrrrrr', cap, [1.45, 1.1, 1.4, 1.75, 1.85, 1.6, 1.4])
    booktabs_tex(TEX / 'table_dynobench.tex', header, rows, 'lrrrrrr', cap)


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
    ax.bar(x, [D[c]['rrtc2']['mean'] / k for c in CASES], w, color=C_OURS, label='ours, r = 2 (unicycle: exact model)')
    ax.bar(x + w, [(D[c]['rrtc4'] or {}).get('mean', np.nan) / k for c in CASES], w, color=C_OURS4, label='ours, r = 4 (full model)')
    ax.set_yscale('log'); ax.set_xticks(x, [c.replace('_', '\n') for c in CASES], fontsize=8); ax.set_ylabel('mean planning time [ms]')
    ax.set_title('(b) DynoBench, FLASK-RRTConnect (paper Table I)', fontsize=10); ax.legend(fontsize=8, frameon=False)
    for a in axes:
        a.grid(axis='y', which='major', alpha=0.25); a.spines[['top', 'right']].set_visible(False)
    fig.text(0.5, -0.02, 'Ours scaled to the paper\'s CPU (divided by k_hw = %.3f). Log scale.' % k, ha='center', fontsize=8.5, style='italic')
    fig.tight_layout(); fig.savefig(FIG / 'planning_times.png', dpi=200, bbox_inches='tight', facecolor='white'); plt.close(fig)


def paths(d):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    style = dict(rrt_connect=(C_OURS, '-', 2.2, 'FLASK RRTConnect'), sst_bvp=(C_OURS4, '--', 1.8, 'FLASK BVP-SST*'),
                 sst=(C_BASE, '-', 1.4, 'SST (Dynoplan)'), idbastar=(C_BASE2, '-', 1.4, 'iDb-A*'))
    for ax, case in zip(axes, ['unicycle_bugtrap', 'unicycle_wall']):
        P = d['paths'][case]; W = P['world']
        for c, s in W['boxes']:
            ax.add_patch(plt.Rectangle((c[0] - s[0] / 2, c[1] - s[1] / 2), s[0], s[1], color='#3a3f47'))
        for name, (col, ls, lw, label) in style.items():
            if name in P:
                xy = np.asarray(P[name]['xy'])
                ax.plot(xy[:, 0], xy[:, 1], ls, color=col, lw=lw, label=f'{label}: {fmt(P[name]["ms"])} ms, {P[name]["length"]:.1f} m')
        ax.plot(*W['start'][:2], 'o', color='black', ms=6); ax.plot(*W['goal'][:2], '*', color='black', ms=11)
        ax.set_xlim(W['lower'][0], W['upper'][0]); ax.set_ylim(W['lower'][1], W['upper'][1]); ax.set_aspect('equal')
        ax.set_title(case.replace('_', ' ').title(), fontsize=10); ax.legend(fontsize=7.5, loc='lower left', framealpha=0.9)
        ax.set_xticks([]); ax.set_yticks([])
    fig.text(0.5, 0.01, 'Representative runs (median planning time). Unicycle |v| ≤ 1 m/s, |ω| ≤ 1.5 rad/s; times on our CPU '
             '(first solution for SST / iDb-A*).', ha='center', fontsize=8.5, color='#333333')
    fig.tight_layout(rect=(0, 0.03, 1, 1)); fig.savefig(FIG / 'trajectories_unicycle.png', dpi=200, facecolor='white'); plt.close(fig)


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


def main():
    FIG.mkdir(parents=True, exist_ok=True); TEX.mkdir(parents=True, exist_ok=True)
    try:
        d = collect(); DATA.write_text(json.dumps(d) + '\n')
    except FileNotFoundError:                              # a clone: raw campaign data is not distributed
        d = json.loads(DATA.read_text())
    tables(d); bars(d); paths(d); ur5(d)
    print('figures ->', FIG, ' tables ->', TEX)


if __name__ == '__main__':
    main()
