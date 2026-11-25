"""V2 (acceptance checks; see "Verification" in README.md): agreement with the authors' released code (KavrakiLab/ompl-flask, branch flat).

  V2a  Steering: native LQMT vs the authors' MinimumEffortSteering on random Panda boundaries
       (delegates to `benchmarks.author_reference bvp`).
  V2b  Edge decisions: for random edges inside the seven MotionBenchMaker scenes, the authors'
       FlatVampMotionValidator (driver command `edges`, build/author_flask_edges) vs the native FLASKCC check
       (`flask_cc`, check_limits=False), with dense ground truth (4000 samples per edge, VAMP fkcc).
       Edges whose dense samples leave the position/velocity bounds are reported separately, because the authors'
       validator also tests those bounds while check_limits=False does not.

Usage: python -m verification.v2_author --output results/verification/<date>_acceptance/v2 [--edges 400 --problems 5]
"""
import argparse
import json
import subprocess
from pathlib import Path

import numpy as np

from flask_repro import mbm, native, robot as R
from flask_repro import lqmt
from benchmarks.author_reference import export_problems

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'build/author_flask_edges/mbm_flat'


def dense(env, mod, q0, v0, q1, v1, T, rb, samples=4000):
    t = np.linspace(0., T, samples)
    q, v = lqmt.evaluate(lqmt.hermite_coefficients(q0, v0, q1, v1, T), t)[:2]
    in_bounds = bool(np.all(q >= rb.q_lo - 1e-9) and np.all(q <= rb.q_hi + 1e-9) and np.all(np.abs(v) <= rb.v_max + 1e-9))
    collisions, _ = mod.count_collisions(env, [list(map(float, x)) for x in q])
    return collisions == 0, in_bounds


def v2b(out, edges_per_problem, problems, seed, profile='published_resolution'):
    rb = R.get_robot('panda'); mod = native.module(rb); rng = np.random.default_rng(seed)
    limits = native.joint_limits(rb, profile)
    dataset = mbm.load_dataset('panda'); rows = []
    for env_name in mbm.ENVIRONMENTS:
        for index in range(problems):
            p = dataset[env_name][index]
            if not p['valid']:
                continue
            prob = out / f'{env_name}_{index}.problem.txt'
            export_problems(prob, [env_name], index + 1)
            lines = prob.read_text().splitlines()
            # keep only the requested problem block (export writes problems 0..index)
            starts = [i for i, l in enumerate(lines) if l.split()[:2] == [env_name, str(index)]]
            block = lines[starts[0]:]
            env = mbm.native_environment(p)
            pairs = []
            while len(pairs) < edges_per_problem:
                q0 = rng.uniform(rb.q_lo, rb.q_hi)
                if not mod.validate(env, q0.tolist()):
                    continue
                d = rng.normal(size=7); d *= rng.uniform(.05, 1.) / np.linalg.norm(d)
                q1 = np.clip(q0 + d, rb.q_lo, rb.q_hi)
                if not mod.validate(env, q1.tolist()):
                    continue
                scale = [0., .3, 1.][len(pairs) % 3]
                v0, v1 = (rng.uniform(-rb.v_max, rb.v_max) * scale for _ in range(2))
                pairs.append((q0, v0, q1, v1))
            src = out / f'{env_name}_{index}.edges.txt'
            src.write_text('\n'.join(block) + f'\n{len(pairs)}\n' +
                           '\n'.join(' '.join(repr(float(x)) for x in np.r_[q0, v0, q1, v1]) for q0, v0, q1, v1 in pairs) + '\n')
            res = out / f'{env_name}_{index}.author.jsonl'
            subprocess.run([str(DRIVER), 'edges', str(src), str(res)], check=True)
            author = [json.loads(l) for l in res.read_text().splitlines()]
            assert len(author) == len(pairs)
            for (q0, v0, q1, v1), a in zip(pairs, author):
                sol = mod.lqmt(q0.tolist(), v0.tolist(), q1.tolist(), v1.tolist(), 1.)
                T = sol[0] if sol is not None else None
                ours = bool(mod.flask_cc(env, q0.tolist(), v0.tolist(), q1.tolist(), v1.tolist(), T, limits, False)) if T else None
                truth, in_bounds = dense(env, mod, q0, v0, q1, v1, T, rb) if T else (None, None)
                rows.append(dict(environment=env_name, problem=index, author_found=a['found'], author_T=a['T'],
                                 native_T=T, author_valid=a['valid'], native_valid=ours, dense_collision_free=truth,
                                 dense_in_bounds=in_bounds))
    (out / 'edges.jsonl').write_text('\n'.join(json.dumps(r) for r in rows) + '\n')
    both = [r for r in rows if r['author_found'] and r['native_T']]
    inb = [r for r in both if r['dense_in_bounds']]
    agree = sum(r['author_valid'] == r['native_valid'] for r in inb)
    dis = [r for r in inb if r['author_valid'] != r['native_valid']]
    summary = dict(edges=len(rows), steering_found_both=len(both),
                   max_rel_T=max(abs(r['author_T'] - r['native_T']) / r['author_T'] for r in both),
                   in_bounds_edges=len(inb), decision_agreement=agree, disagreements=len(dis),
                   disagreements_native_matches_dense=sum(r['native_valid'] == r['dense_collision_free'] for r in dis),
                   disagreements_author_matches_dense=sum(r['author_valid'] == r['dense_collision_free'] for r in dis),
                   native_accepts_dense_collision=sum(r['native_valid'] and not r['dense_collision_free'] for r in inb),
                   author_accepts_dense_collision=sum(r['author_valid'] and not r['dense_collision_free'] for r in inb),
                   native_rejects_dense_free=sum((not r['native_valid']) and r['dense_collision_free'] for r in inb),
                   author_rejects_dense_free=sum((not r['author_valid']) and r['dense_collision_free'] for r in inb),
                   out_of_bounds_edges=len(both) - len(inb),
                   out_of_bounds_author_rejects=sum(not r['author_valid'] for r in both if not r['dense_in_bounds']))
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--edges', type=int, default=400)
    p.add_argument('--problems', type=int, default=5)
    p.add_argument('--bvp-count', type=int, default=10000)
    p.add_argument('--seed', type=int, default=7101)
    p.add_argument('--profiles', nargs='+', default=['published_resolution', 'peak_speed'])
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    for pr in a.profiles: (a.output / pr).mkdir(exist_ok=True)
    subprocess.run([str(ROOT / '.venv/bin/python'), '-m', 'benchmarks.author_reference', 'bvp', '--output',
                    str(a.output / 'v2a_bvp.json'), '--count', str(a.bvp_count)], check=True, cwd=ROOT, stdout=subprocess.DEVNULL)
    bvp = json.loads((a.output / 'v2a_bvp.json').read_text())
    res = dict(V2a_bvp={k: bvp[k] for k in ['count', 'both_found', 'native_only', 'author_only', 'max_rel_T', 'max_rel_cost',
                                             'native_cost_lower_or_equal', 'max_mid_error']},
               **{f'V2b_edges_{pr}': v2b(a.output / pr, a.edges, a.problems, a.seed, pr) for pr in a.profiles})
    (a.output / 'summary.json').write_text(json.dumps(res, indent=2) + '\n')
    print(json.dumps(res, indent=2))


if __name__ == '__main__':
    main()
