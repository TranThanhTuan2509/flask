"""Select cases: FLASK implementation and evaluation utilities."""
import json
import sys
import time
from pathlib import Path
import fire
import numpy as np
import vamp
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'benchmarks'))
from flask_repro import mbm, native, trajectory, robot as R
from mbm_baseline import topp_ra
DT = 0.001

def main(robot='ur5', problem_count=15, seed=0):
    rb = R.get_robot(robot)
    settings = native.settings(velocity_sample_scale=0.25, neighbor_mode='A', max_ns=int(1000000000.0), max_iterations=int(10000000.0))
    dataset = mbm.load_dataset(robot)
    t0 = time.time()
    all_items = []
    for (name, i, s, g, _ev, ec) in mbm.paired_problems(robot):
        d = native.plan(rb, ec, s, g, settings, seed=seed + i)
        if d['solved']:
            all_items.append((name, i, (d['plan_ns'] + d['shortcut_ns']) / 1000000.0))
    print(f'FLASK: solved {len(all_items)} problem inside {time.time() - t0:.1f}s')
    all_items.sort(key=lambda x: x[2])
    n = len(all_items)
    (selected, environment_counts) = ([], {})
    for k in range(problem_count):
        level = round(k * (n - 1) / (problem_count - 1))
        w = max(1, int(0.03 * n))
        candidate = [j for j in range(max(0, level - w), min(n, level + w + 1)) if j not in selected]
        j = min(candidate, key=lambda j: (environment_counts.get(all_items[j][0], 0), abs(j - level)))
        selected.append(j)
        environment_counts[all_items[j][0]] = environment_counts.get(all_items[j][0], 0) + 1
    selected.sort()
    output = ROOT / 'results/legacy' / 'mujoco_mbm' / robot
    output.mkdir(parents=True, exist_ok=True)
    (m, planner, ps, simp) = vamp.configure_robot_and_planner_with_kwargs(robot, 'rrtc')
    sampler = m.halton()
    start_chain = []
    for (index, j) in enumerate(selected):
        (name, i, measured_ms) = all_items[j]
        p = dataset[name][i]
        s = np.asarray(p['start'], float)
        g = [np.asarray(x, float) for x in p['goals']]
        ev = vamp.problem_dict_to_vamp(p)
        ec = mbm.native_environment(p)
        f = native.plan(rb, ec, s, g, settings, seed=seed + i)
        sampler.reset()
        r = planner(list(s), [list(x) for x in g], ev, ps, sampler)
        sp = m.simplify(r.path, ev, simp, sampler)
        wp = np.asarray(sp.path.numpy(), float)
        a = time.perf_counter_ns()
        jt = topp_ra(wp, rb)
        ns_topp = time.perf_counter_ns() - a
        trajectory = f['trajectory']
        tf = np.arange(0.0, trajectory.duration() + DT / 2, DT)
        Qf = trajectory.evaluate(tf)[0]
        tb = np.arange(0.0, float(jt.duration) + DT / 2, DT)
        Qb = np.asarray(jt(tb))
        ec_ = mbm.native_environment(p)
        vcf = np.array([native.module(rb).count_collisions(ec_, [q.tolist()])[0] > 0 for q in Qf])
        vcb = np.array([native.module(rb).count_collisions(ec_, [q.tolist()])[0] > 0 for q in Qb])
        cf = trajectory.evaluate_flask(trajectory, rb, ec)
        tt = dict(index=index + 1, env=name, problem=i, row=j + 1, total_pairs=n, flask_ms=(f['plan_ns'] + f['shortcut_ns']) / 1000000.0, flask_diagnostic_ms=measured_ms, ms_baseline=(r.nanoseconds + sp.nanoseconds + ns_topp) / 1000000.0, T_flask=float(tf[-1]), T_baseline=float(tb[-1]), flask_length_rad=cf['length'], baseline_length_rad=trajectory.sampled_length(Qb), flask_sphere_collision_samples=int(vcf.sum()), baseline_sphere_collision_samples=int(vcb.sum()))
        np.savez_compressed(output / f'problem_{index + 1:02d}.npz', tf=tf, Qf=Qf, tb=tb, Qb=Qb, vcf=vcf, vcb=vcb, start=s, goal=Qf[-1])
        (output / f'problem_{index + 1:02d}_obstacles.json').write_text(json.dumps({k: p[k] for k in ('sphere', 'cylinder', 'box')}, default=float))
        start_chain.append(tt)
        print(f"#{index + 1:2d} row {j + 1:3d}/{n} {name:17s} problem {i:3d}: FLASK {tt['flask_ms']:7.3f} ms (T {tt['T_flask']:5.1f}s, collisions sphere {tt['flask_sphere_collision_samples']}) | baseline {tt['ms_baseline']:6.2f} ms (T {tt['T_baseline']:4.1f}s, collisions sphere {tt['baseline_sphere_collision_samples']})", flush=True)
    (output / 'inventory.json').write_text(json.dumps(start_chain, indent=1, ensure_ascii=False))
    print(f'-> {output}')
if __name__ == '__main__':
    fire.Fire(main)
