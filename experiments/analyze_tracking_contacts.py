"""Analyze tracking contacts: FLASK implementation and evaluation utilities."""
import json
import sys
from collections import Counter
from pathlib import Path
import fire
import numpy as np
import vamp
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'experiments'))
sys.path.insert(0, str(ROOT / 'simulation'))
from flask_repro import mbm, native, robot as R
import ur5_scene
import tracking_experiment as E

def main(directory):
    D = ROOT / directory if not Path(directory).is_absolute() else Path(directory)
    rb = R.get_robot('ur5')
    mod = native.module(rb)
    p = json.loads((D.parent / 'edge.json').read_text())
    planning_output = D.parent / 'iterations.json'
    iterations = json.loads(planning_output.read_text()) if planning_output.exists() else dict(counter=0.0, pick_clearance=0.02)
    ev = vamp.problem_dict_to_vamp(ur5_scene.inflate_scene(p, iterations['counter']))
    (start_q, pick_q, place_q) = ur5_scene.configurations(ev, iterations['pick_clearance'])
    reduced_environment = [mbm.native_environment(dict(p, box=[b for (j, b) in enumerate(p['box']) if j != i])) for i in range(len(p['box']))]
    obstacle_names = [b['name'] for b in p['box']]
    total = json.loads((D / 'summary.json').read_text())
    for name in ('geometric', 'flask'):
        (obstacle_counts, phase_counts) = (Counter(), Counter())
        for x in total:
            z = np.load(D / f"trial_{x['trial']:02d}_{name}.npz")
            if not z['actual_collision'].any():
                continue
            t = z['t']
            (w1, _) = E.plan_geometric(ev, start_q, pick_q, x['seed'])
            (w2, _) = E.plan_geometric(ev, pick_q, place_q, x['seed'])
            T1 = x['flask']['T']
            tc = None
            zf = np.load(D / f"trial_{x['trial']:02d}_flask.npz")
            build = np.where(np.all(np.abs(np.diff(zf['Qd'], axis=0)) < 1e-12, axis=1))[0]
            pick_time = t[build[0]] if len(build) else np.nan
            L = [np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(w, axis=0), axis=1))]) for w in (w1, w2)]
            t_end = [pick_time, T1]
            start_time = [0.0, pick_time + E.PAUSE]
            corner_angles = []
            for k in range(2):
                Tk = t_end[k] - start_time[k]
                corner_angles += list(start_time[k] + Tk * L[k][1:-1] / L[k][-1])
            stop_index = [pick_time, T1]
            idx = np.where(z['actual_collision'])[0]
            clusters = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
            for c in clusters:
                tm = t[c[0]]
                q = z['Qt'][c[len(c) // 2]]
                objects = [obstacle_names[i] for i in range(len(obstacle_names)) if mod.count_collisions(reduced_environment[i], [q.tolist()])[0] == 0]
                corner_error = min([abs(tm - g) for g in corner_angles], default=np.inf)
                stop_error = min((abs(tm - g) for g in stop_index))
                phase = 'turning' if corner_error < stop_error else 'stopping'
                obstacle_counts[','.join(objects) or '?'] += 1
                phase_counts[phase] += 1
                if name == 'geometric' and x['trial'] < 4:
                    print(f"  trial {x['trial']} t {tm:5.2f}s length {len(c) * E.DT * 1000.0:4.0f} ms objects {objects} | distance to turn {corner_error:.2f}s, distance to stop {stop_error:.2f}s -> {phase}")
        print(f'{name}: objects {dict(obstacle_counts)} | phase {dict(phase_counts)}')
if __name__ == '__main__':
    fire.Fire(main)
