"""Run seeds 1..N per robot and pick the showcase run by a rule fixed before looking at the videos:
drop runs with any contact, failure or fall; primary = run whose total FLASK planning time is closest to the median
of the remaining runs; backup = the next closest.  Writes showcase/<FLASK_SHOWCASE>/selection.json.

    taskset -c 10 python -m simulation.showcase.select_runs --seeds 10
    # showcase_2, one robot per isolated core, then merge:
    FLASK_SHOWCASE=showcase_2 taskset -c 2 python -m simulation.showcase.select_runs --robots ur5   # ... each robot
    FLASK_SHOWCASE=showcase_2 python -m simulation.showcase.select_runs --merge
"""
import argparse
import os
import importlib
import json
import shutil
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'showcase' / os.environ.get('FLASK_SHOWCASE', 'showcase_1')   # new campaign: FLASK_SHOWCASE=showcase_2
MODULES = dict(g1='g1_courtyard', drone='drone_hall', ur5='ur5_cell', fetch='fetch_warehouse')
if OUT.name != 'showcase_1':      # showcase_2: pick-and-place arms and the two multi-robot teams
    MODULES = dict(g1='g1_courtyard', drone='drone_hall', ur5='ur5_pickplace', fetch='fetch_pickplace',
                   multiarm='multiarm_handover', multidrone='multidrone_swap')
PLACED = dict(ur5=3, multiarm=3, fetch=1)


def total_ms(robot, s):
    if 'total_ms' in s and robot in ('ur5', 'fetch', 'multidrone'):
        return s['total_ms']
    if robot == 'multiarm':
        return sum(s['total_us']) / 1e3
    if robot == 'g1':
        return sum(s['plan_us']) / 1e3
    if robot == 'drone':
        return sum(s['plan_ms'])
    if robot == 'ur5':
        return sum(s['plan_us']) / 1e3
    return sum(s.get('base_plan_ms', s['base_first_ms'])) + sum(s['arm_plan_us']) / 1e3


def ok(robot, s):
    contacts = s.get('contact_steps', s.get('contact_samples', s.get('contacts', 0)))
    good = contacts == 0 and not s.get('fell', False) and s.get('reference_collision_samples', 0) == 0
    if robot in PLACED:
        good &= s.get('placed', 0) == PLACED[robot]
    if robot == 'multidrone':
        from simulation.showcase.multidrone_swap import MARGIN
        good &= bool(s['interval_certified']) and s['planned_min_surface_gap_m'] >= MARGIN - 1e-3
    return bool(good)


def main():
    a = argparse.ArgumentParser(); a.add_argument('--seeds', type=int, default=10); a.add_argument('--robots', nargs='+', default=list(MODULES))
    a.add_argument('--merge', action='store_true', help='merge selection_<robot>.json files (parallel runs) into selection.json')
    args = a.parse_args()
    if args.merge:
        sel = json.loads((OUT / 'selection.json').read_text()) if (OUT / 'selection.json').exists() else {}
        for robot in MODULES:
            f = OUT / f'selection_{robot}.json'
            if f.exists():
                sel[robot] = json.loads(f.read_text()); f.unlink()
        (OUT / 'selection.json').write_text(json.dumps(sel, indent=2) + '\n')
        return
    parallel = len(args.robots) == 1       # one robot per process: write its own file, merge afterwards
    sel = {} if parallel else (json.loads((OUT / 'selection.json').read_text()) if (OUT / 'selection.json').exists() else {})
    for robot in args.robots:
        mod = importlib.import_module(f'simulation.showcase.{MODULES[robot]}')
        runs = {}
        for seed in range(1, args.seeds + 1):
            d = OUT / robot / 'seeds' / f'seed_{seed}'
            if d.exists():
                shutil.rmtree(d)
            try:
                s = mod.run(seed, d)
                if s is None:
                    s = json.loads((d / 'summary.json').read_text())
                runs[seed] = dict(ok=ok(robot, s), total_ms=total_ms(robot, s))
            except Exception as e:                    # failures stay in the record
                runs[seed] = dict(ok=False, error=repr(e))
        good = {k: v['total_ms'] for k, v in runs.items() if v['ok']}
        med = float(np.median(list(good.values())))
        order = sorted(good, key=lambda k: abs(good[k] - med))
        sel[robot] = dict(rule='closest total planning time to the median of contact-free, successful runs', runs=runs,
                          median_total_ms=med, primary=order[0], backup=order[1] if len(order) > 1 else None,
                          success=f'{len(good)}/{len(runs)}')
        print(robot, sel[robot]['success'], 'median', round(med, 2), 'primary', order[0])
        if parallel:
            (OUT / f'selection_{robot}.json').write_text(json.dumps(sel[robot], indent=2) + '\n')
        else:
            (OUT / 'selection.json').write_text(json.dumps(sel, indent=2) + '\n')


if __name__ == '__main__':
    main()
