"""V5 (acceptance checks; see "Verification" in README.md), low-DOF part: FLASK on NEW random box worlds (not the paper's benchmarks).

For each scene seed (default 7001-7100) a 6 m world receives 3-8 random axis-aligned boxes (0.3-1.5 m) that keep
0.6 m clear of the random start and goal. The same planner settings as the v10 campaign are used (case override of
unicycle_bugtrap / quad2d_bugtrap / quad3d_block from the v10 profiles). Every attempt is checked by the same
independent validator as the campaign (original boxes, original model; order-2 aerial also by the reduced-order
protocol check). Solve rates are reported but not gated (random scenes may be infeasible); the gate is validity of
returned trajectories, termination within budget, and no crash.

Usage: python -m verification.v5_random_low_dof --robot unicycle|quad2d|quad3d --order 2|4 --output DIR
"""
import argparse
import json
import time
import traceback
from pathlib import Path

import numpy as np

from flask_repro.flat_models import Quad2D, Quad3D, Unicycle
from flask_repro.flat_planner import World
from flask_repro.planning import PlannerSettings, plan, validate
from flask_repro.interval_validation import audit_segment

ROOT = Path(__file__).resolve().parents[1]
CASE = dict(unicycle='unicycle_bugtrap', quad2d='quad2d_bugtrap', quad3d='quad3d_block')


def scene(rng, robot):
    dim = 3 if robot == 'quad3d' else 2
    lo, hi = np.zeros(dim), np.full(dim, 6.)
    for _ in range(1000):
        s, g = rng.uniform(.6, 5.4, dim), rng.uniform(.6, 5.4, dim)
        if np.linalg.norm(s - g) > 2.: break
    boxes = []
    for _ in range(int(rng.integers(3, 9))):
        for _ in range(100):
            size = rng.uniform(.3, 1.5, dim); center = rng.uniform(lo + size / 2, hi - size / 2)
            if all(np.any(np.abs(p - center) > size / 2 + .6) for p in (s, g)):
                boxes.append((center.tolist(), size.tolist())); break
    if robot == 'unicycle':
        start, goal = np.r_[s, rng.uniform(-np.pi, np.pi)], np.r_[g, rng.uniform(-np.pi, np.pi)]
    elif robot == 'quad2d':
        start, goal = np.r_[s, 0, 0, 0, 0], np.r_[g, 0, 0, 0, 0]
    else:
        start, goal = np.r_[s, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], np.r_[g, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    return lo, hi, boxes, start, goal


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--robot', choices=list(CASE), required=True)
    p.add_argument('--order', type=int, choices=[2, 4], default=2)
    p.add_argument('--algorithms', nargs='+', default=['rrt_connect', 'sst_bvp'])
    p.add_argument('--seeds', type=int, nargs=2, default=[7001, 7100])
    p.add_argument('--budget', type=float, default=10.)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    profile = json.loads((ROOT / f"config/low_dof_v10_order{a.order if a.robot != 'unicycle' else 2}.json").read_text())
    case = CASE[a.robot]; o = profile['case_overrides'][case]
    geometry = profile['geometry'] | o.get('geometry', {}); boundary = profile['boundary'] | o.get('boundary', {})
    rows = []
    with (a.output / 'attempts.jsonl').open('w') as out:
        for seed in range(a.seeds[0], a.seeds[1] + 1):
            rng = np.random.default_rng(seed)
            lo, hi, boxes, start, goal = scene(rng, a.robot)
            original = World(lo, hi, boxes, [], geometry.get('radius', .1))
            planning = original.sphere_cover(geometry.get('sphere_spacing', .15), geometry.get('shell_only', False),
                                             geometry.get('shell_keep_boxes', True))
            planning.prepare_simd()
            model = dict(unicycle=Unicycle, quad2d=Quad2D, quad3d=Quad3D)[a.robot]()
            if isinstance(model, Unicycle):
                model.lift_speed = boundary['lift_speed']; model.reverse = boundary['reverse_chart']
            for k, v in (profile.get('model', {}) | o.get('model', {})).items():
                setattr(model, k, v)
            for algorithm in a.algorithms:
                cfg = PlannerSettings(**(profile['settings'] | o.get('settings', {}) | dict(algorithm=algorithm, budget_s=a.budget)))
                row = dict(seed=seed, robot=a.robot, order=model.order, algorithm=algorithm, boxes=len(boxes))
                t0 = time.perf_counter()
                try:
                    rec = plan(model, planning, start, goal, cfg, seed)
                    wall = time.perf_counter() - t0
                    val = validate(rec, model, original, cfg) if rec.solved else dict(valid=None)
                    if rec.solved and model.order < 4 and hasattr(model, 'gravity'):
                        val['protocol_validation'] = validate(rec, model, original, cfg, allow_reduced_model=True)
                    row.update(solved=bool(rec.solved), plan_ms=rec.plan_ns / 1e6, wall_s=wall, reason=rec.reason,
                               valid_original=val.get('valid'),
                               valid_protocol=(val.get('protocol_validation') or {}).get('valid'),
                               errors=val.get('errors') if not val.get('valid') else None,
                               within_budget=bool(wall <= a.budget * 1.05 + 1.))
                    if rec.solved and model.order == 4:  # interval oracle on every edge (as stage *_order4_interval)
                        limits = {k: getattr(model, k) for k in ['velocity_max', 'angular_velocity_max']
                                  if a.robot == 'quad2d' and np.isfinite(getattr(model, k))} or None
                        status = 'clear'
                        for e in rec.trajectory.to_records():
                            r = audit_segment(e['coefficients'], e['duration'], a.robot, limits=limits)
                            if r['status'] != 'clear':
                                status = r['status']; break
                        row['interval_status'] = status
                except Exception as error:  # a crash is a finding, not a skipped attempt
                    row.update(solved=False, crashed=True, error=repr(error), trace=traceback.format_exc()[-1500:])
                rows.append(row); out.write(json.dumps(row) + '\n'); out.flush()
    solved = [r for r in rows if r.get('solved')]
    reduced = a.robot != 'unicycle' and a.order < 4
    key = 'valid_protocol' if reduced else 'valid_original'
    summary = dict(robot=a.robot, order=a.order, attempts=len(rows), crashed=sum(bool(r.get('crashed')) for r in rows),
                   over_budget=sum(not r.get('within_budget', True) for r in rows),
                   per_algorithm={alg: dict(attempts=sum(r['algorithm'] == alg for r in rows),
                                            solved=sum(r['algorithm'] == alg and bool(r.get('solved')) for r in rows),
                                            returned_invalid=sum(r['algorithm'] == alg and r.get(key) is False for r in solved),
                                            median_plan_ms=float(np.median([r['plan_ms'] for r in solved if r['algorithm'] == alg]))
                                            if any(r['algorithm'] == alg for r in solved) else None)
                                  for alg in a.algorithms},
                   validity_judged_by=key,
                   interval_status_order4={st: sum(r.get('interval_status') == st for r in solved)
                                           for st in ['clear', 'constraint_violation', 'unresolved']} if a.order == 4 else None,
                   note='order-2 aerial is judged on the reduced-order protocol (never valid on the original model by construction)' if reduced else None)
    (a.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
