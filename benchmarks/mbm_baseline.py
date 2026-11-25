"""Mbm baseline: FLASK implementation and evaluation utilities."""
import csv
import sys
import time
from pathlib import Path
import fire
import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
import vamp
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from flask_repro import mbm, trajectory, robot as R
from flask_repro.provenance import save_manifest
ta.setup_logging('WARNING')

def topp_ra(wp, rb, parameterization='chord_length', gridpoints=None):
    """gridpoints=None: toppra's automatic proposal (pure-Python propose_gridpoints).
    gridpoints=k: k uniformly spaced path parameters (pre-declared fixed-grid condition)."""
    wp = np.asarray(wp, float)
    wp = wp[np.r_[True, np.linalg.norm(np.diff(wp, axis=0), axis=1) > 1e-12]]
    if len(wp) < 2:
        raise ValueError('TOPP-RA needs at least two distinct waypoints')
    if parameterization not in ('uniform', 'chord_length'):
        raise ValueError('unknown spline parameterization')
    if parameterization == 'uniform':
        s = np.linspace(0.0, 1.0, len(wp))
    else:
        L = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(wp, axis=0), axis=1))])
        s = L / L[-1]
    path = ta.SplineInterpolator(s, wp)
    cv = constraint.JointVelocityConstraint(np.vstack([-rb.v_max, rb.v_max]).T)
    ca = constraint.JointAccelerationConstraint(np.vstack([-rb.a_max, rb.a_max]).T)
    grid = None if gridpoints is None else np.linspace(0.0, 1.0, int(gridpoints))
    inst = algo.TOPPRA([cv, ca], path, gridpoints=grid, parametrizer='ParametrizeConstAccel')
    return inst.compute_trajectory()

def main(robot='panda', limit_scale=1.0, parameterization='chord_length', environment=None, problem_count=None, label=None, output=None):
    rb = R.get_robot(robot).scaled_limits(limit_scale)
    (m, planner, ps, simp) = vamp.configure_robot_and_planner_with_kwargs(robot, 'rrtc')
    sampler = m.halton()
    label = label or f'limits{limit_scale:g}_{parameterization}'
    output = Path(output) if output else ROOT / 'results/legacy' / f'baseline_{robot}_{label}.csv'
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    envs = [environment] if isinstance(environment, str) else environment or mbm.ENVIRONMENTS
    column = ['env', 'problem', 'solved', 'total_ms', 'rrt_connect_ms', 'simplify_ms', 'toppra_ms', 'postprocess_ms', 'waypoint_count', 'duration_s', 'length_rad', 'sample_count', 'collision_count', 'collision', 'wall_ms', 'reason']
    inventory = mbm.load_dataset(robot)
    save_manifest(output, dict(robot=robot, limit_scale=limit_scale, parameterization=parameterization, envs=envs, problem_count=problem_count, dataset_inventory={e: dict(available=len(inventory[e]), eligible=sum((bool(x['valid']) for x in inventory[e]))) for e in envs}, spline_note='SplineInterpolator changes the interpolated path; TOPPRA retimes that spline', unpublished=['author waypoint interpolation', 'joint limits', 'VAMP settings']), [mbm.VAMP_ROOT / 'resources' / robot / 'problems.pkl'])
    t0 = time.time()
    with open(output, 'w', newline='') as f:
        w = csv.DictWriter(f, column)
        w.writeheader()
        for name in envs:
            counter = 0
            for (_, i, s, g, ev, ec) in mbm.paired_problems(robot, [name]):
                if problem_count is not None and counter >= problem_count:
                    break
                counter += 1
                sampler.reset()
                t_plan = time.perf_counter_ns()
                r = planner(list(s), [list(x) for x in g], ev, ps, sampler)
                row = dict(env=name, problem=i, solved=int(r.solved), rrt_connect_ms=r.nanoseconds / 1000000.0, total_ms=r.nanoseconds / 1000000.0, reason='search_failed')
                if r.solved:
                    sp = m.simplify(r.path, ev, simp, sampler)
                    wp = np.asarray(sp.path.numpy(), float)
                    a = time.perf_counter_ns()
                    try:
                        jt = topp_ra(wp, rb, parameterization)
                    except (ValueError, RuntimeError):
                        jt = None
                    ns_topp = time.perf_counter_ns() - a
                    if jt is None:
                        row['solved'] = 0
                        row['reason'] = 'parameterization_failed'
                        row.update(simplify_ms=sp.nanoseconds / 1000000.0, toppra_ms=ns_topp / 1000000.0, postprocess_ms=(sp.nanoseconds + ns_topp) / 1000000.0, total_ms=(r.nanoseconds + sp.nanoseconds + ns_topp) / 1000000.0)
                    else:
                        row['reason'] = 'solved'
                        wall_ms = (time.perf_counter_ns() - t_plan) / 1000000.0
                        c = trajectory.evaluate_function(lambda t: np.asarray(jt(t)), float(jt.duration), rb, ec)
                        row.update(rrt_connect_ms=r.nanoseconds / 1000000.0, simplify_ms=sp.nanoseconds / 1000000.0, toppra_ms=ns_topp / 1000000.0, postprocess_ms=(sp.nanoseconds + ns_topp) / 1000000.0, total_ms=(r.nanoseconds + sp.nanoseconds + ns_topp) / 1000000.0, waypoint_count=len(wp), duration_s=c['duration'], length_rad=c['length'], sample_count=c['sample_count'], collision_count=c['collision_count'], collision=int(c['collision']))
                        row['wall_ms'] = wall_ms
                if 'wall_ms' not in row:
                    row['wall_ms'] = (time.perf_counter_ns() - t_plan) / 1000000.0
                w.writerow(row)
            f.flush()
            print(f'  {name}: finished ({time.time() - t0:.0f}s)', flush=True)
    print(f'-> {output}')
if __name__ == '__main__':
    fire.Fire(main)
