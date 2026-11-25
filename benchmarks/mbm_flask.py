"""Mbm flask: FLASK implementation and evaluation utilities."""
import csv
import sys
import time
from pathlib import Path
import fire
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from flask_repro import mbm, native, trajectory, robot as R
from flask_repro.provenance import save_manifest

def main(robot='panda', velocity_sample_scale=1.0, neighbor_mode='A', limit_scale=1.0, seed=0, max_ms=1000.0, max_iterations=100000, environment=None, problem_count=None, label=None, output=None, save_trajectories=True):
    import numpy as np
    rb = R.get_robot(robot).scaled_limits(limit_scale)
    settings = native.settings(velocity_sample_scale=velocity_sample_scale, neighbor_mode=neighbor_mode, max_ns=int(max_ms * 1000000.0), max_iterations=int(max_iterations))
    label = label or f'v{velocity_sample_scale:g}_{neighbor_mode}_gh{limit_scale:g}_s{seed}'
    output = Path(output) if output else ROOT / 'results/legacy' / f'flask_{robot}_{label}.csv'
    if output.exists():
        raise FileExistsError(f'refusing to overwrite existing results: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    path_dir = output.with_suffix('')
    if save_trajectories:
        path_dir.mkdir(parents=True, exist_ok=True)
    envs = [environment] if isinstance(environment, str) else environment or mbm.ENVIRONMENTS
    column = ['env', 'problem', 'solved', 'total_ms', 'rrt_connect_ms', 'postprocess_ms', 'iterations', 'raw_nodes', 'node', 'duration_s', 'length_rad', 'sample_count', 'collision_count', 'collision', 'wall_ms', 'reason']
    inventory = mbm.load_dataset(robot)
    save_manifest(output, dict(robot=robot, velocity_sample_scale=velocity_sample_scale, neighbor_mode=neighbor_mode, limit_scale=limit_scale, seed=seed, max_ms=max_ms, max_iterations=max_iterations, problem_count=problem_count, envs=envs, save_trajectories=save_trajectories, velocity_sampling='full_state' if velocity_sample_scale else 'restricted_zero_velocity', dataset_inventory={e: dict(available=len(inventory[e]), eligible=sum((bool(x['valid']) for x in inventory[e]))) for e in envs}, unpublished=['sampling bounds', 'neighbor metric', 'joint limits', 'timeout']), [mbm.VAMP_ROOT / 'resources' / robot / 'problems.pkl'])
    t0 = time.time()
    with open(output, 'w', newline='') as f:
        w = csv.DictWriter(f, column)
        w.writeheader()
        for name in envs:
            counter = 0
            for (_, i, s, g, _ev, ec) in mbm.paired_problems(robot, [name]):
                if problem_count is not None and counter >= problem_count:
                    break
                counter += 1
                t_plan = time.perf_counter_ns()
                d = native.plan(rb, ec, s, g, settings, seed=seed + i)
                wall_ms = (time.perf_counter_ns() - t_plan) / 1000000.0
                row = dict(env=name, problem=i, solved=int(d['solved']), iterations=d['iterations'], wall_ms=wall_ms, reason='solved' if d['solved'] else 'search_budget_exhausted', rrt_connect_ms=d['plan_ns'] / 1000000.0, postprocess_ms=d['shortcut_ns'] / 1000000.0, total_ms=(d['plan_ns'] + d['shortcut_ns']) / 1000000.0)
                if d['solved']:
                    c = trajectory.evaluate_flask(d['trajectory'], rb, ec)
                    row.update(raw_nodes=len(d['raw_positions']), node=len(d['Q']), duration_s=c['duration'], length_rad=c['length'], sample_count=c['sample_count'], collision_count=c['collision_count'], collision=int(c['collision']))
                    if save_trajectories:
                        np.savez_compressed(path_dir / f'{name}_{i}.npz', **{key: np.asarray(d[key]) for key in ['Q', 'V', 'T', 'raw_positions', 'raw_velocities', 'raw_durations']})
                w.writerow(row)
            f.flush()
            print(f'  {name}: finished ({time.time() - t0:.0f}s)', flush=True)
    print(f'-> {output}')
if __name__ == '__main__':
    fire.Fire(main)
