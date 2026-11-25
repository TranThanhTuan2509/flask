"""Sequential Table II case measurements with retained failures and trajectories.

This measures the disclosed reconstruction, not the unavailable author setup.
Planning, postprocessing, wrapper time, and extra validation are separate.
No physics simulation or rendering is imported or executed.
"""
import argparse
import json
import os
from pathlib import Path
from time import perf_counter_ns

import numpy as np
import vamp
from scipy.spatial.transform import Rotation

from benchmarks.inventory_mbm import SphereOracle, obstacle_hits
from benchmarks.mbm_baseline import topp_ra
from benchmarks.native_campaign import hardware_snapshot
from flask_repro import certify, lqmt, mbm, native, robot, trajectory
from flask_repro.provenance import distribution, file_hash, manifest
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

ROOT = Path(__file__).resolve().parents[1]


def attributes(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, (list, tuple)):
        return [attributes(item) for item in value]
    return {name: attributes(getattr(value, name)) for name in dir(value)
            if not name.startswith('_') and not callable(getattr(value, name))}


def wilson_interval(successes, attempts):
    z = 1.959963984540054
    fraction = successes / attempts;denominator = 1 + z * z / attempts
    center = (fraction + z * z / (2 * attempts)) / denominator
    half = z * np.sqrt(fraction * (1 - fraction) / attempts + z * z / (4 * attempts * attempts)) / denominator
    return [max(0., center - half), min(1., center + half)]


def independent_sphere_centers(oracle, configurations):
    configurations = np.asarray(configurations)
    count = len(configurations)
    coordinates = dict(zip(vamp.panda.joint_names(), configurations.T))
    frames = {name: np.broadcast_to(np.eye(4), (count, 4, 4)) for name in oracle.roots}
    pending = list(oracle.joints)
    while pending:
        progress = False
        for joint in list(pending):
            if joint['parent'] not in frames:
                continue
            motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
            values = coordinates.get(joint['name'], np.zeros(count))
            if joint['kind'] in ['revolute', 'continuous']:
                motion[:, :3, :3] = Rotation.from_rotvec(values[:, None] * joint['axis']).as_matrix()
            elif joint['kind'] == 'prismatic':
                motion[:, :3, 3] = values[:, None] * joint['axis']
            elif joint['kind'] != 'fixed':
                raise ValueError('Unsupported URDF joint type')
            frames[joint['child']] = frames[joint['parent']] @ joint['transform'] @ motion
            pending.remove(joint);progress = True
        if not progress:
            raise ValueError('Disconnected or cyclic URDF')
    return np.stack([frames[link][:, :3, :3] @ position + frames[link][:, :3, 3]
                     for link, position, radius in oracle.spheres], axis=1)


def independent_environment_check(oracle, configurations, problem):
    """Distributed independent URDF FK and NumPy obstacle distances."""
    centers = independent_sphere_centers(oracle, configurations)
    radii = np.asarray([sphere[2] for sphere in oracle.spheres])
    minimum = np.full(len(configurations), np.inf)
    for kind in ['sphere', 'box', 'cylinder']:
        for obstacle in problem[kind]:
            delta = centers - np.asarray(obstacle['position'])
            if kind == 'sphere':
                clearance = np.linalg.norm(delta, axis=-1) - radii - obstacle['radius']
            else:
                rotation = Rotation.from_euler('xyz', obstacle['orientation_euler_xyz']).as_matrix()
                local = delta @ rotation
                if kind == 'box' or problem['problem'] == 'box':
                    half = np.asarray(obstacle['half_extents']) if kind == 'box' else np.array(
                        [obstacle['radius'], obstacle['radius'], obstacle['length'] / 2])
                    excess = np.abs(local) - half
                    clearance = np.linalg.norm(np.maximum(excess, 0.), axis=-1) + np.minimum(
                        np.max(excess, axis=-1), 0.) - radii
                else:
                    nearest = local.copy()
                    nearest[:, :, 2] -= np.clip(local[:, :, 2], -obstacle['length'] / 2, obstacle['length'] / 2)
                    clearance = np.linalg.norm(nearest, axis=-1) - radii - obstacle['radius']
            minimum = np.minimum(minimum, np.min(clearance, axis=1))
    indices = np.flatnonzero(minimum <= 0.)
    hits = []
    if len(indices):
        index = int(indices[0]);q = configurations[index]
        spheres = oracle.forward(dict(zip(vamp.panda.joint_names(), q)))
        hits.append(dict(sample=index, configuration=q.tolist(), hits=obstacle_hits(spheres, problem)))
    return dict(samples_checked=len(configurations), requested_samples=len(configurations),
                minimum_sampled_clearance_m=float(minimum.min()) if np.isfinite(minimum.min()) else None,
                environment_collision=bool(hits), first_witness=hits,
                self_collision_policy='VAMP supplies self-collision exclusions separately')


def validate_flask(result, rb, environment, problem, oracle, profile):
    path = result['trajectory']
    dense = trajectory.evaluate_flask(path, rb, environment)
    strict = native.joint_limits(rb, profile['additional_validation_profile'])
    module = native.module(rb)
    checks = []
    exact_limits = []
    for edge in path.edges():
        args = [a.tolist() for a in (edge.q0, edge.v0, edge.q1, edge.v1)]
        checks.append(bool(module.flask_cc(environment, *args, edge.T, strict)))
        exact_limits.append(bool(lqmt.within_limits(edge.coef, edge.T, rb.q_lo, rb.q_hi,
                                                  rb.v_max, rb.a_max)))
    boundary_ok = bool(np.allclose(path.Q[0], problem['start'], atol=1e-8, rtol=0.)
        and any(np.allclose(path.Q[-1], g, atol=1e-8, rtol=0.) for g in problem['goals'])
        and np.max(np.abs(path.V[[0, -1]])) < 1e-8)
    count = profile['independent_urdf_samples_per_trajectory']
    configurations = path.evaluate(np.linspace(0., path.duration(), count))[0]
    independent = independent_environment_check(oracle, configurations, problem)
    valid = boundary_ok and all(checks) and all(exact_limits) and not dense['collision'] and not independent['environment_collision']
    return dict(valid=bool(valid), boundary_valid=boundary_ok, strict_edges_valid=all(checks),
                analytic_joint_limits_valid=all(exact_limits), dense=dense,
                independent_environment=independent)


def flask_trial(rb, environment, problem, settings, limits, seed, oracle, profile, planning_environment=None,
                lazy_certified_environment=None):
    # planning_environment: delta-inflated scene for the certified profile; validation always uses the original.
    # lazy_certified_environment: delta-inflated scene used only to certify the returned path (v9).
    begin = perf_counter_ns()
    if lazy_certified_environment is not None:
        result = native.plan_lazy_certified(rb, environment, lazy_certified_environment, problem['start'], problem['goals'],
                                            settings, seed=seed, max_attempts=profile.get('lazy_max_attempts', 20),
                                            certify_delta=profile.get('certify_delta', native.CERTIFY_DELTA),
                                            # explicit since acceptance V2: the v9/v10 lazy stages ran with the then
                                            # default 'published_resolution' nominal grid, whatever the profile said
                                            nominal_profile=profile.get('lazy_nominal_profile', profile['planning_validation_profile']))
    else:
        result = native.plan(rb, planning_environment or environment, problem['start'], problem['goals'], settings,
                             seed=seed, limits=limits)
    wrapper_ns = perf_counter_ns() - begin
    keys = ['solved', 'plan_ns', 'shortcut_ns', 'iterations', 'start_nodes', 'goal_nodes',
            'edge_checks', 'batch_checks', 'limit_rejections', 'collision_rejections',
            'task_rejections', 'dynamic_domain_rejections', 'steering_failures',
            'accepted_extensions', 'connection_attempts']
    keys += [k for k in ['certification_ns', 'certification_attempts', 'rejected_paths', 'simplify_only_ns'] if k in result]
    row = {key: result[key] for key in keys}
    row.update(wrapper_ns=wrapper_ns, postprocess_ns=result['shortcut_ns'],
               total_ns=result['plan_ns'] + result['shortcut_ns'],
               reason='solved' if result['solved'] else 'search_budget_exhausted')
    arrays = {}
    begin = perf_counter_ns()
    if result['solved']:
        row['validation'] = validate_flask(result, rb, environment, problem, oracle, profile)
        for key in ['Q', 'V', 'T', 'raw_positions', 'raw_velocities', 'raw_durations']:
            arrays[key] = np.asarray(result[key])
        row['length_rad'] = row['validation']['dense']['length']
        row['duration_s'] = row['validation']['dense']['duration']
        row['collision'] = row['validation']['dense']['collision']
    else:
        row.update(validation=dict(valid=False, errors=['no_solution']),
                   length_rad=None, duration_s=None, collision=None)
    row['additional_validation_ns'] = perf_counter_ns() - begin
    return row, arrays


def baseline_trial(rb, ev, environment, problem, components, seed, oracle, profile):
    module, planner, settings, simplification = components
    sampler = module.halton()
    offset = (seed - 1) * profile['baseline_halton_stride']
    sampler.skip(offset)
    begin = perf_counter_ns()
    result = planner(list(problem['start']), [list(g) for g in problem['goals']], ev, settings, sampler)
    row = dict(solved=bool(result.solved), plan_ns=result.nanoseconds,
               simplify_ns=0, toppra_ns=0, postprocess_ns=0, total_ns=result.nanoseconds,
               reason='search_failed', halton_offset=offset)
    arrays = {}
    timed = None
    if result.solved:
        simplified = module.simplify(result.path, ev, simplification, sampler)
        waypoints = np.asarray(simplified.path.numpy(), float)
        row['simplify_ns'] = simplified.nanoseconds
        stamp = perf_counter_ns()
        try:
            timed = topp_ra(waypoints, rb, profile['baseline_parameterization'], profile.get('baseline_gridpoints'))
        except (ValueError, RuntimeError) as error:
            row['parameterization_error'] = str(error)
        row['toppra_ns'] = perf_counter_ns() - stamp
        row['postprocess_ns'] = row['simplify_ns'] + row['toppra_ns']
        row['total_ns'] += row['postprocess_ns']
        row['solved'] = timed is not None
        row['reason'] = 'solved' if timed is not None else 'parameterization_failed'
        arrays.update(geometric_waypoints=np.asarray(result.path.numpy()),
                      simplified_waypoints=waypoints)
    row['wrapper_ns'] = perf_counter_ns() - begin
    begin = perf_counter_ns()
    if timed is not None:
        duration = float(timed.duration)
        dense = trajectory.evaluate_function(lambda t: np.asarray(timed(t)), duration, rb, environment)
        sample_times = np.linspace(0., duration, dense['sample_count'])
        q = np.asarray(timed(sample_times));v = np.asarray(timed(sample_times, 1))
        acceleration = np.asarray(timed(sample_times, 2))
        limit_error = max(float(np.max(rb.q_lo - q)), float(np.max(q - rb.q_hi)),
                          float(np.max(np.abs(v) - rb.v_max)),
                          float(np.max(np.abs(acceleration) - rb.a_max)), 0.)
        boundary_ok = bool(np.allclose(q[0], problem['start'], atol=1e-6, rtol=0.)
            and any(np.allclose(q[-1], g, atol=1e-6, rtol=0.) for g in problem['goals'])
            and np.max(np.abs(v[[0, -1]])) < 1e-6)
        independent = independent_environment_check(oracle, np.asarray(timed(np.linspace(
            0., duration, profile['independent_urdf_samples_per_trajectory']))), problem)
        row.update(length_rad=dense['length'], duration_s=duration, collision=dense['collision'],
                   validation=dict(valid=bool(boundary_ok and not dense['collision']
                       and not independent['environment_collision'] and limit_error <= 1e-5),
                       boundary_valid=boundary_ok, sampled_limit_violation=limit_error,
                       dense=dense, independent_environment=independent))
        # Version-pinned TOPPRA constant-acceleration representation. Together
        # with the waypoint spline this retains the exact numerical reference.
        for name in ['_ss', '_velocities', '_xs', '_ts', '_us']:
            arrays['toppra' + name] = np.asarray(getattr(timed, name))
    else:
        row.update(validation=dict(valid=False, errors=[row['reason']]),
                   length_rad=None, duration_s=None, collision=None)
    row['additional_validation_ns'] = perf_counter_ns() - begin
    return row, arrays


def summarize(rows):
    result = {}
    for method in ['flask_rrt_connect', 'flask_rrt_connect_lazy_certified', 'vamp_toppra']:
        selected = [r for r in rows if r['method'] == method]
        if not selected:
            continue
        for environment in mbm.ENVIRONMENTS + ['overall']:
            group = selected if environment == 'overall' else [r for r in selected if r['environment'] == environment]
            if not group:
                continue
            solved = [r for r in group if r['solved']]
            valid = [r for r in solved if r['validation']['valid']]
            result[method + '/' + environment] = dict(
                attempts=len(group), solved=len(solved), validated=len(valid),
                solved_rate=len(solved) / len(group), validated_rate=len(valid) / len(group),
                validated_rate_wilson_95=wilson_interval(len(valid), len(group)),
                success_conditioned_total_ms=distribution([r['total_ns'] / 1e6 for r in solved]),
                validated_total_ms=distribution([r['total_ns'] / 1e6 for r in valid]),
                all_attempt_planning_ms=distribution([r['plan_ns'] / 1e6 for r in group]),
                success_conditioned_plan_ms=distribution([r['plan_ns'] / 1e6 for r in solved]),
                success_conditioned_postprocess_ms=distribution([r['postprocess_ns'] / 1e6 for r in solved]),
                success_conditioned_length_rad=distribution([r['length_rad'] for r in solved]),
                dense_collision_trajectories=sum(bool(r['collision']) for r in solved),
                dense_collision_risk=sum(bool(r['collision']) for r in solved) / len(solved) if solved else None,
            )
    return result


def run(args):
    os.sched_setaffinity(0, {args.cpu})
    profile = json.loads(args.profile.read_text())
    registry = json.loads((ROOT / 'config/seed_registry.json').read_text())
    seeds = args.seeds or registry[profile.get('seed_set','manipulator')]
    if not seeds or len(set(seeds)) != len(seeds) or min(seeds) < 1:
        raise ValueError('Unique positive seeds required')
    args.output.mkdir(parents=True, exist_ok=False)
    rb = robot.get_robot(profile['robot'])
    dataset = mbm.load_dataset(rb.name)
    settings = native.settings(**profile['settings'])
    certify_delta = profile.get('certify_delta', native.CERTIFY_DELTA)
    limits = native.joint_limits(rb, profile['planning_validation_profile'], certify_delta)
    certified = profile['planning_validation_profile'] == 'certified'
    for key, value in profile.get('planning_limit_overrides', {}).items():
        setattr(limits, key, value)  # e.g. collision_resolution_scale (v2 resolution profiles)
    components = vamp.configure_robot_and_planner_with_kwargs(rb.name, 'rrtc')
    oracle = SphereOracle(mbm.VAMP_ROOT / 'resources/panda/panda_spherized.urdf')
    configuration = dict(profile=profile, seeds=seeds, environments=args.environments,
        problem_count=args.problem_count, methods=args.methods,
        native_settings=attributes(settings), baseline_settings=attributes(components[2]),
        baseline_simplification=attributes(components[3]),
        joint_limits=dict(q_lo=rb.q_lo.tolist(), q_hi=rb.q_hi.tolist(),
                          v_max=rb.v_max.tolist(), a_max=rb.a_max.tolist()),
        campaign_class='paper_case_reconstruction_measurement',
        isolation_note='Single logical CPU affinity; full physical-core isolation not established',
        seed_note=registry.get('development_note'))
    provenance = manifest(configuration, [args.profile, ROOT / 'config/seed_registry.json',
        ROOT / 'docs/sources/flask_v4.pdf', mbm.VAMP_ROOT / 'resources/panda/problems.pkl',
        mbm.VAMP_ROOT / 'resources/panda/panda_spherized.urdf'])
    provenance['hardware_before'] = hardware_snapshot()
    (args.output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')
    inventory = [dict(environment=env, problem=i, source_valid=bool(p['valid']))
                 for env in args.environments for i, p in enumerate(dataset[env])]
    (args.output / 'inventory.json').write_text(json.dumps(inventory, indent=2) + '\n')
    rows = []
    with (args.output / 'attempts.jsonl').open('w') as stream:
        for env in args.environments:
            for index, problem in enumerate(dataset[env]):
                if args.problem_count is not None and index >= args.problem_count:
                    break
                if not problem['valid']:
                    continue
                ec = mbm.native_environment(problem)
                ec_plan = mbm.native_environment(certify.inflate_problem(problem, certify_delta)) if certified else None
                ev = vamp.problem_dict_to_vamp(problem)
                for seed in seeds:
                    order = args.methods if seed % 2 else list(reversed(args.methods))
                    for method in order:
                        if method == 'flask_rrt_connect':
                            row, arrays = flask_trial(rb, ec, problem, settings, limits, seed, oracle, profile, ec_plan)
                        elif method == 'flask_rrt_connect_lazy_certified':
                            ec_cert = mbm.native_environment(certify.inflate_problem(problem, certify_delta))
                            row, arrays = flask_trial(rb, ec, problem, settings, limits, seed, oracle, profile,
                                                      lazy_certified_environment=ec_cert)
                        else:
                            row, arrays = baseline_trial(rb, ev, ec, problem, components, seed, oracle, profile)
                        row.update(environment=env, problem=index, seed=seed, method=method)
                        if arrays:
                            path = args.output / f'{env}_{index:03d}_seed_{seed:02d}_{method}.npz'
                            np.savez_compressed(path, **arrays)
                            row['trajectory_artifact'] = path.name
                            row['trajectory_sha256'] = file_hash(path)
                        rows.append(row)
                        stream.write(json.dumps(row) + '\n');stream.flush()
                        print(json.dumps({key: row[key] for key in ['environment', 'problem', 'seed', 'method', 'solved', 'total_ns']}
                              | dict(valid=row['validation']['valid'])), flush=True)
                if index % 10 == 9:
                    (args.output / 'summary.json').write_text(json.dumps(summarize(rows), indent=2) + '\n')
            (args.output / 'summary.json').write_text(json.dumps(summarize(rows), indent=2) + '\n')
    after = manifest(configuration)
    changed = [name for name in set(provenance['source_sha256']) | set(after['source_sha256'])
               if provenance['source_sha256'].get(name) != after['source_sha256'].get(name)]
    provenance.update(hardware_after=hardware_snapshot(), completed_attempts=len(rows),
        source_changed_during_campaign=changed,
        artifact_sha256={name: file_hash(args.output / name) for name in ['attempts.jsonl', 'summary.json', 'inventory.json']})
    (args.output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')
    if changed:
        raise RuntimeError('Source changed during measurements; campaign is not frozen')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--profile', type=Path, default=ROOT / 'config/manipulator_measurement_v1.json')
    parser.add_argument('--cpu', type=int, default=default_cpu())
    parser.add_argument('--seeds', nargs='+', type=int)
    parser.add_argument('--environments', nargs='+', choices=mbm.ENVIRONMENTS, default=mbm.ENVIRONMENTS)
    parser.add_argument('--methods', nargs='+', choices=['flask_rrt_connect', 'flask_rrt_connect_lazy_certified', 'vamp_toppra'], default=['flask_rrt_connect', 'vamp_toppra'])
    parser.add_argument('--problem-count', type=int)
    run(parser.parse_args())
