"""Native: FLASK implementation and evaluation utilities."""
import numpy as np
from . import _core, require_native
from .reference import Trajectory

def module(robot):
    require_native()
    return getattr(_core, robot.name)

CERTIFY_DELTA = 0.002  # metres; planning environment must be inflated by the same amount

def joint_limits(robot, validation_profile='peak_speed', certify_delta=CERTIFY_DELTA):
    """Validation limits. Default 'peak_speed' (acceptance V2, results/verification/2026-11-25_acceptance/v2):
    7831/7831 edge decisions equal the authors' FlatVampMotionValidator and none accepts a densely-detected
    collision; 'published_resolution' (arc-length sample count, the pre-acceptance default) accepted 1/7831
    edges that collide over 7.6% of their duration. Benchmarks that must stay comparable with older campaigns
    pass 'published_resolution' explicitly."""
    limits = module(robot).JointLimits(list(robot.q_lo), list(robot.q_hi), list(robot.v_max), list(robot.a_max))
    if validation_profile == 'certified':
        # Sweep-bounded environment certification (flask_repro/certify.py). Callers must pass an
        # environment inflated by certify_delta (certify.inflate_problem) to the planner.
        from .certify import reach
        limits.certify_reach = list(reach(robot.name))
        limits.certify_delta = float(certify_delta)
    elif validation_profile == 'strict_development':
        limits.collision_resolution_scale = 8.
        limits.max_validation_dt = .005
        limits.bound_validation_speed = True
    elif validation_profile == 'peak_speed':
        # v10: sample count from duration x peak joint speed (an upper bound of the authors' T * peakSpeed /
        # resolution, FlatStateSpace::validSegmentCount) at the published 32 samples/rad; arc length is a lower
        # count for the same motion (published_resolution).
        limits.bound_validation_speed = True
    elif validation_profile != 'published_resolution':
        raise ValueError(f'Unknown validation profile: {validation_profile}')
    return limits
DEFAULT_VELOCITY_SCALE = 1.0

def settings(**kw):
    require_native()
    settings = _core.PlannerSettings()
    settings.velocity_sample_scale = DEFAULT_VELOCITY_SCALE
    lg = kw.pop('neighbor_mode', None)
    if lg is not None:
        if lg not in ('A', 'B'):
            raise ValueError('Invalid state, limits, or planner configuration')
        settings.cost_neighbors = lg == 'B'
    for (k, v) in kw.items():
        if not hasattr(settings, k):
            raise AttributeError(k)
        setattr(settings, k, v)
    return settings

def plan(robot, env_cpp, start, goals, settings=None, seed=0, shortcut=True, v_start=None, limits=None):
    """Plan."""
    require_native()
    settings = settings or _core.PlannerSettings()
    for name in ('range', 'rho', 'w'):
        if not np.isfinite(getattr(settings, name)) or getattr(settings, name) <= 0:
            raise ValueError(f'{name} is outside its supported range')
    if not 0 <= settings.velocity_sample_scale <= 1 or settings.k < 1 or settings.max_samples < len(goals) + 2:
        raise ValueError('Invalid state, limits, or planner configuration')
    for a in [start, *goals, *([] if v_start is None else [v_start])]:
        if np.asarray(a).shape != (robot.n,) or not np.isfinite(a).all():
            raise ValueError('Invalid state, limits, or planner configuration')
    vs = None if v_start is None else list(map(float, v_start))
    d = module(robot).plan(env_cpp, list(map(float, start)), [list(map(float, g)) for g in goals], limits or joint_limits(robot), settings, seed, shortcut, vs)
    if d['solved']:
        d['reason'] = 'solved'
    else:                                    # flask_v2: why it failed (checked after the timed native call)
        m = module(robot)
        d['reason'] = ('invalid_start' if not m.validate(env_cpp, list(map(float, start))) else
                       'invalid_goal' if not any(m.validate(env_cpp, list(map(float, g))) for g in goals) else 'not_found')
    if d['solved']:
        d['trajectory'] = Trajectory(np.asarray(d['Q']), np.asarray(d['V']), np.asarray(d['T']))
        d['raw_trajectory'] = Trajectory(np.asarray(d['raw_positions']), np.asarray(d['raw_velocities']), np.asarray(d['raw_durations']))
    return d


def plan_lazy_certified(robot, env_cpp, certified_env_cpp, start, goals, settings=None, seed=0, max_attempts=20,
                        certify_delta=CERTIFY_DELTA, nominal_profile='peak_speed'):
    """Nominal-resolution planning with certification of the returned path only (v9).

    certified_env_cpp must be the scene inflated by certify_delta (certify.inflate_problem)."""
    require_native()
    settings = settings or _core.PlannerSettings()
    d = module(robot).plan_lazy_certified(env_cpp, certified_env_cpp, list(map(float, start)),
                                          [list(map(float, g)) for g in goals], joint_limits(robot, nominal_profile),
                                          joint_limits(robot, 'certified', certify_delta), settings, seed, max_attempts)
    if d['solved']:
        d['trajectory'] = Trajectory(np.asarray(d['Q']), np.asarray(d['V']), np.asarray(d['T']))
        d['raw_trajectory'] = Trajectory(np.asarray(d['raw_positions']), np.asarray(d['raw_velocities']), np.asarray(d['raw_durations']))
    return d
