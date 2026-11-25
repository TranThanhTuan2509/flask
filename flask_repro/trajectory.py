"""Trajectory: FLASK implementation and evaluation utilities."""
import math
import os
import numpy as np
from . import native
DT = 0.001
REFINEMENT = int(os.environ.get('FLASK_GAP', '4'))

def resample_count(total_duration, length, robot):
    return max(math.ceil(total_duration / DT), math.ceil(REFINEMENT * robot.resolution * length), 2)

def sampled_length(Q):
    """Sampled length."""
    return float(np.sum(np.linalg.norm(np.diff(Q, axis=0), axis=1)))

def evaluate_flask(trajectory, robot, env_cpp):
    """Evaluate flask."""
    T = trajectory.duration()
    L = trajectory.length()
    N = resample_count(T, L, robot)
    t = np.linspace(0.0, T, N)
    Q = trajectory.evaluate(t)[0]
    (counter, _) = native.module(robot).count_collisions(env_cpp, Q.tolist())
    return dict(duration=T, length=sampled_length(Q), sample_count=N, collision_count=counter, collision=counter > 0)

def evaluate_function(f, T, robot, env_cpp, estimated_length=None):
    """Evaluate function."""
    if estimated_length is None:
        Q0 = f(np.linspace(0.0, T, 2000))
        estimated_length = sampled_length(Q0)
    N = resample_count(T, estimated_length, robot)
    Q = f(np.linspace(0.0, T, N))
    (counter, _) = native.module(robot).count_collisions(env_cpp, Q.tolist())
    return dict(duration=T, length=sampled_length(Q), sample_count=N, collision_count=counter, collision=counter > 0)
