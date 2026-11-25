"""FLASK-RRTConnect on a MotionBenchMaker Panda problem: plan, then check the trajectory densely."""
import numpy as np
from flask_repro import mbm, native, robot

panda = robot.get_robot('panda')
problem = mbm.load_dataset('panda')['bookshelf_small'][0]
env = mbm.native_environment(problem)

# Settings of the paper's Table II protocol (config/manipulator_v10_rest.json).
settings = native.settings(range=1.0, velocity_sample_scale=0.0, dynamic_domain=True, prefix_steering=False,
                           max_ns=1_000_000_000)
result = native.plan(panda, env, problem['start'], problem['goals'], settings, seed=1)
assert result['solved']

trajectory = result['trajectory']                          # piecewise cubic, C1, within q/v/a limits
q, v, a = trajectory.evaluate(np.linspace(0.0, trajectory.duration(), 2000))[:3]
collisions, _ = native.module(panda).count_collisions(env, q.tolist())
print(f"planned in {(result['plan_ns'] + result['shortcut_ns']) / 1e3:.0f} us, duration {trajectory.duration():.2f} s, "
      f"{len(result['T'])} segments, dense collisions: {collisions}")

# Certified variant: re-checks the returned path in a scene inflated by 2 mm (no false accepts in our tests).
from flask_repro import certify
inflated = mbm.native_environment(certify.inflate_problem(problem, native.CERTIFY_DELTA))
safe = native.plan_lazy_certified(panda, env, inflated, problem['start'], problem['goals'], settings, seed=1)
print('certified solved:', safe['solved'])
