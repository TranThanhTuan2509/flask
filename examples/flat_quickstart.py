"""FLASK on the DynoBench systems (unicycle, 2D/3D quadrotor) with the settings used in our Table I runs.

    python examples/flat_quickstart.py quad3d_block rrt_connect
    python examples/flat_quickstart.py unicycle_wall sst_bvp
"""
import json
import sys
from pathlib import Path

from flask_repro.dynobench import load
from flask_repro.flat_models import Unicycle
from flask_repro.planning import PlannerSettings, plan, validate

case = sys.argv[1] if len(sys.argv) > 1 else 'quad3d_block'
algorithm = sys.argv[2] if len(sys.argv) > 2 else 'rrt_connect'      # rrt_connect | sst_bvp | sst_simd | rrt_dp

# Aerial systems: order 4 = full quadrotor model (recommended); order 2 is the paper's reduced protocol.
config = 'low_dof_v10_order2.json' if case.startswith('unicycle') else 'low_dof_v10_order4.json'
profile = json.loads((Path(__file__).resolve().parents[1] / 'config' / config).read_text())
override = profile['case_overrides'][case]
geometry, boundary = profile['geometry'] | override.get('geometry', {}), profile['boundary'] | override.get('boundary', {})

model, planning_world, world, start, goal, _ = load(case, **geometry, lift_speed=boundary['lift_speed'])
if isinstance(model, Unicycle):
    model.reverse = boundary['reverse_chart']
for key, value in (profile.get('model', {}) | override.get('model', {})).items():
    setattr(model, key, value)

settings = PlannerSettings(**(profile['settings'] | override.get('settings', {}) | dict(algorithm=algorithm)))
record = plan(model, planning_world, start, goal, settings, seed=1)
check = validate(record, model, world, settings)        # independent check on the original model and obstacles
print(f'{case} / {algorithm}: solved={record.solved} valid={check["valid"]} plan={record.plan_ns / 1e6:.2f} ms '
      f'duration={record.trajectory.duration:.2f} s edges={len(record.trajectory.edges)}')
