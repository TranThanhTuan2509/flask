"""Dynobench: FLASK implementation and evaluation utilities."""
from pathlib import Path
import yaml
import numpy as np
from .flat_models import Unicycle, Quad2D, Quad3D
from .flat_planner import World
ROOT = Path(__file__).resolve().parents[1] / 'third_party/dynobench_subset'
CASES = {'unicycle_bugtrap': ('unicycle1_v0/bugtrap_0.yaml', Unicycle), 'unicycle_wall': ('unicycle1_v2/wall_0.yaml', Unicycle), 'quad2d_bugtrap': ('quad2d_v0/quad_bugtrap.yaml', Quad2D), 'quad2d_hole': ('quad2d_v0/fall_through.yaml', Quad2D), 'quad3d_block': ('quadrotor_v0/quad_one_obs.yaml', Quad3D), 'quad3d_window': ('quadrotor_v0/window.yaml', Quad3D)}

def load(name, radius=0.1, sphere_spacing=0.15, simd=True, lift_speed=0.1, shell_only=False, shell_keep_boxes=True):
    (path, factory) = CASES[name]
    source = ROOT / 'envs' / path
    data = yaml.safe_load(source.read_text())
    environment = data['environment']
    robot = data['robots'][0]
    (boxes, spheres) = ([], [])
    for o in environment.get('obstacles', []):
        if o['type'] == 'box':
            boxes.append((o['center'], o['size']))
        elif o['type'] == 'sphere':
            spheres.append((o['center'], o['radius']))
        else:
            raise ValueError(f"Unsupported obstacle type: {o['type']}")
    original = World(environment['min'], environment['max'], boxes, spheres, radius)
    planning = original.sphere_cover(sphere_spacing, shell_only, shell_keep_boxes)
    if simd:
        planning.prepare_simd()
    model = factory()
    if isinstance(model, Unicycle):
        model.lift_speed = lift_speed
    provenance = dict(input=str(source), commit=(ROOT / 'COMMIT').read_text().strip(), correspondence='candidate_matching_figure_not_author_confirmed', robot_radius=radius, sphere_spacing=sphere_spacing, sphere_shell_only=shell_only, sphere_shell_keep_boxes=shell_keep_boxes, unicycle_lift_speed=lift_speed if isinstance(model, Unicycle) else None, geometry='conservative_cell_circumspheres', dynamics='Flatness reconstruction with integrator-chain order 2 for mobile and order 4 for aerial models', derivative_order=factory().order, deviations=['Exact author scene revisions are unconfirmed', 'Boundary thrust/derivatives are reconstructed', 'Unicycle nonzero boundary speed is reconstructed', 'Sphere-cover spacing is reconstructed', 'Steering/sampling settings are reconstructed'])
    return (model, planning, original, np.asarray(robot['start'], float), np.asarray(robot['goal'], float), provenance)
