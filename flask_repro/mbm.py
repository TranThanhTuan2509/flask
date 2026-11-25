"""Mbm: FLASK implementation and evaluation utilities."""
import os
import pickle
from pathlib import Path
import numpy as np
import vamp
ROOT = Path(os.environ.get('FLASK_REPRO_ROOT', Path(__file__).resolve().parent.parent))
VAMP_ROOT = ROOT / 'third_party' / 'vamp'
ENVIRONMENTS = ['bookshelf_small', 'bookshelf_tall', 'bookshelf_thin', 'box', 'cage', 'table_pick', 'table_under_pick']

def load_dataset(robot):
    with open(VAMP_ROOT / 'resources' / robot / 'problems.pkl', 'rb') as f:
        return pickle.load(f)['problems']

def problems(robot, environment=None, valid_only=True):
    """Problems."""
    dataset = load_dataset(robot)
    for name in environment or ENVIRONMENTS:
        for (i, d) in enumerate(dataset[name]):
            if valid_only and (not d['valid']):
                continue
            yield (name, i, np.asarray(d['start'], float), [np.asarray(g, float) for g in d['goals']], vamp.problem_dict_to_vamp(d))

def native_environment(d):
    """Native environment."""
    from . import _core
    e = _core.Environment()
    for o in d['sphere']:
        e.add_sphere(list(o['position']), float(o['radius']))
    for o in d['cylinder']:
        if d['problem'] == 'box':
            e.add_cuboid(list(o['position']), list(o['orientation_euler_xyz']), [o['radius'], o['radius'], o['length'] / 2])
        else:
            e.add_capsule(list(o['position']), list(o['orientation_euler_xyz']), float(o['radius']), float(o['length']))
    for o in d['box']:
        e.add_cuboid(list(o['position']), list(o['orientation_euler_xyz']), list(o['half_extents']))
    return e

def paired_problems(robot, environment=None, valid_only=True):
    """Paired problems."""
    dataset = load_dataset(robot)
    for name in environment or ENVIRONMENTS:
        for (i, d) in enumerate(dataset[name]):
            if valid_only and (not d['valid']):
                continue
            yield (name, i, np.asarray(d['start'], float), [np.asarray(g, float) for g in d['goals']], vamp.problem_dict_to_vamp(d), native_environment(d))
