"""Certified (sweep-bounded) environment collision checking for joint-space robots.

For a collision-sphere centre p on a link downstream of revolute joint j with axis a_j through o_j,
  dp/dt = sum_j qdot_j a_j x (p - o_j),  so  |dp/dt| <= sum_j |qdot_j| |p - o_j| <= sum_j R_j |qdot_j|,
where R_j bounds |p - o_j| over all configurations. By the triangle inequality R_j is at most the sum
of the joint-origin translations between joint j and the sphere's link plus the sphere-centre offset
in that link (prismatic joints add their maximum travel). Rigid rotations preserve these lengths, so
the bound holds for every configuration.

With the environment inflated by delta, a sample that is collision-free certifies the time interval of
half-width h/2 around it whenever sum_j R_j max|qdot_j| h/2 <= delta - float_margin
(src/flask/flask_cc.hh, JointLimits::certify_*). Self-collision is not certified by this construction.
"""
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np

from . import mbm


def _parse(path):
    root = ET.parse(path).getroot()
    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        limit = joint.find('limit')
        travel = 0.
        if joint.get('type') == 'prismatic' and limit is not None:
            travel = max(abs(float(limit.get('lower', 0.))), abs(float(limit.get('upper', 0.))))
        joints[joint.find('child').get('link')] = dict(name=joint.get('name'), kind=joint.get('type'),
                                                       parent=joint.find('parent').get('link'),
                                                       offset=float(np.linalg.norm(xyz)), travel=travel)
    spheres = []
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            sphere = collision.find('geometry/sphere')
            if sphere is None:
                continue
            origin = collision.find('origin')
            centre = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            spheres.append((link.get('name'), float(np.linalg.norm(centre))))
    return joints, spheres


@lru_cache(maxsize=None)
def reach(robot_name='panda', urdf=None):
    """R_j for the robot's actuated joints (VAMP joint order)."""
    import vamp
    path = Path(urdf) if urdf else mbm.VAMP_ROOT / f'resources/{robot_name}/{robot_name}_spherized.urdf'
    joints, spheres = _parse(path)
    names = list(getattr(vamp, robot_name).joint_names())
    child_of = {j['name']: child for child, j in joints.items()}
    out = []
    for name in names:
        axis_link = child_of[name]   # joint axis passes through the child frame origin
        best = 0.
        for link, centre in spheres:
            # walk up from the sphere's link to joint `name`, accumulating translations below it
            distance, current, found = centre, link, False
            while current in joints:
                j = joints[current]
                if j['name'] == name:
                    found = True
                    break
                distance += j['offset'] + j['travel']
                current = j['parent']
            if found:
                best = max(best, distance)
        out.append(best)
    return tuple(out)


def inflate_problem(problem, delta):
    """Copy of a MotionBenchMaker problem with every obstacle grown by delta (Minkowski superset)."""
    p = dict(problem)
    p['sphere'] = [dict(o, radius=o['radius'] + delta) for o in problem['sphere']]
    p['cylinder'] = [dict(o, radius=o['radius'] + delta, length=o['length'] + 2 * delta) for o in problem['cylinder']]
    p['box'] = [dict(o, half_extents=[h + delta for h in o['half_extents']]) for o in problem['box']]
    return p
