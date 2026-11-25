"""Reactive mesh: FLASK implementation and evaluation utilities."""
import json
import sys
from pathlib import Path
import mujoco
import numpy as np
from flask_repro.contact_validation import obstacle_contacts
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'simulation'))
from render_benchmark import quat_wxyz, convert_urdf
D = ROOT / 'results/legacy' / 'e2'
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
TUBE_SEGMENTS = 24

def local_tube_centers(tube, n):
    s = np.linspace(-tube['length'] / 2, tube['length'] / 2, n + 1)
    phi = s / tube['arc_radius']
    return np.stack([tube['arc_radius'] * np.sin(phi), np.zeros_like(phi), -tube['arc_radius'] * (1 - np.cos(phi))], 1)

def tube_quaternion(psi, theta):
    """Tube quaternion."""
    qz = np.array([np.cos(psi / 2), 0, 0, np.sin(psi / 2)])
    qy = np.array([np.cos(theta / 2), 0, np.sin(theta / 2), 0])
    (w1, x1, y1, z1) = qz
    (w2, x2, y2, z2) = qy
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])

def build(cj):
    sp = mujoco.MjSpec()
    wb = sp.worldbody
    for o in cj['edge']['box']:
        wb.add_geom(name=f"vc_{o['name']}", type=mujoco.mjtGeom.mjGEOM_BOX, size=o['half_extents'], pos=o['position'], quat=quat_wxyz(o['orientation_quat_xyzw']))
    tube = wb.add_body(name='tube', mocap=True)
    C = local_tube_centers(cj['tube'], TUBE_SEGMENTS)
    for i in range(TUBE_SEGMENTS):
        tube.add_geom(name=f'obstacle_tube_{i}', type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[cj['tube']['r'], 0, 0], fromto=np.concatenate([C[i], C[i + 1]]))
    fr = wb.add_frame(pos=[0, 0, 0])
    sp.attach(mujoco.MjSpec.from_string(convert_urdf('ur5', D / '_mesh')), prefix='A_', frame=fr)
    m = sp.compile()
    return m

def main():
    cj = json.loads((D / 'edge.json').read_text())
    m = build(cj)
    d = mujoco.MjData(m)
    adr = [m.joint(f'A_{j}').qposadr[0] for j in JOINT_NAMES]
    body_names = [m.body(i).name for i in range(m.nbody)]
    rg = set((i for i in range(m.ngeom) if body_names[m.geom_bodyid[i]].startswith('A_')))
    geom_names = [m.geom(i).name for i in range(m.ngeom)]
    tube_geometry_ids = {i for i,n in enumerate(geom_names) if n.startswith('obstacle_tube_')}
    static_geometry_ids = {i for i,n in enumerate(geom_names) if n.startswith('vc_') and i not in tube_geometry_ids}
    mid = m.body('tube').mocapid[0]
    total = json.loads((D / 'summary.json').read_text())
    for x in total:
        z = np.load(D / f"trial_{x['trial']:02d}.npz")
        X = z['tube_pose']
        for (name, Q) in (('reference', z['Qd']), ('track', z['Qt'])):
            static_contacts = np.zeros(len(Q), bool)
            tube_contacts = np.zeros(len(Q), bool)
            for i in range(len(Q)):
                d.qpos[adr] = Q[i]
                d.mocap_pos[mid] = X[i, :3]
                d.mocap_quat[mid] = tube_quaternion(X[i, 3], X[i, 4])
                mujoco.mj_forward(m, d)
                static_contacts[i],tube_contacts[i]=obstacle_contacts(
                    ((c.geom1,c.geom2) for c in d.contact[:d.ncon]),rg,static_geometry_ids,tube_geometry_ids)
            v = np.linalg.norm(np.gradient(z['Qd'].astype(float), z['t'], axis=0), axis=1) > 0.001
            collisions = static_contacts | tube_contacts
            clusters = lambda b: int(np.sum(b[1:] & ~b[:-1]) + (1 if b.size and b[0] else 0))
            x[name].update(mesh_collision_samples=int(collisions.sum()), mesh_collision_events=clusters(collisions), mesh_static_collision_events=clusters(static_contacts), mesh_tube_collision_events=clusters(tube_contacts), mesh_moving_collision_events=clusters(collisions & v), mesh_stopped_collision_events=clusters(collisions & ~v))
            np.save(D / f"trial_{x['trial']:02d}_mesh_{name}.npy", np.stack([static_contacts, tube_contacts]))
        print(f"trial {x['trial']}: mesh reference collisions {x['reference']['mesh_collision_events']} trial (tube {x['reference']['mesh_tube_collision_events']}) | track collisions {x['track']['mesh_collision_events']} trial (tube {x['track']['mesh_tube_collision_events']}, while stopped {x['track']['mesh_stopped_collision_events']})", flush=True)
        (D / 'summary.json').write_text(json.dumps(total, indent=1))
if __name__ == '__main__':
    main()
