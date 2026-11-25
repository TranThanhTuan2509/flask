"""Validate ur5 mesh: FLASK implementation and evaluation utilities."""
import argparse
import json
import os
from pathlib import Path
import sys
from hashlib import sha256
os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'simulation'))

def main(kind, source, output, stride):
    (source, output) = (Path(source).resolve(), Path(output))
    if output.exists():
        raise FileExistsError(output)
    if stride < 1:
        raise ValueError('stride must be positive')
    rows = []
    hashes = {}

    def load(path):
        hashes[str(path)] = sha256(path.read_bytes()).hexdigest()
        return np.load(path)
    if kind == 'e1':
        from render_benchmark import MeshScene
        scene = json.loads((source.parent / 'edge.json').read_text())
        checker = Edge('ur5', scene, source / '_mesh')
        trials = json.loads((source / 'summary.json').read_text())
        for x in trials:
            row = dict(trial=x['trial'])
            for (name, robot) in [('flask', 'A'), ('geometric', 'B')]:
                data = load(source / f"trial_{x['trial']:02d}_{name}.npz")
                metrics = {}
                for key in ['Qd', 'Qt']:
                    (hits, _) = checker.scan(robot, data[key][::stride])
                    metrics[key] = dict(samples=len(hits), colliding_samples=int(hits.sum()), collision=bool(hits.any()))
                row[name] = metrics
            rows.append(row)
            print(row, flush=True)
    else:
        import reactive_mesh
        reactive_mesh.D = source
        scene = json.loads((source / 'edge.json').read_text())
        model = reactive_mesh.build(scene)
        data = mujoco.MjData(model)
        address = [model.joint(f'A_{j}').qposadr[0] for j in reactive_mesh.JOINT_NAMES]
        names = [model.body(i).name for i in range(model.nbody)]
        robot_geoms = set((i for i in range(model.ngeom) if names[model.geom_bodyid[i]].startswith('A_')))
        obstacle_geoms = set((i for i in range(model.ngeom) if model.geom(i).name.startswith('vc_')))
        mocap = model.body('tube').mocapid[0]
        for path in sorted(source.glob('trial_*.npz')):
            z = load(path)
            row = dict(trial=int(path.stem[-2:]))
            X = z['tube_pose']
            for key in ['Qd', 'Qt']:
                Q = z[key]
                hits = []
                for i in range(0, len(Q), stride):
                    data.qpos[address] = Q[i]
                    x = X[i]
                    data.mocap_pos[mocap] = x[:3]
                    data.mocap_quat[mocap] = reactive_mesh.tube_quaternion(x[3], x[4])
                    mujoco.mj_forward(model, data)
                    hits.append(any((c.geom1 in robot_geoms and c.geom2 in obstacle_geoms or (c.geom2 in robot_geoms and c.geom1 in obstacle_geoms) for c in data.contact[:data.ncon])))
                row[key] = dict(samples=len(hits), colliding_samples=sum(hits), collision=any(hits))
            rows.append(row)
            print(row, flush=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(kind=kind, source=str(source), stride=stride, mujoco_version=mujoco.__version__, numpy_version=np.__version__, input_sha256=hashes, note='sampled convex mesh contacts; environment contacts only; not dynamics or continuous collision checking', rows=rows), indent=2))
if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('kind', choices=['e1', 'e2'])
    p.add_argument('source')
    p.add_argument('--output', required=True)
    p.add_argument('--stride', type=int, default=1)
    a = p.parse_args()
    main(a.kind, a.source, a.output, a.stride)
