"""Tracking mesh: FLASK implementation and evaluation utilities."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'simulation'))
from render_benchmark import MeshScene
D = ROOT / 'results/legacy' / (sys.argv[1] if len(sys.argv) > 1 else 'e1')
obstacles = json.loads((D / 'edge.json').read_text())
C = MeshScene('ur5', obstacles, D / '_mesh')
for directory in sorted((p for p in D.iterdir() if (p / 'summary.json').exists())):
    total = json.loads((directory / 'summary.json').read_text())
    for x in total:
        for (name, rb) in (('flask', 'A'), ('geometric', 'B')):
            z = np.load(directory / f"trial_{x['trial']:02d}_{name}.npz")
            (vt, _) = C.scan(rb, z['Qt'])
            (vd, _) = C.scan(rb, z['Qd'])
            x[name]['actual_mesh_collision_samples'] = int(vt.sum())
            x[name]['reference_mesh_collision_samples'] = int(vd.sum())
    (directory / 'summary.json').write_text(json.dumps(total, indent=1))
    n = len(total)
    tt = ' | '.join((f"{name}: mesh actual_state collisions {sum((x[name]['actual_mesh_collision_samples'] > 0 for x in total))}/{n} (reference_state {sum((x[name]['reference_mesh_collision_samples'] > 0 for x in total))}/{n}), sphere {sum((x[name]['actual_collision_samples'] > 0 for x in total))}/{n}" for name in ('flask', 'geometric')))
    print(f'{directory.name}: {tt}', flush=True)
