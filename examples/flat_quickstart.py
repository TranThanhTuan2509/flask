"""FLASK on the DynoBench systems (unicycle, 2D/3D quadrotor) with the settings behind the README tables.

    python examples/flat_quickstart.py unicycle_wall rrt_connect            # v2 settings (README headline)
    python examples/flat_quickstart.py unicycle_wall rrt_connect --v1       # the paper read literally, for comparison
    python examples/flat_quickstart.py quad3d_block sst_bvp --seeds 10      # full quadrotor model (r = 4)

Every returned path is checked independently on the ORIGINAL model and obstacles (dense |v|, |omega| / attitude and
collision checks). Times are on your CPU; the paper's are on an i7-6900K (README: ours / k_hw).
"""
import argparse
import json
from pathlib import Path

import numpy as np

from flask_repro.planning import plan, validate
from flask_repro.profiles import ALGORITHMS, CASES, load_profile

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument('case', nargs='?', default='unicycle_wall', choices=CASES)
p.add_argument('algorithm', nargs='?', default='rrt_connect', choices=ALGORITHMS)
p.add_argument('--v1', action='store_true', help='unicycle: the literal v1 settings instead of v2')
p.add_argument('--order2', action='store_true', help="quadrotors: the paper's reduced order r = 2 (not full-model feasible)")
p.add_argument('--seeds', type=int, default=5, help='number of seeds, starting at 1')
a = p.parse_args()

profile = 'v1' if a.v1 else 'order2' if a.order2 else 'v2'
s = load_profile(a.case, a.algorithm, profile)
print(f'{a.case} / {a.algorithm}  (profile {profile}: {s.profile_file})')
times, lengths = [], []
for seed in range(1, a.seeds + 1):
    r = plan(s.model, s.planning, s.start, s.goal, s.settings, seed)
    check = validate(r, s.model, s.original, s.settings)
    ms = (r.plan_ns + r.postprocess_ns) / 1e6
    length = r.trajectory.length(len(s.original.lower)) if r.solved else float('nan')
    print(f'  seed {seed}: solved={r.solved} valid={check["valid"]} time={ms:8.3f} ms length={length:5.2f} m '
          f'duration={r.trajectory.duration if r.solved else float("nan"):5.2f} s')
    times.append(ms); lengths.append(length)
paper = json.loads((Path(__file__).resolve().parents[1] / 'config/paper_reference_tables.json').read_text())['table_i']
ref = paper.get(a.case, {}).get(a.algorithm, {})
print(f'mean time {np.mean(times):.3f} ms, mean length {np.nanmean(lengths):.2f} m  |  paper: '
      f'{ref.get("planning_ms", "-")} ms (i7-6900K), {ref.get("length_m", "-")} m')
