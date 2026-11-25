"""Generate an English report directly from stored research CSV/JSON results."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]

def build_report(directory):
    directory=Path(directory)
    lines=['# Historical reconstruction evidence','',
      'These archived numerical results were collected before the current native low-DOF implementation and physical showcase controllers. They are not a completed reproduction of the paper. Timings may include competing work; consult the original per-run manifests.','',
      '| Input | Attempts | Solved | Median total ms (successes) | Mean total ms (successes) | p95 ms (successes) | Colliding returned trajectories |',
      '|---|---:|---:|---:|---:|---:|---:|']
    for path in sorted(directory.glob('*.csv')):
        df=pd.read_csv(path)
        if not {'solved','total_ms'}<=set(df):continue
        successful=df[df.solved.astype(bool)];a=successful.total_ms.to_numpy()
        values=[f'{v:.4f}' for v in (np.median(a),np.mean(a),np.percentile(a,95))] if len(a) else ['N/A']*3
        contacts=str(int(successful.collision.sum())) if 'collision' in successful else 'Not measured'
        lines.append(f'| {path.name} | {len(df)} | {len(a)} | '+ ' | '.join(values)+f' | {contacts} |')
    lines+=['','The Panda source has 700 entries; 699 carry a valid flag. The excluded input is retained in the inventory. Exact correspondence with the author inputs remains unconfirmed. Success-conditioned timing does not imply that failed cases were solved.','',
       '## Between-sample collision witnesses','',
       '| Environment | Pair index | Colliding dense samples | Accepted by C++ | Accepted by Python |',
       '|---|---:|---:|---|---|']
    witness=directory/'mbm_collision_witnesses.json'
    if witness.exists():
        for r in json.loads(witness.read_text()):
            lines.append(f"| {r['env']} | {r['problem']} | {r['dense_collision_samples']} | {r['cpp_edges_valid']} | {r['python_edges_valid']} |")
    lines+=['','These are counterexamples to the sampled validator at its recorded resolution. Dense checking is also discrete and does not prove continuous collision freedom.','',
       '## Limits of the historical evidence','',
       '- The old low-DOF planner used Python search and flatness recovery; its timings cannot substantiate native FLASK speed claims.',
       '- Historical UR5 tracking used Ruckig kinematic integration and synthetic scenes. It does not establish physical robot execution.',
       '- Author sampling, boundary lifting, controller, stopping, and scene revision details remain unresolved.',
       '- Restricted zero-velocity manipulator sampling is a sensitivity setting, not an author-confirmed sampler.',
       '- Real-robot and GPU findings remain attributed paper references until independently measured under the appropriate conditions.','']
    return '\n'.join(lines)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--directory',type=Path,default=ROOT/'results/research');args=parser.parse_args()
    output=args.directory/'summary.md';output.write_text(build_report(args.directory));print(output)
if __name__=='__main__':main()
