"""Validate mbm: FLASK implementation and evaluation utilities."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import pandas as pd
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flask_repro import mbm, native, trajectory, robot as R, reference as TC
from flask_repro.provenance import save_manifest

def main(csv, output):
    (csv, output) = (Path(csv), Path(output))
    if output.exists():
        raise FileExistsError(output)
    old = pd.read_csv(csv)
    manifest = json.loads(Path(str(csv) + '.manifest.json').read_text())['configuration']
    rb = R.get_robot(manifest['robot']).scaled_limits(manifest['limit_scale'])
    cfg = native.settings(velocity_sample_scale=manifest['velocity_sample_scale'], neighbor_mode=manifest['neighbor_mode'], max_ns=int(manifest['max_ms'] * 1000000.0), max_iterations=manifest['max_iterations'])
    witnesses = []
    dest = output.with_suffix('')
    dest.mkdir(parents=True, exist_ok=True)
    for row in old[old.collision == 1].itertuples():
        (_, _, s, goals, ev, ec) = next((x for x in mbm.paired_problems(rb.name, [row.env]) if x[1] == row.problem))
        result = native.plan(rb, ec, s, goals, cfg, seed=manifest['seed'] + row.problem)
        entry = dict(env=row.env, problem=row.problem, reconstructed=result['solved'])
        if result['solved']:
            path = result['trajectory']
            T = path.duration()
            size = trajectory.resample_count(T, path.length(), rb)
            t = np.linspace(0.0, T, size)
            q = path.evaluate(t)[0]
            mask = np.array([not native.module(rb).validate(ec, x.tolist()) for x in q])
            limits = native.joint_limits(rb, 'published_resolution')
            (cpp, py, bracket) = ([], [], [])
            for (i, edge) in enumerate(path.edges()):
                cpp.append(native.module(rb).flask_cc(ec, *[a.tolist() for a in (edge.q0, edge.v0, edge.q1, edge.v1)], edge.T, limits))
                py.append(TC.flask_cc(edge, rb, ev))
                N = TC.sample_count(edge, rb)
                colliding = t[mask] - path.t[i]
                for local in colliding[(colliding >= 0) & (colliding <= edge.T)]:
                    left = np.clip(int(np.floor(local / edge.T * N)), 0, N - 1)
                    (a, b) = (edge.T * left / N, edge.T * (left + 1) / N)
                    Q = edge.evaluate([a, b])[0]
                    bracket.append(dict(edge=i, local_time=float(local), interval=[a, b], endpoints_valid=[bool(rb.vamp.validate(v, ev)) for v in Q]))
                    break
            new_length = trajectory.sampled_length(q)
            entry.update(length_matches_original=bool(np.isclose(new_length, row.length_rad, rtol=1e-07)), reconstructed_length=new_length, original_length=row.length_rad, dense_collision_samples=int(mask.sum()), cpp_edges_valid=all(cpp), python_edges_valid=all(py), colliding_brackets=bracket, conclusion='sampled validators miss a colliding interior point' if all(cpp) and all(py) and mask.any() else 'requires further investigation')
            np.savez_compressed(dest / f'{row.env}_{row.problem}.npz', Q=path.Q, V=path.V, T=path.T, collision_times=t[mask], collision_configurations=q[mask])
        witnesses.append(entry)
    output.write_text(json.dumps(witnesses, indent=2))
    save_manifest(output, manifest, [csv, Path(str(csv) + '.manifest.json')])
if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    main(a.csv, a.output)
