"""V5 (acceptance checks; see "Verification" in README.md), Panda part: FLASK-RRTConnect on NEW random tabletop scenes.

Scene seed s (default 7001-7100): a table slab plus 4-10 random boxes/spheres in front of the robot; 5 start/goal
pairs of collision-free configurations per scene that geometric VAMP RRTConnect connects within 1 s (feasibility
certificate; problems it cannot solve are discarded before FLASK runs). FLASK runs with the v10 `rest` settings
(config/manipulator_v10_rest.json) in two modes: nominal (default validation profile, peak_speed) and
lazy-certified. Each returned trajectory is checked densely (1 kHz, at least 2000 samples) against the ORIGINAL
scene and analytically against position / velocity / acceleration limits.

Usage: python -m verification.v5_random_panda --output DIR [--seeds 7001 7100]
"""
import argparse
import json
import time
import traceback
from pathlib import Path

import numpy as np
import vamp

from flask_repro import certify, mbm, native, robot as R

ROOT = Path(__file__).resolve().parents[1]


def random_scene(rng):
    boxes = [dict(position=[.55, 0., -.05], orientation_euler_xyz=[0, 0, 0], half_extents=[.4, .7, .05])]  # table
    spheres = []
    for _ in range(int(rng.integers(4, 11))):
        pos = [float(rng.uniform(.3, .8)), float(rng.uniform(-.6, .6)), float(rng.uniform(.05, .8))]
        if rng.random() < .6:
            boxes.append(dict(position=pos, orientation_euler_xyz=[0., 0., float(rng.uniform(-np.pi, np.pi))],
                              half_extents=rng.uniform(.03, .15, 3).tolist()))
        else:
            spheres.append(dict(position=pos, radius=float(rng.uniform(.03, .12))))
    return dict(problem='random', sphere=spheres, cylinder=[], box=boxes)


def to_vamp(scene):
    env = vamp.Environment()
    for s in scene['sphere']:
        env.add_sphere(vamp.Sphere(s['position'], s['radius']))
    for b in scene['box']:
        env.add_cuboid(vamp.Cuboid(b['position'], b['orientation_euler_xyz'], b['half_extents']))
    return env


def check(rb, mod, env, d):
    T = d['trajectory']; duration = T.duration()
    n = max(2000, int(np.ceil(duration * 1000)))
    q, v, acc = T.evaluate(np.linspace(0., duration, n))[:3]
    collisions, _ = mod.count_collisions(env, q.tolist())
    excess = max(float(np.max(rb.q_lo - q)), float(np.max(q - rb.q_hi)), float(np.max(np.abs(v) - rb.v_max)),
                 float(np.max(np.abs(acc) - rb.a_max)), 0.)
    return dict(dense_collisions=int(collisions), limit_excess=excess, valid=bool(collisions == 0 and excess <= 1e-6))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seeds', type=int, nargs=2, default=[7001, 7100])
    p.add_argument('--problems', type=int, default=5)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    profile = json.loads((ROOT / 'config/manipulator_v10_rest.json').read_text())
    rb = R.get_robot('panda'); mod = native.module(rb); settings = native.settings(**profile['settings'])
    vplanner = vamp.configure_robot_and_planner_with_kwargs('panda', 'rrtc')
    vm, vplan, vsettings = vplanner[0], vplanner[1], vplanner[2]
    rows = []
    with (a.output / 'attempts.jsonl').open('w') as out:
        for seed in range(a.seeds[0], a.seeds[1] + 1):
            rng = np.random.default_rng(seed)
            scene = random_scene(rng)
            env = mbm.native_environment(scene); venv = to_vamp(scene)
            cert_env = mbm.native_environment(certify.inflate_problem(scene, native.CERTIFY_DELTA))
            sampler = vm.halton(); made = 0; tries = 0
            while made < a.problems and tries < 400:
                tries += 1
                s, g = rng.uniform(rb.q_lo, rb.q_hi), rng.uniform(rb.q_lo, rb.q_hi)
                if not (mod.validate(env, s.tolist()) and mod.validate(env, g.tolist())):
                    continue
                if not (mod.validate(cert_env, s.tolist()) and mod.validate(cert_env, g.tolist())):
                    continue
                sampler.reset()
                if not vplan(s.tolist(), g.tolist(), venv, vsettings, sampler).solved:
                    continue
                made += 1
                for mode in ['nominal', 'lazy_certified']:
                    row = dict(seed=seed, problem=made, mode=mode, objects=len(scene['box']) + len(scene['sphere']) - 1)
                    t0 = time.perf_counter()
                    try:
                        if mode == 'nominal':
                            d = native.plan(rb, env, s, [g], settings, seed=seed * 100 + made)
                        else:
                            d = native.plan_lazy_certified(rb, env, cert_env, s, [g], settings, seed=seed * 100 + made)
                        wall = time.perf_counter() - t0
                        row.update(solved=bool(d['solved']), plan_ms=(d.get('plan_ns', 0) + d.get('shortcut_ns', 0)) / 1e6,
                                   wall_s=wall, within_budget=bool(wall <= 1.05 * settings.max_ns / 1e9 * (20 if mode != 'nominal' else 1) + .5))
                        if d['solved']:
                            row.update(check(rb, mod, env, d))
                    except Exception as error:
                        row.update(solved=False, crashed=True, error=repr(error), trace=traceback.format_exc()[-1500:])
                    rows.append(row); out.write(json.dumps(row) + '\n'); out.flush()
    summary = dict(scenes=a.seeds[1] - a.seeds[0] + 1, problems=len(rows) // 2)
    for mode in ['nominal', 'lazy_certified']:
        r = [x for x in rows if x['mode'] == mode]; s = [x for x in r if x.get('solved')]
        summary[mode] = dict(attempts=len(r), solved=len(s), crashed=sum(bool(x.get('crashed')) for x in r),
                             over_budget=sum(not x.get('within_budget', True) for x in r),
                             returned_invalid=sum(not x['valid'] for x in s),
                             returned_colliding=sum(x['dense_collisions'] > 0 for x in s),
                             limit_excess_max=max([x['limit_excess'] for x in s], default=None),
                             median_ms=float(np.median([x['plan_ms'] for x in s])) if s else None,
                             p95_ms=float(np.percentile([x['plan_ms'] for x in s], 95)) if s else None)
    (a.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
