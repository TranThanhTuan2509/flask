# Changelog

## 2.1.0 (2026-11-29): showcase 2

**New showcases** (`showcase/showcase_2`, 10 seeds each, all 10/10):
- UR5 + Robotiq 2F-85 sorts three cubes;
- Fetch carries a carton between two tables;
- UR5 + Franka Panda hand three bars over;
- three Skydio X2 quadrotors share two windows;
- G1 and the single drone re-run with the v2 profiles.

The README header is now a 3x2 montage.

**Library additions** (off by default; with them off, results are bit-identical, `tests/test_v2_levers.py`,
`tests/test_profiles.py`):
- `Environment.attach(spheres, tf)` / `detach()`: a held object in every manipulator check (FLASKCC,
  `count_collisions`, `validate`), using VAMP's attachments. Also `eefk` / `sphere_fk` for UR5, Panda and Fetch.
- `PlannerSettings.moving_obstacles` / `moving_margin`: prioritized space-time planning for flat robots.
  - Other robots' polynomial trajectories are moving spheres, checked exactly (polynomial squared distance) on every
    edge of a forward tree, at the goal hold, and in the Alg. 4 / L7 shortcuts of the re-timed path.
  - Helpers `planning.moving_pieces`, `hold_piece` and `space_time_clearance` (an independent 1 kHz check).
- `algorithm='rrt_forward'`: FLASK-RRT on a single forward tree (RRTConnect's extension plus an exact BVP connection
  to the goal), so that every node has an absolute time. This is beyond the paper's Table I variants.

**Showcase changes:**
- G1 time scaling 1.4 → 2.0: max tracking error 0.31–0.60 → 0.14–0.22 m.
- Single drone: v2 profile with `rho = 10`: max tracking error 0.12–0.31 → 0.08–0.13 m.
- `sensors.detect_relative`: camera detection of objects (4 mm noise) for the Fetch grasp and place.

**Tests:** 568, including attachment, moving-obstacle and `rrt_forward` controls, and showcase contracts: the 2F-85
lies inside the planner's spheres, grasp poses are reachable, drone starts and goals are clear, and the Fetch carry
footprint is respected.


## 2.0.0 (2026-11-28)

The paper is silent on several unicycle choices. v1 read them literally, and every unicycle row of Table I came
out 3–200× slower than published. v2 adds options for these unspecified parts and corrects one protocol choice.
Each change has a measured cause (README, "What v2 changes and why").

**Planner options.** All are off by default, and with all of them off the paths match v1 bit for bit.
- `parent_candidates` (L1): extend from the cheapest of the k metric-nearest nodes, using the LQMT cost (Eq. 24).
- `duration_multipliers` (L3): when T\* violates a dynamic limit, retry with the fixed-time BVP at c × T\*. The same
  retry is used in the Alg. 4 shortcut.
- Unicycle propagation settings for SIMD-SST\* and DP-RRT: pseudo-control bound 1.0 (L5) and Dynoplan's
  durations of 0.2–2 s (L6). These are profile settings only.
- `shortcut_subdivisions`, `shortcut_rounds` (L7, beyond the paper): after the paper's Alg. 4 pass, further rounds
  of the same shortcut over edges split at interior times; a bypass is accepted only if it lowers the LQMT cost.
  The search is unchanged, and 1 / 1 (the default) is the paper's single pass. It removes the far-node detours that
  Alg. 4 cannot (Wall RRTConnect, held-out: 6.43 → 3.63 m mean). Selected by `config/v2_shortcut_rule.json` on
  development seeds 1101–1130 (`results/diagnostics/v2_shortcut`).

**Protocol.** On the unicycle Wall map the robot now drives forward (κ = 0, Eq. 10), as in the paper's Fig. 5a.
Bugtrap stays backward.

**New in the API**
- `flask_repro.profiles.load_profile(case, algorithm, profile)` returns a DynoBench problem with the settings
  behind the README tables (`v2`, `v1`, `order2`).
- `examples/flat_quickstart.py` uses these profiles, with `--v1` and `--order2` switches.

**Results**
- Headline campaign `results/paper_campaigns/2026-11-28_final_v2d` (v2 with L7, every Table I row on the new
  held-out seeds 8101–8130). Unicycle: all 240 paths valid; four rows inside the band in time and length (two with
  a 95% interval crossing the band edge), BVP-SST\* faster on both maps, Wall RRTConnect in band in time with paths
  25% shorter than the paper's, Wall DP-RRT just above the band (3.14 [2.01, 4.48]). Quadrotors r = 4: 431/431
  solved paths interval-certified.
- Without L7 (release campaign `_v2c`, seeds 8001–8030): every unicycle row inside the time band or faster; Wall
  RRTConnect and DP-RRT paths 33–37% longer than the paper's.
- Quadrotors: search unchanged in method; r = 4 RRTConnect and BVP-SST\* use L7.
- Panda (Table II): unchanged from v1.
- New studies: `results/diagnostics/v2_mechanism`, `v2_factorial` and `v2_chart`. New campaigns:
  `results/paper_campaigns/2026-11-28_final_v2`, `_v2b`, the release campaign `_v2c` and `_v2d` (with L7).
- Panda default validation: the 92/6990 paths that fail the dense check touch obstacles for a median of 20 ms, at
  most 2.1 mm deep (`results/diagnostics/panda_default_contacts`). The README now warns about this.

**Correctness fixes found while testing v2**
- *Exact workspace bounds* (`PlannerSettings.exact_bounds`, on by default). Edges were checked against the
  workspace bounds at sampled points with 1e-5 m slack, so an edge could graze the boundary by up to ~15 µm. The
  generalization check (below) found two such paths, both from v1 settings. Each polynomial edge is now checked at its
  exact extrema (endpoints and roots of the derivative), with no slack. Effect on the release campaign: 49/480
  order-4 quadrotor attempts changed (none changed between solved and unsolved); the unicycle attempts are
  identical.
- *Fail-fast inputs.* `planning.plan` returns `reason='invalid_start'` / `'invalid_goal'` at once for a start or goal
  in collision or out of bounds (checked on the exact obstacles, not only the sphere cover), and rejects NaN/inf
  inputs. `native.plan` reports the same reasons for manipulators.
- *Native build errors.* A missing native extension, or one built for another Python, now raises an `ImportError`
  that names the interpreter and the fix. Before, plans failed with `'NoneType' object has no attribute ...`.
- *Labels.* A Table I row that fails only by running out of time (no invalid path returned) is now reported as
  `UNSOLVED_IN_BUDGET` instead of `DIFFERENT_INVALID`.

**Generalization check.** v1 and v2 settings on problems never used to choose a setting: 3 unseen DynoBench
unicycle maps and 100 random scenes, both driving directions, all four planners, with a pass rule declared before
running (`config/v2_generalization_rule.json`). GENERALIZES both without L7
(`results/diagnostics/v2_generalization_release`) and with it (`v2_generalization_v2d`): 1520/1520 solved, 0 invalid.

**Release campaigns** `results/paper_campaigns/2026-11-28_final_v2c` (without L7) and `_v2d` (with L7, the README
tables) measure every Table I row with the released binary, under the same controls. `load_profile(..., 'v2')`
now returns the v2d settings.

**Tests.** 552 tests (L7: `tests/test_shortcut_l7.py`). `tests/test_profiles.py` re-runs the unicycle rows from a clone. A CI workflow
(`.github/workflows/tests.yml`) installs from scratch and runs them; it has not yet run on GitHub.

**Docs.** The README points to OMPL, whose main branch now contains the authors' flat code (commit `e288901`,
its 32 flat files identical to our pinned `ompl-flask`), and states what this repository adds.

**Unchanged.** The MuJoCo showcases still use v1 planner settings.

## 1.1.0 (2026-11-25)

MuJoCo showcases (Unitree G1, Skydio X2, UR5, Fetch), README figures, release export.

## 1.0.0 (2026-11-25)

Reproduction of Tables I and II and the UR5 experiments, with an evaluation protocol fixed before measuring.
