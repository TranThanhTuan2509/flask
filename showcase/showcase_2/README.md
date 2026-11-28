# Showcase 2: FLASK in MuJoCo, pick-and-place and multi-robot teams

Six scenes, each run with 10 seeds (`selection.json`). The run shown is the one whose total FLASK planning time is
closest to the median over the successful runs. Planning times are measured on one P-core of an i9-14900, with the
v2 profile (v2d) of the planners. Videos and raw logs stay local; the rest is in git:
- `stills/` (4K, ghosts) and `gif/` (GitHub GIFs and the montage: 3 rows x 2 columns);
- `<robot>/seeds/seed_<k>/` with `summary.json` and `plans.json` per run.

| Scene | What FLASK plans | Planning time per motion | 10 seeds |
|---|---|---|---|
| **UR5 + Robotiq 2F-85**: three cubes from tray A to tray B past a post, an arch, a bar and a pillar | every approach, descend, grasp-lift, carry, place and retreat (6-DoF, RRTConnect + Alg. 4, lazy-certified); the held cube is part of the collision model | 0.05–5.9 ms (median 0.15 ms) | 10/10: 3/3 cubes placed, no contact |
| **Fetch**: picks a carton on table A, drives across the warehouse, places it on table B | base (unicycle, BVP-SST\*, v2 profile) and arm (torso + 7 DoF, carton attached while held) | base 0.07–25 ms; arm 0.01–1.6 ms | 10/10: placed within 2–11 mm, upright, no contact |
| **UR5 + Franka Panda hand-over**: three bars, neither arm reaches the other's tray | each arm's motions, prioritized in space-time: an arm avoids the other's committed motion (swept VAMP spheres) | 0.06–39 ms (median 0.38 ms) | 10/10: 3/3 bars handed over and placed, arms never closer than 4.1 cm (sphere models) |
| **Three Skydio X2**: cross a hall through a two-window wall, two of them head-on | full quadrotor model (r = 4), FLASK-RRT forward tree, prioritized space-time planning with exact polynomial clearance | 3–374 ms per drone | 10/10: interval-certified, ≥ 10 cm surface gap planned, centres ≥ 1.01 m in flight, no contact |
| **Skydio X2**: three legs through a hall of windowed walls | full quadrotor model (r = 4), RRTConnect + Alg. 4 + L7, every edge interval-certified | 0.06–128 ms | 10/10: no contact, max tracking error 0.08–0.13 m |
| **Unitree G1**: five legs through a courtyard | base as a fully actuated point (r = 2); Unitree's RL walking policy tracks it | 0.04–9.8 ms | 10/10: no fall, no contact, max tracking error 0.14–0.22 m |

## Multi-robot planning: prioritized space-time planning

Both teams use prioritized planning (Erdmann and Lozano-Pérez 1987; revised prioritized planning, Čáp et al. 2015).
Robots plan one at a time, and each must avoid the motions already committed by the others on a common clock.

- **Drones** (`multidrone_swap.py`). Drones are planned in a fixed priority order; the declared retry rule tries up
  to two other orders, and none was needed in 10 seeds.
  - Drone k avoids the polynomial trajectories of drones 1..k−1 and the places where they then hover. The
    clearance check is exact: the squared distance between two polynomials is minimized at the interval ends and at
    the roots of its derivative (`PlannerSettings.moving_obstacles`, `src/flask/flat_native.hh`).
  - Drone k also avoids the start positions of drones k+1..n, which wait there.
  - This needs a tree whose nodes carry an absolute time. The drones therefore use FLASK-RRT on a single forward tree
    (`algorithm='rrt_forward'`): RRTConnect's extension, plus an exact BVP connection to the goal. This is beyond the
    paper's Table I variants. BVP-SST\* also works, but took seconds here.
  - The 1 kHz pairwise clearance is re-checked independently (`planning.space_time_clearance`).
- **Arms** (`manip.py`). Every motion is planned when it starts, against the other arm's committed motion: its VAMP
  spheres sampled along the rest of its trajectory (swept volume), its rest pose and the object it holds.
  - If that motion blocks the goal, the arm holds still and re-checks every 0.25 s (a microsecond validity probe,
    not a full plan).
  - Hand-overs are synchronized by events. The UR5 lets go only after the Panda's encoders report a grasp, and the
    Panda leaves only after the UR5 has moved clear.

## Grasping

- **UR5:** the fixed URDF Robotiq 85 of the planner's model is replaced, in simulation only, by the actuated MuJoCo
  Menagerie Robotiq 2F-85 on the same frame. A test checks that every vertex of its collision meshes lies inside the
  planner's VAMP spheres (within 1.5 cm).
- **Panda:** keeps its Menagerie fingers.
- **Fetch:** its fingers become prismatic joints with position servos.
- **How a grasp happens:** the fingers close on the object by contact. Once the finger encoders report a stall short
  of fully closed, a MuJoCo weld is switched on, anchored at the object's pose at that moment. It is switched off
  when the gripper opens. The weld is a simulation aid that keeps the object from slipping at speed.
- **Held objects in planning:** while an object is held, it is covered by spheres attached to the end effector
  (`Environment.attach`). Every manipulator check (FLASKCC, dense checks, certification) includes it.

## No privileged information in the loop

Controllers read only sensor models (`simulation/showcase/sensors.py`). Ground truth is used to score runs and to
anchor the weld.

| Robot | Sensing |
|---|---|
| UR5, Panda | joint encoders |
| Fetch | SLAM-like pose + gyro for the base; encoders for the arm; head-camera detection of the carton and of the place marker (4 mm noise) |
| Drones | motion capture (2 mm, 100 Hz, 10 ms latency) + gyro, per drone |
| G1 | IMU, AHRS, encoders, SLAM-like pose (2 cm, 30 Hz, 40 ms latency) |

The Fetch needs the camera detection because its localization error (about 2 cm) is as large as the finger
clearance around the carton.

## Showcase-specific choices (everything else is the library default)

- **Arms:** `rho = 10` (time weight of the LQMT cost, Eq. 20) for the UR5 and the Panda, so motions are brisker
  than with the default of 1 (peak acceleration 43–56 % of the limits). The Fetch arm keeps `rho = 1`, because its
  tight joint limits reject faster edges.
- **Planning padding (arms):** 1.5 cm, with certification at +1 mm. If certification rejects every path, the
  nominal padding is raised by 5 mm and then 10 mm; the certified scene is always the nominal one plus 1 mm.
- **Support surfaces:** the short vertical lift and lower moves ignore the support the held object leaves from or
  lands on (planned with straight-line IK). The Fetch places the carton from 2.5 cm above table B, with a 5 mm pad.
- **Fetch footprint:** the base planner uses a 0.42 m disk with the arm tucked and 0.45 m while carrying (the arm
  and carton reach 0.425 m; tested). Two warehouse columns are moved next to the walls so the aisles admit it.
- **Drone (single):** v2 profile with `rho = 10` (showcase_1 used 100). This lowers the tracking error from
  0.12–0.31 m to 0.08–0.13 m.
- **Drone team:** `rho = 100`, 10 cm drone–drone margin on top of 2 × 0.47 m.
- **G1:** each plan is time-scaled by 2.0 (showcase_1 used 1.4). This lowers the tracking error from 0.31–0.60 m to
  0.14–0.22 m.
- **Planning-time measurement:** one untimed warm-up call per process. GIF playback is sped up to about 12 s, and
  the factor is shown on screen in the full videos.
- **Simulation only.** Nothing here has been run on a physical robot.

## Reproduce

```bash
export FLASK_SHOWCASE=showcase_2
python -m simulation.showcase.ur5_pickplace --seed 3 --output out/ur5        # also fetch_pickplace, multiarm_handover,
                                                                            # multidrone_swap, drone_hall, g1_courtyard
taskset -c 2 python -m simulation.showcase.select_runs --seeds 10 --robots ur5   # one robot per core, then:
python -m simulation.showcase.select_runs --merge
python -m simulation.showcase.media            # 4K/1080p videos, 4K stills, GIFs, 3x2 montage
```
