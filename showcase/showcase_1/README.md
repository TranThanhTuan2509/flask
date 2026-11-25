# Showcase 1: FLASK in MuJoCo (Unitree G1, Skydio X2, UR5, Fetch)

Contents: `stills/` (4K, ghosts), `gif/` (GitHub GIFs + 2x2 montage), `flask_showcase_2x2.mp4`,
`<robot>/seeds/seed_<k>/` (10 runs per robot: `summary.json`, `plans.json`; the selected run also has
`media/master.mp4` (4K/30) and `media/delivery.mp4` (1080p/60)). `selection.json` records the selection.
Videos and raw logs are kept locally (not in git). A new campaign: `FLASK_SHOWCASE=showcase_2 python -m ...`.

| Robot | What FLASK plans | Planning time per motion (1 P-core) | 10 seeds |
|---|---|---|---|
| Unitree G1 | base as a fully actuated point (r = 2, native); Unitree's RL walking policy tracks it | 34–610 µs (+1–55 µs smoothing) | 10/10 no contact, no fall |
| Skydio X2 | full quadrotor model (r = 4), RRTConnect + Alg. 4, every edge interval-certified | 0.05–71 ms | 10/10 no contact |
| UR5 + Robotiq | 6-DoF arm, RRTConnect + Alg. 4, then lazy certification | 16–350 µs (+0.2–1.5 ms certification) | 10/10 no contact |
| Fetch | arm (torso + 7 DoF, native) and diff-drive base (unicycle, BVP-SST*: first solution + Alg. 4, executed as planned) | arm 13–650 µs; base 4–420 ms | 10/10 no contact |

## No privileged information in the loop

Controllers and planners read only what a real robot measures (`simulation/showcase/sensors.py`); ground truth is
used only to *score* runs (contacts, error vs. truth).

| Robot | Measurements used by the controller | Noise / rate / latency |
|---|---|---|
| G1 | gyro (policy), AHRS attitude (policy), joint encoders, SLAM-like pose + gyro (base tracking) | gyro 0.005 rad/s + bias; attitude 0.5 deg; encoders 17 bit; pose 2 cm / 0.5 deg, 30 Hz, 40 ms |
| Skydio X2 | motion-capture pose, gyro; velocity from an alpha-beta filter | 2 mm / 0.3 deg, 100 Hz, 10 ms; gyro as above |
| Fetch | SLAM/AMCL-like pose + gyro (base), joint encoders (arm) | pose 2 cm / 0.5 deg, 30 Hz, 40 ms |
| UR5 | joint encoders (position and velocity) | as delivered by the arm's servo interface |

Measured over the 10 runs: G1 localization error up to 6–8 cm, drone state-estimate error up to 1.2 cm, Fetch up
to about 6 cm / 3 deg; mean G1 tracking error 6–9 cm.

The obstacle map is known (a workcell model / prior map), as in the paper. Plans start from the previously
commanded goal; the feedback controller absorbs the measured offset.

## Safety margins come from the robots' own geometry

Planning radii are derived from the simulated collision geometry and checked by tests
(`tests/test_showcase_contracts.py::test_planning_radius_covers_the_simulated_robot`):
- **Drone:** 0.47 m. The X2 rotor discs reach 0.372 m.
- **G1:** 0.55 m. The fixed arms reach 0.39 m from the pelvis axis.
- **Fetch arm:** obstacles are padded by 4 cm for localization and servo error.

**Fetch collision geometry in MuJoCo.** MuJoCo uses VAMP's sphere model of the Fetch for collisions. The convex
hulls of the URDF meshes over-fill the non-convex gripper and forearm and report contacts the real links do not
have. The meshes are still rendered.

## Showcase-specific choices (everything else is the library default)

- **Drone:** `rho = 100` (the benchmark uses 1) for brisker flight. Planning runs against exact boxes, which is
  about 6x faster than the sphere cover and gives identical validity.
- **G1:** each plan is uniformly time-scaled by 1.4 (same path) for command headroom. Commands are kept inside the
  policy's robust envelope and slew-limited; without this, 2/10 runs fell.
- **Fetch base:** the unicycle model is required for a diff-drive base and is the slowest FLASK case.
- **Planning-time measurement:** one untimed warm-up call per process. The shown run is the one closest to the
  median total planning time over 10 seeds.
- **Playback speed-ups** are written on screen.
- **Simulation only.** Nothing here has been run on a physical robot.
