# Declared before running (2026-11-25): Dynoplan control-duration propagation protocol (labelled variant)

Question: the paper does not state the propagation horizon for SIMD-SST*/DP-RRT (Eq. 29). v10 uses
propagation duration U[0, 0.3] s for every case. The baseline library the paper benchmarks against
(Dynoplan, benchmark/config/algs/sst_v0.yaml) fixes control durations per model:
  unicycle1_v0: min/max_control_duration 2/20 x dt 0.1  -> [0.2, 2.0] s
  quad2d_v0:    10/50 x propagation_step_size 0.01      -> [0.1, 0.5] s
  quad3d_v0:     5/50 x 0.01                            -> [0.05, 0.5] s
Single candidate, no selection: config/low_dof_v10_order2_dynoplan_prop.json = low_dof_v10_order2 with only
propagation_min/propagation_max changed per case. Pseudo-control bound (control_bound 2.0) unchanged.
Dev seeds 1001-1010, algorithms sst_simd and rrt_dp, all six cases, E-cores (development timing, not a claim).
Reported side by side with v10; it never replaces the primary protocol and is labelled VARIANT.
