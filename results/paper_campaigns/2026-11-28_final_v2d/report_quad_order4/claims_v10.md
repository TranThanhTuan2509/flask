# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2d

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.490 (results/paper_campaigns/2026-11-28_final_v2d/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| quad2d_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 7.18 / 5.39 | 2.7 | 5.43 [4.13, 6.81] | 10.73 / 11.46 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 2049.12 / 1377.58 | 45 | 93.02 [58.45, 133.96] | 10.62 / 11.67 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_simd | PAPER_PROTOCOL | 27/30 | 27/30 | 10924.81 / 6424.92 | 388 | 57.52 [40.29, 77.66] | 11.44 / 11.46 | **UNSOLVED_IN_BUDGET** |
| quad2d_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 375.76 / 336.85 | 909 | 0.84 [0.70, 1.01] | 11.38 / 11.5 | **REPRODUCED** |
| quad2d_hole | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.99 / 0.80 | 0.4 | 10.14 [5.81, 16.19] | 4.78 / 5.16 | **DIFFERENT_SLOWER** |
| quad2d_hole | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 13.35 / 1.67 | 270 | 0.10 [0.03, 0.22] | 4.66 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | PAPER_PROTOCOL | 20/30 | 20/30 | 9384.68 / 3794.14 | 1813 | 10.57 [4.49, 18.65] | 6.87 / 5.14 | **UNSOLVED_IN_BUDGET** |
| quad2d_hole | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 332.78 / 167.22 | 2133 | 0.32 [0.19, 0.51] | 6.00 / 5.24 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 3.28 / 2.98 | 0.9 | 7.44 [6.85, 8.12] | 7.56 / 8.08 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 11.76 / 4.32 | 1.4 | 17.16 [6.93, 31.89] | 7.54 / 8.61 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_simd | PAPER_PROTOCOL | 10/30 | 10/30 | 24484.44 / 23893.58 | 4752 | 10.53 [8.33, 12.68] | 9.65 / 9.56 | **UNSOLVED_IN_BUDGET** |
| quad3d_block | rrt_dp | PAPER_PROTOCOL | 29/30 | 29/30 | 13763.97 / 9912.01 | 7602 | 3.70 [2.72, 4.88] | 9.05 / 8.75 | **DIFFERENT_SLOWER** |
| quad3d_window | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.74 / 1.54 | 0.5 | 7.11 [6.15, 8.12] | 5.19 / 5.49 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 141.68 / 39.95 | 22.6 | 12.81 [4.06, 27.57] | 5.01 / 6.28 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_simd | PAPER_PROTOCOL | 15/30 | 15/30 | 26073.77 / 18033.58 | 629 | 84.68 [56.22, 114.94] | 6.26 / 6.96 | **UNSOLVED_IN_BUDGET** |
| quad3d_window | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 6592.79 / 5983.59 | 2631 | 5.12 [3.69, 6.54] | 6.05 / 7.98 | **DIFFERENT_SLOWER** |

