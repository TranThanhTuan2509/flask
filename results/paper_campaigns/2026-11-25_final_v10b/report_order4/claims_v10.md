# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10b

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-25_final_v10b/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| quad2d_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 3.74 / 2.26 | 2.7 | 2.96 [2.03, 4.11] | 13.13 / 11.46 | **REPRODUCED** |
| quad2d_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 3541.49 / 1647.65 | 45 | 168.39 [88.29, 270.15] | 11.57 / 11.67 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_simd | PAPER_PROTOCOL | 24/30 | 24/30 | 12730.56 / 11380.67 | 388 | 70.20 [51.98, 90.98] | 11.93 / 11.46 | **DIFFERENT_INVALID** |
| quad2d_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 307.29 / 297.32 | 909 | 0.72 [0.58, 0.88] | 11.35 / 11.5 | **REPRODUCED** |
| quad2d_hole | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 2.92 / 1.39 | 0.4 | 15.60 [8.23, 25.55] | 6.58 / 5.16 | **DIFFERENT_SLOWER** |
| quad2d_hole | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 7.37 / 2.14 | 270 | 0.06 [0.03, 0.09] | 5.87 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | PAPER_PROTOCOL | 25/30 | 25/30 | 9231.29 / 4123.89 | 1813 | 10.89 [6.51, 15.93] | 7.47 / 5.14 | **DIFFERENT_INVALID** |
| quad2d_hole | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 389.83 / 151.01 | 2133 | 0.39 [0.20, 0.63] | 6.44 / 5.24 | **DIFFERENT_LENGTH** |
| quad3d_block | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 2.30 / 2.17 | 0.9 | 5.47 [4.49, 6.56] | 8.41 / 8.08 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 12.18 / 2.18 | 1.4 | 18.62 [8.06, 31.84] | 8.70 / 8.61 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_simd | PAPER_PROTOCOL | 18/30 | 18/30 | 27551.46 / 25622.79 | 4752 | 12.41 [8.97, 16.25] | 8.76 / 9.56 | **DIFFERENT_INVALID** |
| quad3d_block | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 13927.99 / 9653.58 | 7602 | 3.92 [2.84, 5.18] | 9.48 / 8.75 | **DIFFERENT_SLOWER** |
| quad3d_window | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.27 / 1.17 | 0.5 | 5.45 [4.31, 6.80] | 5.64 / 5.49 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 195.97 / 32.33 | 22.6 | 18.55 [6.59, 36.83] | 6.23 / 6.28 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_simd | PAPER_PROTOCOL | 15/30 | 15/30 | 21986.33 / 18807.89 | 629 | 74.79 [46.95, 106.23] | 6.70 / 6.96 | **DIFFERENT_INVALID** |
| quad3d_window | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 6723.69 / 5492.57 | 2631 | 5.47 [3.95, 7.26] | 6.33 / 7.98 | **DIFFERENT_SLOWER** |

