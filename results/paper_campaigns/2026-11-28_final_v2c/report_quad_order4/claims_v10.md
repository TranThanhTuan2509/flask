# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2c

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-28_final_v2c/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| quad2d_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 5.88 / 2.44 | 2.7 | 4.67 [3.13, 6.64] | 12.62 / 11.46 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 2302.19 / 1742.50 | 45 | 109.61 [69.00, 161.97] | 11.46 / 11.67 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_simd | PAPER_PROTOCOL | 26/30 | 26/30 | 6119.33 / 6426.76 | 388 | 33.79 [26.40, 41.85] | 11.62 / 11.46 | **UNSOLVED_IN_BUDGET** |
| quad2d_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 394.51 / 348.88 | 909 | 0.93 [0.75, 1.11] | 11.80 / 11.5 | **REPRODUCED** |
| quad2d_hole | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.63 / 0.66 | 0.4 | 8.70 [4.37, 14.80] | 6.52 / 5.16 | **DIFFERENT_SLOWER** |
| quad2d_hole | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 8.81 / 2.06 | 270 | 0.07 [0.03, 0.12] | 5.80 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | PAPER_PROTOCOL | 22/30 | 22/30 | 11774.12 / 4367.42 | 1813 | 13.91 [6.93, 22.42] | 6.89 / 5.14 | **UNSOLVED_IN_BUDGET** |
| quad2d_hole | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 742.67 / 244.82 | 2133 | 0.75 [0.31, 1.31] | 6.38 / 5.24 | **DIFFERENT_LENGTH** |
| quad3d_block | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 2.64 / 2.31 | 0.9 | 6.29 [5.40, 7.46] | 9.47 / 8.08 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 7.97 / 1.75 | 1.4 | 12.20 [5.07, 22.43] | 8.67 / 8.61 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_simd | PAPER_PROTOCOL | 12/30 | 12/30 | 26977.43 / 23452.46 | 4752 | 12.16 [7.75, 16.63] | 9.64 / 9.56 | **UNSOLVED_IN_BUDGET** |
| quad3d_block | rrt_dp | PAPER_PROTOCOL | 27/30 | 27/30 | 16250.56 / 10210.56 | 7602 | 4.58 [3.24, 6.15] | 9.70 / 8.75 | **UNSOLVED_IN_BUDGET** |
| quad3d_window | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.33 / 0.97 | 0.5 | 5.71 [4.35, 7.21] | 5.66 / 5.49 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 124.98 / 82.14 | 22.6 | 11.85 [6.92, 17.13] | 5.97 / 6.28 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_simd | PAPER_PROTOCOL | 15/30 | 15/30 | 24105.46 / 24263.46 | 629 | 82.11 [60.53, 102.94] | 6.18 / 6.96 | **UNSOLVED_IN_BUDGET** |
| quad3d_window | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 5120.95 / 3943.73 | 2631 | 4.17 [2.97, 5.61] | 6.55 / 7.98 | **DIFFERENT_SLOWER** |

