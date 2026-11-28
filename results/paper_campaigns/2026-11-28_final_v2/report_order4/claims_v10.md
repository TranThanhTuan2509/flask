# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.476 (results/paper_campaigns/2026-11-28_final_v2/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| quad2d_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 4.21 / 1.79 | 2.7 | 3.28 [2.20, 4.66] | 12.62 / 11.46 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 2163.48 / 1623.57 | 45 | 101.07 [63.33, 149.99] | 11.46 / 11.67 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_simd | PAPER_PROTOCOL | 26/30 | 26/30 | 5940.76 / 6163.97 | 388 | 32.19 [25.09, 39.95] | 11.62 / 11.46 | **UNSOLVED_IN_BUDGET** |
| quad2d_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 343.86 / 302.13 | 909 | 0.80 [0.64, 0.95] | 11.80 / 11.5 | **REPRODUCED** |
| quad2d_hole | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.16 / 0.46 | 0.4 | 6.09 [3.03, 10.34] | 6.52 / 5.16 | **DIFFERENT_SLOWER** |
| quad2d_hole | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 8.08 / 1.56 | 270 | 0.06 [0.03, 0.11] | 5.80 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | PAPER_PROTOCOL | 22/30 | 22/30 | 11551.50 / 4245.43 | 1813 | 13.39 [6.62, 21.60] | 6.89 / 5.14 | **UNSOLVED_IN_BUDGET** |
| quad2d_hole | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 689.91 / 217.84 | 2133 | 0.68 [0.27, 1.21] | 6.38 / 5.24 | **DIFFERENT_LENGTH** |
| quad3d_block | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 2.34 / 2.05 | 0.9 | 5.47 [4.67, 6.52] | 9.47 / 8.08 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 7.40 / 1.45 | 1.4 | 11.11 [4.48, 20.65] | 8.67 / 8.61 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_simd | PAPER_PROTOCOL | 12/30 | 12/30 | 26587.18 / 23069.79 | 4752 | 11.76 [7.49, 16.08] | 9.64 / 9.56 | **UNSOLVED_IN_BUDGET** |
| quad3d_block | rrt_dp | PAPER_PROTOCOL | 27/30 | 27/30 | 14140.90 / 11865.10 | 7602 | 3.91 [2.82, 5.16] | 9.75 / 8.75 | **UNSOLVED_IN_BUDGET** |
| quad3d_window | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.11 / 0.79 | 0.5 | 4.66 [3.56, 5.84] | 5.66 / 5.49 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 125.47 / 83.35 | 22.6 | 11.67 [6.90, 16.73] | 5.97 / 6.28 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_simd | PAPER_PROTOCOL | 15/30 | 15/30 | 23809.37 / 24590.70 | 629 | 79.57 [58.57, 99.70] | 6.18 / 6.96 | **UNSOLVED_IN_BUDGET** |
| quad3d_window | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 4991.40 / 3833.42 | 2631 | 3.99 [2.83, 5.38] | 6.55 / 7.98 | **DIFFERENT_SLOWER** |

