# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.476 (results/paper_campaigns/2026-11-28_final_v2/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 3.97 / 3.75 | 6 | 1.39 [1.19, 1.59] | 11.92 / 12.39 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 4.29 / 3.21 | 48.6 | 0.19 [0.13, 0.25] | 11.58 / 12.51 | **DIFFERENT_FASTER** |
| unicycle_bugtrap | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 96.62 / 55.05 | 407 | 0.50 [0.34, 0.69] | 11.53 / 12.27 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 125.31 / 87.18 | 565 | 0.47 [0.32, 0.64] | 12.95 / 12.25 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_wall | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 0.29 / 0.22 | 0.6 | 1.02 [0.77, 1.33] | 7.50 / 4.82 | **DIFFERENT_LENGTH** |
| unicycle_wall | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 0.09 / 0.08 | 11 | 0.02 [0.01, 0.02] | 7.50 / 5.24 | **DIFFERENT_FASTER** |
| unicycle_wall | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 36.76 / 27.93 | 30 | 2.58 [1.76, 3.61] | 7.88 / 5.27 | **DIFFERENT_LENGTH** |
| unicycle_wall | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 51.31 / 29.12 | 25 | 4.31 [2.89, 6.01] | 8.46 / 5.61 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.27 / 0.25 | 2.7 | 0.21 [0.18, 0.23] | 11.64 / 11.46 | **DIFFERENT_FASTER** |
| quad2d_bugtrap | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 39.27 / 34.86 | 45 | 1.83 [1.39, 2.30] | 10.99 / 11.67 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 131.72 / 113.82 | 388 | 0.71 [0.59, 0.87] | 10.78 / 11.46 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 6.33 / 5.84 | 909 | 0.01 [0.01, 0.02] | 10.96 / 11.5 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.14 / 0.06 | 0.4 | 0.73 [0.37, 1.19] | 5.09 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_hole | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.21 / 0.10 | 270 | 0.00 [0.00, 0.00] | 5.06 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 26.84 / 9.57 | 1813 | 0.03 [0.02, 0.05] | 4.72 / 5.14 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 3.46 / 2.02 | 2133 | 0.00 [0.00, 0.00] | 5.01 / 5.24 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.73 / 0.67 | 0.9 | 1.71 [1.53, 1.92] | 8.71 / 8.08 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.79 / 0.72 | 1.4 | 1.18 [1.01, 1.39] | 8.34 / 8.61 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 16.12 / 13.35 | 4752 | 0.01 [0.01, 0.01] | 8.45 / 9.56 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 10.83 / 9.28 | 7602 | 0.00 [0.00, 0.00] | 8.34 / 8.75 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.31 / 0.29 | 0.5 | 1.32 [1.15, 1.52] | 5.61 / 5.49 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_window | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.29 / 0.66 | 22.6 | 0.12 [0.08, 0.17] | 5.78 / 6.28 | **DIFFERENT_FASTER** |
| quad3d_window | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 8.26 / 7.92 | 629 | 0.03 [0.02, 0.03] | 5.67 / 6.96 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 5.08 / 4.59 | 2631 | 0.00 [0.00, 0.01] | 5.50 / 7.98 | **DIFFERENT_FASTER** |

