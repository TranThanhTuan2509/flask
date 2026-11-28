# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2c

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-28_final_v2c/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.30 / 0.28 | 2.7 | 0.24 [0.21, 0.27] | 11.64 / 11.46 | **DIFFERENT_FASTER** |
| quad2d_bugtrap | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 39.29 / 34.47 | 45 | 1.87 [1.42, 2.34] | 10.98 / 11.67 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 125.15 / 107.56 | 388 | 0.69 [0.57, 0.84] | 10.78 / 11.46 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 6.68 / 6.01 | 909 | 0.02 [0.01, 0.02] | 10.96 / 11.5 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.17 / 0.06 | 0.4 | 0.90 [0.43, 1.49] | 5.09 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_hole | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.25 / 0.12 | 270 | 0.00 [0.00, 0.00] | 5.06 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 26.03 / 9.31 | 1813 | 0.03 [0.02, 0.04] | 4.72 / 5.14 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 3.74 / 2.19 | 2133 | 0.00 [0.00, 0.01] | 5.01 / 5.24 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.74 / 0.68 | 0.9 | 1.77 [1.58, 1.99] | 8.71 / 8.08 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.84 / 0.79 | 1.4 | 1.29 [1.10, 1.52] | 8.34 / 8.61 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 15.45 / 12.68 | 4752 | 0.01 [0.01, 0.01] | 8.45 / 9.56 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 10.60 / 9.24 | 7602 | 0.00 [0.00, 0.00] | 8.34 / 8.75 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.32 / 0.29 | 0.5 | 1.38 [1.20, 1.60] | 5.61 / 5.49 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_window | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.35 / 0.69 | 22.6 | 0.13 [0.08, 0.18] | 5.78 / 6.28 | **DIFFERENT_FASTER** |
| quad3d_window | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 8.11 / 7.63 | 629 | 0.03 [0.02, 0.03] | 5.69 / 6.96 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 5.06 / 4.63 | 2631 | 0.00 [0.00, 0.01] | 5.50 / 7.98 | **DIFFERENT_FASTER** |

