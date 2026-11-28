# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2d

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.490 (results/paper_campaigns/2026-11-28_final_v2d/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.29 / 0.26 | 2.7 | 0.22 [0.19, 0.27] | 11.57 / 11.46 | **DIFFERENT_FASTER** |
| quad2d_bugtrap | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 41.88 / 36.91 | 45 | 1.90 [1.52, 2.32] | 10.96 / 11.67 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 162.60 / 103.52 | 388 | 0.86 [0.57, 1.28] | 10.93 / 11.46 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 7.15 / 7.23 | 909 | 0.02 [0.01, 0.02] | 10.86 / 11.5 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.34 / 0.06 | 0.4 | 1.72 [0.68, 2.94] | 5.21 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_hole | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.18 / 0.09 | 270 | 0.00 [0.00, 0.00] | 5.01 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 18.95 / 12.39 | 1813 | 0.02 [0.01, 0.03] | 4.72 / 5.14 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 4.35 / 2.36 | 2133 | 0.00 [0.00, 0.01] | 5.13 / 5.24 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.75 / 0.73 | 0.9 | 1.69 [1.54, 1.86] | 8.54 / 8.08 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.19 / 0.95 | 1.4 | 1.74 [1.41, 2.14] | 8.17 / 8.61 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 21.07 / 14.97 | 4752 | 0.01 [0.01, 0.01] | 8.24 / 9.56 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 13.60 / 11.87 | 7602 | 0.00 [0.00, 0.00] | 8.60 / 8.75 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.36 / 0.33 | 0.5 | 1.47 [1.27, 1.70] | 5.82 / 5.49 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_window | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.09 / 0.98 | 22.6 | 0.10 [0.08, 0.12] | 5.90 / 6.28 | **DIFFERENT_FASTER** |
| quad3d_window | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 8.62 / 6.30 | 629 | 0.03 [0.02, 0.04] | 5.42 / 6.96 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 4.68 / 3.73 | 2631 | 0.00 [0.00, 0.00] | 5.56 / 7.98 | **DIFFERENT_FASTER** |

