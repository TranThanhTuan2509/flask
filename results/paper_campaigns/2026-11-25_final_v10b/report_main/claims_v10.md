# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10b

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-25_final_v10b/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 234.75 / 100.15 | 6 | 83.71 [52.69, 119.79] | 14.75 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 77.39 / 72.63 | 48.6 | 3.41 [3.02, 3.89] | 11.77 / 12.51 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 1572.85 / 1064.60 | 407 | 8.27 [5.55, 11.34] | 11.81 / 12.27 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 1015.34 / 676.42 | 565 | 3.85 [2.67, 5.27] | 11.81 / 12.25 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 20.04 / 10.34 | 0.6 | 71.46 [40.28, 110.09] | 10.33 / 4.82 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 11.38 / 11.32 | 11 | 2.21 [1.95, 2.47] | 9.04 / 5.24 | **DIFFERENT_LENGTH** |
| unicycle_wall | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 677.97 / 382.03 | 30 | 48.35 [29.02, 71.56] | 8.36 / 5.27 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 435.54 / 324.40 | 25 | 37.28 [26.26, 49.38] | 8.56 / 5.61 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.28 / 0.25 | 2.7 | 0.23 [0.19, 0.27] | 11.61 / 11.46 | **DIFFERENT_FASTER** |
| quad2d_bugtrap | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 47.07 / 56.52 | 45 | 2.24 [1.73, 2.72] | 10.92 / 11.67 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 141.18 / 103.41 | 388 | 0.78 [0.60, 0.98] | 11.09 / 11.46 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 5.81 / 5.74 | 909 | 0.01 [0.01, 0.02] | 10.74 / 11.5 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.16 / 0.07 | 0.4 | 0.83 [0.39, 1.60] | 5.34 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_hole | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.22 / 0.12 | 270 | 0.00 [0.00, 0.00] | 5.28 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 35.51 / 16.56 | 1813 | 0.04 [0.02, 0.08] | 4.71 / 5.14 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 2.87 / 1.88 | 2133 | 0.00 [0.00, 0.00] | 4.80 / 5.24 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.63 / 0.61 | 0.9 | 1.51 [1.37, 1.66] | 8.36 / 8.08 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.81 / 0.67 | 1.4 | 1.24 [1.01, 1.53] | 8.25 / 8.61 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 27.86 / 16.98 | 4752 | 0.01 [0.01, 0.02] | 8.37 / 9.56 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 10.54 / 7.01 | 7602 | 0.00 [0.00, 0.00] | 8.94 / 8.75 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.34 / 0.26 | 0.5 | 1.44 [1.18, 1.74] | 5.86 / 5.49 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_window | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.45 / 1.29 | 22.6 | 0.14 [0.10, 0.18] | 6.16 / 6.28 | **DIFFERENT_FASTER** |
| quad3d_window | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 10.41 / 7.99 | 629 | 0.04 [0.03, 0.04] | 5.95 / 6.96 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 5.26 / 4.71 | 2631 | 0.00 [0.00, 0.01] | 5.56 / 7.98 | **DIFFERENT_FASTER** |

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 979/1000 | 21 | 0.22 / 0.15 | 0.61 / 0.2 | 0.78 / 1.60 [1.45, 1.78] | 5.13 / 5.25 | **REPRODUCED** |
| bookshelf_tall | 988/1000 | 12 | 0.21 / 0.12 | 1.09 / 0.15 | 0.42 / 1.72 [1.56, 1.91] | 5.15 / 5.3 | **REPRODUCED** |
| bookshelf_small | 986/1000 | 14 | 0.35 / 0.14 | 7.8 / 0.17 | 0.10 / 1.72 [1.46, 2.06] | 5.27 / 5.5 | **DIFFERENT_FASTER** |
| cage | 981/1000 | 19 | 1.70 / 1.39 | 12.96 / 6.35 | 0.28 / 0.47 [0.42, 0.52] | 9.04 / 10.07 | **DIFFERENT_FASTER** |
| box | 994/1000 | 6 | 0.35 / 0.33 | 0.86 / 0.25 | 0.86 / 2.78 [2.57, 2.93] | 5.84 / 5.64 | **REPRODUCED** |
| table_under_pick | 987/1000 | 13 | 0.23 / 0.20 | 0.34 / 0.25 | 1.43 / 1.75 [1.65, 1.86] | 7.82 / 8.46 | **REPRODUCED** |
| table_pick | 983/990 | 7 | 0.11 / 0.09 | 0.28 / 0.12 | 0.82 / 1.62 [1.50, 1.70] | 4.97 / 5.24 | **REPRODUCED** |
| overall | 6898/6990 | 92 | 0.45 / 0.18 | 3.49 / 0.24 | 0.28 / 1.62 [1.54, 1.74] | 6.17 / 6.5 | **DIFFERENT_FASTER** |

