# v10 claim evaluation: results/paper_campaigns/2026-11-24_final_v8

Criteria: `config/claim_criteria_v10.json`. k_hw = 1.067 (config/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | VARIANT | 30/30 | 0/30 | 0.63 / 0.52 | 6 | 0.10 [0.08, 0.12] | 11.43 / 12.39 | **VARIANT_NOT_PAPER_PROTOCOL** |
| unicycle_bugtrap | sst_bvp | VARIANT | 30/30 | 0/30 | 540.30 / 547.61 | 48.6 | 10.42 [9.86, 10.93] | 15.20 / 12.51 | **VARIANT_NOT_PAPER_PROTOCOL** |
| unicycle_bugtrap | sst_simd | VARIANT | 30/30 | 0/30 | 756.76 / 707.16 | 407 | 1.74 [1.60, 1.91] | 16.58 / 12.27 | **VARIANT_NOT_PAPER_PROTOCOL** |
| unicycle_bugtrap | rrt_dp | VARIANT | 30/30 | 0/30 | 431.04 / 431.40 | 565 | 0.72 [0.69, 0.75] | 18.09 / 12.25 | **VARIANT_NOT_PAPER_PROTOCOL** |
| unicycle_wall | rrt_connect | VARIANT | 30/30 | 0/30 | 0.08 / 0.08 | 0.6 | 0.12 [0.11, 0.14] | 3.90 / 4.82 | **VARIANT_NOT_PAPER_PROTOCOL** |
| unicycle_wall | sst_bvp | VARIANT | 30/30 | 0/30 | 30.53 / 26.73 | 11 | 2.60 [2.30, 2.95] | 4.11 / 5.24 | **VARIANT_NOT_PAPER_PROTOCOL** |
| unicycle_wall | sst_simd | VARIANT | 30/30 | 0/30 | 115.33 / 114.38 | 30 | 3.60 [3.08, 4.13] | 4.99 / 5.27 | **VARIANT_NOT_PAPER_PROTOCOL** |
| unicycle_wall | rrt_dp | VARIANT | 30/30 | 0/30 | 37.37 / 35.46 | 25 | 1.40 [1.29, 1.52] | 5.23 / 5.61 | **VARIANT_NOT_PAPER_PROTOCOL** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.56 / 0.51 | 2.7 | 0.19 [0.17, 0.22] | 11.27 / 11.46 | **DIFFERENT_FASTER** |
| quad2d_bugtrap | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 76.02 / 60.34 | 45 | 1.58 [1.18, 2.04] | 11.08 / 11.67 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 213.52 / 163.55 | 388 | 0.52 [0.39, 0.67] | 11.06 / 11.46 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_bugtrap | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 8.80 / 7.95 | 909 | 0.01 [0.01, 0.01] | 10.85 / 11.5 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.21 / 0.12 | 0.4 | 0.48 [0.29, 0.80] | 5.07 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_hole | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.31 / 0.16 | 270 | 0.00 [0.00, 0.00] | 5.15 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 51.24 / 35.71 | 1813 | 0.03 [0.02, 0.04] | 4.72 / 5.14 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 4.42 / 3.10 | 2133 | 0.00 [0.00, 0.00] | 4.81 / 5.24 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.09 / 1.03 | 0.9 | 1.14 [1.00, 1.29] | 8.58 / 8.08 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.08 / 0.90 | 1.4 | 0.73 [0.62, 0.86] | 8.24 / 8.61 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 34.69 / 26.18 | 4752 | 0.01 [0.00, 0.01] | 8.41 / 9.56 | **DIFFERENT_FASTER** |
| quad3d_block | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 20.25 / 17.86 | 7602 | 0.00 [0.00, 0.00] | 8.87 / 8.75 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.50 / 0.48 | 0.5 | 0.94 [0.80, 1.09] | 5.69 / 5.49 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_window | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.55 / 1.01 | 22.6 | 0.06 [0.04, 0.10] | 5.96 / 6.28 | **DIFFERENT_FASTER** |
| quad3d_window | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 12.86 / 11.59 | 629 | 0.02 [0.02, 0.02] | 5.64 / 6.96 | **DIFFERENT_FASTER** |
| quad3d_window | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 8.06 / 6.60 | 2631 | 0.00 [0.00, 0.00] | 5.59 / 7.98 | **DIFFERENT_FASTER** |

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 976/1000 | 24 | 0.37 / 0.19 | 0.61 / 0.2 | 0.57 / 0.88 [0.80, 0.98] | 5.66 / 5.25 | **UNCONTROLLED** |
| bookshelf_tall | 983/1000 | 17 | 0.48 / 0.17 | 1.09 / 0.15 | 0.42 / 1.03 [0.91, 1.17] | 5.62 / 5.3 | **UNCONTROLLED** |
| bookshelf_small | 984/1000 | 16 | 1.55 / 0.20 | 7.8 / 0.17 | 0.19 / 1.11 [0.96, 1.33] | 5.86 / 5.5 | **UNCONTROLLED** |
| cage | 982/1000 | 18 | 4.76 / 3.51 | 12.96 / 6.35 | 0.34 / 0.52 [0.47, 0.57] | 11.20 / 10.07 | **UNCONTROLLED** |
| box | 985/1000 | 15 | 0.41 / 0.35 | 0.86 / 0.25 | 0.44 / 1.30 [1.23, 1.38] | 7.13 / 5.64 | **UNCONTROLLED** |
| table_under_pick | 983/1000 | 17 | 0.27 / 0.24 | 0.34 / 0.25 | 0.75 / 0.88 [0.84, 0.93] | 9.09 / 8.46 | **UNCONTROLLED** |
| table_pick | 984/990 | 6 | 0.18 / 0.14 | 0.28 / 0.12 | 0.61 / 1.09 [1.01, 1.18] | 5.50 / 5.24 | **UNCONTROLLED** |
| overall | 6877/6990 | 113 | 1.15 / 0.24 | 3.49 / 0.24 | 0.31 / 0.93 [0.88, 1.00] | 7.15 / 6.5 | **UNCONTROLLED** |

