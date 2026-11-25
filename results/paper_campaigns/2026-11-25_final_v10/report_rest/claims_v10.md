# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.499 (results/paper_campaigns/2026-11-25_final_v10/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 226.45 / 129.51 | 6 | 75.65 [42.51, 117.41] | 13.66 / 12.39 | **UNCONTROLLED** |
| unicycle_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 83.65 / 80.83 | 48.6 | 3.45 [3.00, 3.97] | 11.97 / 12.51 | **UNCONTROLLED** |
| unicycle_bugtrap | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 1871.24 / 1143.70 | 407 | 9.22 [6.45, 12.41] | 11.76 / 12.27 | **UNCONTROLLED** |
| unicycle_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 968.73 / 622.82 | 565 | 3.44 [2.32, 4.66] | 11.74 / 12.25 | **UNCONTROLLED** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 20.70 / 4.93 | 0.6 | 69.14 [31.07, 116.75] | 10.79 / 4.82 | **UNCONTROLLED** |
| unicycle_wall | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 13.46 / 11.69 | 11 | 2.45 [2.01, 3.05] | 8.86 / 5.24 | **UNCONTROLLED** |
| unicycle_wall | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 511.85 / 278.31 | 30 | 34.20 [21.09, 51.60] | 8.81 / 5.27 | **UNCONTROLLED** |
| unicycle_wall | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 370.47 / 328.19 | 25 | 29.70 [23.38, 36.66] | 8.52 / 5.61 | **UNCONTROLLED** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.28 / 0.25 | 2.7 | 0.21 [0.17, 0.26] | 11.47 / 11.46 | **UNCONTROLLED** |
| quad2d_bugtrap | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 42.21 / 30.92 | 45 | 1.88 [1.41, 2.40] | 10.90 / 11.67 | **UNCONTROLLED** |
| quad2d_bugtrap | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 149.87 / 117.47 | 388 | 0.77 [0.60, 0.99] | 10.91 / 11.46 | **UNCONTROLLED** |
| quad2d_bugtrap | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 6.68 / 6.49 | 909 | 0.01 [0.01, 0.02] | 10.90 / 11.5 | **UNCONTROLLED** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.13 / 0.05 | 0.4 | 0.68 [0.37, 1.08] | 5.20 / 5.16 | **UNCONTROLLED** |
| quad2d_hole | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.14 / 0.11 | 270 | 0.00 [0.00, 0.00] | 5.10 / 4.93 | **UNCONTROLLED** |
| quad2d_hole | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 28.37 / 24.18 | 1813 | 0.03 [0.02, 0.04] | 4.67 / 5.14 | **UNCONTROLLED** |
| quad2d_hole | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 3.73 / 3.82 | 2133 | 0.00 [0.00, 0.00] | 4.98 / 5.24 | **UNCONTROLLED** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.78 / 0.74 | 0.9 | 1.73 [1.58, 1.89] | 8.63 / 8.08 | **UNCONTROLLED** |
| quad3d_block | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.03 / 0.88 | 1.4 | 1.47 [1.22, 1.74] | 8.15 / 8.61 | **UNCONTROLLED** |
| quad3d_block | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 22.81 / 17.12 | 4752 | 0.01 [0.01, 0.01] | 8.40 / 9.56 | **UNCONTROLLED** |
| quad3d_block | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 11.62 / 8.60 | 7602 | 0.00 [0.00, 0.00] | 8.44 / 8.75 | **UNCONTROLLED** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.32 / 0.32 | 0.5 | 1.28 [1.13, 1.44] | 5.74 / 5.49 | **UNCONTROLLED** |
| quad3d_window | sst_bvp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.12 / 0.82 | 22.6 | 0.10 [0.07, 0.13] | 6.11 / 6.28 | **UNCONTROLLED** |
| quad3d_window | sst_simd | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 10.29 / 9.75 | 629 | 0.03 [0.03, 0.04] | 5.73 / 6.96 | **UNCONTROLLED** |
| quad3d_window | rrt_dp | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 4.13 / 3.93 | 2631 | 0.00 [0.00, 0.00] | 5.60 / 7.98 | **UNCONTROLLED** |

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 977/1000 | 23 | 0.26 / 0.17 | 0.61 / 0.2 | 0.85 / 1.70 [1.57, 1.86] | 5.12 / 5.25 | **REPRODUCED** |
| bookshelf_tall | 987/1000 | 13 | 0.23 / 0.14 | 1.09 / 0.15 | 0.42 / 1.84 [1.64, 2.05] | 5.17 / 5.3 | **REPRODUCED** |
| bookshelf_small | 986/1000 | 14 | 0.34 / 0.15 | 7.8 / 0.17 | 0.09 / 1.74 [1.45, 2.16] | 5.28 / 5.5 | **DIFFERENT_FASTER** |
| cage | 981/1000 | 19 | 1.76 / 1.46 | 12.96 / 6.35 | 0.27 / 0.46 [0.43, 0.50] | 9.03 / 10.07 | **DIFFERENT_FASTER** |
| box | 998/1000 | 2 | 0.40 / 0.36 | 0.86 / 0.25 | 0.93 / 2.88 [2.72, 3.06] | 6.02 / 5.64 | **REPRODUCED** |
| table_under_pick | 988/1000 | 12 | 0.24 / 0.22 | 0.34 / 0.25 | 1.41 / 1.74 [1.63, 1.84] | 7.77 / 8.46 | **REPRODUCED** |
| table_pick | 990/990 | 0 | 0.12 / 0.10 | 0.28 / 0.12 | 0.87 / 1.66 [1.53, 1.79] | 4.97 / 5.24 | **REPRODUCED** |
| overall | 6907/6990 | 83 | 0.48 / 0.21 | 3.49 / 0.24 | 0.27 / 1.72 [1.60, 1.84] | 6.19 / 6.5 | **DIFFERENT_FASTER** |

