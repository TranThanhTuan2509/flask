# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10b

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-25_final_v10b/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 234.65 / 100.02 | 6 | 83.68 [52.71, 119.76] | 14.75 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 19.78 / 10.19 | 0.6 | 70.54 [39.77, 108.61] | 10.33 / 4.82 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.28 / 0.25 | 2.7 | 0.22 [0.19, 0.26] | 11.61 / 11.46 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.15 / 0.07 | 0.4 | 0.82 [0.39, 1.59] | 5.34 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.63 / 0.61 | 0.9 | 1.51 [1.37, 1.66] | 8.36 / 8.08 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.34 / 0.26 | 0.5 | 1.44 [1.18, 1.75] | 5.86 / 5.49 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 979/1000 | 21 | 0.22 / 0.15 | 0.61 / 0.2 | 0.78 / 1.60 [1.47, 1.79] | 5.13 / 5.25 | **REPRODUCED** |
| bookshelf_tall | 988/1000 | 12 | 0.21 / 0.12 | 1.09 / 0.15 | 0.42 / 1.75 [1.60, 1.93] | 5.15 / 5.3 | **REPRODUCED** |
| bookshelf_small | 986/1000 | 14 | 0.35 / 0.14 | 7.8 / 0.17 | 0.10 / 1.72 [1.46, 2.05] | 5.27 / 5.5 | **DIFFERENT_FASTER** |
| cage | 981/1000 | 19 | 1.71 / 1.39 | 12.96 / 6.35 | 0.28 / 0.47 [0.42, 0.52] | 9.04 / 10.07 | **DIFFERENT_FASTER** |
| box | 994/1000 | 6 | 0.35 / 0.32 | 0.86 / 0.25 | 0.87 / 2.77 [2.61, 2.97] | 5.84 / 5.64 | **REPRODUCED** |
| table_under_pick | 987/1000 | 13 | 0.23 / 0.21 | 0.34 / 0.25 | 1.44 / 1.76 [1.68, 1.87] | 7.82 / 8.46 | **REPRODUCED** |
| table_pick | 983/990 | 7 | 0.11 / 0.09 | 0.28 / 0.12 | 0.83 / 1.64 [1.51, 1.72] | 4.97 / 5.24 | **REPRODUCED** |
| overall | 6898/6990 | 92 | 0.46 / 0.18 | 3.49 / 0.24 | 0.28 / 1.64 [1.55, 1.76] | 6.17 / 6.5 | **DIFFERENT_FASTER** |

