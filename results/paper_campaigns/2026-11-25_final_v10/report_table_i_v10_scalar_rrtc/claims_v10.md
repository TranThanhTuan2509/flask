# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.499 (results/paper_campaigns/2026-11-25_final_v10/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 202.81 / 115.04 | 6 | 67.75 [38.13, 105.22] | 13.66 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 20.52 / 5.16 | 0.6 | 68.55 [30.91, 114.90] | 10.79 / 4.82 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.81 / 0.72 | 2.7 | 0.60 [0.52, 0.70] | 11.47 / 11.46 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.37 / 0.17 | 0.4 | 1.85 [1.14, 2.78] | 5.20 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 2.79 / 2.79 | 0.9 | 6.20 [5.62, 6.77] | 8.63 / 8.08 | **DIFFERENT_SLOWER** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.05 / 1.02 | 0.5 | 4.19 [3.75, 4.68] | 5.74 / 5.49 | **DIFFERENT_SLOWER** |

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 977/1000 | 23 | 0.24 / 0.15 | 0.61 / 0.2 | 0.78 / 1.54 [1.41, 1.71] | 5.12 / 5.25 | **REPRODUCED** |
| bookshelf_tall | 987/1000 | 13 | 0.21 / 0.12 | 1.09 / 0.15 | 0.39 / 1.67 [1.50, 1.86] | 5.17 / 5.3 | **REPRODUCED** |
| bookshelf_small | 986/1000 | 14 | 0.33 / 0.14 | 7.8 / 0.17 | 0.09 / 1.64 [1.35, 2.02] | 5.28 / 5.5 | **DIFFERENT_FASTER** |
| cage | 981/1000 | 19 | 1.68 / 1.39 | 12.96 / 6.35 | 0.26 / 0.44 [0.41, 0.47] | 9.03 / 10.07 | **DIFFERENT_FASTER** |
| box | 998/1000 | 2 | 0.38 / 0.34 | 0.86 / 0.25 | 0.88 / 2.71 [2.56, 2.91] | 6.02 / 5.64 | **REPRODUCED** |
| table_under_pick | 988/1000 | 12 | 0.22 / 0.20 | 0.34 / 0.25 | 1.32 / 1.63 [1.53, 1.72] | 7.77 / 8.46 | **REPRODUCED** |
| table_pick | 990/990 | 0 | 0.11 / 0.09 | 0.28 / 0.12 | 0.80 / 1.50 [1.39, 1.62] | 4.97 / 5.24 | **REPRODUCED** |
| overall | 6907/6990 | 83 | 0.45 / 0.19 | 3.49 / 0.24 | 0.26 / 1.59 [1.49, 1.71] | 6.19 / 6.5 | **DIFFERENT_FASTER** |

