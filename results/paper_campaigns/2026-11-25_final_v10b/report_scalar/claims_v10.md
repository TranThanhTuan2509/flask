# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10b

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-25_final_v10b/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 233.74 / 100.75 | 6 | 83.35 [52.55, 119.04] | 14.75 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 20.26 / 10.69 | 0.6 | 72.26 [41.18, 110.72] | 10.33 / 4.82 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.83 / 0.74 | 2.7 | 0.66 [0.57, 0.76] | 11.61 / 11.46 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.44 / 0.25 | 0.4 | 2.35 [1.32, 4.07] | 5.34 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 2.36 / 2.29 | 0.9 | 5.61 [5.04, 6.23] | 8.36 / 8.08 | **DIFFERENT_SLOWER** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 1.17 / 0.92 | 0.5 | 4.99 [4.16, 5.96] | 5.86 / 5.49 | **DIFFERENT_SLOWER** |

