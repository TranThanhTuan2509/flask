# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.476 (results/paper_campaigns/2026-11-28_final_v2/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 3.91 / 3.75 | 6 | 1.37 [1.17, 1.57] | 11.92 / 12.39 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_wall | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 0.27 / 0.21 | 0.6 | 0.95 [0.71, 1.23] | 7.50 / 4.82 | **DIFFERENT_LENGTH** |
| quad2d_bugtrap | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.25 / 0.24 | 2.7 | 0.20 [0.17, 0.22] | 11.64 / 11.46 | **DIFFERENT_FASTER** |
| quad2d_hole | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.13 / 0.05 | 0.4 | 0.67 [0.34, 1.09] | 5.09 / 5.16 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_block | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.69 / 0.64 | 0.9 | 1.62 [1.45, 1.82] | 8.71 / 8.08 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |
| quad3d_window | rrt_connect | REDUCED_ORDER_PROTOCOL | 30/30 | 0/30 | 0.29 / 0.27 | 0.5 | 1.24 [1.08, 1.44] | 5.61 / 5.49 | **REDUCED_ORDER_PROTOCOL:REPRODUCED** |

