# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.476 (results/paper_campaigns/2026-11-28_final_v2/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 199.90 / 123.83 | 6 | 70.04 [46.01, 95.26] | 13.49 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 54.30 / 44.91 | 48.6 | 2.35 [1.99, 2.78] | 11.87 / 12.51 | **REPRODUCED** |
| unicycle_bugtrap | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 1536.81 / 1284.80 | 407 | 7.94 [5.65, 10.60] | 11.77 / 12.27 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | rrt_dp | PAPER_PROTOCOL | 29/30 | 29/30 | 1137.08 / 568.99 | 565 | 4.23 [2.71, 6.07] | 11.82 / 12.25 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 16.55 / 2.94 | 0.6 | 57.98 [20.20, 118.16] | 10.66 / 4.82 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 7.98 / 7.60 | 11 | 1.53 [1.35, 1.71] | 8.83 / 5.24 | **DIFFERENT_LENGTH** |
| unicycle_wall | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 869.02 / 372.50 | 30 | 60.89 [35.81, 88.76] | 8.29 / 5.27 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 482.26 / 241.56 | 25 | 40.55 [25.73, 59.18] | 8.60 / 5.61 | **DIFFERENT_SLOWER** |

