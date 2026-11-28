# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2b

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.483 (results/paper_campaigns/2026-11-28_final_v2b/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 391.82 / 99.41 | 6 | 135.30 [60.64, 241.62] | 15.34 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 74.65 / 66.53 | 48.6 | 3.18 [2.76, 3.70] | 11.88 / 12.51 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 1225.50 / 964.19 | 407 | 6.24 [4.37, 8.24] | 11.69 / 12.27 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | rrt_dp | PAPER_PROTOCOL | 29/30 | 29/30 | 1167.77 / 894.31 | 565 | 4.28 [2.74, 5.97] | 11.90 / 12.25 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 57.78 / 29.63 | 0.6 | 199.51 [116.60, 305.53] | 11.15 / 4.82 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 67.82 / 70.87 | 11 | 12.77 [10.95, 14.69] | 7.92 / 5.24 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 823.66 / 405.08 | 30 | 56.88 [36.97, 79.46] | 8.09 / 5.27 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 313.16 / 258.00 | 25 | 25.95 [20.71, 31.65] | 8.60 / 5.61 | **DIFFERENT_SLOWER** |

