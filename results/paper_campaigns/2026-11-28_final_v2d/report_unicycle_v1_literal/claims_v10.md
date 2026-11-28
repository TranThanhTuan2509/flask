# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2d

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.490 (results/paper_campaigns/2026-11-28_final_v2d/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 181.03 / 74.29 | 6 | 61.63 [34.72, 93.36] | 13.73 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 78.94 / 63.89 | 48.6 | 3.32 [2.77, 3.88] | 11.59 / 12.51 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_simd | PAPER_PROTOCOL | 29/30 | 29/30 | 1451.23 / 971.14 | 407 | 7.28 [4.95, 9.82] | 11.62 / 12.27 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 1331.17 / 1071.15 | 565 | 4.81 [3.49, 6.40] | 11.78 / 12.25 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 24.40 / 18.18 | 0.6 | 83.07 [51.12, 120.34] | 12.06 / 4.82 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 84.55 / 78.07 | 11 | 15.70 [13.86, 17.66] | 8.27 / 5.24 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 757.03 / 511.94 | 30 | 51.55 [36.33, 68.12] | 7.64 / 5.27 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 280.18 / 228.50 | 25 | 22.89 [19.32, 26.70] | 9.46 / 5.61 | **DIFFERENT_SLOWER** |

