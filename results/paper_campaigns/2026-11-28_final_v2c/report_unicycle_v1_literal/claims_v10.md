# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2c

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-28_final_v2c/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 379.38 / 95.07 | 6 | 135.48 [60.56, 242.86] | 15.34 / 12.39 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 71.91 / 62.83 | 48.6 | 3.17 [2.75, 3.69] | 11.88 / 12.51 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 1215.56 / 961.14 | 407 | 6.40 [4.51, 8.45] | 11.69 / 12.27 | **DIFFERENT_SLOWER** |
| unicycle_bugtrap | rrt_dp | PAPER_PROTOCOL | 29/30 | 29/30 | 1165.24 / 889.78 | 565 | 4.42 [2.84, 6.15] | 11.90 / 12.25 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 56.03 / 28.94 | 0.6 | 200.08 [117.29, 306.85] | 11.15 / 4.82 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 67.17 / 71.55 | 11 | 13.08 [11.25, 15.03] | 7.92 / 5.24 | **DIFFERENT_SLOWER** |
| unicycle_wall | sst_simd | PAPER_PROTOCOL | 30/30 | 30/30 | 809.27 / 402.90 | 30 | 57.80 [37.49, 81.21] | 8.09 / 5.27 | **DIFFERENT_SLOWER** |
| unicycle_wall | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 305.83 / 254.80 | 25 | 26.21 [21.00, 31.86] | 8.60 / 5.61 | **DIFFERENT_SLOWER** |

