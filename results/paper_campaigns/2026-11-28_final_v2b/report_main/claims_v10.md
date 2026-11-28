# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2b

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.483 (results/paper_campaigns/2026-11-28_final_v2b/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 4.39 / 4.35 | 6 | 1.52 [1.28, 1.75] | 11.94 / 12.39 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 4.24 / 4.06 | 48.6 | 0.18 [0.14, 0.23] | 11.56 / 12.51 | **DIFFERENT_FASTER** |
| unicycle_bugtrap | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 120.43 / 103.98 | 407 | 0.61 [0.44, 0.81] | 11.72 / 12.27 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 90.08 / 69.67 | 565 | 0.33 [0.23, 0.46] | 12.03 / 12.25 | **DIFFERENT_FASTER** |
| unicycle_wall | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 0.51 / 0.47 | 0.6 | 1.76 [1.46, 2.08] | 6.41 / 4.82 | **DIFFERENT_LENGTH** |
| unicycle_wall | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 0.28 / 0.22 | 11 | 0.05 [0.04, 0.07] | 4.72 / 5.24 | **DIFFERENT_FASTER** |
| unicycle_wall | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 34.38 / 22.09 | 30 | 2.37 [1.63, 3.25] | 5.84 / 5.27 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_wall | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 29.41 / 18.14 | 25 | 2.44 [1.51, 3.84] | 7.69 / 5.61 | **DIFFERENT_LENGTH** |

