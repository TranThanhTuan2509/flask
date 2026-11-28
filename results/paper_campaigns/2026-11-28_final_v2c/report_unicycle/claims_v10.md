# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2c

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.467 (results/paper_campaigns/2026-11-28_final_v2c/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 4.40 / 4.36 | 6 | 1.57 [1.33, 1.81] | 11.94 / 12.39 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 4.24 / 4.07 | 48.6 | 0.19 [0.15, 0.24] | 11.56 / 12.51 | **DIFFERENT_FASTER** |
| unicycle_bugtrap | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 117.85 / 103.58 | 407 | 0.62 [0.44, 0.82] | 11.72 / 12.27 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 90.12 / 70.32 | 565 | 0.34 [0.24, 0.48] | 12.03 / 12.25 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_wall | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 0.52 / 0.48 | 0.6 | 1.85 [1.54, 2.19] | 6.41 / 4.82 | **DIFFERENT_LENGTH** |
| unicycle_wall | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 0.29 / 0.24 | 11 | 0.06 [0.04, 0.08] | 4.72 / 5.24 | **DIFFERENT_FASTER** |
| unicycle_wall | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 34.42 / 22.06 | 30 | 2.46 [1.69, 3.36] | 5.84 / 5.27 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_wall | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 29.56 / 18.34 | 25 | 2.53 [1.58, 3.96] | 7.69 / 5.61 | **DIFFERENT_LENGTH** |

