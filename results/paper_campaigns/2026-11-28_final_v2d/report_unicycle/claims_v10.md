# v10 claim evaluation: results/paper_campaigns/2026-11-28_final_v2d

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.490 (results/paper_campaigns/2026-11-28_final_v2d/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| unicycle_bugtrap | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 5.17 / 5.04 | 6 | 1.76 [1.50, 2.03] | 10.50 / 12.39 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 4.64 / 3.48 | 48.6 | 0.20 [0.15, 0.24] | 10.61 / 12.51 | **DIFFERENT_FASTER** |
| unicycle_bugtrap | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 113.46 / 77.16 | 407 | 0.57 [0.42, 0.73] | 10.53 / 12.27 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_bugtrap | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 124.95 / 99.20 | 565 | 0.45 [0.32, 0.59] | 10.73 / 12.25 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_wall | rrt_connect | V2_PROTOCOL | 30/30 | 30/30 | 0.73 / 0.67 | 0.6 | 2.49 [2.15, 2.86] | 3.63 / 4.82 | **DIFFERENT_LENGTH** |
| unicycle_wall | sst_bvp | V2_PROTOCOL | 30/30 | 30/30 | 0.63 / 0.58 | 11 | 0.12 [0.10, 0.14] | 3.55 / 5.24 | **DIFFERENT_FASTER** |
| unicycle_wall | sst_simd | V2_PROTOCOL | 30/30 | 30/30 | 41.24 / 30.12 | 30 | 2.81 [1.89, 3.84] | 4.23 / 5.27 | **V2_PROTOCOL:REPRODUCED** |
| unicycle_wall | rrt_dp | V2_PROTOCOL | 30/30 | 30/30 | 38.44 / 18.60 | 25 | 3.14 [2.01, 4.48] | 5.18 / 5.61 | **DIFFERENT_SLOWER** |

