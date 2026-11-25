# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.499 (results/paper_campaigns/2026-11-25_final_v10/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table I

| Case | Variant | Label | Valid (judged) / attempts | Original-model valid | Mean / median ms | Paper ms | Ratio [95% CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|---|---|
| quad2d_bugtrap | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 2.88 / 1.70 | 2.7 | 2.14 [1.50, 2.91] | 12.29 / 11.46 | **REPRODUCED** |
| quad2d_bugtrap | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 4917.21 / 2211.24 | 45 | 219.03 [86.55, 412.34] | 11.39 / 11.67 | **DIFFERENT_SLOWER** |
| quad2d_bugtrap | sst_simd | PAPER_PROTOCOL | 26/30 | 26/30 | 9460.77 / 7301.68 | 388 | 48.88 [31.60, 68.49] | 11.55 / 11.46 | **DIFFERENT_INVALID** |
| quad2d_bugtrap | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 289.17 / 239.85 | 909 | 0.64 [0.50, 0.78] | 11.73 / 11.5 | **REPRODUCED** |
| quad2d_hole | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.66 / 0.63 | 0.4 | 8.31 [4.88, 12.42] | 6.32 / 5.16 | **DIFFERENT_SLOWER** |
| quad2d_hole | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 11.14 / 1.09 | 270 | 0.08 [0.03, 0.15] | 5.57 / 4.93 | **DIFFERENT_FASTER** |
| quad2d_hole | sst_simd | PAPER_PROTOCOL | 24/30 | 24/30 | 11400.53 / 5294.64 | 1813 | 12.60 [6.52, 20.37] | 6.78 / 5.14 | **DIFFERENT_INVALID** |
| quad2d_hole | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 511.54 / 195.96 | 2133 | 0.48 [0.26, 0.77] | 6.08 / 5.24 | **REPRODUCED** |
| quad3d_block | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 2.43 / 2.25 | 0.9 | 5.42 [4.66, 6.20] | 9.17 / 8.08 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 13.26 / 2.53 | 1.4 | 18.99 [7.61, 32.52] | 8.38 / 8.61 | **DIFFERENT_SLOWER** |
| quad3d_block | sst_simd | PAPER_PROTOCOL | 10/30 | 10/30 | 27490.89 / 27642.13 | 4752 | 11.60 [6.33, 16.80] | 9.29 / 9.56 | **DIFFERENT_INVALID** |
| quad3d_block | rrt_dp | PAPER_PROTOCOL | 28/30 | 28/30 | 13809.82 / 10003.74 | 7602 | 3.64 [2.53, 4.91] | 9.18 / 8.75 | **DIFFERENT_INVALID** |
| quad3d_window | rrt_connect | PAPER_PROTOCOL | 30/30 | 30/30 | 1.26 / 0.93 | 0.5 | 5.05 [3.75, 6.59] | 5.85 / 5.49 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_bvp | PAPER_PROTOCOL | 30/30 | 30/30 | 206.50 / 25.79 | 22.6 | 18.31 [4.62, 41.93] | 6.34 / 6.28 | **DIFFERENT_SLOWER** |
| quad3d_window | sst_simd | PAPER_PROTOCOL | 20/30 | 20/30 | 24161.67 / 24601.74 | 629 | 77.00 [54.26, 100.81] | 6.89 / 6.96 | **DIFFERENT_INVALID** |
| quad3d_window | rrt_dp | PAPER_PROTOCOL | 30/30 | 30/30 | 8003.57 / 5700.35 | 2631 | 6.10 [4.35, 8.51] | 6.48 / 7.98 | **DIFFERENT_SLOWER** |

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

