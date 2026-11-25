# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.499 (results/paper_campaigns/2026-11-25_final_v10/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 980/1000 | 0 | 0.76 / 0.60 | 0.61 / 0.2 | 2.49 / 5.99 [5.60, 6.33] | 5.16 / 5.25 | **DIFFERENT_SLOWER** |
| bookshelf_tall | 1000/1000 | 0 | 0.67 / 0.50 | 1.09 / 0.15 | 1.24 / 6.72 [6.41, 7.10] | 5.16 / 5.3 | **DIFFERENT_SLOWER** |
| bookshelf_small | 980/1000 | 0 | 0.89 / 0.52 | 7.8 / 0.17 | 0.23 / 6.12 [5.53, 6.85] | 5.28 / 5.5 | **DIFFERENT_FASTER** |
| cage | 1000/1000 | 0 | 3.67 / 2.67 | 12.96 / 6.35 | 0.57 / 0.84 [0.79, 0.90] | 8.91 / 10.07 | **REPRODUCED** |
| box | 1000/1000 | 0 | 1.05 / 0.96 | 0.86 / 0.25 | 2.45 / 7.69 [7.46, 7.97] | 5.99 / 5.64 | **DIFFERENT_SLOWER** |
| table_under_pick | 1000/1000 | 0 | 0.92 / 0.85 | 0.34 / 0.25 | 5.40 / 6.84 [6.59, 7.13] | 7.75 / 8.46 | **DIFFERENT_SLOWER** |
| table_pick | 990/990 | 0 | 0.57 / 0.51 | 0.28 / 0.12 | 4.05 / 8.58 [8.28, 8.91] | 4.97 / 5.24 | **DIFFERENT_SLOWER** |
| overall | 6950/6990 | 0 | 1.22 / 0.73 | 3.49 / 0.24 | 0.70 / 6.08 [5.77, 6.39] | 6.18 / 6.5 | **DIFFERENT_SLOWER** |

