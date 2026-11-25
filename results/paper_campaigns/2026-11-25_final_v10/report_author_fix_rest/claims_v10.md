# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.499 (results/paper_campaigns/2026-11-25_final_v10/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 984/1000 | 15 | 3.81 / 0.41 | 0.61 / 0.2 | 12.53 / 4.06 [3.43, 5.23] | 9.25 / 5.25 | **DIFFERENT_SLOWER** |
| bookshelf_tall | 997/1000 | 3 | 10.33 / 0.42 | 1.09 / 0.15 | 19.00 / 5.65 [4.33, 7.20] | 8.60 / 5.3 | **DIFFERENT_SLOWER** |
| bookshelf_small | 981/1000 | 2 | 20.48 / 0.43 | 7.8 / 0.17 | 5.26 / 5.13 [3.68, 7.98] | 8.67 / 5.5 | **DIFFERENT_SLOWER** |
| cage | 980/1000 | 9 | 123.46 / 69.88 | 12.96 / 6.35 | 19.10 / 22.06 [18.67, 25.44] | 16.85 / 10.07 | **DIFFERENT_SLOWER** |
| box | 995/1000 | 5 | 0.97 / 0.63 | 0.86 / 0.25 | 2.27 / 5.02 [4.61, 5.69] | 9.96 / 5.64 | **DIFFERENT_SLOWER** |
| table_under_pick | 997/1000 | 3 | 0.45 / 0.21 | 0.34 / 0.25 | 2.63 / 1.65 [1.58, 1.74] | 16.13 / 8.46 | **DIFFERENT_LENGTH** |
| table_pick | 987/990 | 3 | 0.37 / 0.15 | 0.28 / 0.12 | 2.68 / 2.57 [2.36, 2.82] | 8.62 / 5.24 | **DIFFERENT_LENGTH** |
| overall | 6921/6990 | 40 | 22.67 / 0.39 | 3.49 / 0.24 | 13.02 / 3.25 [2.87, 3.72] | 11.15 / 6.5 | **DIFFERENT_SLOWER** |

