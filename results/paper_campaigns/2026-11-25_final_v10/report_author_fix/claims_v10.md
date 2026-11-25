# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.499 (results/paper_campaigns/2026-11-25_final_v10/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 975/1000 | 14 | 10.01 / 0.97 | 0.61 / 0.2 | 32.90 / 9.69 [6.99, 12.76] | 10.72 / 5.25 | **DIFFERENT_SLOWER** |
| bookshelf_tall | 978/1000 | 3 | 14.76 / 0.74 | 1.09 / 0.15 | 27.14 / 9.84 [7.25, 14.66] | 9.74 / 5.3 | **DIFFERENT_SLOWER** |
| bookshelf_small | 925/1000 | 4 | 38.98 / 0.74 | 7.8 / 0.17 | 10.02 / 8.68 [5.81, 14.97] | 10.71 / 5.5 | **DIFFERENT_INVALID** |
| cage | 330/1000 | 0 | 347.20 / 231.29 | 12.96 / 6.35 | 53.70 / 73.01 [60.22, 93.40] | 20.11 / 10.07 | **DIFFERENT_INVALID** |
| box | 995/1000 | 3 | 9.88 / 2.57 | 0.86 / 0.25 | 23.04 / 20.64 [17.14, 24.27] | 13.09 / 5.64 | **DIFFERENT_SLOWER** |
| table_under_pick | 993/1000 | 5 | 4.05 / 0.34 | 0.34 / 0.25 | 23.90 / 2.69 [2.54, 2.96] | 16.68 / 8.46 | **DIFFERENT_SLOWER** |
| table_pick | 988/990 | 2 | 2.81 / 0.23 | 0.28 / 0.12 | 20.09 / 3.81 [3.41, 4.43] | 9.23 / 5.24 | **DIFFERENT_SLOWER** |
| overall | 6184/6990 | 31 | 30.96 / 0.65 | 3.49 / 0.24 | 17.78 / 5.40 [4.75, 6.33] | 12.16 / 6.5 | **DIFFERENT_INVALID** |

