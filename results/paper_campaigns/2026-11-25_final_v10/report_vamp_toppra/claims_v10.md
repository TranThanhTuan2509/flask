# v10 claim evaluation: results/paper_campaigns/2026-11-25_final_v10

Criteria: `config/claim_criteria_v10.json`. k_hw = 0.499 (results/paper_campaigns/2026-11-25_final_v10/hardware_calibration.json). Ratios are local / (k_hw · paper); band [1/3, 3].

## Table II (FLASK-RRTConnect)

| Env | Validated / attempts | Returned-invalid | Mean / median ms | Paper mean / median | Ratio mean / median [median CI] | Length / paper | Status |
|---|---|---|---|---|---|---|---|
| bookshelf_thin | 18/1000 | 982 | 2.93 / 2.69 | 0.61 / 0.2 | 9.64 / 26.98 [26.38, 34.60] | 3.55 / 5.25 | **DIFFERENT_INVALID** |
| bookshelf_tall | 95/1000 | 905 | 2.65 / 2.63 | 1.09 / 0.15 | 4.88 / 35.09 [34.80, 35.53] | 4.45 / 5.3 | **DIFFERENT_INVALID** |
| bookshelf_small | 89/1000 | 911 | 2.62 / 2.61 | 7.8 / 0.17 | 0.67 / 30.72 [30.47, 30.84] | 4.05 / 5.5 | **DIFFERENT_INVALID** |
| cage | 0/1000 | 1000 | – / – | 12.96 / 6.35 | – / – [–, –] | – / 10.07 | **DIFFERENT_INVALID** |
| box | 15/1000 | 985 | 2.81 / 2.61 | 0.86 / 0.25 | 6.55 / 20.93 [20.88, 28.05] | 3.98 / 5.64 | **DIFFERENT_INVALID** |
| table_under_pick | 0/1000 | 1000 | – / – | 0.34 / 0.25 | – / – [–, –] | – / 8.46 | **DIFFERENT_INVALID** |
| table_pick | 132/990 | 858 | 2.63 / 2.60 | 0.28 / 0.12 | 18.83 / 43.48 [43.25, 43.62] | 4.16 / 5.24 | **DIFFERENT_INVALID** |
| overall | 349/6990 | 6641 | 2.66 / 2.61 | 3.49 / 0.24 | 1.53 / 21.79 [21.75, 21.88] | 4.17 / 6.5 | **DIFFERENT_INVALID** |

