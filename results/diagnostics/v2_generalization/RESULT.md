# v2 generalization check: **GENERALIZES**

Rule: `config/v2_generalization_rule.json` (declared before running). G1 safety (0 invalid v2 returns, 0 crashes): **True**. G2 cells failing (v2 solves fewer than v1 minus allowance): **0**. G3 share of cells where v2 is faster (median, unsolved = 10 s): **100 %** (need >= 80 %).
v1 for reference: 2 invalid returns, 0 crashes.

| Problem | Chart | Planner | v1 solved | v2 solved | v1 median ms | v2 median ms | Paired length v1 / v2 (m) | G2 | G3 |
|---|---|---|---|---|---|---|---|---|---|
| random | backward | rrt_connect | 97/100 | 100/100 | 42.27 | 0.43 | 11.01 / 6.01 | ok | ok |
| random | backward | rrt_dp | 98/100 | 100/100 | 297.00 | 46.43 | 6.96 / 7.43 | ok | ok |
| random | backward | sst_bvp | 100/100 | 100/100 | 11.04 | 0.16 | 6.24 / 5.39 | ok | ok |
| random | backward | sst_simd | 96/100 | 100/100 | 338.86 | 28.93 | 6.53 / 6.42 | ok | ok |
| random | forward | rrt_connect | 96/100 | 100/100 | 46.33 | 0.23 | 10.63 / 5.72 | ok | ok |
| random | forward | rrt_dp | 98/100 | 100/100 | 314.54 | 33.79 | 6.89 / 7.14 | ok | ok |
| random | forward | sst_bvp | 100/100 | 100/100 | 16.18 | 0.11 | 5.75 / 5.11 | ok | ok |
| random | forward | sst_simd | 99/100 | 100/100 | 334.80 | 24.52 | 6.49 / 6.23 | ok | ok |
| unicycle_kink | backward | rrt_connect | 17/30 | 30/30 | 5786.30 | 8.26 | 14.72 / 13.22 | ok | ok |
| unicycle_kink | backward | rrt_dp | 30/30 | 30/30 | 908.49 | 83.26 | 14.47 / 14.84 | ok | ok |
| unicycle_kink | backward | sst_bvp | 30/30 | 30/30 | 365.84 | 5.24 | 12.49 / 12.94 | ok | ok |
| unicycle_kink | backward | sst_simd | 30/30 | 30/30 | 1409.95 | 73.36 | 14.23 / 14.19 | ok | ok |
| unicycle_kink | forward | rrt_connect | 12/30 | 30/30 | 10000.00 | 11.32 | 13.93 / 13.24 | ok | ok |
| unicycle_kink | forward | rrt_dp | 30/30 | 30/30 | 601.55 | 44.78 | 12.65 / 11.65 | ok | ok |
| unicycle_kink | forward | sst_bvp | 30/30 | 30/30 | 310.78 | 9.33 | 10.49 / 10.72 | ok | ok |
| unicycle_kink | forward | sst_simd | 30/30 | 30/30 | 595.58 | 25.99 | 13.14 / 10.92 | ok | ok |
| unicycle_kink_v1 | backward | rrt_connect | 18/30 | 30/30 | 6694.35 | 9.33 | 14.32 / 14.13 | ok | ok |
| unicycle_kink_v1 | backward | rrt_dp | 30/30 | 30/30 | 1153.70 | 96.33 | 14.64 / 14.64 | ok | ok |
| unicycle_kink_v1 | backward | sst_bvp | 30/30 | 30/30 | 646.01 | 5.75 | 13.22 / 12.76 | ok | ok |
| unicycle_kink_v1 | backward | sst_simd | 30/30 | 30/30 | 1740.67 | 135.35 | 14.88 / 13.65 | ok | ok |
| unicycle_kink_v1 | forward | rrt_connect | 15/30 | 30/30 | 8927.77 | 8.95 | 13.42 / 13.36 | ok | ok |
| unicycle_kink_v1 | forward | rrt_dp | 30/30 | 30/30 | 1132.19 | 50.50 | 13.05 / 13.03 | ok | ok |
| unicycle_kink_v1 | forward | sst_bvp | 30/30 | 30/30 | 531.11 | 19.11 | 13.35 / 13.11 | ok | ok |
| unicycle_kink_v1 | forward | sst_simd | 30/30 | 30/30 | 712.05 | 35.63 | 13.26 / 13.07 | ok | ok |
| unicycle_parallelpark | backward | rrt_connect | 0/30 | 30/30 | 10000.00 | 93.54 | - / - | ok | ok |
| unicycle_parallelpark | backward | rrt_dp | 15/30 | 30/30 | 9931.59 | 446.25 | 3.06 / 3.98 | ok | ok |
| unicycle_parallelpark | backward | sst_bvp | 22/30 | 30/30 | 5987.88 | 934.82 | 3.28 / 3.91 | ok | ok |
| unicycle_parallelpark | backward | sst_simd | 14/30 | 30/30 | 10000.00 | 1164.17 | 3.35 / 4.07 | ok | ok |
| unicycle_parallelpark | forward | rrt_connect | 2/30 | 30/30 | 10000.00 | 6.58 | 1.33 / 2.79 | ok | ok |
| unicycle_parallelpark | forward | rrt_dp | 30/30 | 30/30 | 9.97 | 4.77 | 1.32 / 1.34 | ok | ok |
| unicycle_parallelpark | forward | sst_bvp | 30/30 | 30/30 | 32.47 | 0.08 | 1.33 / 1.39 | ok | ok |
| unicycle_parallelpark | forward | sst_simd | 30/30 | 30/30 | 24.46 | 4.46 | 1.27 / 1.35 | ok | ok |
