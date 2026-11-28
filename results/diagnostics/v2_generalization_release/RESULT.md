# v2 generalization check: **GENERALIZES**

Rule: `config/v2_generalization_rule.json` (declared before running). G1 safety (0 invalid v2 returns, 0 crashes): **True**. G2 cells failing (v2 solves fewer than v1 minus allowance): **0**. G3 share of cells where v2 is faster (median, unsolved = 10 s): **100 %** (need >= 80 %).
v1 for reference: 0 invalid returns, 0 crashes.

| Problem | Chart | Planner | v1 solved | v2 solved | v1 median ms | v2 median ms | Paired length v1 / v2 (m) | G2 | G3 |
|---|---|---|---|---|---|---|---|---|---|
| random | backward | rrt_connect | 97/100 | 100/100 | 41.90 | 0.46 | 11.01 / 6.01 | ok | ok |
| random | backward | rrt_dp | 99/100 | 100/100 | 293.11 | 46.71 | 6.94 / 7.50 | ok | ok |
| random | backward | sst_bvp | 100/100 | 100/100 | 10.36 | 0.15 | 6.24 / 5.39 | ok | ok |
| random | backward | sst_simd | 96/100 | 100/100 | 340.38 | 29.25 | 6.53 / 6.42 | ok | ok |
| random | forward | rrt_connect | 96/100 | 100/100 | 45.87 | 0.23 | 10.63 / 5.72 | ok | ok |
| random | forward | rrt_dp | 98/100 | 100/100 | 309.06 | 33.57 | 6.89 / 7.14 | ok | ok |
| random | forward | sst_bvp | 100/100 | 100/100 | 16.46 | 0.11 | 5.75 / 5.11 | ok | ok |
| random | forward | sst_simd | 99/100 | 100/100 | 324.32 | 24.33 | 6.49 / 6.23 | ok | ok |
| unicycle_kink | backward | rrt_connect | 17/30 | 30/30 | 5768.67 | 8.44 | 14.72 / 13.22 | ok | ok |
| unicycle_kink | backward | rrt_dp | 30/30 | 30/30 | 911.67 | 83.55 | 14.47 / 14.84 | ok | ok |
| unicycle_kink | backward | sst_bvp | 30/30 | 30/30 | 353.92 | 4.96 | 12.49 / 12.94 | ok | ok |
| unicycle_kink | backward | sst_simd | 30/30 | 30/30 | 1402.74 | 71.99 | 14.23 / 14.20 | ok | ok |
| unicycle_kink | forward | rrt_connect | 12/30 | 30/30 | 10000.00 | 10.92 | 13.93 / 13.24 | ok | ok |
| unicycle_kink | forward | rrt_dp | 30/30 | 30/30 | 594.82 | 43.77 | 12.65 / 11.65 | ok | ok |
| unicycle_kink | forward | sst_bvp | 30/30 | 30/30 | 315.77 | 9.61 | 10.49 / 10.72 | ok | ok |
| unicycle_kink | forward | sst_simd | 30/30 | 30/30 | 593.73 | 26.36 | 13.14 / 10.92 | ok | ok |
| unicycle_kink_v1 | backward | rrt_connect | 18/30 | 30/30 | 6616.70 | 9.62 | 14.32 / 14.13 | ok | ok |
| unicycle_kink_v1 | backward | rrt_dp | 30/30 | 30/30 | 1128.50 | 96.79 | 14.64 / 14.64 | ok | ok |
| unicycle_kink_v1 | backward | sst_bvp | 30/30 | 30/30 | 607.15 | 5.59 | 13.22 / 12.76 | ok | ok |
| unicycle_kink_v1 | backward | sst_simd | 30/30 | 30/30 | 1754.04 | 137.45 | 14.88 / 13.65 | ok | ok |
| unicycle_kink_v1 | forward | rrt_connect | 15/30 | 30/30 | 9020.58 | 9.30 | 13.42 / 13.36 | ok | ok |
| unicycle_kink_v1 | forward | rrt_dp | 30/30 | 30/30 | 1134.75 | 50.72 | 13.05 / 13.03 | ok | ok |
| unicycle_kink_v1 | forward | sst_bvp | 30/30 | 30/30 | 519.99 | 19.30 | 13.35 / 13.11 | ok | ok |
| unicycle_kink_v1 | forward | sst_simd | 30/30 | 30/30 | 691.86 | 35.43 | 13.26 / 13.07 | ok | ok |
| unicycle_parallelpark | backward | rrt_connect | 0/30 | 30/30 | 10000.00 | 90.55 | - / - | ok | ok |
| unicycle_parallelpark | backward | rrt_dp | 16/30 | 30/30 | 9637.76 | 444.03 | 3.08 / 3.92 | ok | ok |
| unicycle_parallelpark | backward | sst_bvp | 22/30 | 30/30 | 6015.12 | 926.44 | 3.28 / 3.91 | ok | ok |
| unicycle_parallelpark | backward | sst_simd | 13/30 | 30/30 | 10000.00 | 1161.04 | 3.33 / 4.07 | ok | ok |
| unicycle_parallelpark | forward | rrt_connect | 2/30 | 30/30 | 10000.00 | 6.02 | 1.33 / 2.79 | ok | ok |
| unicycle_parallelpark | forward | rrt_dp | 30/30 | 30/30 | 9.64 | 4.48 | 1.32 / 1.34 | ok | ok |
| unicycle_parallelpark | forward | sst_bvp | 30/30 | 30/30 | 31.98 | 0.08 | 1.33 / 1.39 | ok | ok |
| unicycle_parallelpark | forward | sst_simd | 30/30 | 30/30 | 25.04 | 4.51 | 1.27 / 1.35 | ok | ok |
