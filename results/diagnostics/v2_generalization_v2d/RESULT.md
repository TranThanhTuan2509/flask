# v2 generalization check: **GENERALIZES**

Rule: `config/v2_generalization_rule.json` (declared before running). G1 safety (0 invalid v2 returns, 0 crashes): **True**. G2 cells failing (v2 solves fewer than v1 minus allowance): **0**. G3 share of cells where v2 is faster (median, unsolved = 10 s): **100 %** (need >= 80 %).
v1 for reference: 0 invalid returns, 0 crashes.

| Problem | Chart | Planner | v1 solved | v2 solved | v1 median ms | v2 median ms | Paired length v1 / v2 (m) | G2 | G3 |
|---|---|---|---|---|---|---|---|---|---|
| random | backward | rrt_connect | 97/100 | 100/100 | 43.66 | 0.76 | 11.01 / 4.78 | ok | ok |
| random | backward | rrt_dp | 98/100 | 100/100 | 302.26 | 49.98 | 6.96 / 6.03 | ok | ok |
| random | backward | sst_bvp | 100/100 | 100/100 | 10.85 | 0.35 | 6.24 / 4.92 | ok | ok |
| random | backward | sst_simd | 96/100 | 100/100 | 338.80 | 29.80 | 6.53 / 5.43 | ok | ok |
| random | forward | rrt_connect | 96/100 | 100/100 | 45.42 | 0.40 | 10.63 / 4.62 | ok | ok |
| random | forward | rrt_dp | 98/100 | 100/100 | 305.13 | 34.31 | 6.89 / 5.60 | ok | ok |
| random | forward | sst_bvp | 100/100 | 100/100 | 15.92 | 0.25 | 5.75 / 4.63 | ok | ok |
| random | forward | sst_simd | 99/100 | 100/100 | 329.01 | 25.64 | 6.49 / 5.39 | ok | ok |
| unicycle_kink | backward | rrt_connect | 17/30 | 30/30 | 5740.87 | 9.39 | 14.72 / 12.09 | ok | ok |
| unicycle_kink | backward | rrt_dp | 30/30 | 30/30 | 928.37 | 92.60 | 14.47 / 13.31 | ok | ok |
| unicycle_kink | backward | sst_bvp | 30/30 | 30/30 | 363.06 | 6.35 | 12.49 / 12.08 | ok | ok |
| unicycle_kink | backward | sst_simd | 30/30 | 30/30 | 1409.54 | 86.05 | 14.23 / 13.00 | ok | ok |
| unicycle_kink | forward | rrt_connect | 12/30 | 30/30 | 10000.00 | 12.12 | 13.93 / 11.71 | ok | ok |
| unicycle_kink | forward | rrt_dp | 30/30 | 30/30 | 599.65 | 51.26 | 12.65 / 10.65 | ok | ok |
| unicycle_kink | forward | sst_bvp | 30/30 | 30/30 | 307.86 | 10.12 | 10.49 / 9.79 | ok | ok |
| unicycle_kink | forward | sst_simd | 30/30 | 30/30 | 585.67 | 28.27 | 13.14 / 10.34 | ok | ok |
| unicycle_kink_v1 | backward | rrt_connect | 18/30 | 30/30 | 6682.42 | 10.83 | 14.32 / 12.68 | ok | ok |
| unicycle_kink_v1 | backward | rrt_dp | 30/30 | 30/30 | 1179.06 | 111.67 | 14.64 / 12.95 | ok | ok |
| unicycle_kink_v1 | backward | sst_bvp | 30/30 | 30/30 | 631.57 | 6.66 | 13.22 / 12.01 | ok | ok |
| unicycle_kink_v1 | backward | sst_simd | 30/30 | 30/30 | 1732.10 | 144.50 | 14.88 / 12.15 | ok | ok |
| unicycle_kink_v1 | forward | rrt_connect | 15/30 | 30/30 | 8828.47 | 10.66 | 13.42 / 12.39 | ok | ok |
| unicycle_kink_v1 | forward | rrt_dp | 30/30 | 30/30 | 1108.65 | 61.44 | 13.05 / 12.24 | ok | ok |
| unicycle_kink_v1 | forward | sst_bvp | 30/30 | 30/30 | 515.78 | 19.72 | 13.35 / 12.29 | ok | ok |
| unicycle_kink_v1 | forward | sst_simd | 30/30 | 30/30 | 694.55 | 44.75 | 13.26 / 12.34 | ok | ok |
| unicycle_parallelpark | backward | rrt_connect | 0/30 | 30/30 | 10000.00 | 91.51 | - / - | ok | ok |
| unicycle_parallelpark | backward | rrt_dp | 15/30 | 30/30 | 9904.89 | 472.96 | 3.06 / 3.86 | ok | ok |
| unicycle_parallelpark | backward | sst_bvp | 22/30 | 30/30 | 5998.22 | 941.61 | 3.28 / 3.73 | ok | ok |
| unicycle_parallelpark | backward | sst_simd | 13/30 | 30/30 | 10000.00 | 1202.42 | 3.33 / 3.99 | ok | ok |
| unicycle_parallelpark | forward | rrt_connect | 2/30 | 30/30 | 10000.00 | 6.28 | 1.33 / 1.33 | ok | ok |
| unicycle_parallelpark | forward | rrt_dp | 30/30 | 30/30 | 9.99 | 4.70 | 1.32 / 1.34 | ok | ok |
| unicycle_parallelpark | forward | sst_bvp | 30/30 | 30/30 | 31.40 | 0.12 | 1.33 / 1.32 | ok | ok |
| unicycle_parallelpark | forward | sst_simd | 30/30 | 30/30 | 24.85 | 4.74 | 1.27 / 1.33 | ok | ok |
