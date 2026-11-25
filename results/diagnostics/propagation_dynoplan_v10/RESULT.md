# Result (dev seeds 1001-1010, E-cores, development timing; work counters are deterministic)

Compared v10 primary (propagation U[0,0.3] s) with Dynoplan sst_v0 durations (DECLARATION.md).

| Case / variant | v10 primary: valid, median ms, iterations | Dynoplan durations | k_hw x paper ms |
|---|---|---|---|
| unicycle_bugtrap sst_simd | 9/10, 1201, 226869 | 10/10, 534, 212268 | 203 |
| unicycle_bugtrap rrt_dp | 10/10, 1544, 252596 | 10/10, 400, 152126 | 282 |
| unicycle_wall sst_simd | 10/10, 283, 65155 | 10/10, 147, 68239 | 15 |
| unicycle_wall rrt_dp | 10/10, 688, 122108 | 10/10, 225, 98296 | 12.5 |
| quad2d_bugtrap sst_simd | 0 orig-valid (order 2), 186, 20744 | 81, 8869 | 194 |
| quad2d_bugtrap rrt_dp | 9.5, 5910 | 5.2, 2938 | 454 |
| quad2d_hole sst_simd | 19.8, 3512 | 5.8, 1558 | 905 |
| quad2d_hole rrt_dp | 3.2, 2221 | 1.3, 825 | 1064 |
| quad3d_block sst_simd | 39.8, 3534 | 37.9, 2184 | 2371 |
| quad3d_block rrt_dp | 20.9, 2578 | 45.7, 3332 | 3793 |
| quad3d_window sst_simd | 16.6, 2490 | 10.9, 1746 | 314 |
| quad3d_window rrt_dp | 12.2, 3458 | 6.7, 1418 | 1313 |

(v10_primary shared CPU 24 with the iDb-A* primitive generator for its first ~60 s; times of its first
attempts are contaminated, iteration counts are not.)

Conclusion: the library-default horizons make aerial order-2 propagation even faster, so the 30-1000x
"faster than paper" gap of order-2 aerial propagation is NOT explained by the propagation horizon. At order 4
(v10 stage table_i_v10_order4) DP-RRT is 0.5-6x k_hw*paper and SIMD-SST* 12-77x (truncated by the 60 s budget),
i.e. the paper's propagation times are of the order-4 magnitude while its RRTConnect/BVP-SST* times match order 2
(paper Sec. IV: quadrotors "can reduce the pseudo-control order to r = 2 ... in practice"). Which order the
paper used per variant is not stated; recorded as an open question, nothing is tuned toward it.
The variant is not adopted as primary (declared beforehand).
