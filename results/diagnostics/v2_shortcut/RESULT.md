# Lever L7: Alg. 4 over subdivided edges

## Why (diagnosis, release campaign `final_v2c`, unicycle Wall, RRTConnect)

Wall path lengths were bimodal on the held-out seeds 8001-8030:
- 9/30 paths were 3.3-3.7 m: one intermediate node next to the wall (box x = 2.5, y in [1.8, 3.2]).
- 21/30 paths were 5.5-11.7 m: an intermediate node far from the wall, e.g. (2.52, 5.10) or (3.27, 5.36).

After the paper's Alg. 4, every path had only 2-5 edges, so the length did not come from a long chain of edges. It
came from where the remaining nodes were. RRTConnect returns its first solution. Alg. 4 (paper p. 10) tries to
connect pairs of states *at the knots of the path*, keeping each knot's velocity. With 2 edges, the only bypass it
can try is start→goal, which collides, so the far node stays. The v2 duration fallback (L3) was not the cause: on
development seeds it made Wall paths shorter, not longer (`v2_factorial`).

## What L7 does (`Settings::shortcut_subdivisions` = m, `shortcut_rounds`)

After the paper's pass, each further round does three things:
1. Split every edge at m − 1 interior times. The restriction of the polynomial is the Hermite polynomial between
   its own end states, so the path is unchanged.
2. Run the same greedy loop over the finer states, using the same steering (T\*, then the L3 fallbacks) and the same
   validation.
3. Accept a bypass only if it lowers the LQMT cost of what it replaces, then merge untouched pieces back.

The search is not changed: the same seed gives the same tree and the same first solution. With 1 / 1 (the
default), the result is the paper's single pass, bit for bit (`tests/test_profiles.py`). L7 goes beyond the paper.

## Selection (`config/v2_shortcut_rule.json`, declared before running; development seeds 1101-1130)

The rule was set so that the paper's numbers play no part:
- every attempt valid on the original model;
- mean time ≤ 1.5× the Alg. 4-only time on every case;
- then the lowest normalized length, which must be at least 5% below our own Alg. 4-only paths.

The selection is in `factorial/selection.json` (raw rows: `factorial/rows.jsonl`).

| Family / planner | Choice (m × rounds) | Mean length relative to Alg. 4 alone | Wall / hardest-case length (m) |
|---|---|---|---|
| unicycle RRTConnect | 4 × 3 | 0.73 | Wall 6.61 → 3.80 |
| unicycle BVP-SST\* | 4 × 3 | 0.80 | Wall 5.54 → 3.86 |
| unicycle SIMD-SST\* | 8 × 4 | 0.80 | Wall 4.95 → 3.63 |
| unicycle DP-RRT | 8 × 4 | 0.78 | Wall 7.96 → 5.34 |
| quadrotor r = 4 RRTConnect | 4 × 3 | 0.85 | Quad2D Bugtrap 12.15 → 10.35 |
| quadrotor r = 4 BVP-SST\* | 8 × 4 | 0.86 | Quad2D Bugtrap 11.64 → 10.72 |

Every configuration of the grid returned 30/30 valid paths on every case. Not in scope, and so left off:
- quadrotor r = 4 SIMD-SST\* and DP-RRT, which are too costly for the grid;
- quadrotor r = 2;
- the v1 literal profile.

The frozen settings are in `config/low_dof_v2d_order2.json` and `config/low_dof_v2d_order4.json`. They are measured
on held-out seeds 8101-8130 in campaign `results/paper_campaigns/2026-11-28_final_v2d`.

Note: L7 makes some paths *shorter* than the paper's. The length criterion is two-sided (±20%,
`config/claim_criteria_v10.json`, unchanged), so such rows are reported as a length difference in the shorter
direction.

## Held-out check (campaign `final_v2d`, seeds 8101-8130)

`heldout_paired/summary.json` compares the same attempts with L7 off (1 / 1, the `final_v2c` settings) and on. The
search was identical in all 240 pairs (same iterations), and every path was valid on the original model with and
without L7. No path became longer.

| Unicycle | Mean length off → on (m) | Longest off → on (m) | Post-processing share of time (on) |
|---|---|---|---|
| Bugtrap RRTConnect | 11.91 → 10.50 | 16.40 → 11.07 | 11.7% |
| Bugtrap BVP-SST\* | 11.72 → 10.61 | 12.94 → 11.68 | 9.7% |
| Bugtrap SIMD-SST\* | 11.97 → 10.53 | 13.76 → 12.40 | 2.5% |
| Bugtrap DP-RRT | 12.17 → 10.73 | 19.10 → 15.75 | 3.0% |
| Wall RRTConnect | 6.43 → 3.63 | 12.02 → 5.63 | 42.0% |
| Wall BVP-SST\* | 5.22 → 3.55 | 9.27 → 5.10 | 35.6% |
| Wall SIMD-SST\* | 5.75 → 4.23 | 10.26 → 8.67 | 2.9% |
| Wall DP-RRT | 7.78 → 5.18 | 15.42 → 7.80 | 4.5% |

The post-processing share includes the paper's own Alg. 4 pass. Lengths are deterministic; the off runs were not
timed.
