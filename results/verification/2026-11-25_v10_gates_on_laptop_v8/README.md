# v10 claim gates applied to the laptop v8 campaign (2026-11-25)

Purpose: check that `benchmarks/report_v10.py` with the pre-declared `config/claim_criteria_v10.json` relabels the
laptop v8 data (`results/paper_campaigns/2026-11-24_final_v8`) honestly. No new measurement; laptop k_hw = 1.067.

Observed effects of the new gates:
- Unicycle stop-turn-go rows: VARIANT_NOT_PAPER_PROTOCOL (were REPRODUCED in v8).
- Aerial order-2 rows: 0/30 original-model valid; judged on the reduced-order protocol and labelled REDUCED_ORDER_PROTOCOL.
- Two-sided band exposes DIFFERENT_FASTER rows: Quad2D Bugtrap RRTConnect (0.19x) and most aerial propagation
  variants (0.00-0.06x of paper). These indicate protocol differences (goal tolerance 0.5/0.7, rest-sample mixture,
  range clipping) to be investigated, not reproduction.
- Table II: 6877/6990 validated (113 returned-invalid); the stage `table_ii_v8` was not controlled (probe drift 10.1%),
  so every Table II verdict is UNCONTROLLED. Overall mean ratio 0.31 (< 1/3): the paper's heavy tail is not reproduced.
