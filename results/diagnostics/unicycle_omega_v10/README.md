# Unicycle |omega| <= 1.5 rejections: diagnosis (25/11/2026, PC, no planner code changed)

- `edge_study.py`: 20 000 BVP edges between states drawn as the v10 planner draws them (uniform |v| in [0.01, 1], uniform
  direction, range 1.5 m, optimal T, rho = 1). Results:
  - 92.5% of the edges violate omega;
  - in 44.7% of all edges the peak omega occurs at a speed below 0.2 m/s (near the Example 2 singularity);
  - the median curvature at the peak is 22 1/m (cusps and loops);
  - only 7.3% of the edges are valid, and this barely depends on the heading change between the endpoints.
- `alignment_study.py`: the validity is governed by the alignment of the endpoint velocities with the displacement.
  At rho = 1: aligned 96.5%, within ±30° 56%, within ±90° 13%, random 4%.
  A larger rho (shorter T) makes it worse: random 0.05% at rho = 10.
- Conclusion: the 70x gap to the paper is explained by the flat-velocity proposal distribution, which the paper does not
  state (Alg. 2: "sample z_f in Z").
  - Uniform sampling stays the primary (literal) protocol.
  - An alignment-biased proposal will only be reported as a labelled sensitivity variant, never as reproduction.
