# Multi-segment prediction accuracy (Milestone 2B.1)

- 300 cases = 5 transitions × 5 speeds × 2 time steps × 3 crossing positions × 2 input sequences (feed-forward, and feed-forward + 0.08 rad sinusoidal steering); 10 excluded because the *reference* trajectory itself is invalid (listed at the end)
- Reference C: event-aware RK4 with max_step = dt/100. Horizon 2.0 s (N = 20 at dt = 0.1, N = 40 at dt = 0.05), shortened on the open lab track so it ends 0.3 m before the track end.
- All predictions use RK4 with one step per stage (as in the NMPC), the same initial state `[s0, 0, 0, v]` and the same inputs.
- Errors at knots vs C: position = Cartesian distance [m], heading = |ψ_pred − ψ_ref| [rad], ey [m], s [m]. "One-step" = knot 2; "horizon" = maximum over all knots.

## Reference cross-check (event-aware Frenet vs independent Cartesian RK4, max_step = 1e-4)

| quantity | value |
|:--|--:|
| max position or heading difference over all knots and cases | 2.5e-11 |
| max |Δs|, |Δey|, |Δepsi| after back-projection (with progress hints) | 7.3e-11 |
| back-projection failures | 0 |
| max crossings resolved in one 2 s horizon | 6 |

## Self-consistent (fixed-point) schedules

The schedule is recomputed from the prediction's own progress until the progress changes by less than 1e-9 m (at most 30 rollouts).

| method | converged | rollouts (median / max) |
|:--|--:|:-:|
| B average | 277 / 290 | 1 / 30 |
| D split | 280 / 290 | 6 / 30 |

## Self-consistent schedules: converged vs not converged (domain failures excluded)

| method | cases | horizon pos. (median / max) | horizon heading (max) |
|:--|--:|:-:|--:|
| B average (self-consistent), converged | 277 | 1.6e-03 / 7.6e-02 | 5.3e-10 |
| B average (self-consistent), **not converged** | 11 | 1.5e-02 / 1.1e+00 | 9.6e-01 |
| D split (self-consistent), converged | 280 | 2.1e-04 / 4.1e-01 | 1.6e-01 |
| D split (self-consistent), **not converged** | 7 | 2.4e-01 / 5.2e-01 | 2.0e-01 |

## Horizon position error [m] by speed, feed-forward inputs (maximum over transitions and crossing positions)

| method | dt | v = 0.5 | v = 1.0 | v = 2.0 | v = 3.0 | v = 4.0 |
|:--|--:|--:|--:|--:|--:|--:|
| A start-of-step | 0.1 | 8.4e-02 | 2.9e-01 | 1.3e+00 | 1.2e+00 | 3.3e+00 |
| A start-of-step | 0.05 | 4.3e-02 | 1.5e-01 | 4.1e-01 | 8.4e-01 | 1.5e+00 |
| B average (self-consistent) | 0.1 | 7.2e-04 | 2.8e-03 | 1.5e-02 | 2.6e-02 | 7.6e-02 |
| B average (self-consistent) | 0.05 | 1.6e-04 | 7.2e-04 | 3.1e-03 | 9.8e-03 | 1.8e-02 |
| B average (reference endpoints) | 0.1 | 8.1e-04 | 3.2e-03 | 1.7e-02 | 3.9e-02 | 1.3e-01 |
| B average (reference endpoints) | 0.05 | 1.6e-04 | 8.3e-04 | 4.5e-03 | 1.2e-02 | 2.2e-02 |
| D split (nominal times) | 0.1 | 2.5e-04 | 1.5e-03 | 1.5e-02 | 6.5e-02 | 6.9e-02 |
| D split (nominal times) | 0.05 | 6.5e-05 | 5.4e-04 | 5.9e-03 | 1.7e-02 | 2.3e-02 |
| D split (self-consistent) | 0.1 | 1.5e-05 | 2.1e-04 | 2.5e-03 | 1.3e-02 | 2.1e-02 |
| D split (self-consistent) | 0.05 | 1.9e-06 | 3.0e-05 | 5.1e-04 | 1.7e-03 | 3.1e-03 |
| D split (exact times) | 0.1 | 7.6e-10 | 5.2e-08 | 2.1e-06 | 2.0e-05 | 8.0e-05 |
| D split (exact times) | 0.05 | 4.6e-11 | 1.1e-09 | 8.9e-08 | 6.7e-07 | 2.1e-06 |

## Summary, feed-forward inputs (150 cases)

| method | model-domain failures (cases) | one-step pos. (median / max) | one-step heading (median / max) | horizon pos. (median / max) | horizon heading (max) | horizon ey (max) | horizon s (max) |
|:--|--:|:-:|:-:|:-:|--:|--:|--:|
| A start-of-step | 1 / 150 | 1.0e-03 / 1.3e-01 | 4.5e-02 / 6.3e-01 | 1.3e-01 / 3.3e+00 | 6.9e-01 | 2.8e+00 | 2.2e+00 |
| B average (nominal plan) | 0 / 150 | 1.0e-03 / 4.0e-02 | 2.4e-15 / 1.4e-12 | 2.2e-03 / 7.6e-02 | 1.7e-12 | 7.5e-02 | 6.7e-02 |
| B average (1 refinement) | 0 / 150 | 1.0e-03 / 4.0e-02 | 2.4e-15 / 1.4e-12 | 2.2e-03 / 7.6e-02 | 1.7e-12 | 7.5e-02 | 6.7e-02 |
| B average (self-consistent) | 0 / 150 | 1.0e-03 / 4.0e-02 | 2.4e-15 / 1.4e-12 | 2.2e-03 / 7.6e-02 | 1.7e-12 | 7.5e-02 | 6.7e-02 |
| B average (reference endpoints) | 0 / 150 | 1.0e-03 / 4.1e-02 | 1.0e-05 / 4.0e-03 | 2.5e-03 / 1.3e-01 | 4.4e-02 | 9.0e-02 | 1.1e-01 |
| D split (nominal times) | 0 / 150 | 2.0e-07 / 1.1e-03 | 3.6e-06 / 5.2e-03 | 2.4e-04 / 6.9e-02 | 3.7e-02 | 5.1e-02 | 6.9e-02 |
| D split (self-consistent) | 0 / 150 | 4.2e-07 / 1.6e-03 | 1.1e-05 / 8.0e-03 | 5.6e-05 / 2.1e-02 | 1.2e-02 | 1.6e-02 | 2.0e-02 |
| D split (exact times) | 0 / 150 | 4.7e-10 / 2.1e-05 | 6.8e-12 / 9.2e-06 | 1.2e-08 / 8.0e-05 | 6.2e-05 | 5.1e-05 | 6.1e-05 |

## Summary, feed-forward + 0.08 rad sinusoidal steering (140 cases)

| method | model-domain failures (cases) | one-step pos. (median / max) | one-step heading (median / max) | horizon pos. (median / max) | horizon heading (max) | horizon ey (max) | horizon s (max) |
|:--|--:|:-:|:-:|:-:|--:|--:|--:|
| A start-of-step | 2 / 140 | 1.0e-03 / 1.3e-01 | 4.5e-02 / 6.3e-01 | 1.3e-01 / 3.6e+00 | 1.1e+00 | 3.4e+00 | 2.0e+00 |
| B average (nominal plan) | 2 / 140 | 9.0e-04 / 4.0e-02 | 2.6e-15 / 1.4e-12 | 2.1e-01 / 3.7e+00 | 1.8e+00 | 2.2e+00 | 3.7e+00 |
| B average (1 refinement) | 6 / 140 | 9.0e-04 / 4.0e-02 | 2.6e-15 / 1.4e-12 | 5.1e-03 / 3.7e+00 | 1.8e+00 | 2.5e+00 | 3.5e+00 |
| B average (self-consistent) | 2 / 140 | 9.0e-04 / 4.0e-02 | 2.6e-15 / 1.4e-12 | 1.7e-03 / 3.7e+00 | 1.8e+00 | 1.6e+00 | 3.5e+00 |
| B average (reference endpoints) | 0 / 140 | 9.0e-04 / 4.1e-02 | 8.5e-06 / 2.8e-03 | 2.2e-03 / 2.1e-01 | 5.6e-02 | 1.0e-01 | 1.8e-01 |
| D split (nominal times) | 3 / 140 | 1.5e-07 / 1.1e-03 | 2.7e-06 / 5.2e-03 | 2.3e-01 / 3.8e+00 | 1.7e+00 | 2.3e+00 | 4.2e+00 |
| D split (self-consistent) | 3 / 140 | 2.6e-07 / 1.6e-03 | 8.4e-06 / 8.0e-03 | 3.3e-03 / 1.8e+00 | 1.7e+00 | 1.7e+00 | 1.6e+00 |
| D split (exact times) | 0 / 140 | 3.9e-10 / 2.1e-05 | 6.7e-12 / 9.2e-06 | 9.6e-07 / 2.2e-03 | 1.6e-03 | 1.9e-03 | 1.5e-03 |

## Summary over all cases

| method | model-domain failures (cases) | one-step pos. (median / max) | one-step heading (median / max) | horizon pos. (median / max) | horizon heading (max) | horizon ey (max) | horizon s (max) | boundary-count mismatches |
|:--|--:|:-:|:-:|:-:|--:|--:|--:|--:|
| A start-of-step | 3 / 290 | 1.0e-03 / 1.3e-01 | 4.5e-02 / 6.3e-01 | 1.3e-01 / 3.6e+00 | 1.1e+00 | 3.4e+00 | 2.2e+00 | 515 |
| B average (nominal plan) | 2 / 290 | 9.0e-04 / 4.0e-02 | 2.4e-15 / 1.4e-12 | 4.7e-03 / 3.7e+00 | 1.8e+00 | 2.2e+00 | 3.7e+00 | 291 |
| B average (1 refinement) | 6 / 290 | 9.0e-04 / 4.0e-02 | 2.4e-15 / 1.4e-12 | 2.6e-03 / 3.7e+00 | 1.8e+00 | 2.5e+00 | 3.5e+00 | 121 |
| B average (self-consistent) | 2 / 290 | 9.0e-04 / 4.0e-02 | 2.4e-15 / 1.4e-12 | 1.9e-03 / 3.7e+00 | 1.8e+00 | 1.6e+00 | 3.5e+00 | 11 |
| B average (reference endpoints) | 0 / 290 | 9.0e-04 / 4.1e-02 | 9.1e-06 / 4.0e-03 | 2.4e-03 / 2.1e-01 | 5.6e-02 | 1.0e-01 | 1.8e-01 | 9 |
| D split (nominal times) | 3 / 290 | 2.0e-07 / 1.1e-03 | 3.5e-06 / 5.2e-03 | 1.6e-03 / 3.8e+00 | 1.7e+00 | 2.3e+00 | 4.2e+00 | 327 |
| D split (self-consistent) | 3 / 290 | 3.3e-07 / 1.6e-03 | 9.5e-06 / 8.0e-03 | 2.8e-04 / 1.8e+00 | 1.7e+00 | 1.7e+00 | 1.6e+00 | 68 |
| D split (exact times) | 0 / 290 | 4.7e-10 / 2.1e-05 | 6.8e-12 / 9.2e-06 | 1.7e-07 / 2.2e-03 | 1.6e-03 | 1.9e-03 | 1.5e-03 | 0 |

## One-step heading error [rad]: maximum over cases, by speed and dt

| method | dt | v = 0.5 | v = 1.0 | v = 2.0 | v = 3.0 | v = 4.0 |
|:--|--:|--:|--:|--:|--:|--:|
| A start-of-step | 0.1 | 9.0e-02 | 1.8e-01 | 3.5e-01 | 5.0e-01 | 6.3e-01 |
| A start-of-step | 0.05 | 4.5e-02 | 9.0e-02 | 1.8e-01 | 2.6e-01 | 3.5e-01 |
| B average (nominal plan) | 0.1 | 1.4e-12 | 5.7e-13 | 9.3e-14 | 4.8e-13 | 2.6e-13 |
| B average (nominal plan) | 0.05 | 6.2e-13 | 1.4e-12 | 5.7e-13 | 3.0e-13 | 9.3e-14 |
| B average (1 refinement) | 0.1 | 1.4e-12 | 5.7e-13 | 9.3e-14 | 4.8e-13 | 2.6e-13 |
| B average (1 refinement) | 0.05 | 6.2e-13 | 1.4e-12 | 5.7e-13 | 3.0e-13 | 9.3e-14 |
| B average (self-consistent) | 0.1 | 1.4e-12 | 5.7e-13 | 9.3e-14 | 4.8e-13 | 2.6e-13 |
| B average (self-consistent) | 0.05 | 6.2e-13 | 1.4e-12 | 5.7e-13 | 3.0e-13 | 9.3e-14 |
| B average (reference endpoints) | 0.1 | 7.6e-06 | 6.1e-05 | 4.9e-04 | 1.7e-03 | 4.0e-03 |
| B average (reference endpoints) | 0.05 | 9.5e-07 | 7.6e-06 | 6.1e-05 | 2.1e-04 | 4.9e-04 |
| D split (nominal times) | 0.1 | 1.0e-05 | 8.3e-05 | 6.6e-04 | 2.2e-03 | 5.2e-03 |
| D split (nominal times) | 0.05 | 1.3e-06 | 1.0e-05 | 8.3e-05 | 2.8e-04 | 6.6e-04 |
| D split (self-consistent) | 0.1 | 1.6e-05 | 1.3e-04 | 1.0e-03 | 3.4e-03 | 8.0e-03 |
| D split (self-consistent) | 0.05 | 2.0e-06 | 1.6e-05 | 1.3e-04 | 4.2e-04 | 1.0e-03 |
| D split (exact times) | 0.1 | 2.8e-10 | 9.0e-09 | 2.9e-07 | 2.2e-06 | 9.2e-06 |
| D split (exact times) | 0.05 | 8.8e-12 | 2.8e-10 | 9.0e-09 | 6.8e-08 | 2.9e-07 |

## One-step position error [m]: maximum over cases, by speed and dt

| method | dt | v = 0.5 | v = 1.0 | v = 2.0 | v = 3.0 | v = 4.0 |
|:--|--:|--:|--:|--:|--:|--:|
| A start-of-step | 0.1 | 2.0e-03 | 8.1e-03 | 3.2e-02 | 7.3e-02 | 1.3e-01 |
| A start-of-step | 0.05 | 5.1e-04 | 2.0e-03 | 8.1e-03 | 1.8e-02 | 3.2e-02 |
| B average (nominal plan) | 0.1 | 6.2e-04 | 2.5e-03 | 1.0e-02 | 2.2e-02 | 4.0e-02 |
| B average (nominal plan) | 0.05 | 1.6e-04 | 6.2e-04 | 2.5e-03 | 5.6e-03 | 1.0e-02 |
| B average (1 refinement) | 0.1 | 6.2e-04 | 2.5e-03 | 1.0e-02 | 2.2e-02 | 4.0e-02 |
| B average (1 refinement) | 0.05 | 1.6e-04 | 6.2e-04 | 2.5e-03 | 5.6e-03 | 1.0e-02 |
| B average (self-consistent) | 0.1 | 6.2e-04 | 2.5e-03 | 1.0e-02 | 2.2e-02 | 4.0e-02 |
| B average (self-consistent) | 0.05 | 1.6e-04 | 6.2e-04 | 2.5e-03 | 5.6e-03 | 1.0e-02 |
| B average (reference endpoints) | 0.1 | 6.3e-04 | 2.5e-03 | 1.0e-02 | 2.3e-02 | 4.1e-02 |
| B average (reference endpoints) | 0.05 | 1.6e-04 | 6.3e-04 | 2.5e-03 | 5.6e-03 | 1.0e-02 |
| D split (nominal times) | 0.1 | 2.6e-07 | 4.2e-06 | 6.7e-05 | 3.4e-04 | 1.1e-03 |
| D split (nominal times) | 0.05 | 1.6e-08 | 2.6e-07 | 4.2e-06 | 2.1e-05 | 6.7e-05 |
| D split (self-consistent) | 0.1 | 3.9e-07 | 6.3e-06 | 1.0e-04 | 5.1e-04 | 1.6e-03 |
| D split (self-consistent) | 0.05 | 2.4e-08 | 3.9e-07 | 6.3e-06 | 3.2e-05 | 1.0e-04 |
| D split (exact times) | 0.1 | 4.7e-10 | 1.5e-08 | 5.3e-07 | 4.5e-06 | 2.1e-05 |
| D split (exact times) | 0.05 | 1.5e-11 | 4.7e-10 | 1.5e-08 | 1.2e-07 | 5.3e-07 |

## Horizon position error [m]: maximum over cases, by speed and dt

| method | dt | v = 0.5 | v = 1.0 | v = 2.0 | v = 3.0 | v = 4.0 |
|:--|--:|--:|--:|--:|--:|--:|
| A start-of-step | 0.1 | 8.4e-02 | 3.0e-01 | 1.3e+00 | 1.9e+00 | 3.6e+00 |
| A start-of-step | 0.05 | 4.3e-02 | 1.5e-01 | 4.1e-01 | 1.1e+00 | 1.7e+00 |
| B average (nominal plan) | 0.1 | 2.3e-03 | 3.7e-02 | 5.2e-01 | 2.3e+00 | 3.5e+00 |
| B average (nominal plan) | 0.05 | 2.5e-03 | 4.1e-02 | 5.4e-01 | 2.3e+00 | 3.7e+00 |
| B average (1 refinement) | 0.1 | 7.5e-04 | 2.8e-03 | 4.3e-02 | 7.1e-01 | 1.9e+00 |
| B average (1 refinement) | 0.05 | 1.9e-04 | 3.6e-03 | 5.0e-02 | 7.7e-01 | 3.7e+00 |
| B average (self-consistent) | 0.1 | 7.2e-04 | 2.8e-03 | 1.7e-02 | 3.3e-02 | 1.1e+00 |
| B average (self-consistent) | 0.05 | 1.6e-04 | 7.2e-04 | 4.2e-03 | 9.8e-03 | 3.7e+00 |
| B average (reference endpoints) | 0.1 | 8.1e-04 | 3.2e-03 | 1.8e-02 | 4.2e-02 | 2.1e-01 |
| B average (reference endpoints) | 0.05 | 1.6e-04 | 8.3e-04 | 4.5e-03 | 1.2e-02 | 5.4e-02 |
| D split (nominal times) | 0.1 | 2.8e-03 | 4.2e-02 | 5.3e-01 | 2.3e+00 | 3.6e+00 |
| D split (nominal times) | 0.05 | 2.7e-03 | 4.3e-02 | 5.4e-01 | 2.3e+00 | 3.8e+00 |
| D split (self-consistent) | 0.1 | 3.2e-04 | 6.0e-03 | 4.9e-02 | 3.6e-01 | 1.7e+00 |
| D split (self-consistent) | 0.05 | 1.5e-04 | 3.4e-03 | 2.9e-02 | 2.3e-01 | 1.8e+00 |
| D split (exact times) | 0.1 | 3.2e-08 | 1.0e-06 | 3.2e-05 | 8.3e-04 | 2.2e-03 |
| D split (exact times) | 0.05 | 2.1e-09 | 7.0e-08 | 2.1e-06 | 4.5e-05 | 1.3e-04 |

## Horizon lateral error |Δey| [m]: maximum over cases, by speed and dt

| method | dt | v = 0.5 | v = 1.0 | v = 2.0 | v = 3.0 | v = 4.0 |
|:--|--:|--:|--:|--:|--:|--:|
| A start-of-step | 0.1 | 7.5e-02 | 2.0e-01 | 9.3e-01 | 1.8e+00 | 3.4e+00 |
| A start-of-step | 0.05 | 3.8e-02 | 9.0e-02 | 3.3e-01 | 1.0e+00 | 1.6e+00 |
| B average (nominal plan) | 0.1 | 2.3e-03 | 3.7e-02 | 4.4e-01 | 1.4e+00 | 2.2e+00 |
| B average (nominal plan) | 0.05 | 2.4e-03 | 4.1e-02 | 4.5e-01 | 1.4e+00 | 2.2e+00 |
| B average (1 refinement) | 0.1 | 7.1e-04 | 2.8e-03 | 3.1e-02 | 4.1e-01 | 2.4e+00 |
| B average (1 refinement) | 0.05 | 1.8e-04 | 3.6e-03 | 3.7e-02 | 4.9e-01 | 2.5e+00 |
| B average (self-consistent) | 0.1 | 7.1e-04 | 2.8e-03 | 1.7e-02 | 3.3e-02 | 4.1e-01 |
| B average (self-consistent) | 0.05 | 1.6e-04 | 7.1e-04 | 4.2e-03 | 9.8e-03 | 1.6e+00 |
| B average (reference endpoints) | 0.1 | 7.6e-04 | 3.0e-03 | 1.8e-02 | 3.7e-02 | 1.0e-01 |
| B average (reference endpoints) | 0.05 | 1.6e-04 | 7.6e-04 | 4.2e-03 | 1.0e-02 | 3.0e-02 |
| D split (nominal times) | 0.1 | 2.7e-03 | 4.2e-02 | 4.4e-01 | 1.4e+00 | 2.3e+00 |
| D split (nominal times) | 0.05 | 2.6e-03 | 4.3e-02 | 4.5e-01 | 1.4e+00 | 2.3e+00 |
| D split (self-consistent) | 0.1 | 3.1e-04 | 5.9e-03 | 4.5e-02 | 2.6e-01 | 1.5e+00 |
| D split (self-consistent) | 0.05 | 1.4e-04 | 3.3e-03 | 2.4e-02 | 1.7e-01 | 1.7e+00 |
| D split (exact times) | 0.1 | 2.1e-08 | 3.4e-07 | 2.6e-05 | 7.1e-04 | 1.9e-03 |
| D split (exact times) | 0.05 | 1.3e-09 | 2.2e-08 | 1.8e-06 | 4.1e-05 | 9.2e-05 |

## One-step heading error [rad] by transition and crossing position (dt = 0.1, v = 3 m/s, feed-forward inputs)

| transition | crossing at | A start-of-step | B average (nominal plan) | B average (1 refinement) | B average (self-consistent) | B average (reference endpoints) | D split (nominal times) | D split (self-consistent) | D split (exact times) |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| straight→curve (κ 0→+1) | 0.10·dt | 2.7e-01 | 1.4e-15 | 1.4e-15 | 1.4e-15 | 7.3e-05 | 3.6e-06 | 7.6e-05 | 5.5e-12 |
| straight→curve (κ 0→+1) | 0.50·dt | 1.5e-01 | 2.0e-14 | 2.0e-14 | 2.0e-14 | 5.7e-04 | 1.4e-04 | 7.0e-04 | 1.7e-09 |
| straight→curve (κ 0→+1) | 0.90·dt | 3.0e-02 | 4.2e-16 | 1.7e-17 | 4.2e-16 | 7.3e-05 | 3.3e-05 | 1.1e-04 | 5.0e-11 |
| curve→curve (κ +1→−1) | 0.10·dt | 5.0e-01 | 1.5e-14 | 1.5e-14 | 1.5e-14 | 2.8e-04 | 4.5e-05 | 3.2e-04 | 2.7e-09 |
| curve→curve (κ +1→−1) | 0.50·dt | 2.8e-01 | 2.2e-15 | 2.2e-15 | 2.2e-15 | 1.1e-03 | 2.2e-03 | 3.4e-03 | 2.1e-06 |
| curve→curve (κ +1→−1) | 0.90·dt | 5.8e-02 | 4.4e-16 | 4.4e-16 | 4.4e-16 | 1.0e-03 | 1.6e-03 | 5.5e-04 | 2.2e-06 |
| curve→straight (κ +0.5→0) | 0.10·dt | 1.3e-01 | 0.0e+00 | 0.0e+00 | 0.0e+00 | 5.1e-07 | 9.6e-07 | 4.6e-07 | 7.8e-12 |
| curve→straight (κ +0.5→0) | 0.50·dt | 7.5e-02 | 5.1e-15 | 5.1e-15 | 5.1e-15 | 3.5e-05 | 5.3e-05 | 1.8e-05 | 3.4e-09 |
| curve→straight (κ +0.5→0) | 0.90·dt | 1.5e-02 | 2.0e-15 | 2.0e-15 | 2.0e-15 | 4.1e-05 | 4.5e-05 | 4.1e-06 | 2.7e-08 |
| lap boundary (κ 0→+1, lap 1→2) | 0.10·dt | 2.7e-01 | 1.1e-14 | 1.1e-14 | 1.1e-14 | 7.3e-05 | 3.6e-06 | 7.6e-05 | 5.5e-12 |
| lap boundary (κ 0→+1, lap 1→2) | 0.50·dt | 1.5e-01 | 2.3e-14 | 2.3e-14 | 2.3e-14 | 5.7e-04 | 1.4e-04 | 7.0e-04 | 1.7e-09 |
| lap boundary (κ 0→+1, lap 1→2) | 0.90·dt | 3.0e-02 | 1.8e-15 | 1.8e-15 | 1.8e-15 | 7.3e-05 | 3.3e-05 | 1.1e-04 | 5.0e-11 |
| lab: short segments, κ ±1.43 | 0.10·dt | 3.8e-01 | 0.0e+00 | 0.0e+00 | 0.0e+00 | 2.1e-04 | 1.1e-05 | 2.2e-04 | 3.3e-11 |
| lab: short segments, κ ±1.43 | 0.50·dt | 2.1e-01 | 4.8e-13 | 4.8e-13 | 4.8e-13 | 1.7e-03 | 4.1e-04 | 2.1e-03 | 9.9e-09 |
| lab: short segments, κ ±1.43 | 0.90·dt | 4.3e-02 | 2.2e-15 | 2.2e-15 | 2.2e-15 | 2.1e-04 | 9.6e-05 | 3.1e-04 | 3.0e-10 |

## Sensitivity to the nominal progress: horizon position error [m], v = 3 m/s, dt = 0.1, crossing at 0.5·dt

| transition | nominal speed / actual | B average | D split |
|:--|--:|--:|--:|
| straight→curve (κ 0→+1) | 0.8 | 6.4e-01 | 6.5e-01 |
| straight→curve (κ 0→+1) | 0.9 | 1.4e+00 | 1.4e+00 |
| straight→curve (κ 0→+1) | 1.0 | 1.4e-02 | 7.0e-04 |
| straight→curve (κ 0→+1) | 1.1 | 1.2e+00 | 1.2e+00 |
| straight→curve (κ 0→+1) | 1.2 | 2.1e+00 | 2.0e+00 |
| curve→curve (κ +1→−1) | 0.8 | 1.6e+00 | 2.2e+00 |
| curve→curve (κ +1→−1) | 0.9 | 1.5e+00 | 1.5e+00 |
| curve→curve (κ +1→−1) | 1.0 | 2.4e-02 | 4.4e-03 |
| curve→curve (κ +1→−1) | 1.1 | 1.3e+00 | 1.2e+00 |
| curve→curve (κ +1→−1) | 1.2 | 2.1e+00 | 2.0e+00 |

## Computational cost (Julia 1.11.5, single thread, wall-clock mean)

| operation | time [µs] |
|:--|--:|
| one RK4 step, constant curvature (`discrete_dynamics`) | 0.10 |
| `integrate_on_track`, 0.1 s with one crossing, max_step = 0.1 | 4.74 |
| `integrate_on_track`, 0.1 s with one crossing, max_step = 0.01 | 8.97 |
| `integrate_on_track`, 0.1 s with one crossing, max_step = 0.001 | 43.13 |
| `curvature_schedule` for N = 20 | 1.12 |
| 20-stage prediction, method A | 5.51 |
| 20-stage prediction, method B | 5.59 |
| 20-stage prediction, method D (nominal splits) | 17.01 |
| 2 s reference (`simulate_on_track`, max_step = 0.001) | 749 |

## Excluded cases (no valid reference trajectory)

- lap boundary (κ 0→+1, lap 1→2), v = 4.0, dt = 0.1, crossing at 0.1·dt, wobble = 0.08: reference status `domain_error` (interval 10: DomainError with -0.07113334270736105:
invalid Frenet geometry: 1 - kappa*ey = -0.07113334270736105 ≤ eps_geometry = 0.1)
- lap boundary (κ 0→+1, lap 1→2), v = 4.0, dt = 0.1, crossing at 0.5·dt, wobble = 0.08: reference status `domain_error` (interval 10: DomainError with -0.0912456064882472:
invalid Frenet geometry: 1 - kappa*ey = -0.0912456064882472 ≤ eps_geometry = 0.1)
- lap boundary (κ 0→+1, lap 1→2), v = 4.0, dt = 0.1, crossing at 0.9·dt, wobble = 0.08: reference status `domain_error` (interval 11: DomainError with -0.11861054804114368:
invalid Frenet geometry: 1 - kappa*ey = -0.11861054804114368 ≤ eps_geometry = 0.1)
- lap boundary (κ 0→+1, lap 1→2), v = 4.0, dt = 0.05, crossing at 0.1·dt, wobble = 0.08: reference status `domain_error` (interval 19: DomainError with -0.10743805237293524:
invalid Frenet geometry: 1 - kappa*ey = -0.10743805237293524 ≤ eps_geometry = 0.1)
- lap boundary (κ 0→+1, lap 1→2), v = 4.0, dt = 0.05, crossing at 0.5·dt, wobble = 0.08: reference status `domain_error` (interval 20: DomainError with -0.11821528985002083:
invalid Frenet geometry: 1 - kappa*ey = -0.11821528985002083 ≤ eps_geometry = 0.1)
- lap boundary (κ 0→+1, lap 1→2), v = 4.0, dt = 0.05, crossing at 0.9·dt, wobble = 0.08: reference status `domain_error` (interval 20: DomainError with -0.13032369915912612:
invalid Frenet geometry: 1 - kappa*ey = -0.13032369915912612 ≤ eps_geometry = 0.1)
- lab: short segments, κ ±1.43, v = 4.0, dt = 0.1, crossing at 0.5·dt, wobble = 0.08: reference status `end_of_track` (interval 19: reached the end of the open track (s = 14.050382858376183) at t = 0.0738620422174292 s)
- lab: short segments, κ ±1.43, v = 4.0, dt = 0.05, crossing at 0.1·dt, wobble = 0.08: reference status `end_of_track` (interval 37: reached the end of the open track (s = 14.050382858376183) at t = 0.012742690127834294 s)
- lab: short segments, κ ±1.43, v = 4.0, dt = 0.05, crossing at 0.5·dt, wobble = 0.08: reference status `end_of_track` (interval 37: reached the end of the open track (s = 14.050382858376183) at t = 0.03833333685293472 s)
- lab: short segments, κ ±1.43, v = 4.0, dt = 0.05, crossing at 0.9·dt, wobble = 0.08: reference status `end_of_track` (interval 38: reached the end of the open track (s = 14.050382858376183) at t = 0.008691766959829823 s)
