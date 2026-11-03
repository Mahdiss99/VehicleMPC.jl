# Progress-linearized curvature benchmark (Milestone 2B.3A)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; `quarter_track()` from `[0, 0, 0, 0.5]`
- 250 control cycles per run (control period 0.1 s), 3 runs per configuration, interleaved by run; event-aware plant
- Time per cycle = the complete `solve!` (schedule, linearization, parameter updates, all NLP solves, validation, consistency test, reference check); failed cycles included
- "TR ρ": trust region `|s_k - s̄_k| ≤ ρ·|Δs̄|` on knots of stages with non-zero sensitivities (an additional constraint, off by default)

## Quarter track: completion, solves and timing

Outcome per run: laps completed (`stop x (cN)` = stopped at cycle N after x laps).

| configuration | outcomes (laps, per run) | failed cycles | fully consistent / qualified accepted | NLP solves: mean / max | Ipopt iterations: mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m: mean / max solves |
|:--|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A  stagewise, 1 solve, reject | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 3 | 0 / 0 | 1.00 / 1 | 15.0 / 15 | 21.6 / 21.8 / 21.8 / 21.9 | 0 / 0 | – |
| B  stagewise, ≤ 8 corrections / 80 ms, reject | stop 0.24 (c37); stop 0.24 (c37); stop 0.24 (c37) | 3 | 108 / 0 | 2.95 / 6 | 13.9 / 20 | 37.0 / 71.6 / 75.9 / 78.4 | 0 / 0 | 3.33 / 5 |
| C3 stagewise, ≤ 8 corr., qualified, margin 0.05 (2B.2 D3) | 2.37; 2.37; 2.37 | 0 | 714 / 36 | 2.49 / 7 | 14.6 / 20 | 30.2 / 69.4 / 76.5 / 110.6 | 1 / 1 | 3.33 / 6 |
| C4 stagewise, ≤ 8 corr., qualified, margin 0.10 (2B.2 D4) | 2.33; 2.33; 2.33 | 0 | 709 / 41 | 2.43 / 6 | 14.5 / 21 | 27.6 / 70.2 / 76.1 / 78.5 | 0 / 0 | 3.24 / 5 |
| D0 linearized, no trust region, 1 solve, reject | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 3 | 9 / 0 | 1.00 / 1 | 16.2 / 18 | 16.6 / 26.3 / 30.1 / 31.0 | 0 / 0 | – |
| D  linearized, TR 0.1, 1 solve, reject, margin 0.05 | stop 0.02 (c5); stop 0.02 (c5); stop 0.02 (c5) | 3 | 12 / 0 | 1.00 / 1 | 17.8 / 19 | 18.0 / 25.7 / 26.2 / 26.3 | 0 / 0 | – |
| D' linearized, TR 0.1, 1 solve, reject, margin 0.10 | stop 0.16 (c26); stop 0.16 (c26); stop 0.16 (c26) | 3 | 75 / 0 | 1.00 / 1 | 20.3 / 84 | 18.1 / 28.1 / 167.4 / 173.3 | 3 / 3 | – |
| E  linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.05 | stop 1.02 (c103); stop 1.02 (c103); stop 1.02 (c103) | 3 | 306 / 0 | 1.06 / 2 | 18.4 / 24 | 18.3 / 35.3 / 37.6 / 40.2 | 0 / 0 | 1.00 / 1 |
| E' linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.10 | stop 0.16 (c26); stop 0.16 (c26); stop 0.16 (c26) | 3 | 75 / 0 | 1.00 / 1 | 20.3 / 84 | 18.8 / 30.0 / 170.3 / 171.7 | 3 / 3 | – |

## Quarter track: constraints, curvature consistency and prediction accuracy (accepted cycles)

| configuration | plant violations | min plant edge distance [m] | max stage / cum. residual [rad] | max κ model error [1/m] | config. changes / saturated stages / trust region active | reference: first-step / max position [m] | heading [rad] / progress / ey [m] | min predicted edge distance [m] |
|:--|--:|--:|:-:|--:|:-:|:-:|:-:|--:|
| A  stagewise, 1 solve, reject | 0 | 0.350 | NaN / NaN | NaN | 0 / 0 / 0 | NaN / NaN | NaN / NaN / NaN | NaN |
| B  stagewise, ≤ 8 corrections / 80 ms, reject | 0 | 0.203 | 4.6e-03 / 4.1e-03 | 1.3e-02 | 0 / 0 / 0 | 1.7e-02 / 3.1e-02 | 4.1e-03 / 3.1e-02 / 2.9e-02 | 0.023 |
| C3 stagewise, ≤ 8 corr., qualified, margin 0.05 (2B.2 D3) | 0 | 0.198 | 2.9e-02 / 4.1e-02 | 7.0e-02 | 20 / 0 / 0 | 1.7e-02 / 8.7e-02 | 4.1e-02 / 5.5e-02 / 6.1e-02 | 0.008 |
| C4 stagewise, ≤ 8 corr., qualified, margin 0.10 (2B.2 D4) | 0 | 0.226 | 2.5e-02 / 2.7e-02 | 5.6e-02 | 51 / 0 / 0 | 1.8e-02 / 7.2e-02 | 2.7e-02 / 4.3e-02 / 5.4e-02 | 0.062 |
| D0 linearized, no trust region, 1 solve, reject | 0 | 0.345 | 2.8e-03 / 2.8e-03 | 1.1e-02 | 0 / 0 / 0 | 5.9e-08 / 5.7e-03 | 2.8e-03 / 3.1e-03 / 5.2e-03 | 0.238 |
| D  linearized, TR 0.1, 1 solve, reject, margin 0.05 | 0 | 0.335 | 4.9e-03 / 4.4e-03 | 1.7e-02 | 0 / 3 / 15 | 5.7e-08 / 1.5e-02 | 4.4e-03 / 4.9e-03 / 1.5e-02 | 0.223 |
| D' linearized, TR 0.1, 1 solve, reject, margin 0.10 | 0 | 0.242 | 3.7e-03 / 3.9e-03 | 1.3e-02 | 0 / 6 / 66 | 1.4e-02 / 3.1e-02 | 3.9e-03 / 2.1e-02 / 3.0e-02 | 0.082 |
| E  linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.05 | 0 | 0.048 | 4.9e-03 / 4.4e-03 | 1.7e-02 | 6 / 3 / 246 | 1.6e-02 / 4.4e-02 | 4.4e-03 / 3.8e-02 / 4.3e-02 | 0.014 |
| E' linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.10 | 0 | 0.242 | 3.7e-03 / 3.9e-03 | 1.3e-02 | 0 / 6 / 66 | 1.4e-02 / 3.1e-02 | 3.9e-03 / 2.1e-02 / 3.0e-02 | 0.082 |

### Stop reasons (run 1)

- **A  stagewise, 1 solve, reject**: control period 1 (t = 0.0 s): schedule_inconsistent: stale curvature schedule: stage residual 0.1788 rad > 0.005 rad; cumulative residual 0.2144 rad > 0.01 rad
- **B  stagewise, ≤ 8 corrections / 80 ms, reject**: control period 37 (t = 3.6 s): schedule_inconsistent: stale curvature schedule: stage residual 0.0105 rad > 0.005 rad; cumulative residual 0.01039 rad > 0.01 rad; schedule correction skipped: time budget 0.08 s would be exceeded
- **C3 stagewise, ≤ 8 corr., qualified, margin 0.05 (2B.2 D3)**: completed
- **C4 stagewise, ≤ 8 corr., qualified, margin 0.10 (2B.2 D4)**: completed
- **D0 linearized, no trust region, 1 solve, reject**: control period 4 (t = 0.30000000000000004 s): schedule_inconsistent: stale curvature schedule: stage residual 0.3227 rad > 0.005 rad; cumulative residual 0.3227 rad > 0.01 rad
- **D  linearized, TR 0.1, 1 solve, reject, margin 0.05**: control period 5 (t = 0.4 s): schedule_inconsistent: stale curvature schedule: stage residual 0.005199 rad > 0.005 rad
- **D' linearized, TR 0.1, 1 solve, reject, margin 0.10**: control period 26 (t = 2.5 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- **E  linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.05**: control period 103 (t = 10.200000000000001 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3564 m is beyond the right track boundary (|ey| ≤ 0.35 m)
- **E' linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.10**: control period 26 (t = 2.5 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT

## Prediction-horizon study (dt = 0.1 s = control period)

| configuration | outcomes (laps, per run) | failed cycles | fully consistent / qualified accepted | NLP solves: mean / max | Ipopt iterations: mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m: mean / max solves |
|:--|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| C4 stagewise, ≤ 8 corr., qualified, margin 0.10, N = 10 | 2.46; 2.46; 2.46 | 0 | 750 / 0 | 2.12 / 8 | 14.3 / 21 | 15.2 / 37.3 / 50.2 / 68.0 | 0 / 0 | 2.95 / 3 |
| B' stagewise, ≤ 8 corr., reject, margin 0.10, N = 10 | 2.46; 2.46; 2.46 | 0 | 750 / 0 | 2.12 / 8 | 14.3 / 21 | 15.8 / 38.3 / 52.4 / 74.5 | 0 / 0 | 2.95 / 3 |
| L1 linearized, no TR, 1 solve, reject, margin 0.10, N = 10 | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 3 | 0 / 0 | 1.00 / 1 | 15.0 / 15 | 12.4 / 13.3 / 13.4 / 13.4 | 0 / 0 | – |
| L8 linearized, no TR, ≤ 8 corr., reject, margin 0.10, N = 10 | 2.48; 2.48; 2.48 | 0 | 750 / 0 | 1.14 / 3 | 15.6 / 27 | 8.9 / 18.8 / 23.0 / 129.4 | 1 / 1 | 1.22 / 2 |
| L8s linearized, no TR, ≤ 8 corr., reject, margin 0.05, N = 10 | stop 0.08 (c13); stop 0.08 (c13); stop 0.08 (c13) | 3 | 36 / 0 | 1.69 / 6 | 15.5 / 19 | 9.4 / 69.7 / 74.0 / 74.3 | 0 / 0 | – |
| T1 linearized, TR 0.1, 1 solve, reject, margin 0.10, N = 10 | stop 0.08 (c13); stop 0.08 (c13); stop 0.08 (c13) | 3 | 36 / 0 | 1.00 / 1 | 16.0 / 19 | 10.2 / 11.9 / 12.4 / 12.5 | 0 / 0 | – |
| C4 stagewise, ≤ 8 corr., qualified, margin 0.10, N = 20 | 2.33; 2.33; 2.33 | 0 | 709 / 41 | 2.43 / 6 | 14.5 / 21 | 27.0 / 70.3 / 76.4 / 79.4 | 0 / 0 | 3.24 / 5 |
| B' stagewise, ≤ 8 corr., reject, margin 0.10, N = 20 | stop 0.00 (c1); stop 0.23 (c37); stop 0.23 (c37) | 3 | 72 / 0 | 2.77 / 6 | 14.4 / 21 | 31.4 / 70.5 / 89.1 / 123.8 | 1 / 1 | 2.83 / 5 |
| L1 linearized, no TR, 1 solve, reject, margin 0.10, N = 20 | stop 0.02 (c5); stop 0.02 (c5); stop 0.02 (c5) | 3 | 12 / 0 | 1.00 / 1 | 16.4 / 19 | 16.2 / 22.4 / 22.4 / 22.4 | 0 / 0 | – |
| L8 linearized, no TR, ≤ 8 corr., reject, margin 0.10, N = 20 | stop 0.11 (c19); stop 0.11 (c19); stop 0.11 (c19) | 3 | 54 / 0 | 1.32 / 4 | 16.2 / 24 | 15.8 / 55.5 / 78.1 / 78.2 | 0 / 0 | – |
| L8s linearized, no TR, ≤ 8 corr., reject, margin 0.05, N = 20 | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 3 | 15 / 0 | 1.67 / 4 | 16.3 / 21 | 18.6 / 74.3 / 75.7 / 76.0 | 0 / 0 | – |
| T1 linearized, TR 0.1, 1 solve, reject, margin 0.10, N = 20 | stop 0.16 (c26); stop 0.16 (c26); stop 0.16 (c26) | 3 | 75 / 0 | 1.00 / 1 | 20.3 / 84 | 18.0 / 26.4 / 163.1 / 164.4 | 3 / 3 | – |

| configuration | plant violations | min plant edge distance [m] | max stage / cum. residual [rad] | max κ model error [1/m] | config. changes / saturated stages / trust region active | reference: first-step / max position [m] | heading [rad] / progress / ey [m] | min predicted edge distance [m] |
|:--|--:|--:|:-:|--:|:-:|:-:|:-:|--:|
| C4 stagewise, ≤ 8 corr., qualified, margin 0.10, N = 10 | 0 | 0.077 | 5.0e-03 / 7.4e-03 | 1.7e-02 | 15 / 0 / 0 | 1.7e-02 / 4.1e-02 | 7.4e-03 / 3.4e-02 / 3.7e-02 | 0.041 |
| B' stagewise, ≤ 8 corr., reject, margin 0.10, N = 10 | 0 | 0.077 | 5.0e-03 / 7.4e-03 | 1.7e-02 | 15 / 0 / 0 | 1.7e-02 / 4.1e-02 | 7.4e-03 / 3.4e-02 / 3.7e-02 | 0.041 |
| L1 linearized, no TR, 1 solve, reject, margin 0.10, N = 10 | 0 | 0.350 | NaN / NaN | NaN | 0 / 0 / 0 | NaN / NaN | NaN / NaN / NaN | NaN |
| L8 linearized, no TR, ≤ 8 corr., reject, margin 0.10, N = 10 | 0 | 0.098 | 4.7e-03 / 4.7e-03 | 1.8e-02 | 72 / 39 / 0 | 1.7e-02 / 3.5e-02 | 4.7e-03 / 3.1e-02 / 3.5e-02 | 0.053 |
| L8s linearized, no TR, ≤ 8 corr., reject, margin 0.05, N = 10 | 0 | 0.050 | 3.6e-03 / 3.6e-03 | 7.5e-03 | 6 / 6 / 0 | 9.0e-03 / 1.6e-02 | 3.6e-03 / 1.7e-02 / 1.5e-02 | 0.024 |
| T1 linearized, TR 0.1, 1 solve, reject, margin 0.10, N = 10 | 0 | 0.100 | 3.0e-03 / 3.0e-03 | 1.3e-02 | 0 / 0 / 30 | 8.0e-03 / 1.5e-02 | 3.0e-03 / 1.4e-02 / 1.4e-02 | 0.070 |
| C4 stagewise, ≤ 8 corr., qualified, margin 0.10, N = 20 | 0 | 0.226 | 2.5e-02 / 2.7e-02 | 5.6e-02 | 51 / 0 / 0 | 1.8e-02 / 7.2e-02 | 2.7e-02 / 4.3e-02 / 5.4e-02 | 0.062 |
| B' stagewise, ≤ 8 corr., reject, margin 0.10, N = 20 | 0 | 0.232 | 4.9e-03 / 5.2e-03 | 1.8e-02 | 4 / 0 / 0 | 1.5e-02 / 3.9e-02 | 5.2e-03 / 3.8e-02 / 3.9e-02 | 0.062 |
| L1 linearized, no TR, 1 solve, reject, margin 0.10, N = 20 | 0 | 0.337 | 6.0e-04 / 6.0e-04 | 2.3e-03 | 0 / 3 / 0 | 4.5e-08 / 3.6e-03 | 6.0e-04 / 3.6e-03 / 3.6e-03 | 0.263 |
| L8 linearized, no TR, ≤ 8 corr., reject, margin 0.10, N = 20 | 0 | 0.298 | 3.3e-03 / 3.3e-03 | 1.3e-02 | 12 / 12 / 0 | 8.2e-04 / 2.5e-02 | 3.3e-03 / 2.6e-02 / 2.4e-02 | 0.078 |
| L8s linearized, no TR, ≤ 8 corr., reject, margin 0.05, N = 20 | 0 | 0.318 | 2.8e-03 / 2.8e-03 | 1.1e-02 | 0 / 3 / 0 | 6.7e-08 / 6.3e-03 | 2.8e-03 / 5.0e-03 / 6.0e-03 | 0.043 |
| T1 linearized, TR 0.1, 1 solve, reject, margin 0.10, N = 20 | 0 | 0.242 | 3.7e-03 / 3.9e-03 | 1.3e-02 | 0 / 6 / 66 | 1.4e-02 / 3.1e-02 | 3.9e-03 / 2.1e-02 / 3.0e-02 | 0.082 |

## Straight → arc (κ 0 → 0.5), one control cycle, cold start

| v [m/s] | φ | formulation | status | NLP solves | stage / cum. residual [rad] | κ model error | reference: first-step / max position [m] |
|:-:|:-:|:--|:--|--:|:-:|--:|:-:|
| 1.0 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.5e-01 / 1.5e-01 | 3.6e-01 | 2.8e-04 / 1.6e-01 |
| 1.0 | 0.1 | linearized TR 0.1 | success | 1 | 2.6e-03 / 2.5e-03 | 7.5e-03 | 2.6e-04 / 5.8e-03 |
| 1.0 | 0.1 | linearized, no TR | schedule_inconsistent | 1 | 7.0e-02 / 7.0e-02 | 1.6e-01 | 2.7e-04 / 9.3e-02 |
| 1.0 | 0.5 | stagewise | schedule_inconsistent | 1 | 1.4e-01 / 1.6e-01 | 3.3e-01 | 9.1e-04 / 1.6e-01 |
| 1.0 | 0.5 | linearized TR 0.1 | success | 1 | 2.5e-03 / 2.6e-03 | 6.7e-03 | 7.9e-04 / 4.6e-03 |
| 1.0 | 0.5 | linearized, no TR | schedule_inconsistent | 1 | 6.4e-03 / 6.3e-03 | 1.5e-02 | 8.0e-04 / 6.1e-03 |
| 1.0 | 0.9 | stagewise | schedule_inconsistent | 1 | 1.1e-01 / 1.7e-01 | 2.7e-01 | 7.3e-04 / 1.5e-01 |
| 1.0 | 0.9 | linearized TR 0.1 | success | 1 | 1.6e-03 / 1.8e-03 | 4.3e-03 | 5.3e-04 / 6.4e-03 |
| 1.0 | 0.9 | linearized, no TR | schedule_inconsistent | 1 | 1.6e-02 / 1.6e-02 | 3.7e-02 | 5.5e-04 / 1.4e-02 |
| 2.0 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.1e-01 / 1.1e-01 | 2.6e-01 | 1.0e-03 / 1.7e-01 |
| 2.0 | 0.1 | linearized TR 0.1 | success | 1 | 2.8e-03 / 2.8e-03 | 8.3e-03 | 9.5e-04 / 6.9e-03 |
| 2.0 | 0.1 | linearized, no TR | schedule_inconsistent | 1 | 7.9e-02 / 7.9e-02 | 1.8e-01 | 9.6e-04 / 1.5e-01 |
| 2.0 | 0.5 | stagewise | schedule_inconsistent | 1 | 6.3e-02 / 1.2e-01 | 1.5e-01 | 3.1e-03 / 1.7e-01 |
| 2.0 | 0.5 | linearized TR 0.1 | success | 1 | 1.4e-03 / 1.7e-03 | 3.9e-03 | 2.7e-03 / 8.5e-03 |
| 2.0 | 0.5 | linearized, no TR | schedule_inconsistent | 1 | 1.8e-02 / 1.8e-02 | 4.1e-02 | 2.7e-03 / 2.6e-02 |
| 2.0 | 0.9 | stagewise | schedule_inconsistent | 1 | 1.2e-01 / 1.4e-01 | 2.7e-01 | 2.0e-03 / 1.7e-01 |
| 2.0 | 0.9 | linearized TR 0.1 | success | 1 | 5.0e-04 / 5.0e-04 | 2.4e-03 | 1.3e-03 / 8.2e-03 |
| 2.0 | 0.9 | linearized, no TR | schedule_inconsistent | 1 | 3.8e-02 / 3.8e-02 | 8.8e-02 | 1.4e-03 / 5.2e-02 |
| 2.8 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.3e-01 / 1.3e-01 | 2.9e-01 | 1.8e-03 / 2.1e-01 |
| 2.8 | 0.1 | linearized TR 0.1 | success | 1 | 1.4e-03 / 1.4e-03 | 4.2e-03 | 1.8e-03 / 9.5e-03 |
| 2.8 | 0.1 | linearized, no TR | schedule_inconsistent | 1 | 5.6e-02 / 5.6e-02 | 1.3e-01 | 1.8e-03 / 1.0e-01 |
| 2.8 | 0.5 | stagewise | schedule_inconsistent | 1 | 1.2e-01 / 1.5e-01 | 2.8e-01 | 4.9e-03 / 2.1e-01 |
| 2.8 | 0.5 | linearized TR 0.1 | success | 1 | 2.1e-03 / 2.2e-03 | 5.9e-03 | 4.9e-03 / 9.1e-03 |
| 2.8 | 0.5 | linearized, no TR | schedule_inconsistent | 1 | 9.7e-03 / 9.7e-03 | 2.2e-02 | 4.9e-03 / 1.7e-02 |
| 2.8 | 0.9 | stagewise | schedule_inconsistent | 1 | 9.3e-02 / 1.5e-01 | 2.1e-01 | 1.8e-03 / 1.9e-01 |
| 2.8 | 0.9 | linearized TR 0.1 | success | 1 | 4.3e-04 / 4.4e-04 | 1.2e-03 | 1.8e-03 / 9.0e-03 |
| 2.8 | 0.9 | linearized, no TR | schedule_inconsistent | 1 | 2.9e-02 / 2.9e-02 | 6.7e-02 | 1.8e-03 / 4.0e-02 |

## Single-segment regression

| track | formulation | status | cycles | s_end [m] | identical to stagewise |
|:--|:--|:--|--:|--:|:-:|
| straight 150 m | stagewise | success | 100 | 47.9466 | true |
| straight 150 m | linearized | success | 100 | 47.9466 | true |
| circle R = 1 m | stagewise | success | 70 | 13.8465 | true |
| circle R = 1 m | linearized | success | 70 | 13.8465 | true |

