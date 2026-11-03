# Multi-segment NMPC experiments (Milestone 2B.2)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; N = 20, dt = 0.1 s, RK4
- Every cycle's `solve!` time includes schedule construction, parameter update, all NLP solves, knot validation, schedule-consistency test and the event-aware reference check.
- Plants: event-aware (`integrate_on_track`) on multi-segment tracks; the Milestone 2A plant on single-segment tracks.

## Experiments A and B: single-segment regression (baseline configuration)

| experiment | status | cycles | statuses | NLP solves/cycle | s_end [m] | median / p99 / max [ms] | mean reference-check time [ms] | plant violations |
|:--|:--|--:|:--|:--|--:|:-:|--:|--:|
| A: straight 150 m | success | 100 | success: 100 | 1: 100 | 47.9466 | 7.6 / 13.0 / 23.0 | 0.06 | 0 |
| B: circle R = 1 m | success | 70 | success: 70 | 1: 70 | 13.8465 | 14.8 / 21.0 / 23.0 | 0.08 | 0 |

The trajectories are compared bitwise with Milestone 2A separately (see docs/multisegment_nmpc.md).

## Experiment C: straight → arc (κ 0 → 0.5), one control cycle

The state is placed so that the reference crosses the boundary at a fraction φ of the first interval (`s0 = 3 − φ·v·dt`, `ey = epsi = 0`). *Warm*: after an accepted solve one period earlier, the measured state is that solve's predicted next state (so φ is only approximate in warm rows). Configurations: **default** (1 solve, reject stale) and **corrections** (≤ 8 corrections, no time limit; this shows how many solves self-consistency needs).

| v [m/s] | φ | init | config | status | NLP solves | schedule it. | stage / cum. residual [rad] | reference: pos / heading / ey error | reference passed |
|:-:|:-:|:--|:--|:--|--:|--:|:-:|:-:|:-:|
| 1.0 | 0.1 | cold | default | schedule_inconsistent | 1 | 4 | 1.5e-01 / 1.5e-01 | 1.6e-01 / 1.5e-01 / 1.6e-01 | true |
| 1.0 | 0.1 | warm | default | schedule_inconsistent | 1 | 3 | 1.0e-02 / 1.0e-02 | 2.8e-02 / 1.0e-02 / 2.8e-02 | true |
| 1.0 | 0.1 | cold | corrections | success | 4 | 11 | 2.1e-03 / 2.1e-03 | 1.5e-03 / 2.1e-03 / 1.4e-03 | true |
| 1.0 | 0.1 | warm | corrections | success | 2 | 6 | 3.1e-03 / 3.1e-03 | 1.6e-02 / 3.1e-03 / 1.6e-02 | true |
| 1.0 | 0.5 | cold | default | schedule_inconsistent | 1 | 4 | 1.4e-01 / 1.6e-01 | 1.6e-01 / 1.6e-01 / 1.6e-01 | true |
| 1.0 | 0.5 | warm | default | schedule_inconsistent | 1 | 3 | 1.1e-02 / 1.1e-02 | 3.1e-02 / 1.1e-02 / 3.1e-02 | true |
| 1.0 | 0.5 | cold | corrections | success | 4 | 12 | 2.2e-03 / 2.2e-03 | 5.3e-03 / 2.2e-03 / 5.2e-03 | true |
| 1.0 | 0.5 | warm | corrections | success | 2 | 6 | 3.4e-03 / 3.4e-03 | 1.8e-02 / 3.4e-03 / 1.8e-02 | true |
| 1.0 | 0.9 | cold | default | schedule_inconsistent | 1 | 4 | 1.1e-01 / 1.7e-01 | 1.5e-01 / 1.7e-01 / 1.5e-01 | true |
| 1.0 | 0.9 | warm | default | schedule_inconsistent | 1 | 3 | 1.2e-02 / 1.2e-02 | 3.3e-02 / 1.2e-02 / 3.3e-02 | true |
| 1.0 | 0.9 | cold | corrections | success | 4 | 13 | 2.2e-03 / 2.2e-03 | 8.3e-03 / 2.2e-03 / 8.3e-03 | true |
| 1.0 | 0.9 | warm | corrections | success | 2 | 6 | 3.7e-03 / 3.7e-03 | 1.9e-02 / 3.7e-03 / 1.9e-02 | true |
| 2.0 | 0.1 | cold | default | schedule_inconsistent | 1 | 3 | 1.1e-01 / 1.1e-01 | 1.7e-01 / 1.1e-01 / 1.7e-01 | true |
| 2.0 | 0.1 | warm | default | success | 1 | 3 | 5.0e-03 / 5.0e-03 | 1.7e-02 / 5.0e-03 / 1.7e-02 | true |
| 2.0 | 0.1 | cold | corrections | success | 4 | 12 | 3.0e-03 / 3.0e-03 | 9.5e-03 / 3.0e-03 / 9.5e-03 | true |
| 2.0 | 0.1 | warm | corrections | success | 1 | 3 | 5.0e-03 / 5.0e-03 | 1.7e-02 / 5.0e-03 / 1.7e-02 | true |
| 2.0 | 0.5 | cold | default | schedule_inconsistent | 1 | 3 | 6.3e-02 / 1.2e-01 | 1.7e-01 / 1.2e-01 / 1.7e-01 | true |
| 2.0 | 0.5 | warm | default | schedule_inconsistent | 1 | 2 | 6.6e-03 / 6.7e-03 | 1.6e-02 / 6.7e-03 / 1.6e-02 | true |
| 2.0 | 0.5 | cold | corrections | success | 4 | 13 | 3.0e-03 / 3.0e-03 | 1.3e-02 / 3.0e-03 / 1.3e-02 | true |
| 2.0 | 0.5 | warm | corrections | success | 2 | 5 | 1.9e-03 / 1.9e-03 | 8.5e-03 / 1.9e-03 / 8.2e-03 | true |
| 2.0 | 0.9 | cold | default | schedule_inconsistent | 1 | 4 | 1.2e-01 / 1.4e-01 | 1.7e-01 / 1.4e-01 / 1.7e-01 | true |
| 2.0 | 0.9 | warm | default | schedule_inconsistent | 1 | 3 | 7.5e-03 / 7.6e-03 | 2.4e-02 / 7.6e-03 / 2.4e-02 | true |
| 2.0 | 0.9 | cold | corrections | success | 4 | 14 | 3.2e-03 / 3.2e-03 | 1.0e-02 / 3.2e-03 / 1.0e-02 | true |
| 2.0 | 0.9 | warm | corrections | success | 2 | 6 | 2.2e-03 / 2.2e-03 | 1.5e-02 / 2.2e-03 / 1.5e-02 | true |
| 2.8 | 0.1 | cold | default | schedule_inconsistent | 1 | 3 | 1.3e-01 / 1.3e-01 | 2.1e-01 / 1.3e-01 / 2.1e-01 | true |
| 2.8 | 0.1 | warm | default | success | 1 | 3 | 2.0e-03 / 2.0e-03 | 1.7e-02 / 2.0e-03 / 1.7e-02 | true |
| 2.8 | 0.1 | cold | corrections | success | 4 | 12 | 4.4e-03 / 4.4e-03 | 5.0e-03 / 4.4e-03 / 4.8e-03 | true |
| 2.8 | 0.1 | warm | corrections | success | 1 | 3 | 2.0e-03 / 2.0e-03 | 1.7e-02 / 2.0e-03 / 1.7e-02 | true |
| 2.8 | 0.5 | cold | default | schedule_inconsistent | 1 | 3 | 1.2e-01 / 1.5e-01 | 2.1e-01 / 1.5e-01 / 2.1e-01 | true |
| 2.8 | 0.5 | warm | default | success | 1 | 3 | 4.3e-03 / 4.3e-03 | 1.9e-02 / 4.3e-03 / 1.9e-02 | true |
| 2.8 | 0.5 | cold | corrections | success | 4 | 13 | 4.0e-03 / 4.0e-03 | 1.0e-02 / 4.0e-03 / 9.7e-03 | true |
| 2.8 | 0.5 | warm | corrections | success | 1 | 3 | 4.3e-03 / 4.3e-03 | 1.9e-02 / 4.3e-03 / 1.9e-02 | true |
| 2.8 | 0.9 | cold | default | schedule_inconsistent | 1 | 4 | 9.3e-02 / 1.5e-01 | 1.9e-01 / 1.5e-01 / 1.9e-01 | true |
| 2.8 | 0.9 | warm | default | schedule_inconsistent | 1 | 3 | 6.8e-03 / 6.9e-03 | 2.1e-02 / 6.9e-03 / 2.1e-02 | true |
| 2.8 | 0.9 | cold | corrections | success | 4 | 14 | 4.3e-03 / 4.3e-03 | 1.2e-02 / 4.3e-03 / 1.2e-02 | true |
| 2.8 | 0.9 | warm | corrections | success | 2 | 6 | 1.9e-03 / 1.9e-03 | 1.3e-02 / 1.9e-03 / 1.3e-02 | true |

## Experiment D: `quarter_track()` closed loop, start `[0, 0, 0, 0.5]`, up to 250 cycles

Each configuration was run three times; the detailed tables below are from run 1.

| configuration | run 1 outcome | cycles | laps | statuses | NLP solves/cycle | outcomes of runs 1–3 |
|:--|:--|--:|--:|:--|:--|:--|
| D1 baseline: 1 solve, reject stale, margin 0.05 | stopped: controller_failure | 1 | 0.00 | schedule_inconsistent: 1 | 1: 1 | stopped at cycle 1 (0.00 laps); stopped at cycle 1 (0.00 laps); stopped at cycle 1 (0.00 laps) |
| D2 reject stale, ≤ 8 corrections in 80 ms, margin 0.05 | stopped: controller_failure | 37 | 0.24 | schedule_inconsistent: 1, success: 36 | 1: 3, 2: 12, 3: 12, 4: 5, 5: 4, 6: 1 | stopped at cycle 37 (0.24 laps); stopped at cycle 37 (0.24 laps); stopped at cycle 37 (0.24 laps) |
| D3 qualified, ≤ 8 corrections in 80 ms, margin 0.05 | completed all cycles | 250 | 2.37 | success: 237, success_stale_schedule: 13 | 1: 70, 2: 68, 3: 62, 4: 24, 5: 19, 6: 5, 7: 1, 8: 1 | 2.37 laps; 2.37 laps; 2.37 laps |
| D4 qualified, ≤ 8 corrections in 80 ms, margin 0.10 (warm) | completed all cycles | 250 | 2.33 | success: 237, success_stale_schedule: 13 | 1: 77, 2: 72, 3: 49, 4: 24, 5: 22, 6: 6 | 2.33 laps; 2.33 laps; 2.33 laps |
| D5 = D4 with cold start every cycle | stopped: controller_failure | 9 | 0.05 | reference_violation: 1, success: 6, success_stale_schedule: 2 | 1: 1, 4: 1, 5: 3, 6: 2, 7: 2 | stopped at cycle 9 (0.05 laps); stopped at cycle 9 (0.05 laps); stopped at cycle 9 (0.05 laps) |

Stop reasons:

- **D1 baseline: 1 solve, reject stale, margin 0.05**: control period 1 (t = 0.0 s): schedule_inconsistent: stale curvature schedule: stage residual 0.1788 rad > 0.005 rad; cumulative residual 0.2144 rad > 0.01 rad
- **D2 reject stale, ≤ 8 corrections in 80 ms, margin 0.05**: control period 37 (t = 3.6 s): schedule_inconsistent: stale curvature schedule: stage residual 0.0105 rad > 0.005 rad; cumulative residual 0.01039 rad > 0.01 rad; schedule correction skipped: time budget 0.08 s would be exceeded
- **D3 qualified, ≤ 8 corrections in 80 ms, margin 0.05**: none
- **D4 qualified, ≤ 8 corrections in 80 ms, margin 0.10 (warm)**: none
- **D5 = D4 with cold start every cycle**: control period 9 (t = 0.8 s): reference_violation: stale curvature schedule: stage residual 0.5503 rad > 0.005 rad; cumulative residual 1.44 rad > 0.01 rad; reference check on the actual track failed: 66 violation(s), first at t = 1.34 s: lateral offset ey = -0.3624 m is beyond the right track boundary (|ey| ≤ 0.35 m); schedule correction did not converge: rollout with the schedule leaves the model domain

### Constraints, schedule and prediction accuracy (accepted cycles)

| configuration | plant violations | max plant \|ey\| [m] | crossings | stale cycles | schedule non-converged | max schedule it. | max stage / cum. residual [rad] | reference pos / heading / s / ey error | max one-step plant mismatch [m] |
|:--|--:|--:|--:|--:|--:|--:|:-:|:-:|--:|
| D1 baseline: 1 solve, reject stale, margin 0.05 | 0 | 0.000 | 0 | 1 | 0 | 4 | NaN / NaN | NaN / NaN / NaN / NaN | NaN |
| D2 reject stale, ≤ 8 corrections in 80 ms, margin 0.05 | 0 | 0.147 | 2 | 1 | 0 | 27 | 4.6e-03 / 4.1e-03 | 3.1e-02 / 4.1e-03 / 3.1e-02 / 2.9e-02 | 1.7e-02 |
| D3 qualified, ≤ 8 corrections in 80 ms, margin 0.05 | 0 | 0.152 | 26 | 13 | 0 | 35 | 5.0e-02 / 5.4e-02 | 1.1e-01 / 5.4e-02 / 7.0e-02 / 7.5e-02 | 1.7e-02 |
| D4 qualified, ≤ 8 corrections in 80 ms, margin 0.10 (warm) | 0 | 0.124 | 25 | 13 | 0 | 33 | 2.5e-02 / 2.7e-02 | 7.2e-02 / 2.7e-02 / 4.3e-02 / 5.4e-02 | 1.8e-02 |
| D5 = D4 with cold start every cycle | 0 | 0.044 | 0 | 2 | 0 | 39 | 7.8e-03 / 7.7e-03 | 3.9e-02 / 7.7e-03 / 3.4e-02 / 3.9e-02 | 1.9e-07 |

### Computation time per control cycle [ms] (all cycles, including the first)

| configuration | median | p95 | p99 | max | > 80 ms | > 100 ms | mean schedule | mean update + guess | mean optimize (wall) | mean Ipopt | mean knot validation | mean reference check | solver / scheduling / reference failures |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:-:|
| D1 baseline: 1 solve, reject stale, margin 0.05 | 20.1 | 20.1 | 20.1 | 20.1 | 0 | 0 | 0.35 | 0.07 | 19.5 | 15.0 | 0.07 | 0.07 | 0 / 0 / 0 |
| D2 reject stale, ≤ 8 corrections in 80 ms, margin 0.05 | 34.2 | 68.6 | 71.2 | 71.7 | 0 | 0 | 0.07 | 0.06 | 35.1 | 35.1 | 1.02 | 0.23 | 0 / 0 / 0 |
| D3 qualified, ≤ 8 corrections in 80 ms, margin 0.05 | 30.3 | 70.1 | 77.1 | 110.9 | 1 | 1 | 0.05 | 0.15 | 32.4 | 32.3 | 0.13 | 0.54 | 0 / 0 / 0 |
| D4 qualified, ≤ 8 corrections in 80 ms, margin 0.10 (warm) | 27.4 | 70.1 | 78.2 | 79.4 | 0 | 0 | 0.04 | 0.08 | 31.0 | 31.1 | 0.15 | 0.20 | 0 / 0 / 0 |
| D5 = D4 with cold start every cycle | 70.0 | 79.9 | 80.8 | 81.1 | 1 | 0 | 1.79 | 0.06 | 61.4 | 61.3 | 0.28 | 0.38 | 0 / 0 / 1 |

