# Segment-pair curvature benchmark (Milestone 2B.3B)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `[0, 0, 0, 0.5]`
- 250 control cycles (control period 0.1 s) per run, 3 interleaved runs per configuration; event-aware plant; all cycles counted
- Corrections use the existing mechanism (≤ 8 per cycle, 80 ms budget). Objective, weights and limits unchanged; horizon N and corridor margin as stated per table
- "Corridor" = tightened planning corridor `|ey| ≤ width/2 − margin`; "track" = physical edge `|ey| ≤ width/2`

Geometric identity: max |heading_increment − Δcenterline_heading| over quarter, lab and a closed loop (up to 3 laps, Δs from 1e-4 to 1.3 m): **1.1e-14 rad**.

## Quarter track, N = 10 (1.0 s horizon), margin 0.05 m

| configuration | laps per run | fully consistent cycles | failures: solver / scheduling / segment-config. / other stale / reference | NLP solves mean / max | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m mean solves |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | stop 0.32 (c42); stop 0.32 (c42); stop 0.32 (c42) | 123 / 126 (97.6 %) | 0 / 0 / 0 / 3 / 0 | 2.40 / 9 | 14.6 / 20 | 16.2 / 41.7 / 77.8 / 80.0 | 0 / 0 | 3.17 |
| B progress-linearized, no TR, ≤ 8 corr., reject | stop 0.08 (c13); stop 0.08 (c13); stop 0.00 (c1) | 24 / 27 (88.9 %) | 0 / 0 / 0 / 3 / 0 | 1.63 / 6 | 15.6 / 23 | 10.6 / 60.6 / 70.5 / 71.8 | 0 / 0 | – |
| C segment-pair, 1 solve | stop 0.01 (c5); stop 0.01 (c5); stop 0.01 (c5) | 12 / 15 (80.0 %) | 0 / 0 / 3 / 0 / 0 | 1.00 / 1 | 16.0 / 17 | 9.9 / 13.2 / 13.7 / 13.8 | 0 / 0 | – |
| D segment-pair, ≤ 8 corr. (residual policy) | stop 0.97 (c97); stop 0.97 (c97); stop 0.97 (c97) | 288 / 291 (99.0 %) | 0 / 0 / 0 / 0 / 3 | 1.09 / 2 | 15.1 / 20 | 9.0 / 18.1 / 20.9 / 112.2 | 1 / 1 | 1.17 |
| D2 segment-pair, ≤ 8 corr., reject segment changes | stop 0.97 (c97); stop 0.97 (c97); stop 0.97 (c97) | 288 / 291 (99.0 %) | 0 / 0 / 0 / 0 / 3 | 1.11 / 2 | 15.1 / 20 | 9.2 / 18.9 / 20.3 / 47.5 | 0 / 0 | 1.17 |
| E segment-pair + membership bounds, 1 solve | stop 0.22 (c32); stop 0.22 (c32); stop 0.22 (c32) | 93 / 96 (96.9 %) | 3 / 0 / 0 / 0 / 0 | 1.00 / 1 | 17.1 / 69 | 9.5 / 14.2 / 69.4 / 77.6 | 0 / 0 | 1.00 |
| E8 segment-pair + membership bounds, ≤ 8 corr. | stop 0.22 (c32); stop 0.22 (c32); stop 0.22 (c32) | 93 / 96 (96.9 %) | 3 / 0 / 0 / 0 / 0 | 1.00 / 1 | 17.1 / 69 | 9.7 / 15.1 / 68.7 / 69.0 | 0 / 0 | 1.00 |

| configuration | plant violations | min plant clearance: track / corridor [m] | predicted (reference) clearance: track / corridor [m] | max stage / cum. residual [rad] | accepted cycles with segment change | reference error: first-step / horizon [m] | heading [rad] / progress [m] | min Δs* [m] |
|:--|--:|:-:|:-:|:-:|--:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.048 / -0.002 | 0.014 / -0.036 | 4.8e-03 / 4.8e-03 | 3 | 1.5e-02 / 2.5e-02 | 4.8e-03 / 2.3e-02 | 0.075 |
| B progress-linearized, no TR, ≤ 8 corr., reject | 0 | 0.050 / -0.000 | 0.024 / -0.026 | 3.6e-03 / 3.6e-03 | 2 | 9.0e-03 / 1.6e-02 | 3.6e-03 / 1.7e-02 | 0.059 |
| C segment-pair, 1 solve | 0 | 0.346 / 0.296 | 0.003 / -0.047 | 6.0e-09 / 6.0e-09 | 0 | 1.2e-08 / 2.8e-02 | 6.0e-09 / 2.6e-02 | 0.058 |
| D segment-pair, ≤ 8 corr. (residual policy) | 0 | 0.046 / -0.004 | 0.003 / -0.047 | 3.6e-03 / 3.6e-03 | 6 | 1.4e-02 / 3.8e-02 | 3.6e-03 / 3.6e-02 | 0.058 |
| D2 segment-pair, ≤ 8 corr., reject segment changes | 0 | 0.046 / -0.004 | 0.003 / -0.047 | 6.0e-09 / 6.0e-09 | 0 | 1.4e-02 / 3.8e-02 | 6.0e-09 / 3.6e-02 | 0.058 |
| E segment-pair + membership bounds, 1 solve | 0 | 0.048 / -0.002 | 0.003 / -0.047 | 1.1e-07 / 1.1e-07 | 0 | 1.4e-02 / 2.8e-02 | 7.5e-08 / 2.6e-02 | 0.058 |
| E8 segment-pair + membership bounds, ≤ 8 corr. | 0 | 0.048 / -0.002 | 0.003 / -0.047 | 1.1e-07 / 1.1e-07 | 0 | 1.4e-02 / 2.8e-02 | 7.5e-08 / 2.6e-02 | 0.058 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: control period 42 (t = 4.1000000000000005 s): schedule_inconsistent: stale curvature schedule: stage residual 0.01915 rad > 0.005 rad; cumulative residual 0.01715 rad > 0.01 rad
- **B progress-linearized, no TR, ≤ 8 corr., reject**: control period 13 (t = 1.2000000000000002 s): schedule_inconsistent: stale curvature schedule: stage residual 0.02976 rad > 0.005 rad; cumulative residual 0.02976 rad > 0.01 rad; schedule correction skipped: time budget 0.08 s would be exceeded
- **C segment-pair, 1 solve**: control period 5 (t = 0.4 s): schedule_inconsistent: stale curvature schedule: stage residual 0.05736 rad > 0.005 rad; cumulative residual 0.05736 rad > 0.01 rad
- **D segment-pair, ≤ 8 corr. (residual policy)**: control period 97 (t = 9.600000000000001 s): reference_violation: reference check on the actual track failed: 1 violation(s), first at t = 0.74 s: lateral offset ey = 0.3515 m is beyond the left track boundary (|ey| ≤ 0.35 m)
- **D2 segment-pair, ≤ 8 corr., reject segment changes**: control period 97 (t = 9.600000000000001 s): reference_violation: reference check on the actual track failed: 1 violation(s), first at t = 0.74 s: lateral offset ey = 0.3515 m is beyond the left track boundary (|ey| ≤ 0.35 m)
- **E segment-pair + membership bounds, 1 solve**: control period 32 (t = 3.1 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- **E8 segment-pair + membership bounds, ≤ 8 corr.**: control period 32 (t = 3.1 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT

## Quarter track, N = 10 (1.0 s horizon), margin 0.1 m

| configuration | laps per run | fully consistent cycles | failures: solver / scheduling / segment-config. / other stale / reference | NLP solves mean / max | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m mean solves |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | 2.46; 2.46; 2.46 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 2.12 / 8 | 14.3 / 21 | 16.1 / 39.3 / 53.2 / 72.5 | 0 / 0 | 2.95 |
| B progress-linearized, no TR, ≤ 8 corr., reject | 2.48; 2.48; 2.48 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 1.14 / 3 | 15.6 / 27 | 9.3 / 19.6 / 23.3 / 28.6 | 0 / 0 | 1.22 |
| C segment-pair, 1 solve | stop 0.01 (c5); stop 0.01 (c5); stop 0.01 (c5) | 12 / 15 (80.0 %) | 0 / 0 / 3 / 0 / 0 | 1.00 / 1 | 16.2 / 18 | 10.3 / 13.2 / 13.4 / 13.4 | 0 / 0 | – |
| D segment-pair, ≤ 8 corr. (residual policy) | 2.48; 2.48; 2.48 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 1.06 / 2 | 15.2 / 20 | 9.1 / 18.9 / 20.7 / 28.2 | 0 / 0 | 1.17 |
| D2 segment-pair, ≤ 8 corr., reject segment changes | 2.48; 2.48; 2.48 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 1.10 / 2 | 15.2 / 21 | 9.2 / 19.4 / 21.7 / 41.9 | 0 / 0 | 1.33 |
| E segment-pair + membership bounds, 1 solve | stop 0.22 (c34); stop 0.22 (c34); stop 0.22 (c34) | 99 / 102 (97.1 %) | 3 / 0 / 0 / 0 / 0 | 1.00 / 1 | 17.9 / 96 | 9.7 / 14.2 / 79.3 / 85.0 | 1 / 0 | 1.00 |
| E8 segment-pair + membership bounds, ≤ 8 corr. | stop 0.22 (c34); stop 0.22 (c34); stop 0.22 (c34) | 99 / 102 (97.1 %) | 3 / 0 / 0 / 0 / 0 | 1.00 / 1 | 17.9 / 96 | 9.5 / 13.4 / 78.4 / 82.1 | 1 / 0 | 1.00 |

| configuration | plant violations | min plant clearance: track / corridor [m] | predicted (reference) clearance: track / corridor [m] | max stage / cum. residual [rad] | accepted cycles with segment change | reference error: first-step / horizon [m] | heading [rad] / progress [m] | min Δs* [m] |
|:--|--:|:-:|:-:|:-:|--:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.077 / -0.023 | 0.041 / -0.059 | 5.0e-03 / 7.4e-03 | 9 | 1.7e-02 / 4.1e-02 | 7.4e-03 / 3.4e-02 | 0.075 |
| B progress-linearized, no TR, ≤ 8 corr., reject | 0 | 0.098 / -0.002 | 0.053 / -0.047 | 4.7e-03 / 4.7e-03 | 36 | 1.7e-02 / 3.5e-02 | 4.7e-03 / 3.1e-02 | 0.058 |
| C segment-pair, 1 solve | 0 | 0.342 / 0.242 | 0.061 / -0.039 | 4.9e-09 / 4.9e-09 | 0 | 3.6e-08 / 2.7e-02 | 4.9e-09 / 2.3e-02 | 0.058 |
| D segment-pair, ≤ 8 corr. (residual policy) | 0 | 0.098 / -0.002 | 0.054 / -0.046 | 3.9e-03 / 3.9e-03 | 27 | 1.6e-02 / 2.8e-02 | 3.9e-03 / 2.5e-02 | 0.058 |
| D2 segment-pair, ≤ 8 corr., reject segment changes | 0 | 0.098 / -0.002 | 0.052 / -0.048 | 4.9e-09 / 4.9e-09 | 0 | 1.6e-02 / 2.7e-02 | 4.9e-09 / 2.3e-02 | 0.058 |
| E segment-pair + membership bounds, 1 solve | 0 | 0.101 / 0.001 | 0.061 / -0.039 | 1.9e-08 / 1.9e-08 | 0 | 1.3e-02 / 2.7e-02 | 1.9e-08 / 2.3e-02 | 0.058 |
| E8 segment-pair + membership bounds, ≤ 8 corr. | 0 | 0.101 / 0.001 | 0.061 / -0.039 | 1.9e-08 / 1.9e-08 | 0 | 1.3e-02 / 2.7e-02 | 1.9e-08 / 2.3e-02 | 0.058 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: completed
- **B progress-linearized, no TR, ≤ 8 corr., reject**: completed
- **C segment-pair, 1 solve**: control period 5 (t = 0.4 s): schedule_inconsistent: stale curvature schedule: stage residual 0.06092 rad > 0.005 rad; cumulative residual 0.06092 rad > 0.01 rad
- **D segment-pair, ≤ 8 corr. (residual policy)**: completed
- **D2 segment-pair, ≤ 8 corr., reject segment changes**: completed
- **E segment-pair + membership bounds, 1 solve**: control period 34 (t = 3.3000000000000003 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- **E8 segment-pair + membership bounds, ≤ 8 corr.**: control period 34 (t = 3.3000000000000003 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT

## Quarter track, N = 20 (2.0 s horizon), margin 0.05 m

| configuration | laps per run | fully consistent cycles | failures: solver / scheduling / segment-config. / other stale / reference | NLP solves mean / max | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m mean solves |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | stop 0.24 (c37); stop 0.24 (c37); stop 0.24 (c37) | 108 / 111 (97.3 %) | 0 / 0 / 0 / 3 / 0 | 2.95 / 6 | 13.9 / 20 | 37.0 / 69.4 / 75.0 / 79.3 | 0 / 0 | 3.33 |
| B progress-linearized, no TR, ≤ 8 corr., reject | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 15 / 18 (83.3 %) | 0 / 0 / 0 / 0 / 3 | 1.67 / 4 | 16.3 / 21 | 19.4 / 76.6 / 77.3 / 77.5 | 0 / 0 | – |
| C segment-pair, 1 solve | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 15 / 18 (83.3 %) | 0 / 0 / 0 / 0 / 3 | 1.00 / 1 | 16.5 / 19 | 17.7 / 24.2 / 25.4 / 25.7 | 0 / 0 | – |
| D segment-pair, ≤ 8 corr. (residual policy) | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 15 / 18 (83.3 %) | 0 / 0 / 0 / 0 / 3 | 1.00 / 1 | 16.5 / 19 | 17.4 / 39.6 / 103.6 / 119.6 | 1 / 1 | – |
| D2 segment-pair, ≤ 8 corr., reject segment changes | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 15 / 18 (83.3 %) | 0 / 0 / 0 / 0 / 3 | 1.00 / 1 | 16.5 / 19 | 17.0 / 23.5 / 23.5 / 23.5 | 0 / 0 | – |
| E segment-pair + membership bounds, 1 solve | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 15 / 18 (83.3 %) | 0 / 0 / 0 / 0 / 3 | 1.00 / 1 | 17.2 / 19 | 18.2 / 24.7 / 25.0 / 25.0 | 0 / 0 | – |
| E8 segment-pair + membership bounds, ≤ 8 corr. | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 15 / 18 (83.3 %) | 0 / 0 / 0 / 0 / 3 | 1.00 / 1 | 17.2 / 19 | 18.0 / 24.3 / 24.4 / 24.5 | 0 / 0 | – |

| configuration | plant violations | min plant clearance: track / corridor [m] | predicted (reference) clearance: track / corridor [m] | max stage / cum. residual [rad] | accepted cycles with segment change | reference error: first-step / horizon [m] | heading [rad] / progress [m] | min Δs* [m] |
|:--|--:|:-:|:-:|:-:|--:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.203 / 0.153 | 0.023 / -0.027 | 4.6e-03 / 4.1e-03 | 0 | 1.7e-02 / 3.1e-02 | 4.1e-03 / 3.1e-02 | 0.075 |
| B progress-linearized, no TR, ≤ 8 corr., reject | 0 | 0.318 / 0.268 | 0.043 / -0.007 | 2.8e-03 / 2.8e-03 | 0 | 6.7e-08 / 6.3e-03 | 2.8e-03 / 5.0e-03 | 0.069 |
| C segment-pair, 1 solve | 0 | 0.319 / 0.269 | 0.042 / -0.008 | 1.5e-11 / 1.5e-11 | 0 | 5.9e-08 / 9.9e-03 | 3.8e-11 / 1.0e-02 | 0.070 |
| D segment-pair, ≤ 8 corr. (residual policy) | 0 | 0.319 / 0.269 | 0.042 / -0.008 | 1.5e-11 / 1.5e-11 | 0 | 5.9e-08 / 9.9e-03 | 3.8e-11 / 1.0e-02 | 0.070 |
| D2 segment-pair, ≤ 8 corr., reject segment changes | 0 | 0.319 / 0.269 | 0.042 / -0.008 | 1.5e-11 / 1.5e-11 | 0 | 5.9e-08 / 9.9e-03 | 3.8e-11 / 1.0e-02 | 0.070 |
| E segment-pair + membership bounds, 1 solve | 0 | 0.319 / 0.269 | 0.042 / -0.008 | 2.5e-11 / 2.5e-11 | 0 | 5.9e-08 / 9.9e-03 | 1.5e-09 / 1.0e-02 | 0.070 |
| E8 segment-pair + membership bounds, ≤ 8 corr. | 0 | 0.319 / 0.269 | 0.042 / -0.008 | 2.5e-11 / 2.5e-11 | 0 | 5.9e-08 / 9.9e-03 | 1.5e-09 / 1.0e-02 | 0.070 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: control period 37 (t = 3.6 s): schedule_inconsistent: stale curvature schedule: stage residual 0.0105 rad > 0.005 rad; cumulative residual 0.01039 rad > 0.01 rad; schedule correction skipped: time budget 0.08 s would be exceeded
- **B progress-linearized, no TR, ≤ 8 corr., reject**: control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.355 m is beyond the right track boundary (|ey| ≤ 0.35 m)
- **C segment-pair, 1 solve**: control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3554 m is beyond the right track boundary (|ey| ≤ 0.35 m)
- **D segment-pair, ≤ 8 corr. (residual policy)**: control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3554 m is beyond the right track boundary (|ey| ≤ 0.35 m)
- **D2 segment-pair, ≤ 8 corr., reject segment changes**: control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3554 m is beyond the right track boundary (|ey| ≤ 0.35 m)
- **E segment-pair + membership bounds, 1 solve**: control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3554 m is beyond the right track boundary (|ey| ≤ 0.35 m)
- **E8 segment-pair + membership bounds, ≤ 8 corr.**: control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3554 m is beyond the right track boundary (|ey| ≤ 0.35 m)

## Quarter track, N = 20 (2.0 s horizon), margin 0.1 m

| configuration | laps per run | fully consistent cycles | failures: solver / scheduling / segment-config. / other stale / reference | NLP solves mean / max | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m mean solves |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | stop 0.00 (c1); stop 0.23 (c37); stop 0.23 (c37) | 72 / 75 (96.0 %) | 0 / 0 / 0 / 3 / 0 | 2.84 / 6 | 14.3 / 21 | 33.9 / 71.7 / 77.3 / 78.8 | 0 / 0 | 3.00 |
| B progress-linearized, no TR, ≤ 8 corr., reject | stop 0.11 (c19); stop 0.11 (c19); stop 0.11 (c19) | 54 / 57 (94.7 %) | 0 / 0 / 0 / 0 / 3 | 1.32 / 4 | 16.2 / 24 | 16.0 / 45.4 / 81.2 / 82.1 | 2 / 0 | – |
| C segment-pair, 1 solve | stop 0.11 (c19); stop 0.11 (c19); stop 0.11 (c19) | 54 / 57 (94.7 %) | 0 / 0 / 3 / 0 / 0 | 1.00 / 1 | 16.6 / 19 | 17.6 / 24.6 / 34.5 / 42.5 | 0 / 0 | – |
| D segment-pair, ≤ 8 corr. (residual policy) | 2.35; 2.35; 2.35 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 1.08 / 2 | 16.4 / 23 | 16.4 / 32.5 / 37.7 / 46.9 | 0 / 0 | 1.17 |
| D2 segment-pair, ≤ 8 corr., reject segment changes | 2.35; 2.35; 2.35 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 1.14 / 4 | 16.3 / 23 | 16.6 / 34.2 / 43.6 / 70.4 | 0 / 0 | 1.28 |
| E segment-pair + membership bounds, 1 solve | 2.35; 2.35; 2.35 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 1.00 / 1 | 16.2 / 22 | 16.4 / 19.9 / 23.3 / 139.9 | 1 / 1 | 1.00 |
| E8 segment-pair + membership bounds, ≤ 8 corr. | 2.35; 2.35; 2.35 | 750 / 750 (100.0 %) | 0 / 0 / 0 / 0 / 0 | 1.00 / 1 | 16.2 / 22 | 16.4 / 19.9 / 23.2 / 90.9 | 1 / 0 | 1.00 |

| configuration | plant violations | min plant clearance: track / corridor [m] | predicted (reference) clearance: track / corridor [m] | max stage / cum. residual [rad] | accepted cycles with segment change | reference error: first-step / horizon [m] | heading [rad] / progress [m] | min Δs* [m] |
|:--|--:|:-:|:-:|:-:|--:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.232 / 0.132 | 0.062 / -0.038 | 4.9e-03 / 5.2e-03 | 2 | 1.5e-02 / 3.9e-02 | 5.2e-03 / 3.8e-02 | 0.075 |
| B progress-linearized, no TR, ≤ 8 corr., reject | 0 | 0.298 / 0.198 | 0.078 / -0.022 | 3.3e-03 / 3.3e-03 | 6 | 8.2e-04 / 2.5e-02 | 3.3e-03 / 2.6e-02 | 0.069 |
| C segment-pair, 1 solve | 0 | 0.294 / 0.194 | 0.049 / -0.051 | 1.3e-11 / 1.3e-11 | 0 | 7.9e-03 / 3.7e-02 | 1.2e-10 / 3.7e-02 | 0.070 |
| D segment-pair, ≤ 8 corr. (residual policy) | 0 | 0.157 / 0.057 | 0.049 / -0.051 | 4.5e-03 / 4.5e-03 | 21 | 2.0e-02 / 3.8e-02 | 4.5e-03 / 3.7e-02 | 0.070 |
| D2 segment-pair, ≤ 8 corr., reject segment changes | 0 | 0.153 / 0.053 | 0.049 / -0.051 | 9.9e-10 / 9.8e-10 | 0 | 2.0e-02 / 4.1e-02 | 9.8e-10 / 3.9e-02 | 0.070 |
| E segment-pair + membership bounds, 1 solve | 0 | 0.152 / 0.052 | 0.050 / -0.050 | 1.2e-06 / 1.2e-06 | 0 | 2.0e-02 / 4.0e-02 | 7.7e-07 / 4.0e-02 | 0.070 |
| E8 segment-pair + membership bounds, ≤ 8 corr. | 0 | 0.152 / 0.052 | 0.050 / -0.050 | 1.2e-06 / 1.2e-06 | 0 | 2.0e-02 / 4.0e-02 | 7.7e-07 / 4.0e-02 | 0.070 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: control period 1 (t = 0.0 s): schedule_inconsistent: stale curvature schedule: stage residual 0.007622 rad > 0.005 rad; schedule correction skipped: time budget 0.08 s would be exceeded
- **B progress-linearized, no TR, ≤ 8 corr., reject**: control period 19 (t = 1.8 s): reference_violation: stale curvature schedule: stage residual 0.2403 rad > 0.005 rad; cumulative residual 0.2331 rad > 0.01 rad; reference check on the actual track failed: 3 violation(s), first at t = 1.96 s: lateral offset ey = 0.3552 m is beyond the left track bound
- **C segment-pair, 1 solve**: control period 19 (t = 1.8 s): schedule_inconsistent: stale curvature schedule: stage residual 0.1541 rad > 0.005 rad; cumulative residual 0.1541 rad > 0.01 rad
- **D segment-pair, ≤ 8 corr. (residual policy)**: completed
- **D2 segment-pair, ≤ 8 corr., reject segment changes**: completed
- **E segment-pair + membership bounds, 1 solve**: completed
- **E8 segment-pair + membership bounds, ≤ 8 corr.**: completed

## Straight → arc (κ 0 → 0.5), one cycle, cold start, N = 20, margin 0.05

| v [m/s] | φ | formulation | status | NLP solves | stage residual [rad] | segment changes | reference: first-step / horizon [m] |
|:-:|:-:|:--|:--|--:|--:|--:|:-:|
| 1.0 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.5e-01 | 0 | 2.8e-04 / 1.6e-01 |
| 1.0 | 0.1 | progress-linearized | schedule_inconsistent | 1 | 7.0e-02 | 0 | 2.7e-04 / 9.3e-02 |
| 1.0 | 0.1 | segment-pair | success | 1 | 8.1e-10 | 0 | 2.8e-04 / 2.8e-04 |
| 1.0 | 0.5 | stagewise | schedule_inconsistent | 1 | 1.4e-01 | 2 | 9.1e-04 / 1.6e-01 |
| 1.0 | 0.5 | progress-linearized | schedule_inconsistent | 1 | 6.4e-03 | 0 | 8.0e-04 / 6.1e-03 |
| 1.0 | 0.5 | segment-pair | success | 1 | 2.9e-09 | 0 | 7.9e-04 / 7.9e-04 |
| 1.0 | 0.9 | stagewise | schedule_inconsistent | 1 | 1.1e-01 | 2 | 7.3e-04 / 1.5e-01 |
| 1.0 | 0.9 | progress-linearized | schedule_inconsistent | 1 | 1.6e-02 | 0 | 5.5e-04 / 1.4e-02 |
| 1.0 | 0.9 | segment-pair | success | 1 | 2.8e-09 | 0 | 5.6e-04 / 5.6e-04 |
| 2.0 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.1e-01 | 0 | 1.0e-03 / 1.7e-01 |
| 2.0 | 0.1 | progress-linearized | schedule_inconsistent | 1 | 7.9e-02 | 0 | 9.6e-04 / 1.5e-01 |
| 2.0 | 0.1 | segment-pair | success | 1 | 7.5e-16 | 0 | 9.7e-04 / 9.7e-04 |
| 2.0 | 0.5 | stagewise | schedule_inconsistent | 1 | 6.3e-02 | 2 | 3.1e-03 / 1.7e-01 |
| 2.0 | 0.5 | progress-linearized | schedule_inconsistent | 1 | 1.8e-02 | 0 | 2.7e-03 / 2.6e-02 |
| 2.0 | 0.5 | segment-pair | success | 1 | 2.9e-09 | 0 | 2.7e-03 / 2.7e-03 |
| 2.0 | 0.9 | stagewise | schedule_inconsistent | 1 | 1.2e-01 | 2 | 2.0e-03 / 1.7e-01 |
| 2.0 | 0.9 | progress-linearized | schedule_inconsistent | 1 | 3.8e-02 | 0 | 1.4e-03 / 5.2e-02 |
| 2.0 | 0.9 | segment-pair | success | 1 | 2.5e-09 | 0 | 1.5e-03 / 1.5e-03 |
| 2.8 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.3e-01 | 0 | 1.8e-03 / 2.1e-01 |
| 2.8 | 0.1 | progress-linearized | schedule_inconsistent | 1 | 5.6e-02 | 0 | 1.8e-03 / 1.0e-01 |
| 2.8 | 0.1 | segment-pair | success | 1 | 3.6e-09 | 0 | 1.8e-03 / 1.8e-03 |
| 2.8 | 0.5 | stagewise | schedule_inconsistent | 1 | 1.2e-01 | 2 | 4.9e-03 / 2.1e-01 |
| 2.8 | 0.5 | progress-linearized | schedule_inconsistent | 1 | 9.7e-03 | 0 | 4.9e-03 / 1.7e-02 |
| 2.8 | 0.5 | segment-pair | success | 1 | 3.2e-09 | 0 | 4.9e-03 / 4.9e-03 |
| 2.8 | 0.9 | stagewise | schedule_inconsistent | 1 | 9.3e-02 | 2 | 1.8e-03 / 1.9e-01 |
| 2.8 | 0.9 | progress-linearized | schedule_inconsistent | 1 | 2.9e-02 | 0 | 1.8e-03 / 4.0e-02 |
| 2.8 | 0.9 | segment-pair | success | 1 | 3.9e-09 | 0 | 1.9e-03 / 1.9e-03 |

## Single-segment regression

| track | segment-pair identical to stagewise | s_end [m] |
|:--|:-:|--:|
| straight 150 m | true | 47.9466 |
| circle R = 1 m | true | 13.8465 |

