# Cartesian-endpoint NMPC benchmark (Milestone 2B.4)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `[0.0, 0.0, 0.0, 0.5]`
- 250 control cycles (control period 0.1 s, prediction step 0.1 s) per run, 3 interleaved runs per configuration; event-aware plant (10 substeps); all cycles counted
- Corrections/reassignments use the existing mechanism (≤ 8 per cycle, 80 ms budget). Objective, weights, limits, vehicle and Ipopt options are identical in all configurations
- "Corridor" = tightened planning corridor `|ey| ≤ width/2 − margin`; "track" = physical edge `|ey| ≤ width/2`. Predicted clearances are from the event-aware re-simulation of each accepted plan at 5 samples per interval; corridor violations are counted with tolerance 1.0e-6 m
- "Exact-map error" = optimised knots (true geometry) vs the exact Cartesian reconstruction of the plan's inputs from the measured state, computed after the run for every configuration

## Exact Cartesian map: independent validation

| case (dt = 0.1 s) | exact vs BigFloat RK4 (4000 steps): position / heading / speed | `integrate_cartesian` (RK4, 1 ms) vs BigFloat: position |
|:--|:-:|:-:|
| straight | 0.0e+00 / 0.0e+00 / 0.0e+00 | 1.4e-16 |
| left | 0.0e+00 / 0.0e+00 / 0.0e+00 | 2.8e-14 |
| right | 0.0e+00 / 2.8e-17 / 0.0e+00 | 2.7e-14 |
| accelerate | 2.8e-17 / 0.0e+00 / 0.0e+00 | 2.2e-14 |
| brake | 3.9e-17 / 0.0e+00 / 0.0e+00 | 1.2e-13 |
| zero steering, accelerate | 0.0e+00 / 0.0e+00 / 0.0e+00 | 5.3e-16 |
| steering 1e-9 rad | 0.0e+00 / 0.0e+00 / 0.0e+00 | 5.1e-16 |
| from rest | 3.5e-18 / 6.9e-18 / 0.0e+00 | 2.2e-14 |
| heading 40 rad (unwrapped) | 4.4e-16 / 0.0e+00 / 0.0e+00 | 9.0e-14 |
| δ = 25°, v = 5 m/s, a = 5 | 5.6e-17 / 0.0e+00 / 0.0e+00 | 1.9e-12 |
| brake to rest at dt | 3.9e-18 / 0.0e+00 / 2.8e-17 | 1.0e-14 |

| event-aware Frenet plant (RK4, 1 ms, located crossings) mapped to Cartesian vs exact map | crossings | position [m] | heading [rad] | speed [m/s] |
|:--|--:|:-:|:-:|:-:|
| quarter: straight → R 1 m, accelerating, 20 steps | 2 | 5.1e-12 | 5.8e-13 | 3.4e-13 |
| quarter: R 1 m → R −1 m S-bend | 1 | 5.2e-13 | 1.2e-14 | 0.0e+00 |
| quarter: lap boundary | 1 | 6.4e-14 | 8.9e-16 | 0.0e+00 |
| quarter: second lap, braking, 60 steps | 4 | 7.2e-12 | 1.1e-12 | 1.7e-13 |
| quarter: from rest at s = 1.9 | 1 | 2.3e-14 | 1.7e-14 | 3.3e-14 |

First construction / first solve in this process, compilation included: stagewise 2.69 s / 5.94 s, Cartesian 0.24 s / 0.98 s (the Cartesian figures exclude what the stagewise run had already compiled). All other timings below are warmed up.

Benchmark loop with a primed controller vs `simulate_closed_loop` without priming (40 cycles, Cartesian, no time budget): states identical = true.

## Quarter track, N = 10 (1.0 s horizon), margin 0.05 m

| configuration | laps per run | accepted / rejected cycles | rejections by status | NLP solves mean / max | reassignments per cycle | Ipopt iter. mean / max | model build / priming solve [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms |
|:--|:--|:-:|:--|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | stop 0.32 (c42); stop 0.32 (c42); stop 0.32 (c42) | 123 / 3 | schedule_inconsistent 3 | 2.40 / 9 | 1.40 | 14.6 / 20 | 3 / 24 | 15.0 / 38.1 / 70.7 / 71.1 | 0 / 0 |
| B segment-pair D2, ≤ 8 corr. | stop 0.97 (c97); stop 0.97 (c97); stop 0.97 (c97) | 288 / 3 | reference_violation 3 | 1.11 / 2 | 0.11 | 15.1 / 20 | 3 / 13 | 8.8 / 17.5 / 19.4 / 134.7 | 1 / 1 |
| C Cartesian, 1 solve | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | segment_assignment_invalid 3 | 1.00 / 1 | 0.00 | 13.0 / 13 | 2 / 8 | 5.0 / 5.4 / 5.5 / 5.5 | 0 / 0 |
| D Cartesian, ≤ 8 reassignments | 2.58; 2.58; 2.58 | 750 / 0 | – | 1.13 / 2 | 0.13 | 15.3 / 23 | 2 / 12 | 6.1 / 12.8 / 14.4 / 19.9 | 0 / 0 |
| E Cartesian + membership bounds, 1 solve | stop 0.22 (c32); stop 0.22 (c32); stop 0.22 (c32) | 93 / 3 | solver_failure 3 | 1.00 / 1 | 0.00 | 17.5 / 72 | 2 / 9 | 6.5 / 7.5 / 37.5 / 48.3 | 0 / 0 |
| F = D + strict corridor | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 11.0 / 11 | 2 / 13 | 10.2 / 10.5 / 10.6 / 10.6 | 0 / 0 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted clearance: track / corridor [m] | predicted reference violations / corridor violations (accepted) | exact-map error: first step / horizon [m] | heading [rad] / speed [m/s] | event-aware reference: first step / horizon [m] | NLP dyn. residual / geometry residual | accepted cycles with segment change |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.048 / -0.002 | 0.014 / -0.036 | 0 / 111 | 1.5e-02 / 2.5e-02 | 4.8e-03 / 4.2e-08 | 1.5e-02 / 2.5e-02 | 4.3e-08 / NaN | 3 |
| B segment-pair D2, ≤ 8 corr. | 0 | 0.046 / -0.004 | 0.003 / -0.047 | 3 / 195 | 1.4e-02 / 3.8e-02 | 6.0e-09 / 4.7e-08 | 1.4e-02 / 3.8e-02 | 4.5e-08 / NaN | 0 |
| C Cartesian, 1 solve | 0 | 0.350 / 0.300 | NaN / NaN | 0 / 0 | NaN / NaN | NaN / NaN | NaN / NaN | NaN / NaN | 0 |
| D Cartesian, ≤ 8 reassignments | 0 | 0.046 / -0.004 | 0.015 / -0.035 | 0 / 429 | 1.0e-08 / 1.1e-08 | 1.4e-09 / 4.7e-08 | 4.4e-08 / 1.2e-06 | 4.7e-08 / 7.3e-15 | 0 |
| E Cartesian + membership bounds, 1 solve | 0 | 0.049 / -0.001 | 0.020 / -0.030 | 0 / 69 | 9.9e-09 / 8.1e-08 | 7.7e-08 / 4.0e-08 | 3.4e-08 / 9.3e-07 | 9.6e-08 / 8.9e-16 | 0 |
| F = D + strict corridor | 0 | 0.350 / 0.300 | NaN / NaN | 0 / 0 | NaN / NaN | NaN / NaN | NaN / NaN | NaN / NaN | 0 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: control period 42 (t = 4.1 s): schedule_inconsistent: stale curvature schedule: stage residual 0.01915 rad > 0.005 rad; cumulative residual 0.01715 rad > 0.01 rad
- **B segment-pair D2, ≤ 8 corr.**: control period 97 (t = 9.6 s): reference_violation: reference check on the actual track failed: 1 violation(s), first at t = 0.74 s: lateral offset ey = 0.3515 m is beyond the left track boundary (|ey| ≤ 0.35 m)
- **C Cartesian, 1 solve**: control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.4234 m)
- **D Cartesian, ≤ 8 reassignments**: completed
- **E Cartesian + membership bounds, 1 solve**: control period 32 (t = 3.1 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- **F = D + strict corridor**: control period 1 (t = 0.0 s): corridor_violation: re-simulation leaves the planning corridor by 0.01446 m (strict corridor)

Predicted track clearance vs re-simulation sampling (run 1, accepted plans; min over cycles) [m]:

| configuration | 5 / interval | 20 / interval | 100 / interval |
|:--|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | 0.0142 | 0.0130 | 0.0130 |
| B segment-pair D2, ≤ 8 corr. | 0.0027 | 0.0013 | 0.0013 |
| D Cartesian, ≤ 8 reassignments | 0.0151 | 0.0137 | 0.0137 |
| E Cartesian + membership bounds, 1 solve | 0.0196 | 0.0186 | 0.0186 |

Timing by run (median / p99 / max [ms]; Ipopt time median / max [ms]; allocations median / max [MB]; GC time total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A stagewise, ≤ 8 corr., reject | 15.7 / 60.3 / 71.1; Ipopt 16.0 / 68.0; 0.4 / 1.7 MB; GC 0.0 / 0.0 | 14.5 / 59.8 / 71.0; Ipopt 15.0 / 64.0; 0.4 / 1.7 MB; GC 0.0 / 0.0 | 15.0 / 58.5 / 69.9; Ipopt 14.5 / 65.0; 0.4 / 1.7 MB; GC 0.0 / 0.0 |
| B segment-pair D2, ≤ 8 corr. | 9.0 / 24.0 / 134.7; Ipopt 10.0 / 21.0; 0.2 / 7.8 MB; GC 0.0 / 0.0 | 8.8 / 18.8 / 20.0; Ipopt 9.0 / 24.0; 0.2 / 0.4 MB; GC 0.0 / 0.0 | 8.6 / 19.3 / 19.8; Ipopt 9.0 / 21.0; 0.2 / 0.4 MB; GC 0.0 / 0.0 |
| C Cartesian, 1 solve | 5.0 / 5.0 / 5.0; Ipopt 7.0 / 7.0; 0.4 / 0.4 MB; GC 0.0 / 0.0 | 4.9 / 4.9 / 4.9; Ipopt 4.0 / 4.0; 0.4 / 0.4 MB; GC 0.0 / 0.0 | 5.5 / 5.5 / 5.5; Ipopt 9.0 / 9.0; 0.4 / 0.4 MB; GC 0.0 / 0.0 |
| D Cartesian, ≤ 8 reassignments | 6.1 / 14.5 / 14.7; Ipopt 6.0 / 18.0; 0.3 / 0.6 MB; GC 5.4 / 5.4 | 6.1 / 14.5 / 19.9; Ipopt 6.0 / 20.0; 0.3 / 0.6 MB; GC 8.8 / 8.8 | 6.1 / 14.0 / 14.5; Ipopt 6.0 / 20.0; 0.3 / 0.6 MB; GC 12.4 / 7.3 |
| E Cartesian + membership bounds, 1 solve | 6.5 / 35.6 / 48.3; Ipopt 7.0 / 44.0; 0.3 / 0.9 MB; GC 0.0 / 0.0 | 6.6 / 27.8 / 36.8; Ipopt 7.0 / 38.0; 0.3 / 0.9 MB; GC 0.0 / 0.0 | 6.4 / 28.0 / 37.0; Ipopt 6.0 / 41.0; 0.3 / 0.9 MB; GC 0.0 / 0.0 |
| F = D + strict corridor | 9.8 / 9.8 / 9.8; Ipopt 10.0 / 10.0; 0.6 / 0.6 MB; GC 0.0 / 0.0 | 10.6 / 10.6 / 10.6; Ipopt 10.0 / 10.0; 0.6 / 0.6 MB; GC 0.0 / 0.0 | 10.2 / 10.2 / 10.2; Ipopt 10.0 / 10.0; 0.6 / 0.6 MB; GC 0.0 / 0.0 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):

- B segment-pair D2, ≤ 8 corr., run 1, cycle 5: 134.7 / 20.0 / 17.6 ms, GC 0.0 ms, 7.8 MB, 2 solve(s), 16 iterations

## Quarter track, N = 10 (1.0 s horizon), margin 0.1 m

| configuration | laps per run | accepted / rejected cycles | rejections by status | NLP solves mean / max | reassignments per cycle | Ipopt iter. mean / max | model build / priming solve [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms |
|:--|:--|:-:|:--|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | 2.46; 2.46; 2.46 | 750 / 0 | – | 2.12 / 8 | 1.12 | 14.3 / 21 | 3 / 25 | 15.0 / 36.5 / 49.0 / 70.0 | 0 / 0 |
| B segment-pair D2, ≤ 8 corr. | 2.48; 2.48; 2.48 | 750 / 0 | – | 1.10 / 2 | 0.10 | 15.2 / 21 | 3 / 12 | 8.9 / 19.0 / 20.5 / 21.3 | 0 / 0 |
| C Cartesian, 1 solve | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | segment_assignment_invalid 3 | 1.00 / 1 | 0.00 | 14.0 / 14 | 2 / 9 | 5.8 / 6.0 / 6.0 / 6.0 | 0 / 0 |
| D Cartesian, ≤ 8 reassignments | 2.50; 2.50; 2.50 | 750 / 0 | – | 1.11 / 2 | 0.11 | 15.3 / 22 | 2 / 14 | 6.1 / 12.5 / 14.7 / 25.8 | 0 / 0 |
| E Cartesian + membership bounds, 1 solve | stop 0.23 (c34); stop 0.23 (c34); stop 0.23 (c34) | 99 / 3 | solver_failure 3 | 1.00 / 1 | 0.00 | 16.9 / 43 | 2 / 9 | 6.5 / 7.8 / 20.2 / 20.4 | 0 / 0 |
| F = D + strict corridor | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 13.0 / 13 | 2 / 14 | 10.7 / 10.9 / 10.9 / 10.9 | 0 / 0 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted clearance: track / corridor [m] | predicted reference violations / corridor violations (accepted) | exact-map error: first step / horizon [m] | heading [rad] / speed [m/s] | event-aware reference: first step / horizon [m] | NLP dyn. residual / geometry residual | accepted cycles with segment change |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.077 / -0.023 | 0.041 / -0.059 | 0 / 504 | 1.7e-02 / 4.1e-02 | 7.4e-03 / 4.7e-08 | 1.7e-02 / 4.1e-02 | 4.7e-08 / NaN | 9 |
| B segment-pair D2, ≤ 8 corr. | 0 | 0.098 / -0.002 | 0.052 / -0.048 | 0 / 438 | 1.6e-02 / 2.7e-02 | 4.9e-09 / 4.7e-08 | 1.6e-02 / 2.7e-02 | 4.6e-08 / NaN | 0 |
| C Cartesian, 1 solve | 0 | 0.350 / 0.250 | NaN / NaN | 0 / 0 | NaN / NaN | NaN / NaN | NaN / NaN | NaN / NaN | 0 |
| D Cartesian, ≤ 8 reassignments | 0 | 0.097 / -0.003 | 0.064 / -0.036 | 0 / 366 | 9.9e-09 / 1.0e-08 | 1.2e-09 / 4.7e-08 | 3.9e-08 / 9.1e-07 | 4.7e-08 / 7.4e-15 | 0 |
| E Cartesian + membership bounds, 1 solve | 0 | 0.108 / 0.008 | 0.077 / -0.023 | 0 / 42 | 1.5e-10 / 5.8e-08 | 5.0e-08 / 3.9e-08 | 2.5e-08 / 6.7e-07 | 6.9e-08 / 8.9e-16 | 0 |
| F = D + strict corridor | 0 | 0.350 / 0.250 | NaN / NaN | 0 / 0 | NaN / NaN | NaN / NaN | NaN / NaN | NaN / NaN | 0 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: completed
- **B segment-pair D2, ≤ 8 corr.**: completed
- **C Cartesian, 1 solve**: control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.4298 m)
- **D Cartesian, ≤ 8 reassignments**: completed
- **E Cartesian + membership bounds, 1 solve**: control period 34 (t = 3.3 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- **F = D + strict corridor**: control period 1 (t = 0.0 s): corridor_violation: re-simulation leaves the planning corridor by 0.01601 m (strict corridor)

Predicted track clearance vs re-simulation sampling (run 1, accepted plans; min over cycles) [m]:

| configuration | 5 / interval | 20 / interval | 100 / interval |
|:--|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | 0.0406 | 0.0399 | 0.0398 |
| B segment-pair D2, ≤ 8 corr. | 0.0524 | 0.0524 | 0.0524 |
| D Cartesian, ≤ 8 reassignments | 0.0643 | 0.0632 | 0.0632 |
| E Cartesian + membership bounds, 1 solve | 0.0768 | 0.0762 | 0.0761 |

Timing by run (median / p99 / max [ms]; Ipopt time median / max [ms]; allocations median / max [MB]; GC time total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A stagewise, ≤ 8 corr., reject | 15.2 / 49.4 / 65.1; Ipopt 15.0 / 63.0; 0.4 / 1.5 MB; GC 19.2 / 10.4 | 14.9 / 47.9 / 64.3; Ipopt 14.0 / 56.0; 0.4 / 1.5 MB; GC 11.9 / 6.3 | 15.0 / 48.2 / 70.0; Ipopt 15.0 / 67.0; 0.4 / 1.5 MB; GC 11.1 / 6.0 |
| B segment-pair D2, ≤ 8 corr. | 8.9 / 20.6 / 21.3; Ipopt 10.0 / 28.0; 0.2 / 0.4 MB; GC 0.0 / 0.0 | 8.7 / 20.1 / 20.9; Ipopt 9.0 / 24.0; 0.2 / 0.4 MB; GC 7.7 / 7.7 | 8.8 / 20.4 / 20.9; Ipopt 10.0 / 27.0; 0.2 / 0.4 MB; GC 5.4 / 5.4 |
| C Cartesian, 1 solve | 5.5 / 5.5 / 5.5; Ipopt 6.0 / 6.0; 0.4 / 0.4 MB; GC 0.0 / 0.0 | 5.8 / 5.8 / 5.8; Ipopt 10.0 / 10.0; 0.4 / 0.4 MB; GC 0.0 / 0.0 | 6.0 / 6.0 / 6.0; Ipopt 12.0 / 12.0; 0.4 / 0.4 MB; GC 0.0 / 0.0 |
| D Cartesian, ≤ 8 reassignments | 6.2 / 14.6 / 15.6; Ipopt 6.0 / 21.0; 0.3 / 0.7 MB; GC 9.0 / 4.9 | 6.0 / 15.0 / 25.8; Ipopt 6.0 / 28.0; 0.3 / 0.7 MB; GC 12.7 / 6.9 | 6.0 / 14.3 / 15.0; Ipopt 6.0 / 21.0; 0.3 / 0.7 MB; GC 9.3 / 4.8 |
| E Cartesian + membership bounds, 1 solve | 6.5 / 16.1 / 20.2; Ipopt 6.0 / 24.0; 0.3 / 0.5 MB; GC 0.0 / 0.0 | 6.7 / 16.1 / 20.1; Ipopt 7.0 / 19.0; 0.3 / 0.5 MB; GC 0.0 / 0.0 | 6.4 / 16.2 / 20.4; Ipopt 6.0 / 29.0; 0.3 / 0.5 MB; GC 0.0 / 0.0 |
| F = D + strict corridor | 10.9 / 10.9 / 10.9; Ipopt 10.0 / 10.0; 0.7 / 0.7 MB; GC 0.0 / 0.0 | 10.7 / 10.7 / 10.7; Ipopt 13.0 / 13.0; 0.7 / 0.7 MB; GC 0.0 / 0.0 | 10.4 / 10.4 / 10.4; Ipopt 4.0 / 4.0; 0.7 / 0.7 MB; GC 0.0 / 0.0 |

## Quarter track, N = 20 (2.0 s horizon), margin 0.05 m

| configuration | laps per run | accepted / rejected cycles | rejections by status | NLP solves mean / max | reassignments per cycle | Ipopt iter. mean / max | model build / priming solve [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms |
|:--|:--|:-:|:--|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | stop 0.24 (c37); stop 0.24 (c37); stop 0.24 (c37) | 108 / 3 | schedule_inconsistent 3 | 2.95 / 6 | 1.95 | 13.9 / 20 | 9 / 73 | 36.2 / 63.0 / 73.4 / 73.6 | 0 / 0 |
| B segment-pair D2, ≤ 8 corr. | stop 0.02 (c6); stop 0.02 (c6); stop 0.02 (c6) | 15 / 3 | reference_violation 3 | 1.00 / 1 | 0.00 | 16.5 / 19 | 5 / 28 | 16.2 / 17.7 / 17.9 / 17.9 | 0 / 0 |
| C Cartesian, 1 solve | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | segment_assignment_invalid 3 | 1.00 / 1 | 0.00 | 15.0 / 15 | 4 / 14 | 9.1 / 9.5 / 9.5 / 9.6 | 0 / 0 |
| D Cartesian, ≤ 8 reassignments | 2.39; 2.39; 2.39 | 750 / 0 | – | 1.22 / 3 | 0.22 | 16.4 / 22 | 3 / 20 | 9.8 / 20.7 / 22.8 / 110.0 | 1 / 1 |
| E Cartesian + membership bounds, 1 solve | stop 1.99 (c202); stop 1.99 (c202); stop 1.99 (c202) | 603 / 3 | invalid_solution 3 | 1.00 / 1 | 0.00 | 16.2 / 25 | 4 / 14 | 9.5 / 11.3 / 17.1 / 37.4 | 0 / 0 |
| F = D + strict corridor | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 3 | corridor_violation 3 | 1.50 / 2 | 0.50 | 14.5 / 16 | 4 / 21 | 15.0 / 21.5 / 21.6 / 21.6 | 0 / 0 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted clearance: track / corridor [m] | predicted reference violations / corridor violations (accepted) | exact-map error: first step / horizon [m] | heading [rad] / speed [m/s] | event-aware reference: first step / horizon [m] | NLP dyn. residual / geometry residual | accepted cycles with segment change |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.203 / 0.153 | 0.023 / -0.027 | 0 / 15 | 1.7e-02 / 3.1e-02 | 4.1e-03 / 3.6e-08 | 1.7e-02 / 3.1e-02 | 3.6e-08 / NaN | 0 |
| B segment-pair D2, ≤ 8 corr. | 0 | 0.319 / 0.269 | 0.042 / -0.008 | 3 / 3 | 5.9e-08 / 9.9e-03 | 3.8e-11 / 8.9e-16 | 5.9e-08 / 9.9e-03 | 3.8e-09 / NaN | 0 |
| C Cartesian, 1 solve | 0 | 0.350 / 0.300 | NaN / NaN | 0 / 0 | NaN / NaN | NaN / NaN | NaN / NaN | NaN / NaN | 0 |
| D Cartesian, ≤ 8 reassignments | 0 | 0.129 / 0.079 | 0.009 / -0.041 | 0 / 81 | 1.3e-10 / 8.8e-09 | 5.1e-09 / 4.5e-08 | 6.1e-08 / 1.2e-06 | 4.5e-08 / 8.0e-15 | 0 |
| E Cartesian + membership bounds, 1 solve | 0 | 0.115 / 0.065 | 0.035 / -0.015 | 0 / 60 | 4.4e-11 / 6.8e-07 | 7.0e-07 / 4.7e-08 | 5.5e-08 / 9.6e-07 | 7.0e-07 / 4.9e-15 | 0 |
| F = D + strict corridor | 0 | 0.344 / 0.294 | 0.244 / 0.194 | 0 / 0 | 4.6e-17 / 4.3e-12 | 2.9e-12 / 8.9e-16 | 1.0e-10 / 2.3e-08 | 9.1e-13 / 2.2e-16 | 0 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: control period 37 (t = 3.6 s): schedule_inconsistent: stale curvature schedule: stage residual 0.0105 rad > 0.005 rad; cumulative residual 0.01039 rad > 0.01 rad; schedule correction skipped: time budget 0.08 s would be exceeded
- **B segment-pair D2, ≤ 8 corr.**: control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3554 m is beyond the right track boundary (|ey| ≤ 0.35 m)
- **C Cartesian, 1 solve**: control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.005224 m)
- **D Cartesian, ≤ 8 reassignments**: completed
- **E Cartesian + membership bounds, 1 solve**: control period 202 (t = 20.1 s): invalid_solution: dynamics residual 1.017324874230674e-6 exceeds acceptance_tol
- **F = D + strict corridor**: control period 4 (t = 0.3 s): corridor_violation: re-simulation leaves the planning corridor by 0.02537 m (strict corridor)

Predicted track clearance vs re-simulation sampling (run 1, accepted plans; min over cycles) [m]:

| configuration | 5 / interval | 20 / interval | 100 / interval |
|:--|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | 0.0225 | 0.0219 | 0.0219 |
| B segment-pair D2, ≤ 8 corr. | 0.0418 | 0.0415 | 0.0415 |
| D Cartesian, ≤ 8 reassignments | 0.0089 | 0.0076 | 0.0076 |
| E Cartesian + membership bounds, 1 solve | 0.0347 | 0.0347 | 0.0347 |
| F = D + strict corridor | 0.2443 | 0.2443 | 0.2443 |

Timing by run (median / p99 / max [ms]; Ipopt time median / max [ms]; allocations median / max [MB]; GC time total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A stagewise, ≤ 8 corr., reject | 36.2 / 70.6 / 73.6; Ipopt 37.0 / 70.0; 0.9 / 2.0 MB; GC 0.0 / 0.0 | 35.6 / 68.6 / 71.8; Ipopt 34.0 / 70.0; 0.9 / 2.0 MB; GC 0.0 / 0.0 | 36.7 / 70.4 / 73.6; Ipopt 34.0 / 73.0; 0.9 / 2.0 MB; GC 0.0 / 0.0 |
| B segment-pair D2, ≤ 8 corr. | 16.2 / 17.7 / 17.7; Ipopt 17.5 / 21.0; 0.3 / 0.6 MB; GC 0.0 / 0.0 | 16.2 / 17.9 / 17.9; Ipopt 16.5 / 21.0; 0.3 / 0.6 MB; GC 0.0 / 0.0 | 15.8 / 17.2 / 17.2; Ipopt 17.5 / 21.0; 0.3 / 0.6 MB; GC 0.0 / 0.0 |
| C Cartesian, 1 solve | 9.1 / 9.1 / 9.1; Ipopt 4.0 / 4.0; 0.8 / 0.8 MB; GC 0.0 / 0.0 | 8.9 / 8.9 / 8.9; Ipopt 0.0 / 0.0; 0.8 / 0.8 MB; GC 0.0 / 0.0 | 9.6 / 9.6 / 9.6; Ipopt 17.0 / 17.0; 0.8 / 0.8 MB; GC 0.0 / 0.0 |
| D Cartesian, ≤ 8 reassignments | 9.6 / 29.6 / 110.0; Ipopt 10.0 / 104.0; 0.5 / 1.4 MB; GC 136.6 / 99.2 | 9.9 / 22.5 / 28.8; Ipopt 10.0 / 28.0; 0.5 / 1.4 MB; GC 17.6 / 7.1 | 9.8 / 21.9 / 28.8; Ipopt 10.0 / 30.0; 0.5 / 1.4 MB; GC 18.4 / 8.8 |
| E Cartesian + membership bounds, 1 solve | 9.5 / 13.4 / 33.6; Ipopt 10.0 / 20.0; 0.5 / 0.8 MB; GC 11.6 / 5.8 | 9.4 / 13.2 / 37.4; Ipopt 10.0 / 20.0; 0.5 / 0.8 MB; GC 15.5 / 9.2 | 9.5 / 17.2 / 25.4; Ipopt 10.0 / 22.0; 0.5 / 0.8 MB; GC 15.3 / 5.8 |
| F = D + strict corridor | 15.0 / 20.9 / 21.1; Ipopt 11.0 / 22.0; 0.7 / 1.2 MB; GC 5.9 / 5.9 | 12.3 / 21.5 / 21.6; Ipopt 15.5 / 21.0; 0.7 / 1.2 MB; GC 0.0 / 0.0 | 12.3 / 21.2 / 21.4; Ipopt 15.0 / 20.0; 0.7 / 1.2 MB; GC 0.0 / 0.0 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):

- D Cartesian, ≤ 8 reassignments, run 1, cycle 249: 110.0 / 104.0 / 109.6 ms, GC 99.2 ms, 0.5 MB, 1 solve(s), 15 iterations

## Quarter track, N = 20 (2.0 s horizon), margin 0.1 m

| configuration | laps per run | accepted / rejected cycles | rejections by status | NLP solves mean / max | reassignments per cycle | Ipopt iter. mean / max | model build / priming solve [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms |
|:--|:--|:-:|:--|:-:|:-:|:-:|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | stop 0.23 (c37); stop 0.23 (c37); stop 0.23 (c37) | 108 / 3 | schedule_inconsistent 3 | 2.81 / 6 | 1.81 | 14.4 / 21 | 5 / 71 | 30.6 / 67.0 / 76.4 / 78.2 | 0 / 0 |
| B segment-pair D2, ≤ 8 corr. | 2.35; 2.35; 2.35 | 750 / 0 | – | 1.14 / 4 | 0.14 | 16.3 / 23 | 5 / 23 | 16.1 / 32.3 / 42.2 / 67.8 | 0 / 0 |
| C Cartesian, 1 solve | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | segment_assignment_invalid 3 | 1.00 / 1 | 0.00 | 15.0 / 15 | 5 / 15 | 9.0 / 9.2 / 9.2 / 9.2 | 0 / 0 |
| D Cartesian, ≤ 8 reassignments | 2.34; 2.34; 2.34 | 750 / 0 | – | 1.18 / 2 | 0.18 | 16.5 / 25 | 3 / 24 | 9.9 / 20.5 / 22.3 / 26.3 | 0 / 0 |
| E Cartesian + membership bounds, 1 solve | 2.35; 2.35; 2.35 | 750 / 0 | – | 1.00 / 1 | 0.00 | 16.4 / 27 | 3 / 15 | 9.7 / 11.7 / 16.9 / 53.8 | 0 / 0 |
| F = D + strict corridor | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 3 | corridor_violation 3 | 1.50 / 2 | 0.50 | 14.8 / 17 | 5 / 21 | 12.5 / 22.3 / 22.4 / 22.4 | 0 / 0 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted clearance: track / corridor [m] | predicted reference violations / corridor violations (accepted) | exact-map error: first step / horizon [m] | heading [rad] / speed [m/s] | event-aware reference: first step / horizon [m] | NLP dyn. residual / geometry residual | accepted cycles with segment change |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|--:|
| A stagewise, ≤ 8 corr., reject | 0 | 0.232 / 0.132 | 0.062 / -0.038 | 0 / 18 | 1.5e-02 / 3.9e-02 | 5.2e-03 / 3.9e-08 | 1.5e-02 / 3.9e-02 | 3.9e-08 / NaN | 3 |
| B segment-pair D2, ≤ 8 corr. | 0 | 0.153 / 0.053 | 0.049 / -0.051 | 0 / 126 | 2.0e-02 / 4.1e-02 | 9.8e-10 / 4.6e-08 | 2.0e-02 / 4.1e-02 | 4.6e-08 / NaN | 0 |
| C Cartesian, 1 solve | 0 | 0.350 / 0.250 | NaN / NaN | 0 / 0 | NaN / NaN | NaN / NaN | NaN / NaN | NaN / NaN | 0 |
| D Cartesian, ≤ 8 reassignments | 0 | 0.166 / 0.066 | 0.073 / -0.027 | 0 / 63 | 1.4e-11 / 6.2e-09 | 1.8e-09 / 4.7e-08 | 4.8e-08 / 9.1e-07 | 4.7e-08 / 7.6e-15 | 0 |
| E Cartesian + membership bounds, 1 solve | 0 | 0.164 / 0.064 | 0.074 / -0.026 | 0 / 66 | 1.0e-10 / 8.4e-07 | 7.7e-07 / 4.7e-08 | 4.8e-08 / 8.6e-07 | 9.4e-07 / 7.6e-15 | 0 |
| F = D + strict corridor | 0 | 0.345 / 0.245 | 0.268 / 0.168 | 0 / 0 | 8.6e-17 / 7.5e-12 | 5.2e-12 / 8.9e-16 | 8.0e-11 / 1.8e-08 | 1.7e-12 / 4.5e-16 | 0 |

Stop reasons (run 1):

- **A stagewise, ≤ 8 corr., reject**: control period 37 (t = 3.6 s): schedule_inconsistent: stale curvature schedule: stage residual 0.007236 rad > 0.005 rad; schedule correction skipped: time budget 0.08 s would be exceeded
- **B segment-pair D2, ≤ 8 corr.**: completed
- **C Cartesian, 1 solve**: control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.01495 m)
- **D Cartesian, ≤ 8 reassignments**: completed
- **E Cartesian + membership bounds, 1 solve**: completed
- **F = D + strict corridor**: control period 4 (t = 0.3 s): corridor_violation: re-simulation leaves the planning corridor by 0.027 m (strict corridor)

Predicted track clearance vs re-simulation sampling (run 1, accepted plans; min over cycles) [m]:

| configuration | 5 / interval | 20 / interval | 100 / interval |
|:--|:-:|:-:|:-:|
| A stagewise, ≤ 8 corr., reject | 0.0617 | 0.0615 | 0.0615 |
| B segment-pair D2, ≤ 8 corr. | 0.0494 | 0.0486 | 0.0486 |
| D Cartesian, ≤ 8 reassignments | 0.0729 | 0.0718 | 0.0718 |
| E Cartesian + membership bounds, 1 solve | 0.0744 | 0.0735 | 0.0735 |
| F = D + strict corridor | 0.2677 | 0.2677 | 0.2677 |

Timing by run (median / p99 / max [ms]; Ipopt time median / max [ms]; allocations median / max [MB]; GC time total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A stagewise, ≤ 8 corr., reject | 30.6 / 72.7 / 76.2; Ipopt 31.0 / 77.0; 0.9 / 2.0 MB; GC 7.3 / 7.3 | 30.4 / 77.4 / 78.2; Ipopt 32.0 / 79.0; 0.9 / 2.0 MB; GC 10.3 / 10.3 | 31.0 / 73.7 / 76.4; Ipopt 34.0 / 72.0; 0.9 / 2.0 MB; GC 9.6 / 9.6 |
| B segment-pair D2, ≤ 8 corr. | 15.8 / 42.4 / 66.9; Ipopt 16.0 / 69.0; 0.3 / 1.3 MB; GC 13.8 / 7.2 | 16.1 / 40.4 / 67.1; Ipopt 16.0 / 75.0; 0.3 / 1.3 MB; GC 10.0 / 10.0 | 16.3 / 40.4 / 67.8; Ipopt 16.0 / 67.0; 0.3 / 1.3 MB; GC 11.4 / 11.4 |
| C Cartesian, 1 solve | 8.7 / 8.7 / 8.7; Ipopt 10.0 / 10.0; 0.8 / 0.8 MB; GC 0.0 / 0.0 | 9.0 / 9.0 / 9.0; Ipopt 14.0 / 14.0; 0.8 / 0.8 MB; GC 0.0 / 0.0 | 9.2 / 9.2 / 9.2; Ipopt 10.0 / 10.0; 0.8 / 0.8 MB; GC 0.0 / 0.0 |
| D Cartesian, ≤ 8 reassignments | 9.7 / 21.3 / 26.3; Ipopt 10.0 / 25.0; 0.5 / 1.2 MB; GC 21.9 / 8.2 | 10.0 / 21.7 / 22.7; Ipopt 10.0 / 26.0; 0.5 / 1.2 MB; GC 15.1 / 8.7 | 10.1 / 22.4 / 23.7; Ipopt 10.0 / 28.0; 0.5 / 1.2 MB; GC 16.8 / 7.1 |
| E Cartesian + membership bounds, 1 solve | 9.9 / 16.6 / 46.5; Ipopt 10.0 / 40.0; 0.5 / 0.8 MB; GC 36.1 / 23.5 | 9.6 / 13.6 / 53.8; Ipopt 10.0 / 58.0; 0.5 / 0.8 MB; GC 19.2 / 9.9 | 9.7 / 17.8 / 28.4; Ipopt 10.0 / 23.0; 0.5 / 0.8 MB; GC 18.1 / 6.7 |
| F = D + strict corridor | 12.9 / 22.2 / 22.4; Ipopt 12.5 / 15.0; 0.8 / 1.2 MB; GC 0.0 / 0.0 | 12.5 / 22.1 / 22.3; Ipopt 16.0 / 23.0; 0.8 / 1.2 MB; GC 0.0 / 0.0 | 12.4 / 21.5 / 21.7; Ipopt 11.0 / 20.0; 0.8 / 1.2 MB; GC 0.0 / 0.0 |

## Without the correction time budget (one run per configuration)

The 80 ms budget makes the correction/reassignment decisions depend on wall time. Here every configuration is rerun once with the budget lifted (`correction_time_budget = 1000 s`; corrections still capped at 8), so the outcome depends only on the computation. Timings are not reported.

| N | margin | configuration | laps / stop | accepted cycles | NLP solves mean / max | stop reason |
|--:|--:|:--|:--|--:|:-:|:--|
| 10 | 0.05 | A stagewise, ≤ 8 corr., reject | stop 0.32 (c42) | 41 | 2.40 / 9 | control period 42 (t = 4.1 s): schedule_inconsistent: stale curvature schedule: stage residual 0.01915 rad > 0.005 rad; cumulative residual 0.01715 rad > 0.01 rad |
| 10 | 0.05 | B segment-pair D2, ≤ 8 corr. | stop 0.97 (c97) | 96 | 1.11 / 2 | control period 97 (t = 9.6 s): reference_violation: reference check on the actual track failed: 1 violation(s), first at t = 0.74 s: lateral offset ey = 0.3515 m is beyond the left track boundary (|ey |
| 10 | 0.05 | C Cartesian, 1 solve | stop 0.00 (c1) | 0 | 1.00 / 1 | control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.4234 m) |
| 10 | 0.05 | D Cartesian, ≤ 8 reassignments | 2.58 | 250 | 1.13 / 2 | completed |
| 10 | 0.05 | E Cartesian + membership bounds, 1 solve | stop 0.22 (c32) | 31 | 1.00 / 1 | control period 32 (t = 3.1 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT |
| 10 | 0.05 | F = D + strict corridor | stop 0.00 (c1) | 0 | 2.00 / 2 | control period 1 (t = 0.0 s): corridor_violation: re-simulation leaves the planning corridor by 0.01446 m (strict corridor) |
| 10 | 0.10 | A stagewise, ≤ 8 corr., reject | 2.46 | 250 | 2.12 / 8 | completed |
| 10 | 0.10 | B segment-pair D2, ≤ 8 corr. | 2.48 | 250 | 1.10 / 2 | completed |
| 10 | 0.10 | C Cartesian, 1 solve | stop 0.00 (c1) | 0 | 1.00 / 1 | control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.4298 m) |
| 10 | 0.10 | D Cartesian, ≤ 8 reassignments | 2.50 | 250 | 1.11 / 2 | completed |
| 10 | 0.10 | E Cartesian + membership bounds, 1 solve | stop 0.23 (c34) | 33 | 1.00 / 1 | control period 34 (t = 3.3 s): solver_failure: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT |
| 10 | 0.10 | F = D + strict corridor | stop 0.00 (c1) | 0 | 2.00 / 2 | control period 1 (t = 0.0 s): corridor_violation: re-simulation leaves the planning corridor by 0.01601 m (strict corridor) |
| 20 | 0.05 | A stagewise, ≤ 8 corr., reject | stop 0.24 (c38) | 37 | 3.16 / 9 | control period 38 (t = 3.7 s): schedule_inconsistent: stale curvature schedule: stage residual 0.01403 rad > 0.005 rad; cumulative residual 0.01434 rad > 0.01 rad |
| 20 | 0.05 | B segment-pair D2, ≤ 8 corr. | stop 0.02 (c6) | 5 | 1.00 / 1 | control period 6 (t = 0.5 s): reference_violation: reference check on the actual track failed: 2 violation(s), first at t = 1.84 s: lateral offset ey = -0.3554 m is beyond the right track boundary (|e |
| 20 | 0.05 | C Cartesian, 1 solve | stop 0.00 (c1) | 0 | 1.00 / 1 | control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.005224 m) |
| 20 | 0.05 | D Cartesian, ≤ 8 reassignments | 2.39 | 250 | 1.22 / 3 | completed |
| 20 | 0.05 | E Cartesian + membership bounds, 1 solve | stop 1.99 (c202) | 201 | 1.00 / 1 | control period 202 (t = 20.1 s): invalid_solution: dynamics residual 1.017324874230674e-6 exceeds acceptance_tol |
| 20 | 0.05 | F = D + strict corridor | stop 0.01 (c4) | 3 | 1.50 / 2 | control period 4 (t = 0.3 s): corridor_violation: re-simulation leaves the planning corridor by 0.02537 m (strict corridor) |
| 20 | 0.10 | A stagewise, ≤ 8 corr., reject | stop 0.24 (c38) | 37 | 3.00 / 9 | control period 38 (t = 3.7 s): schedule_inconsistent: stale curvature schedule: stage residual 0.01183 rad > 0.005 rad; cumulative residual 0.01273 rad > 0.01 rad |
| 20 | 0.10 | B segment-pair D2, ≤ 8 corr. | 2.35 | 250 | 1.14 / 4 | completed |
| 20 | 0.10 | C Cartesian, 1 solve | stop 0.00 (c1) | 0 | 1.00 / 1 | control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.01495 m) |
| 20 | 0.10 | D Cartesian, ≤ 8 reassignments | 2.34 | 250 | 1.18 / 2 | completed |
| 20 | 0.10 | E Cartesian + membership bounds, 1 solve | 2.35 | 250 | 1.00 / 1 | completed |
| 20 | 0.10 | F = D + strict corridor | stop 0.01 (c4) | 3 | 1.50 / 2 | control period 4 (t = 0.3 s): corridor_violation: re-simulation leaves the planning corridor by 0.027 m (strict corridor) |

## Straight → arc (κ 0 → 0.5), one cycle, cold start, N = 20, margin 0.05

Each controller is primed (one solve at the same state, then reset), so the timed cycle excludes the evaluator construction.

| v [m/s] | φ | formulation | status | NLP solves | exact-map horizon error [m] | event-aware reference horizon error [m] |
|:-:|:-:|:--|:--|--:|:-:|:-:|
| 1.0 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.6e-01 | 1.6e-01 |
| 1.0 | 0.1 | segment-pair | success | 1 | 2.8e-04 | 2.8e-04 |
| 1.0 | 0.1 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 1.0e-02 | 1.0e-02 |
| 1.0 | 0.1 | Cartesian, ≤ 8 reassignments | success | 2 | 1.2e-13 | 8.4e-09 |
| 1.0 | 0.5 | stagewise | schedule_inconsistent | 1 | 1.6e-01 | 1.6e-01 |
| 1.0 | 0.5 | segment-pair | success | 1 | 7.9e-04 | 7.9e-04 |
| 1.0 | 0.5 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 7.5e-02 | 7.5e-02 |
| 1.0 | 0.5 | Cartesian, ≤ 8 reassignments | success | 2 | 3.6e-13 | 9.5e-09 |
| 1.0 | 0.9 | stagewise | schedule_inconsistent | 1 | 1.5e-01 | 1.5e-01 |
| 1.0 | 0.9 | segment-pair | success | 1 | 5.6e-04 | 5.6e-04 |
| 1.0 | 0.9 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 6.2e-02 | 6.2e-02 |
| 1.0 | 0.9 | Cartesian, ≤ 8 reassignments | success | 2 | 3.0e-12 | 8.7e-09 |
| 2.0 | 0.1 | stagewise | schedule_inconsistent | 1 | 1.7e-01 | 1.7e-01 |
| 2.0 | 0.1 | segment-pair | success | 1 | 9.7e-04 | 9.7e-04 |
| 2.0 | 0.1 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 1.9e-03 | 1.9e-03 |
| 2.0 | 0.1 | Cartesian, ≤ 8 reassignments | success | 2 | 6.4e-13 | 8.3e-09 |
| 2.0 | 0.5 | stagewise | schedule_inconsistent | 1 | 1.7e-01 | 1.7e-01 |
| 2.0 | 0.5 | segment-pair | success | 1 | 2.7e-03 | 2.7e-03 |
| 2.0 | 0.5 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 3.9e-02 | 3.9e-02 |
| 2.0 | 0.5 | Cartesian, ≤ 8 reassignments | success | 2 | 5.8e-11 | 8.5e-09 |
| 2.0 | 0.9 | stagewise | schedule_inconsistent | 1 | 1.7e-01 | 1.7e-01 |
| 2.0 | 0.9 | segment-pair | success | 1 | 1.5e-03 | 1.5e-03 |
| 2.0 | 0.9 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 2.5e-02 | 2.5e-02 |
| 2.0 | 0.9 | Cartesian, ≤ 8 reassignments | success | 2 | 3.8e-12 | 9.0e-09 |
| 2.8 | 0.1 | stagewise | schedule_inconsistent | 1 | 2.1e-01 | 2.1e-01 |
| 2.8 | 0.1 | segment-pair | success | 1 | 1.8e-03 | 1.8e-03 |
| 2.8 | 0.1 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 8.4e-03 | 8.4e-03 |
| 2.8 | 0.1 | Cartesian, ≤ 8 reassignments | success | 2 | 1.2e-10 | 8.4e-09 |
| 2.8 | 0.5 | stagewise | schedule_inconsistent | 1 | 2.1e-01 | 2.1e-01 |
| 2.8 | 0.5 | segment-pair | success | 1 | 4.9e-03 | 4.9e-03 |
| 2.8 | 0.5 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 6.2e-02 | 6.2e-02 |
| 2.8 | 0.5 | Cartesian, ≤ 8 reassignments | success | 2 | 3.7e-11 | 9.6e-09 |
| 2.8 | 0.9 | stagewise | schedule_inconsistent | 1 | 1.9e-01 | 1.9e-01 |
| 2.8 | 0.9 | segment-pair | success | 1 | 1.9e-03 | 1.9e-03 |
| 2.8 | 0.9 | Cartesian, 1 solve | segment_assignment_invalid | 1 | 3.7e-02 | 3.7e-02 |
| 2.8 | 0.9 | Cartesian, ≤ 8 reassignments | success | 2 | 1.4e-10 | 8.7e-09 |

## Single-segment tracks (Cartesian mode vs default Frenet RK4)

| track | Cartesian status | max state difference | s_end Frenet / Cartesian [m] | Cartesian max exact-map error [m] |
|:--|:--|:-:|:-:|:-:|
| straight 150 m | success | 2.2e-06 | 47.9466 / 47.9466 | 1.2e-08 |
| circle R = 1 m | success | 1.2e-05 | 13.8465 / 13.8465 | 2.9e-10 |

