# Between-knot corridor constraints benchmark (Milestone 2B.5)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `[0.0, 0.0, 0.0, 0.5]`
- 250 control cycles (control period 0.1 s, prediction step 0.1 s) per run, 3 interleaved runs per configuration; event-aware plant (10 substeps); all cycles counted
- All configurations: Cartesian-endpoint prediction, ≤ 8 segment reassignments within the 80 ms budget, dense exact-map corridor check at 20 samples per interval in every cycle. Objective, weights, limits, vehicle, Ipopt options and `acceptance_tol` = 1.0e-6 are identical
- "strict" = `strict_corridor = true`: a candidate whose event-aware re-simulation (5/interval) or dense check (20/interval) leaves the planning corridor by more than 1.0e-6 m is rejected. A0/B0/C0 are the same controllers without that rule (diagnostic acceptance; A0 is the Milestone 2B.4 controller)
- "Corridor" = `|ey| ≤ width/2 − margin`; "track" = `|ey| ≤ width/2`. Predicted clearances are minima over accepted plans
- Timing covers everything in `solve!` (assignments, all solves, reference check, exact-map check, interior check, dense check); "validation" is the part spent in the independent checks

## Quarter track, N = 10 (1.0 s horizon), margin 0.05 m

| configuration | laps per run | accepted cycles | rejections by status | NLP solves mean / max | corrections per cycle | Ipopt iter. mean / max | decision variables / constraints | build / priming [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms | validation median / max [ms] |
|:--|:--|--:|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 11.0 / 11 | 60 / 80 | 2 / 13 | 10.4 / 10.8 / 10.8 / 10.8 | 0 / 0 | 0.50 / 0.6 |
| B Cartesian + midpoint, strict | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 13.0 / 13 | 60 / 100 | 4 / 24 | 16.3 / 17.6 / 17.7 / 17.8 | 0 / 0 | 0.56 / 0.6 |
| C Cartesian + 3 samples, strict | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 14.0 / 14 | 60 / 140 | 9 / 44 | 28.0 / 34.0 / 34.6 / 34.7 | 0 / 0 | 0.58 / 0.6 |
| A0 = A without strict corridor (2B.4) | 2.58; 2.58; 2.58 | 750 / 750 | – | 1.13 / 2 | 0.13 | 15.3 / 23 | 60 / 80 | 2 / 19 | 6.3 / 13.2 / 15.1 / 26.3 | 0 / 0 | 0.26 / 12.6 |
| B0 = B without strict corridor | 2.58; 2.58; 2.58 | 750 / 750 | – | 1.24 / 2 | 0.24 | 15.6 / 22 | 60 / 100 | 5 / 23 | 10.0 / 20.7 / 23.9 / 26.2 | 0 / 0 | 0.28 / 5.2 |
| C0 = C without strict corridor | 2.58; 2.58; 2.58 | 750 / 750 | – | 1.40 / 3 | 0.40 | 16.0 / 22 | 60 / 140 | 9 / 48 | 17.6 / 36.7 / 44.2 / 102.8 | 1 / 1 | 0.33 / 4.3 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted (dense, 20/interval): track / corridor [m] | worst at t [s] (stage) | accepted cycles with corridor violation / largest excursion [m] | interior: min margin / assignment failures / prediction error [m] | event-aware re-simulation corridor [m] |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | 0 | 0.3500 / 0.3000 | NaN / NaN | NaN (0) | 0 / NaN | NaN / 0 / NaN | NaN |
| B Cartesian + midpoint, strict | 0 | 0.3500 / 0.3000 | NaN / NaN | NaN (0) | 0 / NaN | NaN / 0 / NaN | NaN |
| C Cartesian + 3 samples, strict | 0 | 0.3500 / 0.3000 | NaN / NaN | NaN (0) | 0 / NaN | NaN / 0 / NaN | NaN |
| A0 = A without strict corridor (2B.4) | 0 | 0.0463 / -0.0037 | 0.0137 / -0.0363 | 0.750 (8) | 492 / 0.0363 | NaN / 0 / NaN | -0.0349 |
| B0 = B without strict corridor | 0 | 0.0493 / -0.0007 | 0.0409 / -0.0091 | 0.975 (10) | 489 / 0.0091 | -1.0e-08 / 0 / 1.0e-08 | -0.0089 |
| C0 = C without strict corridor | 0 | 0.0498 / -0.0002 | 0.0477 / -0.0023 | 0.765 (8) | 432 / 0.0023 | -1.0e-08 / 0 / 1.6e-08 | -0.0023 |

Stop reasons (run 1):

- **A Cartesian, no interior samples, strict**: control period 1 (t = 0.0 s): corridor_violation: plan leaves the planning corridor by 0.01521 m (strict corridor; event-aware re-simulation -0.01446 m, dense check -0.01521 m at t = 0.87 s, stage 9)
- **B Cartesian + midpoint, strict**: control period 1 (t = 0.0 s): corridor_violation: plan leaves the planning corridor by 0.00433 m (strict corridor; event-aware re-simulation -0.003547 m, dense check -0.00433 m at t = 0.87 s, stage 9)
- **C Cartesian + 3 samples, strict**: control period 1 (t = 0.0 s): corridor_violation: plan leaves the planning corridor by 0.000824 m (strict corridor; event-aware re-simulation -0.0001834 m, dense check -0.000824 m at t = 0.865 s, stage 9)
- **A0 = A without strict corridor (2B.4)**: completed
- **B0 = B without strict corridor**: completed
- **C0 = C without strict corridor**: completed

Predicted corridor clearance vs validation resolution (run 1, accepted plans; minimum over cycles) [m], and analytic − dense(100) gap:

| configuration | plans | 20 / interval | 50 / interval | 100 / interval | analytic (continuous) | max (dense 100 − analytic) | analytic: candidates / projection failures |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A0 = A without strict corridor (2B.4) | 250 | -0.03635 | -0.03635 | -0.03635 | -0.03635 | 3.0e-06 | 6739 / 0 |
| B0 = B without strict corridor | 250 | -0.00910 | -0.00911 | -0.00911 | -0.00911 | 3.3e-06 | 6734 / 0 |
| C0 = C without strict corridor | 250 | -0.00230 | -0.00239 | -0.00239 | -0.00239 | 3.8e-06 | 6729 / 0 |

Timing by run (median / p99 / max [ms]; GC total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A Cartesian, no interior samples, strict | 10.4 / 10.4 / 10.4; GC 0.0 / 0.0 | 10.3 / 10.3 / 10.3; GC 0.0 / 0.0 | 10.8 / 10.8 / 10.8; GC 0.0 / 0.0 |
| B Cartesian + midpoint, strict | 17.8 / 17.8 / 17.8; GC 0.0 / 0.0 | 16.3 / 16.3 / 16.3; GC 0.0 / 0.0 | 15.9 / 15.9 / 15.9; GC 0.0 / 0.0 |
| C Cartesian + 3 samples, strict | 28.0 / 28.0 / 28.0; GC 0.0 / 0.0 | 34.7 / 34.7 / 34.7; GC 7.4 / 7.4 | 25.4 / 25.4 / 25.4; GC 0.0 / 0.0 |
| A0 = A without strict corridor (2B.4) | 6.5 / 15.7 / 26.3; GC 30.1 / 17.9 | 6.4 / 14.7 / 17.4; GC 14.0 / 3.9 | 6.2 / 14.4 / 15.6; GC 11.3 / 6.0 |
| B0 = B without strict corridor | 10.0 / 23.9 / 25.8; GC 19.4 / 5.7 | 10.1 / 23.8 / 26.2; GC 24.5 / 7.3 | 9.7 / 21.5 / 24.8; GC 23.6 / 5.9 |
| C0 = C without strict corridor | 18.1 / 41.6 / 51.2; GC 43.4 / 6.3 | 17.3 / 45.1 / 102.8; GC 134.9 / 89.5 | 17.4 / 42.6 / 51.2; GC 43.1 / 5.1 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! / validation [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):

- C0 = C without strict corridor, run 2, cycle 75: 102.8 / 103.0 / 102.4 / 0.3 ms, GC 89.5 ms, 1.0 MB, 1 solve(s), 13 iterations

## Quarter track, N = 10 (1.0 s horizon), margin 0.1 m

| configuration | laps per run | accepted cycles | rejections by status | NLP solves mean / max | corrections per cycle | Ipopt iter. mean / max | decision variables / constraints | build / priming [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms | validation median / max [ms] |
|:--|:--|--:|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 13.0 / 13 | 60 / 80 | 2 / 15 | 11.6 / 20.1 / 20.9 / 21.1 | 0 / 0 | 0.52 / 9.8 |
| B Cartesian + midpoint, strict | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 12.0 / 12 | 60 / 100 | 5 / 24 | 16.4 / 16.6 / 16.7 / 16.7 | 0 / 0 | 0.52 / 0.5 |
| C Cartesian + 3 samples, strict | stop 0.00 (c1); stop 0.00 (c1); stop 0.00 (c1) | 0 / 3 | corridor_violation 3 | 2.00 / 2 | 1.00 | 13.0 / 13 | 60 / 140 | 9 / 48 | 28.8 / 29.0 / 29.0 / 29.0 | 0 / 0 | 0.69 / 0.8 |
| A0 = A without strict corridor (2B.4) | 2.50; 2.50; 2.50 | 750 / 750 | – | 1.11 / 2 | 0.11 | 15.3 / 22 | 60 / 80 | 2 / 14 | 6.2 / 12.5 / 15.0 / 27.5 | 0 / 0 | 0.26 / 13.0 |
| B0 = B without strict corridor | 2.50; 2.50; 2.50 | 750 / 750 | – | 1.21 / 2 | 0.21 | 15.7 / 22 | 60 / 100 | 5 / 24 | 9.6 / 20.7 / 23.0 / 106.6 | 1 / 1 | 0.28 / 98.8 |
| C0 = C without strict corridor | 2.50; 2.50; 2.50 | 750 / 750 | – | 1.37 / 2 | 0.37 | 15.6 / 22 | 60 / 140 | 9 / 45 | 16.8 / 34.6 / 37.1 / 42.4 | 0 / 0 | 0.33 / 5.8 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted (dense, 20/interval): track / corridor [m] | worst at t [s] (stage) | accepted cycles with corridor violation / largest excursion [m] | interior: min margin / assignment failures / prediction error [m] | event-aware re-simulation corridor [m] |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | 0 | 0.3500 / 0.2500 | NaN / NaN | NaN (0) | 0 / NaN | NaN / 0 / NaN | NaN |
| B Cartesian + midpoint, strict | 0 | 0.3500 / 0.2500 | NaN / NaN | NaN (0) | 0 / NaN | NaN / 0 / NaN | NaN |
| C Cartesian + 3 samples, strict | 0 | 0.3500 / 0.2500 | NaN / NaN | NaN (0) | 0 / NaN | NaN / 0 / NaN | NaN |
| A0 = A without strict corridor (2B.4) | 0 | 0.0969 / -0.0031 | 0.0632 / -0.0368 | 0.950 (10) | 411 / 0.0368 | NaN / 0 / NaN | -0.0357 |
| B0 = B without strict corridor | 0 | 0.0994 / -0.0006 | 0.0927 / -0.0073 | 0.820 (9) | 393 / 0.0073 | -1.0e-08 / 0 / 9.9e-09 | -0.0073 |
| C0 = C without strict corridor | 0 | 0.0999 / -0.0001 | 0.0979 / -0.0021 | 0.890 (9) | 369 / 0.0021 | -1.0e-08 / 0 / 1.0e-08 | -0.0013 |

Stop reasons (run 1):

- **A Cartesian, no interior samples, strict**: control period 1 (t = 0.0 s): corridor_violation: plan leaves the planning corridor by 0.01636 m (strict corridor; event-aware re-simulation -0.01601 m, dense check -0.01636 m at t = 0.865 s, stage 9)
- **B Cartesian + midpoint, strict**: control period 1 (t = 0.0 s): corridor_violation: plan leaves the planning corridor by 0.002773 m (strict corridor; event-aware re-simulation -0.002485 m, dense check -0.002773 m at t = 0.865 s, stage 9)
- **C Cartesian + 3 samples, strict**: control period 1 (t = 0.0 s): corridor_violation: plan leaves the planning corridor by 0.001478 m (strict corridor; event-aware re-simulation -0.001244 m, dense check -0.001478 m at t = 0.865 s, stage 9)
- **A0 = A without strict corridor (2B.4)**: completed
- **B0 = B without strict corridor**: completed
- **C0 = C without strict corridor**: completed

Predicted corridor clearance vs validation resolution (run 1, accepted plans; minimum over cycles) [m], and analytic − dense(100) gap:

| configuration | plans | 20 / interval | 50 / interval | 100 / interval | analytic (continuous) | max (dense 100 − analytic) | analytic: candidates / projection failures |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A0 = A without strict corridor (2B.4) | 250 | -0.03675 | -0.03678 | -0.03678 | -0.03678 | 2.8e-06 | 6789 / 0 |
| B0 = B without strict corridor | 250 | -0.00730 | -0.00738 | -0.00738 | -0.00738 | 3.6e-06 | 6784 / 0 |
| C0 = C without strict corridor | 250 | -0.00213 | -0.00220 | -0.00220 | -0.00220 | 3.3e-06 | 6782 / 0 |

Timing by run (median / p99 / max [ms]; GC total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A Cartesian, no interior samples, strict | 11.2 / 11.2 / 11.2; GC 0.0 / 0.0 | 21.1 / 21.1 / 21.1; GC 8.8 / 8.8 | 11.6 / 11.6 / 11.6; GC 0.0 / 0.0 |
| B Cartesian + midpoint, strict | 16.3 / 16.3 / 16.3; GC 0.0 / 0.0 | 16.4 / 16.4 / 16.4; GC 0.0 / 0.0 | 16.7 / 16.7 / 16.7; GC 0.0 / 0.0 |
| C Cartesian + 3 samples, strict | 28.8 / 28.8 / 28.8; GC 0.0 / 0.0 | 29.0 / 29.0 / 29.0; GC 0.0 / 0.0 | 28.1 / 28.1 / 28.1; GC 0.0 / 0.0 |
| A0 = A without strict corridor (2B.4) | 6.3 / 14.9 / 27.5; GC 12.4 / 12.4 | 6.1 / 15.5 / 18.6; GC 16.3 / 4.4 | 6.1 / 14.2 / 17.9; GC 22.2 / 11.5 |
| B0 = B without strict corridor | 9.6 / 25.2 / 106.6; GC 135.2 / 96.9 | 9.7 / 22.7 / 23.5; GC 24.1 / 4.6 | 9.5 / 22.6 / 25.8; GC 21.7 / 6.6 |
| C0 = C without strict corridor | 16.6 / 36.7 / 41.1; GC 33.8 / 13.7 | 16.8 / 36.0 / 42.4; GC 41.9 / 8.1 | 16.9 / 38.3 / 40.2; GC 40.9 / 6.1 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! / validation [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):

- B0 = B without strict corridor, run 1, cycle 222: 106.6 / 10.0 / 7.6 / 98.8 ms, GC 96.9 ms, 0.7 MB, 1 solve(s), 14 iterations

## Quarter track, N = 20 (2.0 s horizon), margin 0.05 m

| configuration | laps per run | accepted cycles | rejections by status | NLP solves mean / max | corrections per cycle | Ipopt iter. mean / max | decision variables / constraints | build / priming [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms | validation median / max [ms] |
|:--|:--|--:|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | corridor_violation 3 | 1.50 / 2 | 0.50 | 14.5 / 16 | 120 / 160 | 4 / 26 | 13.0 / 21.8 / 21.8 / 21.8 | 0 / 0 | 0.80 / 1.0 |
| B Cartesian + midpoint, strict | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | corridor_violation 3 | 1.75 / 3 | 0.75 | 14.2 / 16 | 120 / 200 | 8 / 41 | 25.8 / 48.9 / 49.8 / 50.0 | 0 / 0 | 1.04 / 11.7 |
| C Cartesian + 3 samples, strict | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | interior_assignment_invalid 3 | 1.50 / 2 | 0.50 | 15.2 / 19 | 120 / 280 | 18 / 69 | 36.2 / 66.2 / 66.3 / 66.3 | 0 / 0 | 0.94 / 1.5 |
| A0 = A without strict corridor (2B.4) | 2.39; 2.39; 2.39 | 750 / 750 | – | 1.22 / 3 | 0.22 | 16.4 / 22 | 120 / 160 | 3 / 22 | 10.1 / 21.2 / 26.7 / 119.1 | 1 / 1 | 0.50 / 110.8 |
| B0 = B without strict corridor | 2.39; 2.39; stop 0.99 (c104) | 603 / 604 | segment_assignment_invalid 1 | 1.37 / 3 | 0.37 | 16.3 / 23 | 120 / 200 | 8 / 44 | 17.1 / 38.5 / 50.6 / 134.8 | 1 / 1 | 0.56 / 18.5 |
| C0 = C without strict corridor | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | interior_assignment_invalid 3 | 1.50 / 2 | 0.50 | 15.2 / 19 | 120 / 280 | 22 / 65 | 37.9 / 68.2 / 69.9 / 70.4 | 0 / 0 | 0.83 / 1.5 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted (dense, 20/interval): track / corridor [m] | worst at t [s] (stage) | accepted cycles with corridor violation / largest excursion [m] | interior: min margin / assignment failures / prediction error [m] | event-aware re-simulation corridor [m] |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | 0 | 0.3438 / 0.2938 | 0.2443 / 0.1943 | 1.205 (13) | 0 / 0.0000 | NaN / 0 / NaN | 0.1943 |
| B Cartesian + midpoint, strict | 0 | 0.3438 / 0.2938 | 0.2443 / 0.1943 | 1.205 (13) | 0 / 0.0000 | 1.6e-01 / 0 / 4.0e-12 | 0.1943 |
| C Cartesian + 3 samples, strict | 0 | 0.3438 / 0.2938 | 0.2443 / 0.1943 | 1.205 (13) | 0 / 0.0000 | 1.5e-01 / 6 / 4.3e-12 | 0.1943 |
| A0 = A without strict corridor (2B.4) | 0 | 0.1288 / 0.0788 | 0.0076 / -0.0424 | 1.950 (20) | 96 / 0.0424 | NaN / 0 / NaN | -0.0411 |
| B0 = B without strict corridor | 0 | 0.1306 / 0.0806 | 0.0400 / -0.0100 | 1.925 (20) | 74 / 0.0100 | -9.1e-09 / 1 / 8.2e-09 | -0.0095 |
| C0 = C without strict corridor | 0 | 0.3438 / 0.2938 | 0.2443 / 0.1943 | 1.205 (13) | 0 / 0.0000 | 1.5e-01 / 6 / 4.3e-12 | 0.1943 |

Stop reasons (run 1):

- **A Cartesian, no interior samples, strict**: control period 4 (t = 0.3 s): corridor_violation: plan leaves the planning corridor by 0.02538 m (strict corridor; event-aware re-simulation -0.02537 m, dense check -0.02538 m at t = 1.94 s, stage 20)
- **B Cartesian + midpoint, strict**: control period 4 (t = 0.3 s): corridor_violation: plan leaves the planning corridor by 0.003959 m (strict corridor; event-aware re-simulation -0.003464 m, dense check -0.003959 m at t = 1.935 s, stage 20)
- **C Cartesian + 3 samples, strict**: control period 4 (t = 0.3 s): interior_assignment_invalid: invalid interior segment assignment: 2 interior sample(s) with an invalid segment assignment (missing 0, ambiguous 0, outside 2); first: stage 19, alpha = 0.5: projects to s = 5.226 outside its assigned segment [2.0, 5.142]; plan leaves the planning corridor by 0.002426 m (strict corridor; event-aware re-simulation -0.002425 m, dense check -0.002426 m at t = 
- **A0 = A without strict corridor (2B.4)**: completed
- **B0 = B without strict corridor**: completed
- **C0 = C without strict corridor**: control period 4 (t = 0.3 s): interior_assignment_invalid: invalid interior segment assignment: 2 interior sample(s) with an invalid segment assignment (missing 0, ambiguous 0, outside 2); first: stage 19, alpha = 0.5: projects to s = 5.226 outside its assigned segment [2.0, 5.142]; schedule correction skipped: time budget 0.08 s would be exceeded

Predicted corridor clearance vs validation resolution (run 1, accepted plans; minimum over cycles) [m], and analytic − dense(100) gap:

| configuration | plans | 20 / interval | 50 / interval | 100 / interval | analytic (continuous) | max (dense 100 − analytic) | analytic: candidates / projection failures |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | 3 | 0.19428 | 0.19428 | 0.19428 | 0.19428 | 5.9e-08 | 164 / 0 |
| B Cartesian + midpoint, strict | 3 | 0.19428 | 0.19428 | 0.19428 | 0.19428 | 5.9e-08 | 164 / 0 |
| C Cartesian + 3 samples, strict | 3 | 0.19428 | 0.19428 | 0.19428 | 0.19428 | 5.9e-08 | 164 / 0 |
| A0 = A without strict corridor (2B.4) | 250 | -0.04240 | -0.04243 | -0.04243 | -0.04243 | 2.9e-06 | 13698 / 0 |
| B0 = B without strict corridor | 250 | -0.01000 | -0.00999 | -0.01000 | -0.01000 | 3.4e-06 | 13697 / 0 |
| C0 = C without strict corridor | 3 | 0.19428 | 0.19428 | 0.19428 | 0.19428 | 5.9e-08 | 164 / 0 |

Timing by run (median / p99 / max [ms]; GC total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A Cartesian, no interior samples, strict | 12.7 / 21.6 / 21.8; GC 0.0 / 0.0 | 12.9 / 21.6 / 21.8; GC 0.0 / 0.0 | 13.2 / 21.5 / 21.6; GC 0.0 / 0.0 |
| B Cartesian + midpoint, strict | 26.1 / 49.3 / 50.0; GC 10.9 / 10.9 | 20.7 / 47.3 / 48.0; GC 0.0 / 0.0 | 21.5 / 47.3 / 47.9; GC 0.0 / 0.0 |
| C Cartesian + 3 samples, strict | 36.4 / 65.5 / 66.1; GC 0.0 / 0.0 | 35.5 / 64.3 / 64.9; GC 0.0 / 0.0 | 36.8 / 65.7 / 66.3; GC 0.0 / 0.0 |
| A0 = A without strict corridor (2B.4) | 10.1 / 35.0 / 119.1; GC 161.4 / 108.6 | 10.2 / 24.8 / 29.2; GC 31.2 / 7.2 | 10.1 / 24.3 / 38.0; GC 26.5 / 9.6 |
| B0 = B without strict corridor | 17.2 / 50.3 / 55.9; GC 42.3 / 17.1 | 17.0 / 49.4 / 50.6; GC 58.7 / 13.9 | 17.2 / 54.7 / 134.8; GC 151.8 / 113.8 |
| C0 = C without strict corridor | 37.5 / 65.9 / 66.5; GC 0.0 / 0.0 | 38.9 / 69.7 / 70.4; GC 0.0 / 0.0 | 36.5 / 65.0 / 65.6; GC 0.0 / 0.0 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! / validation [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):

- A0 = A without strict corridor (2B.4), run 1, cycle 214: 119.1 / 10.0 / 8.1 / 110.8 ms, GC 108.6 ms, 0.8 MB, 1 solve(s), 16 iterations
- B0 = B without strict corridor, run 3, cycle 104: 134.8 / 132.0 / 134.1 / 0.5 ms, GC 113.8 ms, 1.5 MB, 1 solve(s), 21 iterations

## Quarter track, N = 20 (2.0 s horizon), margin 0.1 m

| configuration | laps per run | accepted cycles | rejections by status | NLP solves mean / max | corrections per cycle | Ipopt iter. mean / max | decision variables / constraints | build / priming [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms | validation median / max [ms] |
|:--|:--|--:|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | corridor_violation 3 | 1.50 / 2 | 0.50 | 14.8 / 17 | 120 / 160 | 3 / 22 | 12.7 / 22.2 / 22.5 / 22.6 | 0 / 0 | 0.76 / 1.1 |
| B Cartesian + midpoint, strict | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | corridor_violation 3 | 1.50 / 2 | 0.50 | 15.5 / 20 | 120 / 200 | 8 / 47 | 20.4 / 38.0 / 38.2 / 38.2 | 0 / 0 | 0.84 / 1.0 |
| C Cartesian + 3 samples, strict | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | segment_assignment_invalid 3 | 1.50 / 2 | 0.50 | 16.8 / 23 | 120 / 280 | 24 / 59 | 37.6 / 73.0 / 73.0 / 73.0 | 0 / 0 | 0.85 / 1.2 |
| A0 = A without strict corridor (2B.4) | 2.34; 2.34; 2.34 | 750 / 750 | – | 1.18 / 2 | 0.18 | 16.5 / 25 | 120 / 160 | 3 / 21 | 10.1 / 20.9 / 22.8 / 35.1 | 0 / 0 | 0.49 / 15.5 |
| B0 = B without strict corridor | 2.34; 2.34; 2.34 | 750 / 750 | – | 1.33 / 3 | 0.33 | 16.4 / 22 | 120 / 200 | 8 / 41 | 16.8 / 36.9 / 51.7 / 155.5 | 1 / 1 | 0.55 / 26.8 |
| C0 = C without strict corridor | stop 0.01 (c4); stop 0.01 (c4); stop 0.01 (c4) | 9 / 12 | segment_assignment_invalid 3 | 1.50 / 2 | 0.50 | 16.8 / 23 | 120 / 280 | 19 / 76 | 37.9 / 72.7 / 73.5 / 73.7 | 0 / 0 | 0.87 / 1.4 |

| configuration | plant violations | plant clearance: track / corridor [m] | predicted (dense, 20/interval): track / corridor [m] | worst at t [s] (stage) | accepted cycles with corridor violation / largest excursion [m] | interior: min margin / assignment failures / prediction error [m] | event-aware re-simulation corridor [m] |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | 0 | 0.3447 / 0.2447 | 0.2677 / 0.1677 | 2.000 (20) | 0 / 0.0000 | NaN / 0 / NaN | 0.1677 |
| B Cartesian + midpoint, strict | 0 | 0.3447 / 0.2447 | 0.2677 / 0.1677 | 2.000 (20) | 0 / 0.0000 | 1.4e-01 / 0 / 7.0e-12 | 0.1677 |
| C Cartesian + 3 samples, strict | 0 | 0.3447 / 0.2447 | 0.2677 / 0.1677 | 2.000 (20) | 0 / 0.0000 | 1.4e-01 / 0 / 7.5e-12 | 0.1677 |
| A0 = A without strict corridor (2B.4) | 0 | 0.1663 / 0.0663 | 0.0718 / -0.0282 | 1.850 (19) | 81 / 0.0282 | NaN / 0 / NaN | -0.0271 |
| B0 = B without strict corridor | 0 | 0.1662 / 0.0662 | 0.0921 / -0.0079 | 1.875 (19) | 72 / 0.0079 | -8.6e-09 / 0 / 2.5e-09 | -0.0076 |
| C0 = C without strict corridor | 0 | 0.3447 / 0.2447 | 0.2677 / 0.1677 | 2.000 (20) | 0 / 0.0000 | 1.4e-01 / 0 / 7.5e-12 | 0.1677 |

Stop reasons (run 1):

- **A Cartesian, no interior samples, strict**: control period 4 (t = 0.3 s): corridor_violation: plan leaves the planning corridor by 0.027 m (strict corridor; event-aware re-simulation -0.027 m, dense check -0.027 m at t = 1.96 s, stage 20)
- **B Cartesian + midpoint, strict**: control period 4 (t = 0.3 s): corridor_violation: plan leaves the planning corridor by 0.002037 m (strict corridor; event-aware re-simulation -0.002036 m, dense check -0.002037 m at t = 1.96 s, stage 20)
- **C Cartesian + 3 samples, strict**: control period 4 (t = 0.3 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.03951 m); schedule correction skipped: time budget 0.08 s would be exceeded
- **A0 = A without strict corridor (2B.4)**: completed
- **B0 = B without strict corridor**: completed
- **C0 = C without strict corridor**: control period 4 (t = 0.3 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.03951 m); schedule correction skipped: time budget 0.08 s would be exceeded

Predicted corridor clearance vs validation resolution (run 1, accepted plans; minimum over cycles) [m], and analytic − dense(100) gap:

| configuration | plans | 20 / interval | 50 / interval | 100 / interval | analytic (continuous) | max (dense 100 − analytic) | analytic: candidates / projection failures |
|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
| A Cartesian, no interior samples, strict | 3 | 0.16766 | 0.16766 | 0.16766 | 0.16766 | 0.0e+00 | 165 / 0 |
| B Cartesian + midpoint, strict | 3 | 0.16766 | 0.16766 | 0.16766 | 0.16766 | 0.0e+00 | 165 / 0 |
| C Cartesian + 3 samples, strict | 3 | 0.16766 | 0.16766 | 0.16766 | 0.16766 | 0.0e+00 | 165 / 0 |
| A0 = A without strict corridor (2B.4) | 250 | -0.02818 | -0.02818 | -0.02818 | -0.02818 | 1.8e-06 | 13842 / 0 |
| B0 = B without strict corridor | 250 | -0.00792 | -0.00792 | -0.00792 | -0.00792 | 3.0e-06 | 13839 / 0 |
| C0 = C without strict corridor | 3 | 0.16766 | 0.16766 | 0.16766 | 0.16766 | 0.0e+00 | 165 / 0 |

Timing by run (median / p99 / max [ms]; GC total / max [ms]):

| configuration | run 1 | run 2 | run 3 |
|:--|:--|:--|:--|
| A Cartesian, no interior samples, strict | 12.8 / 22.4 / 22.6; GC 0.0 / 0.0 | 12.9 / 21.8 / 21.9; GC 0.0 / 0.0 | 12.4 / 21.5 / 21.7; GC 0.0 / 0.0 |
| B Cartesian + midpoint, strict | 20.9 / 37.8 / 38.2; GC 0.0 / 0.0 | 20.9 / 37.6 / 37.9; GC 0.0 / 0.0 | 19.6 / 35.4 / 35.7; GC 0.0 / 0.0 |
| C Cartesian + 3 samples, strict | 37.9 / 72.2 / 72.9; GC 0.0 / 0.0 | 38.6 / 72.3 / 73.0; GC 0.0 / 0.0 | 37.1 / 71.7 / 72.5; GC 0.0 / 0.0 |
| A0 = A without strict corridor (2B.4) | 10.1 / 23.0 / 30.3; GC 42.7 / 20.6 | 10.3 / 22.7 / 35.1; GC 29.7 / 13.1 | 10.1 / 22.5 / 29.9; GC 46.0 / 17.2 |
| B0 = B without strict corridor | 16.9 / 50.7 / 53.1; GC 55.8 / 10.5 | 16.5 / 51.8 / 155.5; GC 194.7 / 126.7 | 16.8 / 51.9 / 54.3; GC 63.2 / 11.7 |
| C0 = C without strict corridor | 37.8 / 71.2 / 71.9; GC 0.0 / 0.0 | 38.3 / 69.3 / 69.9; GC 0.0 / 0.0 | 37.9 / 72.9 / 73.7; GC 0.0 / 0.0 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! / validation [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):

- B0 = B without strict corridor, run 2, cycle 138: 155.5 / 150.0 / 154.1 / 1.1 ms, GC 126.7 ms, 2.4 MB, 2 solve(s), 14 iterations

## Without the correction time budget (diagnostic acceptance, one run each)

`correction_time_budget = 1000 s` (still ≤ 8 reassignments), primed controllers. The outcome then depends only on the computation; the timings show what the 80 ms budget would have cut.

| N | margin | configuration | laps / stop | NLP solves mean / max | median / p99 / max [ms] | > 80 / > 100 ms | predicted dense corridor clearance [m] |
|--:|--:|:--|:--|:-:|:-:|:-:|:-:|
| 10 | 0.05 | A0 = A without strict corridor (2B.4) | 2.58 | 1.13 / 2 | 6.2 / 15.0 / 17.1 | 0 / 0 | -0.0363 |
| 10 | 0.05 | B0 = B without strict corridor | 2.58 | 1.24 / 2 | 9.8 / 27.4 / 30.0 | 0 / 0 | -0.0091 |
| 10 | 0.05 | C0 = C without strict corridor | 2.58 | 1.40 / 3 | 17.2 / 44.7 / 53.7 | 0 / 0 | -0.0023 |
| 10 | 0.10 | A0 = A without strict corridor (2B.4) | 2.50 | 1.11 / 2 | 6.3 / 14.9 / 17.7 | 0 / 0 | -0.0368 |
| 10 | 0.10 | B0 = B without strict corridor | 2.50 | 1.21 / 2 | 9.7 / 23.5 / 24.1 | 0 / 0 | -0.0073 |
| 10 | 0.10 | C0 = C without strict corridor | 2.50 | 1.37 / 2 | 16.6 / 38.6 / 45.8 | 0 / 0 | -0.0021 |
| 20 | 0.05 | A0 = A without strict corridor (2B.4) | 2.39 | 1.22 / 3 | 10.2 / 23.3 / 29.9 | 0 / 0 | -0.0424 |
| 20 | 0.05 | B0 = B without strict corridor | 2.39 | 1.38 / 3 | 16.9 / 49.9 / 166.8 | 1 / 1 | -0.0100 |
| 20 | 0.05 | C0 = C without strict corridor | 2.39 | 1.56 / 3 | 51.8 / 143.1 / 160.8 | 70 / 21 | -0.0024 |
| 20 | 0.10 | A0 = A without strict corridor (2B.4) | 2.34 | 1.18 / 2 | 10.2 / 23.8 / 32.5 | 0 / 0 | -0.0282 |
| 20 | 0.10 | B0 = B without strict corridor | 2.34 | 1.33 / 3 | 16.8 / 51.2 / 54.1 | 0 / 0 | -0.0079 |
| 20 | 0.10 | C0 = C without strict corridor | 2.34 | 1.51 / 4 | 35.7 / 99.9 / 143.5 | 10 / 3 | -0.0020 |

## Other tracks (N = 20, margin 0.05, one run of 100 cycles)

| track | configuration | status | accepted cycles | predicted dense corridor clearance [m] | analytic corridor clearance [m] | plant clearance: track / corridor [m] | median / max [ms] |
|:--|:--|:--|--:|:-:|:-:|:-:|:-:|
| straight 150 m | A Cartesian, no interior samples, strict | completed | 100 / 100 | 0.0850 | 0.0850 | 0.1350 / 0.0850 | 6.1 / 16.8 |
| straight 150 m | B Cartesian + midpoint, strict | completed | 100 / 100 | 0.0850 | 0.0850 | 0.1350 / 0.0850 | 9.5 / 27.1 |
| straight 150 m | C Cartesian + 3 samples, strict | completed | 100 / 100 | 0.0850 | 0.0850 | 0.1350 / 0.0850 | 17.4 / 47.7 |
| circle R = 1 m | A Cartesian, no interior samples, strict | completed | 100 / 100 | 0.1937 | 0.1937 | 0.3176 / 0.2676 | 10.5 / 157.1 |
| circle R = 1 m | B Cartesian + midpoint, strict | completed | 100 / 100 | 0.1937 | 0.1937 | 0.3176 / 0.2676 | 18.4 / 40.7 |
| circle R = 1 m | C Cartesian + 3 samples, strict | completed | 100 / 100 | 0.1937 | 0.1937 | 0.3176 / 0.2676 | 31.5 / 44.3 |
| straight → arc → straight | A Cartesian, no interior samples, strict | control period 19 (t = 1.8 s): scheduling_failure: nominal rollout leaves the open track (interval 20: reached the end of the open track (s = 13.71238898038469) | 18 / 19 | 0.0836 | 0.0836 | 0.2164 / 0.1664 | 10.2 / 20.2 |
| straight → arc → straight | B Cartesian + midpoint, strict | control period 19 (t = 1.8 s): scheduling_failure: nominal rollout leaves the open track (interval 20: reached the end of the open track (s = 13.71238898038469) | 18 / 19 | 0.0836 | 0.0836 | 0.2164 / 0.1664 | 17.9 / 36.0 |
| straight → arc → straight | C Cartesian + 3 samples, strict | control period 1 (t = 0.0 s): segment_assignment_invalid: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08894 m); s | 0 / 1 | NaN | NaN | 0.3500 / 0.3000 | 64.0 / 64.0 |

