# Stationary-obstacle collision avoidance benchmark (Milestone 3A)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; straight track 80 m, width 0.7 m (planning corridor |ey| ≤ 0.30 m with the 5 cm margin); start `[0.0, 0.0, 0.0, 0.5]`
- Cartesian-endpoint prediction, ≤ 8 segment reassignments within the 80 ms budget, `interior_samples = 0`, `strict_corridor = false`; objective, weights, limits and Ipopt options unchanged
- Ego collision disk radius 0.05 m (= the planning margin, so the disk stays inside the physical track whenever its reference point is in the corridor), safety buffer 0.03 m
- Obstacles (radius / centre): centre = 0.10 m at (8, 0); offset = 0.10 m at (8, 0.08); blocked = 0.30 m at (8, 0). Required separation D = 0.18 m (0.38 m blocked); collision (disks touching) at 0.15 m (0.35 m)
- 100 control cycles per run, 3 interleaved runs per configuration, event-aware plant (10 substeps); dense collision evaluation of every candidate at 50 samples per interval; plant distances evaluated at the 10 plant substeps per period
- Timing covers the whole `solve!` cycle (candidate initialisations, all NLP solves, reassignments, validation, collision evaluation)

First construction / first solve with an obstacle in this process (compilation included): 5.67 s / 13.82 s. All other timings are warmed up.

## Scenario `centre`, N = 10 (1.0 s horizon)

| configuration | outcome per run (s_end [m] or stop) | accepted cycles | plant min distance to obstacle [m] | plant clearance above D / above collision [m] | plant track clearance [m] / violations | predicted (dense) clearance above D / above collision [m] | accepted plans entering the buffer (between knots) | passing side |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 47.96; 47.96; 47.96 | 300 / 300 | 0.0121 | -0.1679 / -0.1379 | 0.3500 / 0 | NaN / NaN | 0 (0) | – |
| knots only | 47.95; 47.95; 47.95 | 300 / 300 | 0.1743 | -0.0057 / 0.0243 | 0.1757 / 0 | -0.0057 / 0.0243 | 30 (30) | left, right |
| knots + 1 interior sample | 47.95; 47.95; 47.95 | 300 / 300 | 0.1743 | -0.0057 / 0.0243 | 0.1757 / 0 | -0.0057 / 0.0243 | 30 (30) | left |
| knots + 3 interior samples | 47.95; 47.95; 47.95 | 300 / 300 | 0.1743 | -0.0057 / 0.0243 | 0.1757 / 0 | -0.0057 / 0.0243 | 30 (30) | right |

| configuration | NLP solves mean / max | Ipopt iter. mean / max | build / priming [ms] | mean / median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles (2 candidate solves): count, median / max [ms] | single-attempt cycles median [ms] |
|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 1.00 / 1 | 7.2 / 10 | 4 / 12 | 6.0 / 5.5 / 7.7 / 9.5 / 17.6 | 0 / 0 | 0, – | 5.5 |
| knots only | 1.01 / 2 | 10.1 / 37 | 5 / 33 | 9.9 / 7.9 / 19.2 / 44.9 / 61.4 | 0 / 0 | 3, 54.9 / 61.4 | 7.8 |
| knots + 1 interior sample | 1.01 / 2 | 9.7 / 29 | 6 / 18 | 11.0 / 9.0 / 18.1 / 31.6 / 83.2 | 1 / 0 | 3, 63.2 / 83.2 | 9.0 |
| knots + 3 interior samples | 1.01 / 2 | 9.8 / 40 | 11 / 40 | 13.8 / 12.8 / 19.1 / 44.0 / 85.6 | 1 / 0 | 3, 71.2 / 85.6 | 12.8 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, attempts):

- knots + 1 interior sample, run 2, cycle 11: 83.2 / 77.0 / 81.7 ms, GC 0.0 ms, 1.2 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 2, cycle 11: 85.6 / 88.0 / 84.6 ms, GC 0.0 ms, 1.7 MB, 2 solve(s), 2 attempt(s)

## Scenario `centre`, N = 20 (2.0 s horizon)

| configuration | outcome per run (s_end [m] or stop) | accepted cycles | plant min distance to obstacle [m] | plant clearance above D / above collision [m] | plant track clearance [m] / violations | predicted (dense) clearance above D / above collision [m] | accepted plans entering the buffer (between knots) | passing side |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 47.96; 47.96; 47.96 | 300 / 300 | 0.0101 | -0.1699 / -0.1399 | 0.3500 / 0 | NaN / NaN | 0 (0) | – |
| knots only | stop c2; stop c2; stop c2 | 3 / 6 | 7.9250 | 7.7450 / 7.7750 | 0.3500 / 0 | 0.0000 / 0.0300 | 0 (0) | right |
| knots + 1 interior sample | stop c2; stop c2; stop c2 | 3 / 6 | 7.9309 | 7.7509 / 7.7809 | 0.3500 / 0 | -0.0000 / 0.0300 | 0 (0) | right |
| knots + 3 interior samples | 47.93; 47.93; 47.93 | 300 / 300 | 0.1696 | -0.0104 / 0.0196 | 0.1808 / 0 | -0.0109 / 0.0191 | 60 (60) | left |

| configuration | NLP solves mean / max | Ipopt iter. mean / max | build / priming [ms] | mean / median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles (2 candidate solves): count, median / max [ms] | single-attempt cycles median [ms] |
|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 1.00 / 1 | 9.3 / 14 | 8 / 27 | 12.8 / 12.5 / 17.3 / 19.9 / 30.2 | 0 / 0 | 0, – | 12.5 |
| knots only | 2.00 / 2 | 36.0 / 45 | 10 / 180 | 125.3 / 121.1 / 155.8 / 158.1 / 158.7 | 6 / 5 | 6, 121.1 / 158.7 | NaN |
| knots + 1 interior sample | 2.00 / 2 | 53.5 / 78 | 11 / 113 | 197.0 / 194.8 / 297.4 / 302.8 / 304.1 | 6 / 6 | 6, 194.8 / 304.1 | NaN |
| knots + 3 interior samples | 1.01 / 2 | 14.6 / 115 | 17 / 325 | 39.9 / 26.8 / 141.9 / 300.8 / 345.5 | 22 / 20 | 3, 300.8 / 323.1 | 26.8 |

Stops:

- **knots only** (run 1): control period 2 (t = 0.1 s): collision_detected: collision in the dense evaluation: minimum distance 5.888e-8 m < 0.15 m (first separation violation at t = 1.902 s) [attempts: warm → collision_detected, right → collision_detected]
- **knots + 1 interior sample** (run 1): control period 2 (t = 0.1 s): collision_detected: collision in the dense evaluation: minimum distance 0.1365 m < 0.15 m (first separation violation at t = 1.904 s) [attempts: warm → collision_detected, right → collision_detected]

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, attempts):

- knots only, run 1, cycle 1: 158.7 / 158.0 / 154.3 ms, GC 0.0 ms, 3.1 MB, 2 solve(s), 2 attempt(s)
- knots only, run 1, cycle 2: 98.8 / 85.0 / 96.5 ms, GC 0.0 ms, 2.4 MB, 2 solve(s), 2 attempt(s)
- knots only, run 2, cycle 1: 147.3 / 142.0 / 144.4 ms, GC 0.0 ms, 3.1 MB, 2 solve(s), 2 attempt(s)
- knots only, run 2, cycle 2: 110.7 / 109.0 / 108.6 ms, GC 0.0 ms, 2.4 MB, 2 solve(s), 2 attempt(s)
- knots only, run 3, cycle 1: 131.6 / 125.0 / 129.0 ms, GC 0.0 ms, 3.1 MB, 2 solve(s), 2 attempt(s)
- knots only, run 3, cycle 2: 104.9 / 106.0 / 102.0 ms, GC 0.0 ms, 2.4 MB, 2 solve(s), 2 attempt(s)
- knots + 1 interior sample, run 1, cycle 1: 103.4 / 99.0 / 100.3 ms, GC 0.0 ms, 2.8 MB, 2 solve(s), 2 attempt(s)
- knots + 1 interior sample, run 1, cycle 2: 277.4 / 275.0 / 275.0 ms, GC 0.0 ms, 5.0 MB, 2 solve(s), 2 attempt(s)
- knots + 1 interior sample, run 2, cycle 1: 114.9 / 111.0 / 112.2 ms, GC 0.0 ms, 2.8 MB, 2 solve(s), 2 attempt(s)
- knots + 1 interior sample, run 2, cycle 2: 274.8 / 271.0 / 272.1 ms, GC 0.0 ms, 5.0 MB, 2 solve(s), 2 attempt(s)
- knots + 1 interior sample, run 3, cycle 1: 107.1 / 101.0 / 104.1 ms, GC 0.0 ms, 2.8 MB, 2 solve(s), 2 attempt(s)
- knots + 1 interior sample, run 3, cycle 2: 304.1 / 310.0 / 301.8 ms, GC 0.0 ms, 5.0 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 1, cycle 1: 323.1 / 317.0 / 319.9 ms, GC 0.0 ms, 6.1 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 1, cycle 2: 199.7 / 197.0 / 199.0 ms, GC 0.0 ms, 3.3 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 1, cycle 3: 260.6 / 265.0 / 259.9 ms, GC 0.0 ms, 4.7 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 1, cycle 4: 154.7 / 155.0 / 153.8 ms, GC 0.0 ms, 2.5 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 1, cycle 5: 128.9 / 129.0 / 128.2 ms, GC 0.0 ms, 2.9 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 1, cycle 6: 110.5 / 107.0 / 109.6 ms, GC 0.0 ms, 2.1 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 1, cycle 7: 138.6 / 136.0 / 137.8 ms, GC 0.0 ms, 2.5 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 1: 293.7 / 289.0 / 290.4 ms, GC 0.0 ms, 6.1 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 2, cycle 2: 216.4 / 221.0 / 215.6 ms, GC 0.0 ms, 3.3 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 3: 345.5 / 340.0 / 344.4 ms, GC 0.0 ms, 4.7 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 4: 160.5 / 160.0 / 159.3 ms, GC 0.0 ms, 2.5 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 5: 141.9 / 141.0 / 140.6 ms, GC 0.0 ms, 2.9 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 6: 95.8 / 90.0 / 95.0 ms, GC 0.0 ms, 2.1 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 7: 188.9 / 189.0 / 187.8 ms, GC 52.2 ms, 2.5 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 79: 211.9 / 24.0 / 27.5 ms, GC 182.6 ms, 0.9 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 1: 300.8 / 302.0 / 297.8 ms, GC 0.0 ms, 6.1 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 3, cycle 2: 212.6 / 213.0 / 211.7 ms, GC 0.0 ms, 3.3 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 3: 301.5 / 299.0 / 300.3 ms, GC 0.0 ms, 4.7 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 4: 143.1 / 144.0 / 142.2 ms, GC 0.0 ms, 2.5 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 5: 161.6 / 160.0 / 160.9 ms, GC 0.0 ms, 2.9 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 6: 81.6 / 72.0 / 80.9 ms, GC 0.0 ms, 2.1 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 7: 130.5 / 139.0 / 129.7 ms, GC 0.0 ms, 2.5 MB, 1 solve(s), 1 attempt(s)

## Scenario `offset`, N = 10 (1.0 s horizon)

| configuration | outcome per run (s_end [m] or stop) | accepted cycles | plant min distance to obstacle [m] | plant clearance above D / above collision [m] | plant track clearance [m] / violations | predicted (dense) clearance above D / above collision [m] | accepted plans entering the buffer (between knots) | passing side |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 47.96 | 100 / 100 | 0.0809 | -0.0991 / -0.0691 | 0.3500 / 0 | NaN / NaN | 0 (0) | – |
| knots only | 47.96; 47.96; 47.96 | 300 / 300 | 0.1757 | -0.0043 / 0.0257 | 0.2545 / 0 | -0.0046 / 0.0254 | 30 (30) | right |
| knots + 1 interior sample | 47.96; 47.96; 47.96 | 300 / 300 | 0.1757 | -0.0043 / 0.0257 | 0.2545 / 0 | -0.0046 / 0.0254 | 30 (30) | right |
| knots + 3 interior samples | 47.96; 47.96; 47.96 | 300 / 300 | 0.1757 | -0.0043 / 0.0257 | 0.2545 / 0 | -0.0046 / 0.0254 | 30 (30) | right |

| configuration | NLP solves mean / max | Ipopt iter. mean / max | build / priming [ms] | mean / median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles (2 candidate solves): count, median / max [ms] | single-attempt cycles median [ms] |
|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 1.00 / 1 | 7.2 / 10 | 4 / 12 | 7.0 / 6.7 / 9.3 / 10.5 / 12.1 | 0 / 0 | 0, – | 6.7 |
| knots only | 1.01 / 2 | 9.3 / 19 | 5 / 19 | 9.8 / 8.8 / 15.1 / 25.9 / 33.9 | 0 / 0 | 3, 29.2 / 33.9 | 8.8 |
| knots + 1 interior sample | 1.01 / 2 | 9.1 / 16 | 6 / 22 | 10.4 / 9.3 / 14.1 / 28.1 / 36.3 | 0 / 0 | 3, 29.2 / 31.5 | 9.3 |
| knots + 3 interior samples | 1.01 / 2 | 9.1 / 23 | 11 / 28 | 12.2 / 11.0 / 18.5 / 34.5 / 43.5 | 0 / 0 | 3, 37.5 / 43.5 | 10.9 |

## Scenario `offset`, N = 20 (2.0 s horizon)

| configuration | outcome per run (s_end [m] or stop) | accepted cycles | plant min distance to obstacle [m] | plant clearance above D / above collision [m] | plant track clearance [m] / violations | predicted (dense) clearance above D / above collision [m] | accepted plans entering the buffer (between knots) | passing side |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 47.96 | 100 / 100 | 0.0806 | -0.0994 / -0.0694 | 0.3500 / 0 | NaN / NaN | 0 (0) | – |
| knots only | 47.95; 47.95; 47.95 | 300 / 300 | 0.1726 | -0.0074 / 0.0226 | 0.2573 / 0 | -0.0074 / 0.0226 | 60 (60) | right |
| knots + 1 interior sample | 47.95; 47.95; 47.95 | 300 / 300 | 0.1726 | -0.0074 / 0.0226 | 0.2573 / 0 | -0.0074 / 0.0226 | 60 (60) | right |
| knots + 3 interior samples | 47.95; 47.95; 47.95 | 300 / 300 | 0.1726 | -0.0074 / 0.0226 | 0.2573 / 0 | -0.0074 / 0.0226 | 60 (60) | right |

| configuration | NLP solves mean / max | Ipopt iter. mean / max | build / priming [ms] | mean / median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles (2 candidate solves): count, median / max [ms] | single-attempt cycles median [ms] |
|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| no obstacle (baseline) | 1.00 / 1 | 9.3 / 14 | 8 / 24 | 12.8 / 12.1 / 19.0 / 21.1 / 23.1 | 0 / 0 | 0, – | 12.1 |
| knots only | 1.01 / 2 | 12.6 / 36 | 8 / 59 | 19.1 / 16.5 / 30.5 / 45.9 / 213.3 | 1 / 1 | 3, 44.9 / 48.2 | 16.4 |
| knots + 1 interior sample | 1.01 / 2 | 12.3 / 47 | 12 / 81 | 23.3 / 19.9 / 47.5 / 71.0 / 94.0 | 3 / 0 | 3, 58.2 / 69.6 | 19.9 |
| knots + 3 interior samples | 1.01 / 2 | 11.3 / 40 | 21 / 154 | 28.0 / 24.4 / 44.5 / 102.3 / 116.6 | 8 / 4 | 3, 104.7 / 116.6 | 24.3 |

Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, attempts):

- knots only, run 2, cycle 20: 213.3 / 217.0 / 212.5 ms, GC 197.3 ms, 0.6 MB, 1 solve(s), 1 attempt(s)
- knots + 1 interior sample, run 1, cycle 3: 82.9 / 81.0 / 82.0 ms, GC 0.0 ms, 1.5 MB, 1 solve(s), 1 attempt(s)
- knots + 1 interior sample, run 2, cycle 3: 94.0 / 92.0 / 93.0 ms, GC 0.0 ms, 1.5 MB, 1 solve(s), 1 attempt(s)
- knots + 1 interior sample, run 3, cycle 3: 82.0 / 83.0 / 80.9 ms, GC 0.0 ms, 1.5 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 1, cycle 1: 116.6 / 110.0 / 112.5 ms, GC 0.0 ms, 3.1 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 1, cycle 3: 91.8 / 94.0 / 91.0 ms, GC 0.0 ms, 2.0 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 1: 94.8 / 93.0 / 92.0 ms, GC 0.0 ms, 3.1 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 2, cycle 2: 81.3 / 77.0 / 80.4 ms, GC 0.0 ms, 1.7 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 2, cycle 3: 113.6 / 111.0 / 112.8 ms, GC 0.0 ms, 2.0 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 1: 104.7 / 98.0 / 101.7 ms, GC 0.0 ms, 3.1 MB, 2 solve(s), 2 attempt(s)
- knots + 3 interior samples, run 3, cycle 2: 102.2 / 106.0 / 101.4 ms, GC 0.0 ms, 1.7 MB, 1 solve(s), 1 attempt(s)
- knots + 3 interior samples, run 3, cycle 3: 82.1 / 80.0 / 80.7 ms, GC 0.0 ms, 2.0 MB, 1 solve(s), 1 attempt(s)

## Scenario `blocked` (no lateral passage), one control cycle

| start state | N | configuration | status | failure cause | attempts | passage left / right | NLP solves | time [ms] |
|:--|--:|:--|:--|:--|:--|:-:|--:|--:|
| [0.0, 0.0, 0.0, 1.0] | 10 | knots + 3 interior samples | success | none | default → success | false / false | 1 | 12.7 |
| [0.0, 0.0, 0.0, 1.0] | 20 | knots + 3 interior samples | solver_failure | infeasible_geometry | left → solver_failure, right → solver_failure | false / false | 2 | 241.0 |
| [6.6, 0.0, 0.0, 5.0] | 10 | knots + 3 interior samples | solver_failure | infeasible_geometry | left → solver_failure, right → solver_failure | false / false | 2 | 1082.5 |
| [6.6, 0.0, 0.0, 5.0] | 20 | knots + 3 interior samples | solver_failure | infeasible_geometry | left → solver_failure, right → solver_failure | false / false | 2 | 847.5 |

