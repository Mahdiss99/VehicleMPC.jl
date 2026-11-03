# Multi-curvature stationary-obstacle avoidance benchmark (Milestone 3B)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; unchanged `quarter_track()` (L = 34.85 m, width 0.7 m); start `[0.0, 0.0, 0.0, 0.5]`
- Primary configuration (all options explicit): Cartesian-endpoint, N = 10, dt = 0.1 s, track_margin = 0.05, schedule_corrections = 8 (80 ms budget), interior_samples = 0, strict_corridor = false, ego_radius = 0.05, obstacle_buffer = 0.03, obstacle_samples = 3, obstacle_acceptance = :collision; objective, weights, limits and Ipopt options unchanged
- Obstacles are placed with `obstacle_on_track(quarter_track(), s, ey; radius)` and stay fixed in Cartesian space; radius 0.1 m (D = 0.18 m, contact at 0.15 m) unless stated
- 250 control cycles per run, 3 interleaved runs; event-aware plant (10 substeps); plant distances at every substep; dense collision evaluation of every candidate at 50 samples per interval
- "Encounter cycles" are cycles with a passing-side initialisation (two candidate solves at a new encounter, or a warm-start retry); timing covers the whole `solve!` cycle

First construction / first solve with an obstacle in this process (compilation included): 2.78 s / 7.26 s.

## Primary: N = 10, obstacle_acceptance = :collision

| scenario | laps per run | accepted cycles | plant min distance [m] | plant clearance above contact / above D [m] | plant track clearance / corridor clearance [m] | max |ey| [m] | predicted (dense) clearance above contact / above D [m] | accepted plans in the buffer | encounters: cycle:side (run 1) | failure causes |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:--|:--|
| A no obstacle | 2.58; 2.58; 2.58 | 750 / 750 | NaN | NaN / NaN | 0.0463 / -0.0037 | 0.304 | NaN / NaN | 0 | – | – |
| B R 1 m left bend, s = 3.5 | 2.46; 2.46; 2.46 | 750 / 750 | 0.1795 | 0.0295 / -0.0005 | 0.0472 / -0.0028 | 0.303 | 0.0193 / -0.0107 | 132 | c1:left c97:left c193:left | – |
| C R 1 m right bend (S-bend), s = 6.6 | 2.53; 2.53; 2.53 | 750 / 750 | 0.1795 | 0.0295 / -0.0005 | 0.0468 / -0.0032 | 0.303 | 0.0275 / -0.0025 | 54 | c6:right c100:right c194:right c205:left | – |
| D R 2 m curve, s = 9.7 | 2.58; 2.58; stop c178 (1.84) | 677 / 678 | 0.2942 | 0.1442 / 0.1142 | 0.0463 / -0.0037 | 0.304 | 0.0820 / 0.0520 | 0 | c17:left c111:left c204:left | segment_assignment |
| E1 before straight→R 1 m boundary, s = 1.95 | stop c95 (0.95); stop c95 (0.95); stop c46 (0.37) | 233 / 236 | 0.1759 | 0.0259 / -0.0041 | 0.0465 / -0.0035 | 0.303 | 0.0189 / -0.0111 | 19 | c1:left c89:right c95:right | segment_assignment |
| E2 after straight→R 1 m boundary, s = 2.10 | stop c95 (0.95); stop c95 (0.95); stop c191 (1.95) | 378 / 381 | 0.1786 | 0.0286 / -0.0014 | 0.0470 / -0.0030 | 0.303 | 0.0189 / -0.0111 | 19 | c1:left c89:right c95:right | segment_assignment |
| E3 before R −1→R 2 boundary, s = 8.20 | stop c25 (0.17); stop c25 (0.17); stop c118 (1.17) | 165 / 168 | 0.1821 | 0.0321 / 0.0021 | 0.0438 / -0.0062 | 0.306 | 0.0271 / -0.0029 | 10 | c11:right c25:left | segment_assignment |
| E4 after R −1→R 2 boundary, s = 8.40 | stop c215 (2.17); stop c216 (2.18); stop c25 (0.17) | 453 / 456 | 0.1771 | 0.0271 / -0.0029 | 0.0470 / -0.0030 | 0.303 | 0.0266 / -0.0034 | 30 | c12:right c26:left c107:right c121:left c202:right c215:left | segment_assignment |
| F lap boundary, s = L − 0.3 | 2.58; 2.58; 2.58 | 750 / 750 | 0.1855 | 0.0355 / 0.0055 | 0.0463 / -0.0037 | 0.304 | 0.0314 / 0.0014 | 0 | c84:right c177:right c184:right | – |
| G1 offset +0.08 on R 1 m left bend | 2.48; 2.48; 2.48 | 750 / 750 | 0.1794 | 0.0294 / -0.0006 | 0.0452 / -0.0048 | 0.305 | 0.0228 / -0.0072 | 132 | c1:left c96:left c99:right c192:left c195:right | – |
| G2 offset −0.08 on R 1 m right bend | 2.46; 2.46; 2.46 | 750 / 750 | 0.1800 | 0.0300 / -0.0000 | 0.0464 / -0.0036 | 0.304 | 0.0197 / -0.0103 | 126 | c6:right c18:left c103:right c200:right | – |
| G3 blocked: radius 0.30 at s = 3.5 | 0.08; 0.08; 0.08 | 750 / 750 | 0.8135 | 0.4635 / 0.4335 | 0.3173 / 0.2673 | 0.033 | 0.0273 / -0.0027 | 9 | c1:left | – |

| scenario | NLP solves mean / max | reassignment solves per cycle | initialisation attempts | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: n, median / max [ms] | ordinary cycles: median / max [ms] | plant: input / lateral-acc. violations, max |ay| | max dyn. residual / min knot clearance above D |
|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | 1.13 / 2 | 0.13 | 0 | 15.3 / 23 | 10.9 / 23.4 / 29.3 / 51.1 | 0 / 0 | 0, – | 10.9 / 51.1 | 0 / 0, 4.00 | NaN / NaN |
| B R 1 m left bend, s = 3.5 | 1.15 / 4 | 0.14 | 759 | 17.7 / 42 | 17.8 / 41.8 / 60.1 / 87.2 | 3 / 0 | 9, 67.7 / 87.2 | 17.7 / 73.0 | 0 / 0, 4.00 | NaN / -0.0000 |
| C R 1 m right bend (S-bend), s = 6.6 | 1.14 / 3 | 0.12 | 762 | 17.2 / 50 | 11.9 / 36.8 / 51.3 / 184.5 | 3 / 2 | 12, 25.5 / 184.5 | 11.8 / 79.5 | 0 / 0, 4.00 | NaN / -0.0000 |
| D R 2 m curve, s = 9.7 | 1.17 / 6 | 0.15 | 686 | 16.3 / 34 | 16.1 / 37.7 / 52.5 / 98.6 | 2 / 0 | 8, 53.0 / 98.6 | 15.8 / 61.0 | 0 / 0, 4.00 | NaN / 0.0558 |
| E1 before straight→R 1 m boundary, s = 1.95 | 1.19 / 4 | 0.17 | 243 | 17.1 / 53 | 22.6 / 50.7 / 100.7 / 126.9 | 5 / 3 | 7, 96.8 / 126.9 | 22.4 / 68.8 | 0 / 0, 4.00 | NaN / -0.0000 |
| E2 after straight→R 1 m boundary, s = 2.10 | 1.17 / 6 | 0.14 | 391 | 17.4 / 47 | 20.7 / 47.8 / 112.1 / 173.5 | 8 / 7 | 10, 109.9 / 173.5 | 20.6 / 65.8 | 0 / 0, 4.00 | NaN / -0.0000 |
| E3 before R −1→R 2 boundary, s = 8.20 | 1.21 / 6 | 0.17 | 175 | 17.0 / 46 | 16.6 / 42.8 / 137.0 / 155.1 | 3 / 3 | 7, 38.3 / 155.1 | 15.2 / 61.2 | 0 / 0, 4.00 | NaN / -0.0000 |
| E4 after R −1→R 2 boundary, s = 8.40 | 1.18 / 3 | 0.15 | 470 | 16.8 / 38 | 22.8 / 51.9 / 134.0 / 216.8 | 12 / 9 | 14, 98.1 / 145.6 | 22.6 / 216.8 | 0 / 0, 4.00 | NaN / -0.0000 |
| F lap boundary, s = L − 0.3 | 1.15 / 4 | 0.14 | 757 | 16.6 / 38 | 21.2 / 48.3 / 83.7 / 152.0 | 8 / 5 | 7, 101.3 / 118.0 | 21.1 / 152.0 | 0 / 0, 4.00 | NaN / 0.0034 |
| G1 offset +0.08 on R 1 m left bend | 1.17 / 4 | 0.15 | 762 | 17.5 / 35 | 20.9 / 47.7 / 73.7 / 136.9 | 7 / 4 | 12, 89.6 / 136.9 | 20.7 / 76.0 | 0 / 0, 4.00 | NaN / -0.0000 |
| G2 offset −0.08 on R 1 m right bend | 1.11 / 3 | 0.10 | 761 | 18.0 / 46 | 19.7 / 46.5 / 59.5 / 123.9 | 2 / 2 | 11, 36.7 / 123.9 | 19.6 / 69.6 | 0 / 0, 4.00 | NaN / -0.0000 |
| G3 blocked: radius 0.30 at s = 3.5 | 1.02 / 4 | 0.01 | 753 | 20.9 / 26 | 24.9 / 34.9 / 41.5 / 86.7 | 1 / 0 | 3, 64.1 / 86.7 | 24.9 / 54.2 | 0 / 0, 1.60 | NaN / -0.0000 |

Stops:

- **E1 before straight→R 1 m boundary, s = 1.95** (run 1): control period 95 (t = 9.4 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.3991 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, right → segment_assignment_invalid]
- **E2 after straight→R 1 m boundary, s = 2.10** (run 1): control period 95 (t = 9.4 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.3021 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, right → segment_assignment_invalid]
- **E3 before R −1→R 2 boundary, s = 8.20** (run 1): control period 25 (t = 2.4 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.09525 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- **E4 after R −1→R 2 boundary, s = 8.40** (run 1): control period 215 (t = 21.4 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.01393 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]

Cycles above 80 ms (scenario, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, attempts):

- B R 1 m left bend, s = 3.5, run 2, cycle 193: 81.9 / 75.0 / 72.7 ms, GC 6.9 ms, 2.1 MB, 4 solve(s), 2 attempt(s)
- B R 1 m left bend, s = 3.5, run 3, cycle 1: 81.3 / 79.0 / 78.2 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- B R 1 m left bend, s = 3.5, run 3, cycle 97: 87.2 / 82.0 / 83.6 ms, GC 0.0 ms, 2.1 MB, 4 solve(s), 2 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 205: 106.7 / 93.0 / 99.8 ms, GC 0.0 ms, 4.7 MB, 3 solve(s), 2 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 2, cycle 205: 184.5 / 173.0 / 182.8 ms, GC 0.0 ms, 4.7 MB, 3 solve(s), 2 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 3, cycle 205: 98.5 / 95.0 / 97.5 ms, GC 4.8 ms, 4.7 MB, 3 solve(s), 2 attempt(s)
- D R 2 m curve, s = 9.7, run 2, cycle 17: 98.6 / 84.0 / 95.3 ms, GC 0.0 ms, 2.8 MB, 6 solve(s), 2 attempt(s)
- D R 2 m curve, s = 9.7, run 2, cycle 111: 84.4 / 72.0 / 72.2 ms, GC 9.6 ms, 2.1 MB, 4 solve(s), 2 attempt(s)
- E1 before straight→R 1 m boundary, s = 1.95, run 1, cycle 1: 96.8 / 95.0 / 93.9 ms, GC 0.0 ms, 2.9 MB, 4 solve(s), 2 attempt(s)
- E1 before straight→R 1 m boundary, s = 1.95, run 1, cycle 95: 126.9 / 127.0 / 125.4 ms, GC 0.0 ms, 2.5 MB, 2 solve(s), 2 attempt(s)
- E1 before straight→R 1 m boundary, s = 1.95, run 2, cycle 1: 126.7 / 123.0 / 123.4 ms, GC 0.0 ms, 2.9 MB, 4 solve(s), 2 attempt(s)
- E1 before straight→R 1 m boundary, s = 1.95, run 2, cycle 95: 102.8 / 100.0 / 101.2 ms, GC 0.0 ms, 2.5 MB, 2 solve(s), 2 attempt(s)
- E1 before straight→R 1 m boundary, s = 1.95, run 3, cycle 1: 90.7 / 87.0 / 87.8 ms, GC 0.0 ms, 2.1 MB, 3 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 1, cycle 1: 112.0 / 109.0 / 109.7 ms, GC 0.0 ms, 2.9 MB, 3 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 1, cycle 89: 86.0 / 86.0 / 84.2 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 1, cycle 95: 107.7 / 110.0 / 106.2 ms, GC 0.0 ms, 2.4 MB, 2 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 2, cycle 1: 170.2 / 170.0 / 167.4 ms, GC 0.0 ms, 3.9 MB, 4 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 2, cycle 95: 104.6 / 105.0 / 102.6 ms, GC 0.0 ms, 2.4 MB, 2 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 3, cycle 1: 173.5 / 177.0 / 170.8 ms, GC 0.0 ms, 3.9 MB, 4 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 3, cycle 185: 112.1 / 112.0 / 109.0 ms, GC 0.0 ms, 2.9 MB, 6 solve(s), 2 attempt(s)
- E2 after straight→R 1 m boundary, s = 2.10, run 3, cycle 191: 134.7 / 126.0 / 131.4 ms, GC 0.0 ms, 2.4 MB, 2 solve(s), 2 attempt(s)
- E3 before R −1→R 2 boundary, s = 8.20, run 1, cycle 25: 155.1 / 155.0 / 153.3 ms, GC 0.0 ms, 3.4 MB, 3 solve(s), 2 attempt(s)
- E3 before R −1→R 2 boundary, s = 8.20, run 2, cycle 25: 131.7 / 123.0 / 129.9 ms, GC 0.0 ms, 3.4 MB, 3 solve(s), 2 attempt(s)
- E3 before R −1→R 2 boundary, s = 8.20, run 3, cycle 118: 147.7 / 153.0 / 146.0 ms, GC 0.0 ms, 6.3 MB, 6 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 1, cycle 26: 139.0 / 140.0 / 136.4 ms, GC 0.0 ms, 3.1 MB, 3 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 1, cycle 37: 96.5 / 32.0 / 33.0 ms, GC 62.1 ms, 0.6 MB, 1 solve(s), 1 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 1, cycle 121: 136.4 / 138.0 / 133.6 ms, GC 0.0 ms, 3.1 MB, 3 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 1, cycle 125: 216.8 / 220.0 / 215.7 ms, GC 190.5 ms, 0.6 MB, 1 solve(s), 1 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 1, cycle 215: 110.7 / 106.0 / 107.9 ms, GC 0.0 ms, 2.3 MB, 3 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 2, cycle 5: 213.4 / 208.0 / 212.9 ms, GC 193.0 ms, 0.6 MB, 1 solve(s), 1 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 2, cycle 26: 145.6 / 146.0 / 142.6 ms, GC 0.0 ms, 3.1 MB, 3 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 2, cycle 121: 132.1 / 126.0 / 128.8 ms, GC 0.0 ms, 3.1 MB, 3 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 2, cycle 215: 81.4 / 81.0 / 80.2 ms, GC 0.0 ms, 1.6 MB, 2 solve(s), 1 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 2, cycle 216: 92.4 / 91.0 / 90.7 ms, GC 0.0 ms, 2.1 MB, 2 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 3, cycle 12: 103.7 / 40.0 / 46.5 ms, GC 55.4 ms, 1.1 MB, 2 solve(s), 2 attempt(s)
- E4 after R −1→R 2 boundary, s = 8.40, run 3, cycle 25: 109.1 / 110.0 / 106.7 ms, GC 0.0 ms, 2.3 MB, 3 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 1, cycle 84: 97.6 / 95.0 / 95.1 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 1, cycle 177: 111.8 / 109.0 / 109.3 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 1, cycle 184: 118.0 / 120.0 / 114.7 ms, GC 0.0 ms, 2.6 MB, 3 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 2, cycle 84: 113.7 / 111.0 / 110.0 ms, GC 9.7 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 2, cycle 177: 97.1 / 95.0 / 94.3 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 3, cycle 46: 152.0 / 146.0 / 151.5 ms, GC 123.9 ms, 1.2 MB, 2 solve(s), 1 attempt(s)
- F lap boundary, s = L − 0.3, run 3, cycle 177: 101.3 / 100.0 / 98.5 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 3, cycle 184: 96.2 / 100.0 / 95.0 ms, GC 0.0 ms, 1.7 MB, 2 solve(s), 1 attempt(s)
- G1 offset +0.08 on R 1 m left bend, run 1, cycle 96: 98.6 / 96.0 / 96.1 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- G1 offset +0.08 on R 1 m left bend, run 1, cycle 99: 96.4 / 95.0 / 94.3 ms, GC 0.0 ms, 2.3 MB, 3 solve(s), 2 attempt(s)
- G1 offset +0.08 on R 1 m left bend, run 1, cycle 195: 111.1 / 106.0 / 108.7 ms, GC 0.0 ms, 2.3 MB, 3 solve(s), 2 attempt(s)
- G1 offset +0.08 on R 1 m left bend, run 2, cycle 96: 136.9 / 136.0 / 133.6 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)
- G1 offset +0.08 on R 1 m left bend, run 2, cycle 192: 82.8 / 85.0 / 80.2 ms, GC 0.0 ms, 2.1 MB, 4 solve(s), 2 attempt(s)
- G1 offset +0.08 on R 1 m left bend, run 3, cycle 192: 101.8 / 93.0 / 98.3 ms, GC 0.0 ms, 2.1 MB, 4 solve(s), 2 attempt(s)
- G1 offset +0.08 on R 1 m left bend, run 3, cycle 195: 101.2 / 89.0 / 98.8 ms, GC 0.0 ms, 2.3 MB, 3 solve(s), 2 attempt(s)
- G2 offset −0.08 on R 1 m right bend, run 1, cycle 18: 110.5 / 110.0 / 108.8 ms, GC 0.0 ms, 2.6 MB, 3 solve(s), 2 attempt(s)
- G2 offset −0.08 on R 1 m right bend, run 2, cycle 18: 123.9 / 124.0 / 121.5 ms, GC 0.0 ms, 2.6 MB, 3 solve(s), 2 attempt(s)
- G3 blocked: radius 0.30 at s = 3.5, run 1, cycle 1: 86.7 / 85.0 / 82.8 ms, GC 0.0 ms, 2.2 MB, 4 solve(s), 2 attempt(s)

## Failed primary scenarios rerun without the correction time budget (one run, budget 1000 s)

| scenario | laps per run | accepted cycles | plant min distance [m] | plant clearance above contact / above D [m] | plant track clearance / corridor clearance [m] | max |ey| [m] | predicted (dense) clearance above contact / above D [m] | accepted plans in the buffer | encounters: cycle:side (run 1) | failure causes |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:--|:--|
| D R 2 m curve, s = 9.7 | 2.58 | 250 / 250 | 0.2942 | 0.1442 / 0.1142 | 0.0463 / -0.0037 | 0.304 | 0.0820 / 0.0520 | 0 | c17:left c111:left c204:left | – |
| E1 before straight→R 1 m boundary, s = 1.95 | 2.54 | 250 / 250 | 0.1759 | 0.0259 / -0.0041 | 0.0463 / -0.0037 | 0.304 | 0.0189 / -0.0111 | 29 | c1:left c89:right c184:right | – |
| E2 after straight→R 1 m boundary, s = 2.10 | 2.52 | 250 / 250 | 0.1786 | 0.0286 / -0.0014 | 0.0469 / -0.0031 | 0.303 | 0.0189 / -0.0111 | 26 | c1:left c89:right c185:right | – |
| E3 before R −1→R 2 boundary, s = 8.20 | stop c118 (1.17) | 117 / 118 | 0.1821 | 0.0321 / 0.0021 | 0.0438 / -0.0062 | 0.306 | 0.0271 / -0.0029 | 8 | c11:right c104:right c118:left | segment_assignment |
| E4 after R −1→R 2 boundary, s = 8.40 | 2.52 | 250 / 250 | 0.1771 | 0.0271 / -0.0029 | 0.0470 / -0.0030 | 0.303 | 0.0266 / -0.0034 | 21 | c12:right c107:right c202:right | – |

| scenario | NLP solves mean / max | reassignment solves per cycle | initialisation attempts | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: n, median / max [ms] | ordinary cycles: median / max [ms] | plant: input / lateral-acc. violations, max |ay| | max dyn. residual / min knot clearance above D |
|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| D R 2 m curve, s = 9.7 | 1.17 / 6 | 0.16 | 253 | 16.4 / 34 | 10.4 / 22.3 / 30.2 / 53.9 | 0 / 0 | 3, 51.1 / 53.9 | 10.3 / 25.2 | 0 / 0, 4.00 | NaN / 0.0558 |
| E1 before straight→R 1 m boundary, s = 1.95 | 1.19 / 4 | 0.18 | 253 | 17.2 / 51 | 11.1 / 24.5 / 45.0 / 57.6 | 0 / 0 | 3, 39.0 / 56.9 | 11.1 / 57.6 | 0 / 0, 4.00 | NaN / -0.0000 |
| E2 after straight→R 1 m boundary, s = 2.10 | 1.15 / 6 | 0.14 | 253 | 17.7 / 52 | 10.7 / 24.3 / 57.4 / 80.5 | 1 / 0 | 3, 57.4 / 80.5 | 10.7 / 64.7 | 0 / 0, 4.00 | NaN / -0.0000 |
| E3 before R −1→R 2 boundary, s = 8.20 | 1.31 / 18 | 0.28 | 121 | 16.8 / 46 | 10.3 / 23.0 / 49.3 / 444.6 | 1 / 1 | 3, 20.6 / 444.6 | 10.3 / 51.6 | 0 / 0, 4.00 | NaN / -0.0000 |
| E4 after R −1→R 2 boundary, s = 8.40 | 1.16 / 2 | 0.15 | 253 | 17.0 / 46 | 10.4 / 22.0 / 41.6 / 168.4 | 1 / 1 | 3, 19.0 / 20.4 | 10.4 / 168.4 | 0 / 0, 4.00 | NaN / -0.0000 |

Stops:

- **E3 before R −1→R 2 boundary, s = 8.20** (run 1): control period 118 (t = 11.7 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m) [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]

## Strict-buffer experiment: N = 10, obstacle_acceptance = :separation

| scenario | laps per run | accepted cycles | plant min distance [m] | plant clearance above contact / above D [m] | plant track clearance / corridor clearance [m] | max |ey| [m] | predicted (dense) clearance above contact / above D [m] | accepted plans in the buffer | encounters: cycle:side (run 1) | failure causes |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:--|:--|
| B R 1 m left bend, s = 3.5 | stop c2 (0.00); stop c2 (0.00); stop c2 (0.00) | 3 / 6 | 3.0667 | 2.9167 / 2.8867 | 0.3499 / 0.2999 | 0.000 | 0.1720 / 0.1420 | 0 | c1:left c2:left | separation |
| C R 1 m right bend (S-bend), s = 6.6 | stop c15 (0.10); stop c15 (0.10); stop c15 (0.10) | 42 / 45 | 2.6365 | 2.4865 / 2.4565 | 0.0470 / -0.0030 | 0.303 | 0.0895 / 0.0595 | 0 | c6:right c15:right | separation |
| D R 2 m curve, s = 9.7 | 2.58; 2.58; 2.58 | 750 / 750 | 0.2942 | 0.1442 / 0.1142 | 0.0463 / -0.0037 | 0.304 | 0.0820 / 0.0520 | 0 | c17:left c111:left c204:left | – |

Stops:

- **B R 1 m left bend, s = 3.5** (run 1): control period 2 (t = 0.1 s): collision_detected [cause separation]: required separation violated in the dense evaluation at t = 0.952 s (distance 0.1693 m < D = 0.18 m) [attempts: warm → collision_detected, left → collision_detected]
- **C R 1 m right bend (S-bend), s = 6.6** (run 1): control period 15 (t = 1.4 s): segment_assignment_invalid [cause separation]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.06161 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → collision_detected, right → segment_assignment_invalid]

## Secondary: N = 20 (one run each)

| scenario | laps per run | accepted cycles | plant min distance [m] | plant clearance above contact / above D [m] | plant track clearance / corridor clearance [m] | max |ey| [m] | predicted (dense) clearance above contact / above D [m] | accepted plans in the buffer | encounters: cycle:side (run 1) | failure causes |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:--|:--|
| A no obstacle | 2.39 | 250 / 250 | NaN | NaN / NaN | 0.1288 / 0.0788 | 0.221 | NaN / NaN | 0 | – | – |
| B R 1 m left bend, s = 3.5 | stop c101 (0.96) | 100 / 101 | 0.1783 | 0.0283 / -0.0017 | 0.1097 / 0.0597 | 0.240 | 0.0255 / -0.0045 | 25 | c1:left c85:right c101:right | segment_assignment |
| C R 1 m right bend (S-bend), s = 6.6 | stop c130 (1.18) | 129 / 130 | 0.1795 | 0.0295 / -0.0005 | 0.1576 / 0.1076 | 0.192 | 0.0214 / -0.0086 | 46 | c1:right c11:left c95:right c130:right | segment_assignment |
| D R 2 m curve, s = 9.7 | stop c30 (0.19) | 29 / 30 | 2.7911 | 2.6411 / 2.6111 | 0.2424 / 0.1924 | 0.108 | 0.0191 / -0.0109 | 9 | c1:right c21:left c30:right | segment_assignment |
| F lap boundary, s = L − 0.3 | 2.39 | 250 / 250 | 0.1706 | 0.0206 / -0.0094 | 0.1247 / 0.0747 | 0.225 | 0.0191 / -0.0109 | 43 | c78:right c178:right | – |

| scenario | NLP solves mean / max | reassignment solves per cycle | initialisation attempts | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: n, median / max [ms] | ordinary cycles: median / max [ms] | plant: input / lateral-acc. violations, max |ay| | max dyn. residual / min knot clearance above D |
|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | 1.22 / 3 | 0.22 | 0 | 16.4 / 22 | 9.7 / 20.4 / 21.9 / 28.5 | 0 / 0 | 0, – | 9.7 / 28.5 | 0 / 0, 4.00 | NaN / NaN |
| B R 1 m left bend, s = 3.5 | 1.26 / 6 | 0.23 | 104 | 21.2 / 89 | 21.0 / 46.5 / 132.1 / 188.9 | 3 / 3 | 3, 132.1 / 188.9 | 20.9 / 69.1 | 0 / 0, 4.00 | NaN / -0.0000 |
| C R 1 m right bend (S-bend), s = 6.6 | 1.35 / 8 | 0.32 | 134 | 22.9 / 126 | 21.1 / 74.6 / 138.6 / 151.8 | 7 / 6 | 4, 122.8 / 151.8 | 20.9 / 142.3 | 0 / 0, 4.00 | NaN / -0.0000 |
| D R 2 m curve, s = 9.7 | 1.47 / 4 | 0.37 | 33 | 27.1 / 81 | 30.8 / 109.5 / 131.2 / 133.5 | 3 / 2 | 3, 125.6 / 133.5 | 23.7 / 90.0 | 0 / 0, 4.00 | NaN / -0.0000 |
| F lap boundary, s = L − 0.3 | 1.23 / 4 | 0.22 | 252 | 20.0 / 89 | 21.3 / 48.2 / 89.5 / 173.0 | 3 / 3 | 2, 57.8 / 77.8 | 21.2 / 173.0 | 0 / 0, 4.00 | NaN / -0.0000 |

Stops:

- **B R 1 m left bend, s = 3.5** (run 1): control period 101 (t = 10.0 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1283 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, right → segment_assignment_invalid]
- **C R 1 m right bend (S-bend), s = 6.6** (run 1): control period 130 (t = 12.9 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.03094 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, right → segment_assignment_invalid]
- **D R 2 m curve, s = 9.7** (run 1): control period 30 (t = 2.9 s): segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.01963 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, right → segment_assignment_invalid]

Cycles above 80 ms (scenario, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, attempts):

- B R 1 m left bend, s = 3.5, run 1, cycle 1: 132.1 / 130.0 / 130.4 ms, GC 0.0 ms, 6.4 MB, 2 solve(s), 2 attempt(s)
- B R 1 m left bend, s = 3.5, run 1, cycle 85: 126.8 / 119.0 / 124.6 ms, GC 5.5 ms, 6.3 MB, 6 solve(s), 2 attempt(s)
- B R 1 m left bend, s = 3.5, run 1, cycle 101: 188.9 / 189.0 / 187.5 ms, GC 0.0 ms, 7.9 MB, 2 solve(s), 2 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 11: 115.8 / 119.0 / 114.3 ms, GC 0.0 ms, 5.9 MB, 3 solve(s), 2 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 15: 83.9 / 85.0 / 83.4 ms, GC 0.0 ms, 3.6 MB, 1 solve(s), 1 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 16: 103.3 / 104.0 / 102.9 ms, GC 9.3 ms, 3.7 MB, 1 solve(s), 1 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 17: 109.7 / 109.0 / 109.2 ms, GC 0.0 ms, 4.4 MB, 1 solve(s), 1 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 18: 142.3 / 139.0 / 141.8 ms, GC 0.0 ms, 6.0 MB, 1 solve(s), 1 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 95: 129.7 / 119.0 / 121.2 ms, GC 0.0 ms, 7.2 MB, 6 solve(s), 2 attempt(s)
- C R 1 m right bend (S-bend), s = 6.6, run 1, cycle 130: 151.8 / 145.0 / 149.0 ms, GC 0.0 ms, 7.8 MB, 8 solve(s), 2 attempt(s)
- D R 2 m curve, s = 9.7, run 1, cycle 21: 125.6 / 110.0 / 123.8 ms, GC 9.8 ms, 5.7 MB, 4 solve(s), 2 attempt(s)
- D R 2 m curve, s = 9.7, run 1, cycle 29: 90.0 / 86.0 / 89.0 ms, GC 0.0 ms, 4.0 MB, 3 solve(s), 1 attempt(s)
- D R 2 m curve, s = 9.7, run 1, cycle 30: 133.5 / 134.0 / 132.2 ms, GC 0.0 ms, 6.1 MB, 2 solve(s), 2 attempt(s)
- F lap boundary, s = L − 0.3, run 1, cycle 89: 173.0 / 174.0 / 172.1 ms, GC 124.7 ms, 2.3 MB, 2 solve(s), 1 attempt(s)
- F lap boundary, s = L − 0.3, run 1, cycle 185: 100.7 / 100.0 / 100.2 ms, GC 0.0 ms, 3.7 MB, 1 solve(s), 1 attempt(s)
- F lap boundary, s = L − 0.3, run 1, cycle 187: 104.1 / 96.0 / 103.4 ms, GC 0.0 ms, 4.0 MB, 1 solve(s), 1 attempt(s)

## Single-cycle feasibility cases

| case | status | failure cause | attempts | guesses (side, knot clearance above D, track clearance) [m] | passage left / right |
|:--|:--|:--|:--|:--|:-:|
| dynamically infeasible: obstacle (s 25.9, ey 0) at 0.9 m ahead, v = 5 m/s | solver_failure | dynamically_infeasible | left → solver_failure, right → solver_failure | left -0.079 0.279; right -0.079 0.283 | true / true |
| geometrically blocked: radius 0.30 at s = 3.5, from s = 2.4, v = 2 m/s | solver_failure | infeasible_geometry | left → solver_failure, right → solver_failure | left -0.303 0.205; right -0.239 0.187 | false / false |
| feasible reference: obstacle (s 27.9, ey 0) 2.9 m ahead, v = 5 m/s | success | none | left → success, right → success | left -0.010 0.209; right -0.010 0.209 | true / true |

## Diagnosis: E3 (s = 8.20) without the budget, failing cycle 118 replayed with k reassignments

State `[40.8162, -0.2797, -0.1613, 2.3058]`. Left-side guess, then k reassignment solves. The knot outside its assigned segment alternates:

| k | status | NLP solves | optimised knot(s) outside the assigned segment [m] | nearest segment boundary [m] |
|--:|:--|--:|:--|:-:|
| 0 | segment_assignment_invalid | 1 | 43.213 | 43.133 |
| 1 | segment_assignment_invalid | 2 | 43.219 | 43.133 |
| 2 | segment_assignment_invalid | 3 | 43.015 | 43.133 |
| 3 | segment_assignment_invalid | 4 | 43.219 | 43.133 |
| 4 | segment_assignment_invalid | 5 | 43.015 | 43.133 |
| 5 | segment_assignment_invalid | 6 | 43.219 | 43.133 |
| 6 | segment_assignment_invalid | 7 | 43.015 | 43.133 |
| 7 | segment_assignment_invalid | 8 | 43.219 | 43.133 |
| 8 | segment_assignment_invalid | 9 | 43.015 | 43.133 |

