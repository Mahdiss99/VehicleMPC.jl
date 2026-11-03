# Segment-reassignment reliability benchmark (Milestone 3B.1)

- Julia 1.11.5, 13th Gen Intel(R) Core(TM) i9-13900HX, 1 Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `[0.0, 0.0, 0.0, 0.5]`
- Milestone 3B scenarios and parameters: N = 10, dt = 0.1 s, margin 0.05 m, ≤ 8 reassignments, obstacle radius 0.1 m, ego radius 0.05 m, buffer 0.03 m (D = 0.18 m, contact 0.15 m), 3 interior obstacle samples, `obstacle_acceptance = :collision`
- Variants: A original (3B); B `reassignment_cycle_detection` + `branch_resolution`; C `encounter_strategy = :sequential`; D both
- Phase 1: no restrictive budget (correction 1000 s, cycle Inf), one run each. Phase 2: correction budget 80 ms only (the 3B setting), 3 runs each. Phase 3: correction budget 80 ms and `cycle_budget = 0.09 s`, 3 runs each
- 250 control cycles per run; event-aware plant; all cycles counted; timing = the whole `solve!` cycle

## E3 replay: failing cycle 118 of the original controller (no budget)

State `[40.8162, -0.2797, -0.1613, 2.3058]`; R −1 → R 2 boundary at s = 43.133 m (lap 2). Left-side initial guess, `solve_once!` with 8 reassignments:

**A original**: status `segment_assignment_invalid`, 9 NLP solves. 

| solve | status | Ipopt it. | objective | dyn. residual | knot clearance above D [m] | solver [ms] | knots changed | knots outside: progress [m] | knot segment ids 2…11 |
|--:|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:--|
| 1 | segment_assignment_invalid | 26 | -4.1623 | 4.3e-12 | +0.0130 | 16.0 | Int64[] | 11: 43.213 | 14 14 14 14 14 14 14 14 14 14 |
| 2 | segment_assignment_invalid | 41 | -4.2492 | 6.5e-09 | +0.0312 | 22.0 | [11] | 10: 43.219 | 14 14 14 14 14 14 14 14 14 15 |
| 3 | segment_assignment_invalid | 46 | -4.2645 | 8.5e-09 | +0.0128 | 31.0 | [10] | 10: 43.015 | 14 14 14 14 14 14 14 14 15 15 |
| 4 | segment_assignment_invalid | 36 | -4.2492 | 6.5e-09 | +0.0312 | 20.0 | [10] | 10: 43.219 | 14 14 14 14 14 14 14 14 14 15 |
| 5 | segment_assignment_invalid | 46 | -4.2645 | 8.5e-09 | +0.0128 | 28.0 | [10] | 10: 43.015 | 14 14 14 14 14 14 14 14 15 15 |
| 6 | segment_assignment_invalid | 36 | -4.2492 | 6.5e-09 | +0.0312 | 20.0 | [10] | 10: 43.219 | 14 14 14 14 14 14 14 14 14 15 |
| 7 | segment_assignment_invalid | 46 | -4.2645 | 8.5e-09 | +0.0128 | 30.0 | [10] | 10: 43.015 | 14 14 14 14 14 14 14 14 15 15 |
| 8 | segment_assignment_invalid | 36 | -4.2492 | 6.5e-09 | +0.0312 | 23.0 | [10] | 10: 43.219 | 14 14 14 14 14 14 14 14 14 15 |
| 9 | segment_assignment_invalid | 46 | -4.2645 | 8.5e-09 | +0.0128 | 27.0 | [10] | 10: 43.015 | 14 14 14 14 14 14 14 14 15 15 |


**B cycle detection + branch**: status `success`, 5 NLP solves. reassignment cycle of length 2 detected at knot(s) [10]; branch resolution of knot 10 selected segment 14 (segment 14: objective -4.183)

| solve | status | Ipopt it. | objective | dyn. residual | knot clearance above D [m] | solver [ms] | knots changed | knots outside: progress [m] | knot segment ids 2…11 |
|--:|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:--|
| 1 | segment_assignment_invalid | 26 | -4.1623 | 4.3e-12 | +0.0130 | 18.0 | Int64[] | 11: 43.213 | 14 14 14 14 14 14 14 14 14 14 |
| 2 | segment_assignment_invalid | 41 | -4.2492 | 6.5e-09 | +0.0312 | 20.0 | [11] | 10: 43.219 | 14 14 14 14 14 14 14 14 14 15 |
| 3 | segment_assignment_invalid | 46 | -4.2645 | 8.5e-09 | +0.0128 | 31.0 | [10] | 10: 43.015 | 14 14 14 14 14 14 14 14 15 15 |
| branch 14 | success | 25 | -4.1828 | 4.3e-07 | -0.0000 | 15.0 | Int64[] |  | 14 14 14 14 14 14 14 14 14 15 |
| branch 15 | solver_failure | 41 | -3.8701 | NaN | +0.0072 | 31.0 | Int64[] |  | 14 14 14 14 14 14 14 14 15 15 |

branch candidate: knot 10 bounded to segment 14 [39.991, 43.133] → success, objective -4.1828, 32.7 ms
branch candidate: knot 10 bounded to segment 15 [43.133, 46.274] → solver_failure, objective -3.8701, 32.1 ms

## Phase 1: no restrictive budget (one run)

| scenario | variant | laps / stop per run | accepted / cycles | cycles with reassignment cycle: resolved / not | branch solves | max solves in one attempt | NLP solves mean | encounters, solves per encounter | side switches | first encounter: side, objective | plant: min distance [m] (contact 0.15, D 0.18), track clearance [m], violations |
|:--|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | A original | 2.58 | 250 / 250 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | B cycle detection + branch | 2.58 | 250 / 250 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | C sequential encounters | 2.58 | 250 / 250 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | D both | 2.58 | 250 / 250 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| B R 1 m left, s = 3.5 | A original | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.15 | 3, 2.00 | 5 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | B cycle detection + branch | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.15 | 3, 2.00 | 5 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | C sequential encounters | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.12 | 3, 1.00 | 5 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | D both | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.12 | 3, 1.00 | 5 | left -5.8625 | 0.1795, 0.0472, 0 |
| C R 1 m right, s = 6.6 | A original | 2.53 | 250 / 250 | 0 / 0 | 0 | 2 | 1.14 | 4, 2.00 | 1 | right -5.6144 | 0.1795, 0.0468, 0 |
| C R 1 m right, s = 6.6 | B cycle detection + branch | 2.53 | 250 / 250 | 0 / 0 | 0 | 2 | 1.14 | 4, 2.00 | 1 | right -5.6144 | 0.1795, 0.0468, 0 |
| C R 1 m right, s = 6.6 | C sequential encounters | 2.53 | 250 / 250 | 0 / 0 | 0 | 2 | 1.12 | 4, 1.25 | 1 | right -5.6144 | 0.1795, 0.0468, 0 |
| C R 1 m right, s = 6.6 | D both | 2.53 | 250 / 250 | 0 / 0 | 0 | 2 | 1.12 | 4, 1.25 | 1 | right -5.6144 | 0.1795, 0.0468, 0 |
| D R 2 m, s = 9.7 | A original | 2.58 | 250 / 250 | 0 / 0 | 0 | 3 | 1.17 | 3, 2.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | B cycle detection + branch | 2.58 | 250 / 250 | 0 / 0 | 0 | 3 | 1.17 | 3, 2.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | C sequential encounters | 2.58 | 250 / 250 | 0 / 0 | 0 | 3 | 1.14 | 3, 1.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | D both | 2.58 | 250 / 250 | 0 / 0 | 0 | 3 | 1.14 | 3, 1.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| E1 s = 1.95 | A original | 2.54 | 250 / 250 | 0 / 0 | 0 | 2 | 1.19 | 3, 2.00 | 1 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | B cycle detection + branch | 2.54 | 250 / 250 | 0 / 0 | 0 | 2 | 1.19 | 3, 2.00 | 1 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | C sequential encounters | 2.54 | 250 / 250 | 0 / 0 | 0 | 2 | 1.16 | 3, 1.00 | 1 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | D both | 2.54 | 250 / 250 | 0 / 0 | 0 | 2 | 1.16 | 3, 1.00 | 1 | left -5.8625 | 0.1759, 0.0463, 0 |
| E2 s = 2.10 | A original | 2.52 | 250 / 250 | 0 / 0 | 0 | 3 | 1.15 | 3, 2.00 | 1 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | B cycle detection + branch | 2.52 | 250 / 250 | 0 / 0 | 0 | 3 | 1.15 | 3, 2.00 | 1 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | C sequential encounters | 2.52 | 250 / 250 | 0 / 0 | 0 | 3 | 1.12 | 3, 1.00 | 1 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | D both | 2.52 | 250 / 250 | 0 / 0 | 0 | 3 | 1.12 | 3, 1.00 | 1 | left -5.8625 | 0.1786, 0.0469, 0 |
| E3 s = 8.20 | A original | stop c118 | 117 / 118 | 0 / 0 | 0 | 9 | 1.31 | 3, 2.00 | 3 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | B cycle detection + branch | 2.60 | 250 / 250 | 1 / 0 | 2 | 4 | 1.18 | 3, 2.00 | 6 | right -4.3804 | 0.1816, 0.0437, 0 |
| E3 s = 8.20 | C sequential encounters | stop c118 | 117 / 118 | 0 / 0 | 0 | 9 | 1.29 | 3, 1.33 | 3 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | D both | 2.60 | 250 / 250 | 1 / 0 | 2 | 4 | 1.16 | 3, 1.00 | 6 | right -4.3804 | 0.1816, 0.0437, 0 |
| E4 s = 8.40 | A original | 2.52 | 250 / 250 | 0 / 0 | 0 | 2 | 1.16 | 3, 2.00 | 5 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | B cycle detection + branch | 2.52 | 250 / 250 | 0 / 0 | 0 | 2 | 1.16 | 3, 2.00 | 5 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | C sequential encounters | 2.52 | 250 / 250 | 0 / 0 | 0 | 2 | 1.15 | 3, 1.00 | 5 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | D both | 2.52 | 250 / 250 | 0 / 0 | 0 | 2 | 1.15 | 3, 1.00 | 5 | right -4.2217 | 0.1771, 0.0470, 0 |
| F lap boundary | A original | 2.58 | 250 / 250 | 0 / 0 | 0 | 2 | 1.15 | 2, 2.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | B cycle detection + branch | 2.58 | 250 / 250 | 0 / 0 | 0 | 2 | 1.15 | 2, 2.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | C sequential encounters | 2.58 | 250 / 250 | 0 / 0 | 0 | 2 | 1.14 | 2, 1.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | D both | 2.58 | 250 / 250 | 0 / 0 | 0 | 2 | 1.14 | 2, 1.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| G1 +0.08 on R 1 m left | A original | 2.48 | 250 / 250 | 0 / 0 | 0 | 2 | 1.16 | 3, 2.00 | 5 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | B cycle detection + branch | 2.48 | 250 / 250 | 0 / 0 | 0 | 2 | 1.16 | 3, 2.00 | 5 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | C sequential encounters | 2.48 | 250 / 250 | 0 / 0 | 0 | 2 | 1.14 | 3, 1.00 | 5 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | D both | 2.48 | 250 / 250 | 0 / 0 | 0 | 2 | 1.14 | 3, 1.00 | 5 | left -5.8625 | 0.1794, 0.0452, 0 |
| G2 −0.08 on R 1 m right | A original | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.11 | 3, 2.00 | 5 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | B cycle detection + branch | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.11 | 3, 2.00 | 5 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | C sequential encounters | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.10 | 3, 1.00 | 5 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | D both | 2.46 | 250 / 250 | 0 / 0 | 0 | 2 | 1.10 | 3, 1.00 | 5 | right -5.6144 | 0.1800, 0.0464, 0 |

| scenario | variant | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: median / max [ms] | ordinary median [ms] | Ipopt share of total time | GC share | mean allocation per cycle [MB] |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | A original | 6.3 / 13.2 / 14.8 / 26.2 | 0 / 0 | – | 6.3 | 96 % | 1.1 % | 0.32 |
| A no obstacle | B cycle detection + branch | 6.1 / 12.7 / 13.8 / 14.2 | 0 / 0 | – | 6.1 | 94 % | 0.5 % | 0.32 |
| A no obstacle | C sequential encounters | 6.2 / 13.1 / 14.2 / 14.6 | 0 / 0 | – | 6.2 | 98 % | 0.8 % | 0.32 |
| A no obstacle | D both | 6.1 / 12.5 / 14.1 / 14.5 | 0 / 0 | – | 6.1 | 98 % | 0.3 % | 0.32 |
| B R 1 m left, s = 3.5 | A original | 10.2 / 22.2 / 29.8 / 37.0 | 0 / 0 | 35.8 / 37.0 | 10.1 | 97 % | 0.7 % | 0.71 |
| B R 1 m left, s = 3.5 | B cycle detection + branch | 10.4 / 22.3 / 30.7 / 39.8 | 0 / 0 | 35.7 / 39.8 | 10.4 | 98 % | 0.6 % | 0.71 |
| B R 1 m left, s = 3.5 | C sequential encounters | 10.3 / 21.5 / 24.7 / 30.0 | 0 / 0 | 17.9 / 18.4 | 10.3 | 96 % | 0.6 % | 0.70 |
| B R 1 m left, s = 3.5 | D both | 10.4 / 21.8 / 27.7 / 105.6 | 1 / 1 | 18.6 / 18.6 | 10.3 | 98 % | 3.9 % | 0.70 |
| C R 1 m right, s = 6.6 | A original | 10.4 / 22.1 / 27.9 / 103.0 | 1 / 1 | 17.8 / 103.0 | 10.4 | 97 % | 0.6 % | 0.71 |
| C R 1 m right, s = 6.6 | B cycle detection + branch | 10.2 / 22.2 / 27.8 / 94.3 | 1 / 0 | 18.1 / 94.3 | 10.2 | 97 % | 0.6 % | 0.71 |
| C R 1 m right, s = 6.6 | C sequential encounters | 10.0 / 22.0 / 27.9 / 97.4 | 1 / 0 | 9.0 / 97.4 | 10.0 | 98 % | 0.7 % | 0.71 |
| C R 1 m right, s = 6.6 | D both | 10.1 / 21.6 / 27.1 / 94.6 | 1 / 0 | 8.8 / 94.6 | 10.1 | 97 % | 0.7 % | 0.71 |
| D R 2 m, s = 9.7 | A original | 10.1 / 21.9 / 28.8 / 50.6 | 0 / 0 | 40.5 / 50.6 | 10.1 | 97 % | 0.7 % | 0.69 |
| D R 2 m, s = 9.7 | B cycle detection + branch | 10.3 / 21.9 / 30.9 / 51.0 | 0 / 0 | 39.3 / 51.0 | 10.3 | 97 % | 0.8 % | 0.69 |
| D R 2 m, s = 9.7 | C sequential encounters | 10.3 / 21.7 / 25.8 / 114.9 | 1 / 1 | 19.3 / 25.3 | 10.3 | 96 % | 4.5 % | 0.68 |
| D R 2 m, s = 9.7 | D both | 9.9 / 21.3 / 23.3 / 24.8 | 0 / 0 | 19.8 / 24.8 | 9.9 | 97 % | 0.6 % | 0.68 |
| E1 s = 1.95 | A original | 10.5 / 23.0 / 42.1 / 53.2 | 0 / 0 | 36.9 / 52.7 | 10.5 | 98 % | 0.5 % | 0.73 |
| E1 s = 1.95 | B cycle detection + branch | 10.4 / 23.3 / 43.4 / 55.6 | 0 / 0 | 41.9 / 54.7 | 10.4 | 96 % | 0.7 % | 0.73 |
| E1 s = 1.95 | C sequential encounters | 10.3 / 21.4 / 36.0 / 54.8 | 0 / 0 | 18.8 / 20.3 | 10.3 | 97 % | 0.7 % | 0.72 |
| E1 s = 1.95 | D both | 10.4 / 21.8 / 36.3 / 53.6 | 0 / 0 | 18.0 / 19.8 | 10.4 | 97 % | 0.7 % | 0.72 |
| E2 s = 2.10 | A original | 10.4 / 23.2 / 56.5 / 81.8 | 1 / 0 | 55.1 / 81.8 | 10.4 | 97 % | 0.7 % | 0.73 |
| E2 s = 2.10 | B cycle detection + branch | 10.5 / 23.6 / 58.4 / 104.0 | 2 / 1 | 53.8 / 80.1 | 10.4 | 92 % | 3.8 % | 0.73 |
| E2 s = 2.10 | C sequential encounters | 10.3 / 22.9 / 42.3 / 63.1 | 0 / 0 | 26.5 / 36.2 | 10.3 | 97 % | 0.5 % | 0.71 |
| E2 s = 2.10 | D both | 10.3 / 22.3 / 43.1 / 61.6 | 0 / 0 | 28.9 / 37.2 | 10.3 | 98 % | 0.6 % | 0.71 |
| E3 s = 8.20 | A original | 10.0 / 22.0 / 45.2 / 432.4 | 1 / 1 | 20.2 / 432.4 | 9.9 | 95 % | 0.5 % | 0.86 |
| E3 s = 8.20 | B cycle detection + branch | 10.0 / 21.5 / 36.6 / 263.4 | 1 / 1 | 18.4 / 19.5 | 9.9 | 93 % | 0.8 % | 0.80 |
| E3 s = 8.20 | C sequential encounters | 10.1 / 21.6 / 45.1 / 442.4 | 1 / 1 | 9.8 / 442.4 | 10.1 | 98 % | 0.9 % | 0.85 |
| E3 s = 8.20 | D both | 9.9 / 21.3 / 33.1 / 104.6 | 1 / 1 | 9.2 / 9.6 | 10.0 | 98 % | 0.8 % | 0.77 |
| E4 s = 8.40 | A original | 10.1 / 21.3 / 47.4 / 108.0 | 1 / 1 | 18.0 / 19.2 | 10.1 | 94 % | 4.0 % | 0.72 |
| E4 s = 8.40 | B cycle detection + branch | 10.1 / 21.0 / 43.5 / 60.3 | 0 / 0 | 18.7 / 19.4 | 10.1 | 98 % | 0.6 % | 0.72 |
| E4 s = 8.40 | C sequential encounters | 10.2 / 21.7 / 40.1 / 52.6 | 0 / 0 | 9.9 / 10.3 | 10.2 | 98 % | 0.5 % | 0.71 |
| E4 s = 8.40 | D both | 10.1 / 21.3 / 41.0 / 52.4 | 0 / 0 | 9.9 / 9.9 | 10.1 | 96 % | 0.7 % | 0.71 |
| F lap boundary | A original | 10.0 / 21.5 / 30.0 / 41.7 | 0 / 0 | 39.9 / 41.7 | 9.9 | 96 % | 0.6 % | 0.69 |
| F lap boundary | B cycle detection + branch | 10.1 / 21.6 / 29.6 / 42.6 | 0 / 0 | 42.2 / 42.6 | 10.0 | 97 % | 0.7 % | 0.69 |
| F lap boundary | C sequential encounters | 10.1 / 21.3 / 24.2 / 35.4 | 0 / 0 | 20.4 / 21.0 | 10.1 | 96 % | 0.6 % | 0.69 |
| F lap boundary | D both | 10.4 / 21.8 / 30.3 / 103.3 | 1 / 1 | 19.8 / 20.2 | 10.4 | 97 % | 4.2 % | 0.69 |
| G1 +0.08 on R 1 m left | A original | 10.5 / 21.2 / 31.3 / 40.4 | 0 / 0 | 36.6 / 40.4 | 10.4 | 98 % | 0.5 % | 0.72 |
| G1 +0.08 on R 1 m left | B cycle detection + branch | 10.5 / 21.4 / 32.0 / 41.0 | 0 / 0 | 36.0 / 41.0 | 10.4 | 98 % | 0.6 % | 0.72 |
| G1 +0.08 on R 1 m left | C sequential encounters | 10.3 / 20.5 / 23.9 / 30.2 | 0 / 0 | 18.0 / 20.2 | 10.3 | 98 % | 0.7 % | 0.71 |
| G1 +0.08 on R 1 m left | D both | 10.3 / 20.3 / 23.2 / 30.2 | 0 / 0 | 17.4 / 20.0 | 10.2 | 97 % | 0.7 % | 0.71 |
| G2 −0.08 on R 1 m right | A original | 10.3 / 23.0 / 27.4 / 37.1 | 0 / 0 | 17.2 / 18.1 | 10.2 | 97 % | 0.8 % | 0.71 |
| G2 −0.08 on R 1 m right | B cycle detection + branch | 10.1 / 22.4 / 26.8 / 36.4 | 0 / 0 | 16.5 / 18.5 | 10.1 | 98 % | 0.8 % | 0.71 |
| G2 −0.08 on R 1 m right | C sequential encounters | 10.4 / 23.3 / 27.1 / 104.7 | 1 / 1 | 8.9 / 9.4 | 10.4 | 95 % | 4.2 % | 0.70 |
| G2 −0.08 on R 1 m right | D both | 10.1 / 22.7 / 24.9 / 36.8 | 0 / 0 | 8.3 / 9.0 | 10.2 | 98 % | 0.5 % | 0.70 |

Stops:

- E3 s = 8.20 / A original: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m) [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / C sequential encounters: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m) [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]

## Phase 2: correction budget 80 ms only (3 runs)

| scenario | variant | laps / stop per run | accepted / cycles | cycles with reassignment cycle: resolved / not | branch solves | max solves in one attempt | NLP solves mean | encounters, solves per encounter | side switches | first encounter: side, objective | plant: min distance [m] (contact 0.15, D 0.18), track clearance [m], violations |
|:--|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | A original | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | B cycle detection + branch | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | C sequential encounters | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | D both | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| B R 1 m left, s = 3.5 | A original | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 9, 2.00 | 15 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | B cycle detection + branch | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 9, 2.00 | 15 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | C sequential encounters | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.12 | 9, 1.00 | 15 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | D both | 2.46; stop c86; 2.46 | 585 / 586 | 0 / 0 | 0 | 2 | 1.12 | 7, 1.00 | 11 | left -5.8625 | 0.1795, 0.0472, 0 |
| C R 1 m right, s = 6.6 | A original | 2.53; 2.53; 2.53 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 12, 2.00 | 3 | right -5.6144 | 0.1795, 0.0468, 0 |
| C R 1 m right, s = 6.6 | B cycle detection + branch | 2.53; 2.53; 2.53 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 12, 2.00 | 3 | right -5.6144 | 0.1795, 0.0468, 0 |
| C R 1 m right, s = 6.6 | C sequential encounters | 2.53; 2.53; 2.53 | 750 / 750 | 0 / 0 | 0 | 2 | 1.12 | 12, 1.25 | 3 | right -5.6144 | 0.1795, 0.0468, 0 |
| C R 1 m right, s = 6.6 | D both | 2.53; 2.53; 2.53 | 750 / 750 | 0 / 0 | 0 | 2 | 1.12 | 12, 1.25 | 3 | right -5.6144 | 0.1795, 0.0468, 0 |
| D R 2 m, s = 9.7 | A original | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.17 | 9, 2.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | B cycle detection + branch | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.17 | 9, 2.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | C sequential encounters | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.14 | 9, 1.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | D both | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.14 | 9, 1.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| E1 s = 1.95 | A original | 2.54; stop c120; 2.54 | 619 / 620 | 0 / 0 | 0 | 2 | 1.19 | 8, 2.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | B cycle detection + branch | 2.54; 2.54; 2.54 | 750 / 750 | 0 / 0 | 0 | 2 | 1.19 | 9, 2.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | C sequential encounters | 2.54; 2.54; 2.54 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 1.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | D both | 2.54; 2.54; 2.54 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 1.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E2 s = 2.10 | A original | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.15 | 9, 2.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | B cycle detection + branch | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.15 | 9, 2.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | C sequential encounters | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.12 | 9, 1.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | D both | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.12 | 9, 1.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E3 s = 8.20 | A original | stop c118; stop c118; stop c118 | 351 / 354 | 0 / 0 | 0 | 3 | 1.20 | 9, 2.00 | 9 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | B cycle detection + branch | stop c118; stop c118; stop c64 | 297 / 300 | 0 / 2 | 4 | 3 | 1.19 | 7, 2.00 | 8 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | C sequential encounters | stop c118; stop c118; stop c118 | 351 / 354 | 0 / 0 | 0 | 3 | 1.19 | 9, 1.33 | 9 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | D both | stop c118; stop c118; stop c118 | 351 / 354 | 0 / 2 | 4 | 3 | 1.18 | 9, 1.33 | 9 | right -4.3804 | 0.1821, 0.0438, 0 |
| E4 s = 8.40 | A original | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 2.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | B cycle detection + branch | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 2.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | C sequential encounters | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 10, 1.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | D both | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 9, 1.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| F lap boundary | A original | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 6, 2.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | B cycle detection + branch | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 6, 2.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | C sequential encounters | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 6, 1.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | D both | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 6, 1.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| G1 +0.08 on R 1 m left | A original | 2.48; 2.48; 2.48 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 2.00 | 15 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | B cycle detection + branch | stop c181; 2.48; 2.48 | 680 / 681 | 0 / 0 | 0 | 2 | 1.16 | 8, 2.00 | 13 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | C sequential encounters | 2.48; 2.48; 2.48 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 9, 1.00 | 15 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | D both | 2.48; 2.48; 2.48 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 9, 1.00 | 15 | left -5.8625 | 0.1794, 0.0452, 0 |
| G2 −0.08 on R 1 m right | A original | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.11 | 9, 2.00 | 15 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | B cycle detection + branch | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.11 | 9, 2.00 | 15 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | C sequential encounters | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.10 | 9, 1.00 | 15 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | D both | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.10 | 9, 1.00 | 15 | right -5.6144 | 0.1800, 0.0464, 0 |

| scenario | variant | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: median / max [ms] | ordinary median [ms] | Ipopt share of total time | GC share | mean allocation per cycle [MB] |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | A original | 6.0 / 12.9 / 14.2 / 17.2 | 0 / 0 | – | 6.0 | 97 % | 0.7 % | 0.32 |
| A no obstacle | B cycle detection + branch | 6.0 / 12.7 / 14.0 / 16.6 | 0 / 0 | – | 6.0 | 97 % | 0.6 % | 0.32 |
| A no obstacle | C sequential encounters | 6.0 / 12.5 / 14.3 / 31.6 | 0 / 0 | – | 6.0 | 97 % | 0.9 % | 0.32 |
| A no obstacle | D both | 6.0 / 12.5 / 13.8 / 15.2 | 0 / 0 | – | 6.0 | 97 % | 0.4 % | 0.32 |
| B R 1 m left, s = 3.5 | A original | 10.2 / 22.1 / 33.4 / 39.0 | 0 / 0 | 36.0 / 39.0 | 10.2 | 97 % | 0.7 % | 0.71 |
| B R 1 m left, s = 3.5 | B cycle detection + branch | 10.2 / 21.9 / 32.6 / 40.2 | 0 / 0 | 36.0 / 40.2 | 10.2 | 97 % | 0.6 % | 0.71 |
| B R 1 m left, s = 3.5 | C sequential encounters | 10.2 / 21.2 / 24.6 / 28.5 | 0 / 0 | 18.1 / 19.2 | 10.2 | 97 % | 0.6 % | 0.70 |
| B R 1 m left, s = 3.5 | D both | 10.1 / 21.7 / 25.0 / 102.0 | 1 / 1 | 18.1 / 19.2 | 10.1 | 97 % | 2.2 % | 0.70 |
| C R 1 m right, s = 6.6 | A original | 10.1 / 21.8 / 29.5 / 106.4 | 4 / 1 | 17.6 / 93.4 | 10.1 | 97 % | 2.0 % | 0.71 |
| C R 1 m right, s = 6.6 | B cycle detection + branch | 10.0 / 21.7 / 29.3 / 95.0 | 3 / 0 | 18.6 / 95.0 | 10.0 | 98 % | 0.7 % | 0.71 |
| C R 1 m right, s = 6.6 | C sequential encounters | 10.0 / 21.5 / 29.3 / 102.7 | 4 / 1 | 8.9 / 94.0 | 10.0 | 98 % | 1.8 % | 0.71 |
| C R 1 m right, s = 6.6 | D both | 10.1 / 21.7 / 28.8 / 102.9 | 3 / 1 | 9.1 / 102.9 | 10.1 | 97 % | 0.6 % | 0.71 |
| D R 2 m, s = 9.7 | A original | 9.9 / 21.2 / 34.1 / 51.1 | 0 / 0 | 38.9 / 51.1 | 9.9 | 98 % | 0.7 % | 0.69 |
| D R 2 m, s = 9.7 | B cycle detection + branch | 10.0 / 21.4 / 34.1 / 50.0 | 0 / 0 | 38.8 / 50.0 | 9.9 | 97 % | 0.7 % | 0.69 |
| D R 2 m, s = 9.7 | C sequential encounters | 9.9 / 21.1 / 23.2 / 27.1 | 0 / 0 | 19.7 / 25.5 | 9.9 | 97 % | 0.7 % | 0.68 |
| D R 2 m, s = 9.7 | D both | 10.0 / 21.4 / 24.2 / 105.1 | 2 / 1 | 20.0 / 28.5 | 10.0 | 96 % | 2.8 % | 0.68 |
| E1 s = 1.95 | A original | 10.2 / 22.5 / 50.8 / 101.1 | 1 / 1 | 35.6 / 57.9 | 10.2 | 96 % | 2.0 % | 0.73 |
| E1 s = 1.95 | B cycle detection + branch | 10.2 / 22.5 / 44.0 / 110.3 | 1 / 1 | 35.2 / 53.2 | 10.2 | 97 % | 1.8 % | 0.73 |
| E1 s = 1.95 | C sequential encounters | 10.2 / 21.5 / 39.0 / 55.0 | 0 / 0 | 18.0 / 19.8 | 10.2 | 97 % | 0.6 % | 0.72 |
| E1 s = 1.95 | D both | 10.3 / 21.5 / 38.5 / 54.4 | 0 / 0 | 18.0 / 19.4 | 10.3 | 98 % | 0.7 % | 0.72 |
| E2 s = 2.10 | A original | 10.1 / 22.6 / 55.0 / 84.2 | 2 / 0 | 53.3 / 84.2 | 10.1 | 97 % | 0.7 % | 0.73 |
| E2 s = 2.10 | B cycle detection + branch | 10.1 / 22.5 / 56.9 / 79.0 | 0 / 0 | 53.1 / 79.0 | 10.1 | 98 % | 0.6 % | 0.73 |
| E2 s = 2.10 | C sequential encounters | 10.2 / 22.5 / 47.7 / 62.0 | 0 / 0 | 27.0 / 40.1 | 10.2 | 98 % | 0.7 % | 0.71 |
| E2 s = 2.10 | D both | 10.1 / 22.6 / 48.3 / 61.4 | 0 / 0 | 26.9 / 36.5 | 10.1 | 97 % | 0.5 % | 0.71 |
| E3 s = 8.20 | A original | 9.9 / 21.2 / 48.7 / 139.5 | 3 / 3 | 19.3 / 139.5 | 9.9 | 97 % | 0.7 % | 0.76 |
| E3 s = 8.20 | B cycle detection + branch | 9.9 / 21.9 / 48.7 / 120.9 | 3 / 3 | 19.2 / 120.9 | 9.9 | 95 % | 3.5 % | 0.75 |
| E3 s = 8.20 | C sequential encounters | 9.8 / 21.3 / 48.2 / 138.0 | 3 / 3 | 9.6 / 138.0 | 9.8 | 97 % | 0.4 % | 0.75 |
| E3 s = 8.20 | D both | 9.7 / 21.7 / 48.3 / 127.8 | 3 / 3 | 9.4 / 127.8 | 9.7 | 98 % | 0.8 % | 0.74 |
| E4 s = 8.40 | A original | 10.1 / 20.9 / 45.1 / 51.4 | 0 / 0 | 18.9 / 19.3 | 10.0 | 97 % | 0.6 % | 0.72 |
| E4 s = 8.40 | B cycle detection + branch | 10.0 / 20.9 / 45.3 / 51.4 | 0 / 0 | 18.5 / 19.0 | 10.0 | 97 % | 0.5 % | 0.72 |
| E4 s = 8.40 | C sequential encounters | 9.9 / 21.0 / 46.1 / 153.3 | 1 / 1 | 9.6 / 153.3 | 10.0 | 97 % | 1.7 % | 0.71 |
| E4 s = 8.40 | D both | 9.9 / 21.0 / 44.9 / 100.0 | 1 / 1 | 9.4 / 9.6 | 9.9 | 96 % | 1.8 % | 0.71 |
| F lap boundary | A original | 10.0 / 21.7 / 35.2 / 46.4 | 0 / 0 | 40.9 / 46.4 | 10.0 | 97 % | 0.6 % | 0.69 |
| F lap boundary | B cycle detection + branch | 10.0 / 21.6 / 35.1 / 41.0 | 0 / 0 | 39.4 / 41.0 | 10.0 | 97 % | 0.6 % | 0.69 |
| F lap boundary | C sequential encounters | 9.9 / 21.4 / 23.9 / 101.4 | 1 / 1 | 20.0 / 21.2 | 9.9 | 97 % | 1.9 % | 0.69 |
| F lap boundary | D both | 10.0 / 21.8 / 23.8 / 36.4 | 0 / 0 | 19.9 / 20.9 | 10.0 | 97 % | 0.5 % | 0.69 |
| G1 +0.08 on R 1 m left | A original | 10.2 / 21.4 / 32.0 / 40.3 | 0 / 0 | 36.2 / 40.3 | 10.2 | 96 % | 0.6 % | 0.72 |
| G1 +0.08 on R 1 m left | B cycle detection + branch | 10.2 / 21.2 / 33.0 / 103.1 | 1 / 1 | 35.8 / 40.2 | 10.2 | 96 % | 2.0 % | 0.72 |
| G1 +0.08 on R 1 m left | C sequential encounters | 10.2 / 20.4 / 26.4 / 112.7 | 1 / 1 | 17.6 / 20.3 | 10.2 | 96 % | 1.7 % | 0.71 |
| G1 +0.08 on R 1 m left | D both | 10.2 / 20.0 / 25.4 / 31.0 | 0 / 0 | 17.8 / 20.0 | 10.1 | 97 % | 0.6 % | 0.71 |
| G2 −0.08 on R 1 m right | A original | 10.1 / 22.4 / 25.3 / 41.0 | 0 / 0 | 16.4 / 18.5 | 10.1 | 97 % | 0.7 % | 0.71 |
| G2 −0.08 on R 1 m right | B cycle detection + branch | 10.1 / 22.4 / 25.4 / 36.4 | 0 / 0 | 17.3 / 25.5 | 10.1 | 97 % | 0.7 % | 0.71 |
| G2 −0.08 on R 1 m right | C sequential encounters | 10.0 / 22.5 / 27.3 / 99.1 | 1 / 0 | 8.3 / 9.5 | 10.1 | 96 % | 1.9 % | 0.70 |
| G2 −0.08 on R 1 m right | D both | 10.1 / 22.3 / 25.9 / 37.2 | 0 / 0 | 8.6 / 9.2 | 10.1 | 97 % | 0.7 % | 0.70 |

Stops:

- E1 s = 1.95 / A original, run 2: c120: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.04402 m); schedule correction skipped: time budget 0.08 s would be exceeded
- E3 s = 8.20 / A original, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / A original, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / A original, run 3: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / B cycle detection + branch, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / B cycle detection + branch, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / B cycle detection + branch, run 3: c64: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.04396 m); schedule correction skipped: time budget 0.08 s would be exceeded
- G1 +0.08 on R 1 m left / B cycle detection + branch, run 1: c181: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1052 m); schedule correction skipped: time budget 0.08 s would be exceeded
- E3 s = 8.20 / C sequential encounters, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / C sequential encounters, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / C sequential encounters, run 3: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- B R 1 m left, s = 3.5 / D both, run 2: c86: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.09896 m); schedule correction skipped: time budget 0.08 s would be exceeded
- E3 s = 8.20 / D both, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / D both, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / D both, run 3: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.1182 m); schedule correction skipped: time budget 0.08 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]

## Phase 3: correction budget 80 ms + cycle budget 90 ms (3 runs)

| scenario | variant | laps / stop per run | accepted / cycles | cycles with reassignment cycle: resolved / not | branch solves | max solves in one attempt | NLP solves mean | encounters, solves per encounter | side switches | first encounter: side, objective | plant: min distance [m] (contact 0.15, D 0.18), track clearance [m], violations |
|:--|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | A original | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | B cycle detection + branch | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | C sequential encounters | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| A no obstacle | D both | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 0 | 1.13 | 0, NaN | 0 | none NaN | NaN, 0.0463, 0 |
| B R 1 m left, s = 3.5 | A original | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 9, 2.00 | 15 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | B cycle detection + branch | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 9, 2.00 | 15 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | C sequential encounters | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.12 | 9, 1.00 | 15 | left -5.8625 | 0.1795, 0.0472, 0 |
| B R 1 m left, s = 3.5 | D both | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.12 | 9, 1.00 | 15 | left -5.8625 | 0.1795, 0.0472, 0 |
| C R 1 m right, s = 6.6 | A original | stop c193; stop c205; stop c205 | 600 / 603 | 0 / 0 | 0 | 2 | 1.13 | 10, 1.75 | 0 | right -5.6144 | 0.2935, 0.0468, 0 |
| C R 1 m right, s = 6.6 | B cycle detection + branch | stop c205; stop c205; stop c205 | 612 / 615 | 0 / 0 | 0 | 2 | 1.13 | 12, 1.75 | 0 | right -5.6144 | 0.2935, 0.0468, 0 |
| C R 1 m right, s = 6.6 | C sequential encounters | stop c205; stop c205; stop c205 | 612 / 615 | 0 / 0 | 0 | 2 | 1.12 | 12, 1.00 | 0 | right -5.6144 | 0.2935, 0.0468, 0 |
| C R 1 m right, s = 6.6 | D both | stop c205; stop c205; stop c205 | 612 / 615 | 0 / 0 | 0 | 2 | 1.12 | 12, 1.00 | 0 | right -5.6144 | 0.2935, 0.0468, 0 |
| D R 2 m, s = 9.7 | A original | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.17 | 9, 2.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | B cycle detection + branch | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.17 | 9, 2.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | C sequential encounters | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.14 | 9, 1.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| D R 2 m, s = 9.7 | D both | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 3 | 1.14 | 9, 1.00 | 0 | left -5.7330 | 0.2942, 0.0463, 0 |
| E1 s = 1.95 | A original | 2.54; 2.54; 2.54 | 750 / 750 | 0 / 0 | 0 | 2 | 1.19 | 9, 2.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | B cycle detection + branch | 2.54; 2.54; 2.54 | 750 / 750 | 0 / 0 | 0 | 2 | 1.19 | 9, 2.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | C sequential encounters | 2.54; 2.54; 2.54 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 1.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E1 s = 1.95 | D both | 2.54; 2.54; 2.54 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 1.00 | 3 | left -5.8625 | 0.1759, 0.0463, 0 |
| E2 s = 2.10 | A original | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.15 | 9, 2.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | B cycle detection + branch | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.15 | 9, 2.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | C sequential encounters | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.12 | 9, 1.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E2 s = 2.10 | D both | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 3 | 1.12 | 9, 1.00 | 3 | left -5.8625 | 0.1786, 0.0469, 0 |
| E3 s = 8.20 | A original | stop c118; stop c118; stop c118 | 351 / 354 | 0 / 0 | 0 | 3 | 1.18 | 9, 1.67 | 9 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | B cycle detection + branch | stop c118; stop c118; stop c118 | 351 / 354 | 0 / 2 | 4 | 2 | 1.19 | 9, 2.00 | 9 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | C sequential encounters | stop c118; stop c118; stop c118 | 351 / 354 | 0 / 0 | 0 | 3 | 1.16 | 9, 1.00 | 9 | right -4.3804 | 0.1821, 0.0438, 0 |
| E3 s = 8.20 | D both | stop c118; stop c118; stop c118 | 351 / 354 | 0 / 3 | 6 | 2 | 1.17 | 9, 1.33 | 9 | right -4.3804 | 0.1821, 0.0438, 0 |
| E4 s = 8.40 | A original | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 2.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | B cycle detection + branch | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 2.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | C sequential encounters | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 9, 1.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| E4 s = 8.40 | D both | 2.52; 2.52; 2.52 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 9, 1.00 | 15 | right -4.2217 | 0.1771, 0.0470, 0 |
| F lap boundary | A original | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 6, 2.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | B cycle detection + branch | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.15 | 6, 2.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | C sequential encounters | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 6, 1.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| F lap boundary | D both | 2.58; 2.58; 2.58 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 6, 1.00 | 0 | right -7.4190 | 0.1855, 0.0463, 0 |
| G1 +0.08 on R 1 m left | A original | 2.48; 2.48; 2.48 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 2.00 | 15 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | B cycle detection + branch | 2.48; 2.48; 2.48 | 750 / 750 | 0 / 0 | 0 | 2 | 1.16 | 9, 2.00 | 15 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | C sequential encounters | 2.48; 2.48; 2.48 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 9, 1.00 | 15 | left -5.8625 | 0.1794, 0.0452, 0 |
| G1 +0.08 on R 1 m left | D both | 2.48; 2.48; 2.48 | 750 / 750 | 0 / 0 | 0 | 2 | 1.14 | 9, 1.00 | 15 | left -5.8625 | 0.1794, 0.0452, 0 |
| G2 −0.08 on R 1 m right | A original | stop c230; stop c188; 2.46 | 666 / 668 | 0 / 0 | 0 | 2 | 1.11 | 8, 2.00 | 13 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | B cycle detection + branch | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.11 | 9, 2.00 | 15 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | C sequential encounters | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.10 | 9, 1.00 | 15 | right -5.6144 | 0.1800, 0.0464, 0 |
| G2 −0.08 on R 1 m right | D both | 2.46; 2.46; 2.46 | 750 / 750 | 0 / 0 | 0 | 2 | 1.10 | 9, 1.00 | 15 | right -5.6144 | 0.1800, 0.0464, 0 |

| scenario | variant | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: median / max [ms] | ordinary median [ms] | Ipopt share of total time | GC share | mean allocation per cycle [MB] |
|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| A no obstacle | A original | 6.0 / 12.5 / 13.9 / 19.6 | 0 / 0 | – | 6.0 | 97 % | 0.5 % | 0.32 |
| A no obstacle | B cycle detection + branch | 6.0 / 12.5 / 14.0 / 17.7 | 0 / 0 | – | 6.0 | 96 % | 0.3 % | 0.32 |
| A no obstacle | C sequential encounters | 6.0 / 12.5 / 13.9 / 15.1 | 0 / 0 | – | 6.0 | 97 % | 0.5 % | 0.32 |
| A no obstacle | D both | 6.0 / 12.6 / 13.7 / 14.7 | 0 / 0 | – | 6.0 | 97 % | 0.6 % | 0.32 |
| B R 1 m left, s = 3.5 | A original | 10.1 / 21.8 / 31.5 / 37.5 | 0 / 0 | 35.2 / 37.5 | 10.1 | 97 % | 0.6 % | 0.71 |
| B R 1 m left, s = 3.5 | B cycle detection + branch | 10.2 / 22.2 / 34.6 / 101.2 | 1 / 1 | 35.8 / 37.4 | 10.2 | 97 % | 1.8 % | 0.71 |
| B R 1 m left, s = 3.5 | C sequential encounters | 10.1 / 21.2 / 24.4 / 29.2 | 0 / 0 | 17.7 / 18.5 | 10.1 | 97 % | 0.7 % | 0.70 |
| B R 1 m left, s = 3.5 | D both | 10.0 / 21.2 / 24.6 / 28.2 | 0 / 0 | 17.4 / 18.9 | 10.0 | 97 % | 0.8 % | 0.70 |
| C R 1 m right, s = 6.6 | A original | 10.0 / 21.2 / 24.8 / 103.2 | 1 / 1 | 17.2 / 63.8 | 10.0 | 97 % | 2.3 % | 0.70 |
| C R 1 m right, s = 6.6 | B cycle detection + branch | 10.0 / 21.0 / 23.9 / 65.6 | 0 / 0 | 17.5 / 65.6 | 10.0 | 97 % | 0.7 % | 0.70 |
| C R 1 m right, s = 6.6 | C sequential encounters | 10.0 / 21.3 / 24.6 / 102.3 | 1 / 1 | 9.1 / 65.3 | 10.0 | 97 % | 2.1 % | 0.69 |
| C R 1 m right, s = 6.6 | D both | 10.1 / 21.3 / 24.1 / 66.0 | 0 / 0 | 9.4 / 66.0 | 10.1 | 97 % | 0.6 % | 0.69 |
| D R 2 m, s = 9.7 | A original | 10.0 / 21.1 / 34.6 / 52.6 | 0 / 0 | 38.0 / 52.6 | 10.0 | 97 % | 0.7 % | 0.69 |
| D R 2 m, s = 9.7 | B cycle detection + branch | 9.9 / 21.2 / 37.9 / 102.4 | 1 / 1 | 38.5 / 55.9 | 9.9 | 97 % | 1.8 % | 0.69 |
| D R 2 m, s = 9.7 | C sequential encounters | 10.0 / 21.3 / 23.2 / 28.1 | 0 / 0 | 19.6 / 24.9 | 9.9 | 97 % | 0.8 % | 0.68 |
| D R 2 m, s = 9.7 | D both | 10.0 / 20.9 / 24.0 / 28.4 | 0 / 0 | 19.0 / 25.1 | 10.0 | 97 % | 0.8 % | 0.68 |
| E1 s = 1.95 | A original | 10.2 / 22.7 / 43.9 / 102.3 | 1 / 1 | 35.9 / 52.7 | 10.2 | 98 % | 1.8 % | 0.73 |
| E1 s = 1.95 | B cycle detection + branch | 10.2 / 23.0 / 43.0 / 57.5 | 0 / 0 | 35.6 / 57.5 | 10.2 | 97 % | 0.7 % | 0.73 |
| E1 s = 1.95 | C sequential encounters | 10.2 / 21.3 / 39.1 / 97.4 | 1 / 0 | 18.0 / 19.4 | 10.2 | 97 % | 1.7 % | 0.72 |
| E1 s = 1.95 | D both | 10.3 / 21.4 / 38.5 / 55.0 | 0 / 0 | 18.3 / 21.4 | 10.3 | 98 % | 0.6 % | 0.72 |
| E2 s = 2.10 | A original | 10.2 / 22.5 / 56.9 / 79.5 | 0 / 0 | 54.0 / 79.5 | 10.2 | 97 % | 0.6 % | 0.73 |
| E2 s = 2.10 | B cycle detection + branch | 10.1 / 22.8 / 55.0 / 78.9 | 0 / 0 | 54.6 / 78.9 | 10.1 | 97 % | 0.6 % | 0.73 |
| E2 s = 2.10 | C sequential encounters | 10.1 / 22.6 / 46.9 / 61.2 | 0 / 0 | 26.8 / 36.5 | 10.1 | 98 % | 1.8 % | 0.71 |
| E2 s = 2.10 | D both | 10.1 / 22.8 / 50.1 / 113.6 | 1 / 1 | 27.3 / 37.1 | 10.1 | 98 % | 1.9 % | 0.71 |
| E3 s = 8.20 | A original | 9.8 / 21.4 / 48.5 / 73.3 | 0 / 0 | 18.5 / 73.3 | 9.8 | 97 % | 0.5 % | 0.73 |
| E3 s = 8.20 | B cycle detection + branch | 9.8 / 21.6 / 48.0 / 92.8 | 3 / 0 | 19.1 / 92.8 | 9.8 | 97 % | 0.9 % | 0.74 |
| E3 s = 8.20 | C sequential encounters | 9.7 / 21.2 / 48.7 / 71.8 | 0 / 0 | 9.6 / 71.8 | 9.7 | 98 % | 0.6 % | 0.72 |
| E3 s = 8.20 | D both | 9.8 / 21.5 / 68.3 / 99.9 | 4 / 0 | 9.5 / 93.1 | 9.8 | 96 % | 3.0 % | 0.73 |
| E4 s = 8.40 | A original | 10.1 / 21.3 / 45.7 / 58.7 | 0 / 0 | 19.1 / 20.1 | 10.0 | 97 % | 0.6 % | 0.72 |
| E4 s = 8.40 | B cycle detection + branch | 10.0 / 20.9 / 45.2 / 52.9 | 0 / 0 | 18.5 / 20.8 | 10.0 | 97 % | 0.7 % | 0.72 |
| E4 s = 8.40 | C sequential encounters | 9.9 / 21.0 / 46.5 / 101.0 | 1 / 1 | 9.5 / 10.6 | 9.9 | 96 % | 1.8 % | 0.71 |
| E4 s = 8.40 | D both | 9.9 / 20.8 / 44.6 / 51.7 | 0 / 0 | 9.2 / 9.5 | 9.9 | 97 % | 0.7 % | 0.71 |
| F lap boundary | A original | 10.0 / 21.8 / 35.9 / 41.1 | 0 / 0 | 39.8 / 41.1 | 10.0 | 97 % | 0.8 % | 0.69 |
| F lap boundary | B cycle detection + branch | 10.0 / 21.7 / 35.4 / 106.3 | 1 / 1 | 39.4 / 40.7 | 9.9 | 98 % | 2.0 % | 0.69 |
| F lap boundary | C sequential encounters | 10.0 / 21.5 / 23.5 / 39.9 | 0 / 0 | 20.1 / 21.1 | 10.0 | 97 % | 0.7 % | 0.69 |
| F lap boundary | D both | 10.1 / 21.6 / 23.6 / 115.0 | 1 / 1 | 20.1 / 20.7 | 10.1 | 96 % | 1.8 % | 0.69 |
| G1 +0.08 on R 1 m left | A original | 10.2 / 21.4 / 32.2 / 41.1 | 0 / 0 | 36.1 / 41.1 | 10.2 | 97 % | 0.7 % | 0.72 |
| G1 +0.08 on R 1 m left | B cycle detection + branch | 10.1 / 21.2 / 31.8 / 40.4 | 0 / 0 | 35.7 / 40.4 | 10.1 | 98 % | 0.6 % | 0.72 |
| G1 +0.08 on R 1 m left | C sequential encounters | 10.3 / 20.4 / 24.8 / 30.4 | 0 / 0 | 18.1 / 21.1 | 10.3 | 98 % | 0.6 % | 0.71 |
| G1 +0.08 on R 1 m left | D both | 10.2 / 20.4 / 26.7 / 111.7 | 1 / 1 | 18.2 / 20.1 | 10.2 | 96 % | 1.9 % | 0.71 |
| G2 −0.08 on R 1 m right | A original | 9.9 / 22.4 / 29.9 / 106.4 | 2 / 2 | 16.7 / 18.4 | 9.9 | 97 % | 3.4 % | 0.71 |
| G2 −0.08 on R 1 m right | B cycle detection + branch | 10.0 / 22.3 / 25.4 / 35.7 | 0 / 0 | 16.6 / 18.0 | 10.0 | 97 % | 0.6 % | 0.71 |
| G2 −0.08 on R 1 m right | C sequential encounters | 10.0 / 22.4 / 26.3 / 101.9 | 1 / 1 | 8.4 / 12.0 | 10.0 | 97 % | 1.8 % | 0.70 |
| G2 −0.08 on R 1 m right | D both | 10.0 / 22.1 / 27.1 / 36.2 | 0 / 0 | 8.4 / 9.6 | 10.0 | 97 % | 0.6 % | 0.70 |

Stops:

- C R 1 m right, s = 6.6 / A original, run 1: c193: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.0201 m); schedule correction skipped: time budget 0.08 s would be exceeded
- C R 1 m right, s = 6.6 / A original, run 2: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- C R 1 m right, s = 6.6 / A original, run 3: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- E3 s = 8.20 / A original, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: time budget 0.08 s would be exceeded
- E3 s = 8.20 / A original, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: time budget 0.08 s would be exceeded
- E3 s = 8.20 / A original, run 3: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: time budget 0.08 s would be exceeded
- G2 −0.08 on R 1 m right / A original, run 1: c230: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.09409 m); schedule correction skipped: time budget 0.08 s would be exceeded
- G2 −0.08 on R 1 m right / A original, run 2: c188: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.01266 m); schedule correction skipped: time budget 0.08 s would be exceeded
- C R 1 m right, s = 6.6 / B cycle detection + branch, run 1: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- C R 1 m right, s = 6.6 / B cycle detection + branch, run 2: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- C R 1 m right, s = 6.6 / B cycle detection + branch, run 3: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- E3 s = 8.20 / B cycle detection + branch, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: cycle budget 0.09 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / B cycle detection + branch, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: cycle budget 0.09 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / B cycle detection + branch, run 3: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: cycle budget 0.09 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- C R 1 m right, s = 6.6 / C sequential encounters, run 1: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- C R 1 m right, s = 6.6 / C sequential encounters, run 2: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- C R 1 m right, s = 6.6 / C sequential encounters, run 3: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- E3 s = 8.20 / C sequential encounters, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: time budget 0.08 s would be exceeded
- E3 s = 8.20 / C sequential encounters, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: time budget 0.08 s would be exceeded
- E3 s = 8.20 / C sequential encounters, run 3: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: time budget 0.08 s would be exceeded
- C R 1 m right, s = 6.6 / D both, run 1: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- C R 1 m right, s = 6.6 / D both, run 2: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- C R 1 m right, s = 6.6 / D both, run 3: c205: solver_failure [cause initialization]: solver returned termination status LOCALLY_INFEASIBLE with primal status INFEASIBLE_POINT
- E3 s = 8.20 / D both, run 1: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: cycle budget 0.09 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / D both, run 2: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: cycle budget 0.09 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]
- E3 s = 8.20 / D both, run 3: c118: segment_assignment_invalid [cause segment_assignment]: invalid segment assignment: 1 knot(s) outside their assigned segments (largest overshoot 0.08675 m); schedule correction skipped: cycle budget 0.09 s would be exceeded [attempts: warm → segment_assignment_invalid, left → segment_assignment_invalid]

