# Nonlinear vs linearized MPC: consolidated benchmark comparison

This page consolidates existing benchmark reports, plus the Milestone 5 example runs. It
**does not merge timings from different sessions into one experiment**.

**Sources.** Each table below states its source, recorded in one of these sessions on one
Windows laptop (Intel i9-13900HX, Julia 1.11.5, one Julia thread):

| label | source | recorded | controllers in the same session |
|---|---|---|---|
| **S-4A** | `benchmarks/results/linear_mpc.md` (`benchmarks/linear_mpc.jl`) | Milestone 4A, commit `4df19a4` | LMPC and NMPC (Frenet, N = 10/20), straight track |
| **S-4BC** | `benchmarks/results/linear_multicurvature_obstacle.md` (`benchmarks/linear_multicurvature_obstacle.jl`) | Milestone 4BC, commit `992abf0` | LMPC and NMPC (Cartesian-endpoint), quarter track, obstacles |
| **S-M5** | standalone examples `examples/*.jl` (default settings) | Milestone 5 (this release preparation) | one controller per process |

**Labels used below:**
- *recorded*: copied from a report;
- *newly run*: S-M5;
- *derived*: a difference or ratio computed from recorded numbers of **one** session.

No controller was retuned for this comparison. Both use the same vehicle, tracks,
obstacle, start states, horizon, period, objective weights and limits, as described in the
reports.

## 1. Straight track, no obstacle (S-4A, recorded)

400 m straight, 1.0 m wide, N = 10, 250 cycles.
- LMPC: the straight track model, warm start.
- NMPC: Frenet RK4 model, Ipopt.

| scenario | progress LMPC / NMPC [m] | RMS ey LMPC / NMPC [m] | cycle median LMPC / NMPC [ms] | cycle max LMPC / NMPC [ms] |
|---|---|---|---|---|
| A nominal progression from 1 m/s | 123.4 / 123.4 | 0 / 0 | 0.7 / 3.7 | 3.8 / 13.6 |
| B lateral/heading recovery | 124.1 / 124.1 | 0.0355 / 0.0355 | 0.4 / 3.2 | 17.8 / 10.1 |
| C near the speed limit | 125.0 / 125.0 | 0.0125 / 0.0127 | 0.4 / 3.5 | 2.0 / 14.0 |

**Derived (S-4A):**
- The trajectories coincide.
- At sampled states the LMPC plan's original objective was within 6e-4 of Ipopt's.
- The LMPC median cycle is 5–9× shorter.

**Also in S-4A:**
- every cycle of both controllers was accepted, with no violations;
- at N = 20 the NMPC median was 5.9–6.0 ms and the LMPC's 0.8 ms.

## 2. Quarter track, no obstacle (S-4BC stage A, recorded)

`quarter_track()`, N = 10, 250 cycles.

| | LMPC (3 runs, identical) | NMPC (1 run) |
|---|---|---|
| progress | 2.34 laps (81.5 m) | 2.58 laps (90.0 m) |
| first lap | 10.6 s | 9.8 s |
| RMS / max abs ey (plant) | 0.135 / 0.296 m | 0.202 / 0.304 m |
| min physical track clearance | 0.054 m | 0.046 m |
| cycle median / p99 / max | 3.7 / 11.8 / 20.8 ms | 6.2 / 14.6 / 23.2 ms |
| solves per cycle (mean / max) | 1.95 / 3 QPs | 1.13 / 2 NLPs |
| plant violations, failed validations | 0, 0 | 0, 0 |

**Derived (S-4BC):** the NMPC made 10 % more progress. The LMPC stays inside the planning
corridor (0.30 m). The NMPC leaves it between knots by up to 4 mm (its constraints apply
at the knots only).

## 3. Stationary obstacle

### Straight track, Milestone 3A geometry (S-4BC stage B, recorded)

Obstacle at (8, 0) with r = 0.10 m; ego radius 0.05 m, buffer 0.03 m (contact at 0.15 m,
D = 0.18 m); N = 10.

| | LMPC M = 3 (3 runs) | NMPC M = 3 |
|---|---|---|
| distance in 250 cycles | 122.9 m | 122.9 m |
| min clearance above contact / above D (plant) | 0.0299 / −0.0001 m | 0.0243 / −0.0057 m |
| cycle median / p99 / max | 1.1 / 8.0 / 14.2 ms | 5.8 / 15.0 / 45.7 ms |

### Quarter track, Milestone 3B scenarios (S-4BC stage C, recorded)

Ten scenarios: both R = 1 m bends, the R = 2 m bend, both sides of two curvature
transitions, the lap boundary, and two offset obstacles. N = 10, M = 3, 250 cycles.

| | LMPC (3 runs per scenario) | NMPC (1 run per scenario) |
|---|---|---|
| completed runs | **30 / 30** | **9 / 10** (E3 stops at cycle 118: segment-assignment correction skipped by the 80 ms budget) |
| progress | 2.22–2.32 laps | 2.46–2.58 laps |
| min clearance above contact (plant) | 0.026 m | 0.026 m |
| deepest entry into the buffer (plant) | 3.7 mm | 4.1 mm |
| plant violations / failed validations of accepted plans | 0 / 0 | 0 / 0 |
| cycle median (per scenario) | 4.7–5.4 ms | 9.9–10.8 ms |
| cycle p99 (per scenario) | 13.6–19.5 ms | 22.6–46.4 ms |
| cycle max | 24.5 ms, except three cycles of 105–137 ms during 100–128 ms GC pauses | 318 ms (E3) |
| cycles > 100 ms | 3 of 7 500 | 1 of 2 368 |
| solves per cycle | ≈ 2 QPs (max 5), OSQP median 350–400 iterations per QP | ≈ 1.1 NLPs (max 5), Ipopt median 16 iterations |

**Derived (S-4BC):**
- The NMPC made 7–10 % more progress in every scenario it completed.
- With M = 0 (knots only), the LMPC failed 4 of 10 scenarios: three collisions between
  knots, detected and rejected by the dense check, and one measured state inside D. The
  NMPC was run with M = 3 only.

## 4. Prediction accuracy (S-4BC, recorded)

| | LMPC | NMPC (Cartesian-endpoint) |
|---|---|---|
| plan knots vs exact rollout of the plan's inputs | ≤ 1 mm (the acceptance tolerance) | ≈ 1e-5 mm |
| first predicted step vs plant one period later | mean 0.0015–0.002 mm, max ≤ 0.06 mm | mean ≈ 1e-6 mm, max ≈ 1e-5 mm |

The NMPC predicts with the exact map. The LMPC's prediction is the affine model, accepted
only within the mismatch tolerance.

## 5. Objective values (S-4BC, recorded)

The original nonlinear objective (lower is better), evaluated on the exact rollout of each
plan's inputs in true track coordinates, at the same states and previous input. The QP's
surrogate objective is not compared.

| situation | LMPC plan used in closed loop − fresh NMPC plan |
|---|---|
| straight track with the obstacle | 0.001–0.015 |
| quarter track, no obstacle | 0.44–1.56 |
| quarter track, obstacle on the R = 1 m bend | 0.26–1.11 |

The objective values themselves are about −3 to −10. The LMPC's first validated
candidate is feasible, but not a converged local optimum.

## 6. Newly run: standalone examples (S-M5)

Default (budgeted) configurations. Each process first ran a discarded warm-up closed loop
of the same configuration.

| example | outcome | progress | min track clearance | obstacle clearance above contact / D | cycle median / p99 / max |
|---|---|---|---|---|---|
| A LMPC straight | success, 250 cycles | 124.06 m | 0.15 m (starts at ey = 0.35 m, half-width 0.5 m) | – | 0.41 / 1.93 / 6.77 ms |
| B LMPC quarter | success, 250 cycles | 2.34 laps | 0.054 m | – | 9.36 / 29.5 / 39.4 ms |
| C LMPC obstacle (scenario B) | success, 250 cycles | 2.26 laps | – | 0.0336 / 0.0036 m | 12.45 / 39.2 / 62.7 ms |
| D NMPC obstacle (scenario B) | success, 250 cycles | 2.46 laps | 0.047 m | 0.0295 / −0.0005 m | 23.2 / 55.2 / 63.9 ms |

- The outcomes, progress and clearances reproduce S-4BC: identical laps and clearances
  for B, C and D, and the S-4A recovery scenario for A.
- The **timings do not**. In this session the machine ran about 2–2.5× slower (for
  example B: median 9.4 ms against 3.7 ms in S-4BC). Session-to-session timing variation of
  this size has been seen throughout the project.
- Two further observations in S-M5, which the examples now handle or report:
  - Before the repeated warm-up was added, budgeted runs of B and C failed in their first
    cycles: code paths compiled during the measured run exceeded the 80 ms budget.
  - One budgeted run of D failed at cycle 133: a segment-assignment correction was
    skipped by the 80 ms budget on the slower machine. A later run of the same
    configuration completed.

  Budget-dependent outcomes are expected: use `--no-time-budget` for
  machine-independent runs.

## 7. The measured trade-off

- **Speed:** the linearized MPC was faster. Its median cycle was about 2× shorter on the
  quarter track and 5–9× shorter on the straight track, within each session.
- **Reliability:** it completed all tested obstacle scenarios (30/30 runs). The nonlinear
  MPC completed 9 of 10 at N = 10.
- **Progress:** the nonlinear MPC made more progress on the quarter track (7–10 %), and
  its individual plans had better objective values.
- **Approximation:** the linearized MPC relies on several mechanisms (see
  `docs/linearized_multicurvature_collision_avoidance.md`):
  - convex approximations (affine track coordinates, linearised lateral acceleration,
    supporting halfspaces);
  - QP-side tightenings and softened inequalities;
  - a trust region with successive linearisation;
  - mandatory nonlinear validation of every plan.
- **No guarantees.** Neither controller provides a hard real-time guarantee, a
  continuous-time collision certificate, recursive feasibility or global optimality.
  - Both accept plans that enter the buffer between constrained points, without contact.
  - Both can fail a cycle and stop the closed loop; there is no fallback controller.
  - The occasional cycles above 100 ms coincided with garbage-collection pauses (LMPC) or
    repeated corrections (NMPC).

## 8. Final milestone: dynamic bicycle NMPC (S-final, recorded)

This section has a different scope from sections 1–7:
- a different vehicle model (six-state dynamic bicycle, synthetic tyres), so its results
  are not comparable one-to-one with the kinematic rows above;
- starting conditions: rear axle at `s = 0`, 1.0 m/s;
- N = 10, 250 cycles, no wall-clock budget.

Source: `benchmarks/results/dynamic_showcase.md` (3 runs per configuration, commit
`b982f1b`).

| configuration | completed | laps | median cycle time [ms] | ≤ 100 ms | obstacles |
|:--|:--|--:|--:|--:|:--|
| dynamic NMPC, no obstacle | 3 of 3 | 2.56 | 88–98 (slow machine state) | 55–73 % | – |
| dynamic NMPC, six obstacles | 3 of 3 | 2.29 | 62 | 81–83 % | no contact; up to 1.06 cm inside the 3 cm buffer |
| kinematic NMPC (Cartesian endpoint), no obstacle | 3 of 3 | 2.59 | 6.6–7.0 | 100 % | – |

The dynamic NLP costs roughly an order of magnitude more per cycle than the kinematic one.
It does not run within the 0.1 s period in every cycle on this laptop. The linearized MPC
has no dynamic-model counterpart.
