# Stationary-obstacle avoidance on the multi-curvature quarter track (Milestone 3B, experimental)

**Status: experimental.** One stationary circular obstacle on `quarter_track()`, with a
1 s horizon. This is an engineering application of standard obstacle-avoidance MPC. It
carries no safety guarantee, has had no real-world validation, and claims no novelty.
Results are in `benchmarks/results/multicurvature_obstacle.md`, and the figures are
`docs/figures/multicurvature_obstacle_trajectory.png` (whole track, zooms on the R 1 m
bends, and the predicted plan at the encounter) and `_signals.png` (steering, speed and
obstacle distance).

**Summary (N = 10, 3 runs × 250 cycles per scenario):**

- **Completed.** With the 80 ms correction budget, these completed all 3 runs, with no
  contact and no physical-track violation:
  - the obstacle on the R = 1 m left bend;
  - the R = 1 m right bend of the S-bend;
  - the lap boundary, with repeated encounters on laps 2 and 3;
  - both off-centre cases.

  The R = 2 m curve completed 2 of 3 runs.
- **Plant clearances.** Over all accepted runs, the closest plant approach left 2.6–14.4 cm
  between the disks. The vehicle entered the 3 cm buffer by at most 0.41 cm. The plant
  always stayed at least 4.4 cm inside the physical track.
- **Obstacles near segment boundaries** (E1–E4) stopped in most runs with
  `:segment_assignment_invalid`, after the 80 ms budget skipped a needed reassignment.
  - Without the budget, three of the four complete.
  - **E3** (s = 8.20, just before the R −1 → R 2 boundary) fails on its second lap: a
    knot's segment assignment oscillates with period 2 and does not converge within 8
    reassignments (§8). This is unresolved.
- **Timing.**
  - Encounter cycles (two candidate solves plus reassignments) have medians of 26–110 ms.
  - Ordinary cycles have medians of 12–23 ms, against about 11 ms without the budget in
    the same process (§9).
  - 47 of the 54 cycles above 80 ms were encounter cycles.
- **Strict buffer** (`obstacle_acceptance = :separation`): only the R = 2 m case
  completes. The R = 1 m bends are rejected within 15 cycles.
- **N = 20:** only the lap-boundary case completes (one run). It is not real-time capable
  here.

## 1. What was extended

The collision formulation of Milestone 3A is unchanged.
- **Constraint:** `D = r_e + r_o + d_b` and `(|p_k − o|² − D²)/(2D) ≥ 0` at the knots and
  at 3 interior samples per interval.
- **Positions:** exact segment-local Cartesian positions of the decision variables.
- **Obstacle:** `(ox, oy, D)` as JuMP Parameters.
- **Other settings:** the objective, weights, tolerances, vehicle and track are unchanged.

What changed is the obstacle relevance and association (no longer a single projection),
encounter tracking across laps, the curvature-aware initialisation, and the diagnostics.
The NLP is unchanged.

## 2. Obstacle placement

`obstacle_on_track(track, s, ey; radius)` places an obstacle at `frenet_to_cartesian(track,
s, ey)`, i.e. with the track's actual geometry. The obstacle is fixed in Cartesian space
from then on.

## 3. Relevance and projection

**Problem.** In 3A, `obstacle_relevant` projected the obstacle centre once with
`cartesian_to_frenet` (nearest progress to a hint, within the track width). A failed or
ambiguous projection dismissed the obstacle for initialisation.

**Replacement** (`obstacle_associations`), computed from the obstacle's Cartesian
position over the unwrapped progress window `[s − D − E, s + v̄·N·dt + D + E]`, where
`v̄ = max(v, v_max)` and `E = ey_max`:

1. For every lap-aware segment overlapping the window, find the analytically closest
   centreline point within the overlap:
   - lines: the orthogonal projection, clamped;
   - arcs: the angle of the obstacle around the arc centre, both branches, plus the
     overlap end points.
2. Keep the segment if that point is within `E + D` of the obstacle centre. This is
   conservative: an obstacle whose centre lies outside the track, but whose disk reaches
   the planning corridor, is relevant.
3. The reported association `(s, ey, dist)` is the segment's own closest point. It is not
   clamped by the window, so an obstacle at the window's edge gets its true local offset.
4. Group the results into separate passes (progress gaps larger than `2(E + D)`), one
   association each. Several associations are reported where several parts of the track
   are within reach, e.g. both legs of a hairpin (tested).

**How it is used:**
- `obstacle_relevant` is true when any association exists.
- The candidate initialisation needs an association **ahead** of the vehicle
  (`s ≥ s_vehicle − D`). An obstacle just behind the start no longer triggers two solves.
- **Forward unwrapped progress** is preserved: the same physical obstacle appears at
  `s_o`, `s_o + L`, `s_o + 2L` on successive laps (tested).
- **The collision constraints are active in every solve**; only the choice of
  initialisation depends on relevance.

**Encounters and side retention.** The chosen side is remembered with the association's
unwrapped progress. A new encounter, which triggers two candidate solves, is detected
when the association moves by more than `2(D + E)` (the same obstacle one lap later).
- **Observed:**
  - F: candidate encounters at cycles 84 and 177 (laps 2 and 3);
  - B: at cycles 1, 97 and 193.
- **Side switch.** With the off-centre obstacle G1 (4 cm gap on the left, 20 cm on the
  right), the warm-start retry at cycles 99 and 195 was initialised from the remembered
  left side, but converged to the right. The side is retained by the initialisation, not
  enforced as a constraint.

## 4. Passing-side initialisation on curves

`side_guess` keeps the 3A structure: a lateral target window, pure pursuit, and an
event-aware rollout of piecewise-constant inputs, hence a dynamically consistent guess.
It is made curvature-aware:

- **Ramps.**
  - Approach: `clamp(s_o − D − s, obstacle_ramp_min, obstacle_ramp)`, so it is shortened
    when the obstacle is already close.
  - Departure: `obstacle_ramp`.
  - Defaults: 1.5 m and 0.3 m, configurable. The 3A value is kept: on the R = 1 m bends,
    the cases B, C, G1 and G2 complete with it.
- **Speed.** The acceleration is reduced where needed so that, over the next second, the
  speed stays below `sqrt(0.9·ay_max/|κ_path|)` and stays reachable by braking at 80 % of
  `|a_min|`.
  - `κ_path = κ/(1 − κ·ey_target)` is the curvature of the offset parallel path, so a
    guess on the inside of a bend is slower.
  - The extra curvature of the ramp itself is neglected.
  - On straight segments the 3A acceleration is unchanged.
- **Look-ahead.** Pure pursuit with look-ahead `clamp(0.5·v, 0.5, 2) m`, but at most the
  smallest radius within the next 2 m, so the guess does not cut into a tight bend or
  across a curvature transition early.
  - On a constant-curvature path, pure pursuit reproduces that curvature exactly.
  - The steering is clipped to the steering and lateral-acceleration limits.
- **Diagnostics.** Each guess records its clearance above `D` at the knots and its track
  clearance (`result.obstacle.guesses`).

**Passage test.** `passage_sides` (`ey_o ± D` against `±E`) is labelled a heuristic. On a
single segment it is the exact closest approach of a constant-offset path, since lines
and concentric arcs are parallel. It says nothing about reaching that offset in time, or
about steering and lateral-acceleration limits.

## 5. Failure classification

`result.obstacle.failure_cause` distinguishes:

| cause | meaning |
|:--|:--|
| `:infeasible_geometry` | geometrically blocked (lateral-gap heuristic) and the solver failed |
| `:dynamically_infeasible` | both sides open, every solve failed, and no side guess (a dynamically consistent rollout) stayed free of contact on the track. Suspected, not proven |
| `:solver_convergence` | every solve failed although some side guess was contact-free and on the track |
| `:initialization` | the solver failed with only one initialisation tried |
| `:segment_assignment` | only invalid segment assignments (e.g. a reassignment skipped by the budget, or the E3 oscillation) |
| `:constraint_violation`, `:collision`, `:separation` | an imposed constraint violated, contact, or a buffer intrusion found by the dense evaluation |

The public statuses are unchanged.

**Single-cycle checks:**
- an obstacle 0.9 m ahead at 5 m/s on the 6 m straight: `:dynamically_infeasible`; the
  guesses come 7.9 cm too close;
- radius 0.30 m on the R = 1 m bend (`D = 0.38 m > E`): `:infeasible_geometry`;
- the same speed with 2.9 m of room: success.

## 6. Scenarios

**Common setup.**
- **Configuration:** N = 10, `dt = 0.1`, `track_margin = 0.05`, `schedule_corrections = 8`
  (80 ms budget), `interior_samples = 0`, `strict_corridor = false`, `ego_radius = 0.05`,
  `obstacle_buffer = 0.03`, `obstacle_samples = 3`, `obstacle_acceptance = :collision`.
- **Obstacle:** radius 0.10 m, so `D = 0.18 m` and contact at 0.15 m.
- **Start:** `[0, 0, 0, 0.5]`.
- **Track:** `quarter_track()` unchanged, `L = 34.85 m`.

**Scenarios:**
- A: no obstacle.
- B: R 1 m left bend, s = 3.5.
- C: R 1 m right bend (S-bend), s = 6.6.
- D: R 2 m curve, s = 9.7.
- E1/E2: s = 1.95 / 2.10, either side of the straight → R 1 m boundary.
- E3/E4: s = 8.20 / 8.40, either side of the R −1 → R 2 boundary.
- F: lap boundary, s = L − 0.3.
- G1/G2: B and C offset by +0.08 / −0.08 m.
- G3: blocked, radius 0.30 at s = 3.5.

## 7. Results

Per scenario: the outcome of each run with the 80 ms budget (laps, or the stop cycle),
the plant's closest approach to the obstacle (margin to contact / to `D` in metres), and,
for scenarios that stopped, one run without the budget.

| scenario | runs (budget 80 ms) | closest plant approach [m] (to contact / to D) | without budget |
|:--|:--|:-:|:--|
| A no obstacle | 2.58 ×3 (reproduces 2B.4) | – | |
| B R 1 m left | 2.46 ×3 | 0.1795 (+0.0295 / −0.0005) | |
| C R 1 m right (S-bend) | 2.53 ×3 | 0.1795 (+0.0295 / −0.0005) | |
| D R 2 m | 2.58; 2.58; stop c178 | 0.2942 (+0.1442 / +0.1142) | 2.58 |
| E1 s 1.95 | stop c95; c95; c46 | 0.1759 (+0.0259 / −0.0041) | 2.54 |
| E2 s 2.10 | stop c95; c95; c191 | 0.1786 (+0.0286 / −0.0014) | 2.52 |
| E3 s 8.20 | stop c25; c25; c118 | 0.1821 (+0.0321 / +0.0021) | **stop c118** (oscillation, §8) |
| E4 s 8.40 | stop c215; c216; c25 | 0.1771 (+0.0271 / −0.0029) | 2.52 |
| F lap boundary | 2.58 ×3 | 0.1855 (+0.0355 / +0.0055) | |
| G1 B + 0.08 | 2.48 ×3 | 0.1794 (+0.0294 / −0.0006) | |
| G2 C − 0.08 | 2.46 ×3 | 0.1800 (+0.0300 / −0.0000) | |
| G3 blocked | 0.08 laps ×3: decelerates to a stop before the obstacle and waits (750/750 accepted) | 0.8135 | |

- **Every budget stop** is `:segment_assignment_invalid` with
  `schedule correction skipped: time budget`. Which run stops varies between repetitions
  and between benchmark runs, with the machine's speed (§9).
- **Physical track (plant).** Minimum clearance 4.4–4.7 cm; no violations of the track,
  inputs or lateral acceleration (max |ay| = 4.00 m/s², the limit).
- **Planning corridor (diagnostic).** Down to −6.2 mm, as with the obstacle-free
  controller.
- **Predicted plans** (dense evaluation, 50 samples per interval): at least 1.89 cm
  above contact, and up to 1.11 cm inside the buffer between constrained points.
- **Plant buffer intrusions:** up to 0.41 cm (E1).
- **G3 (blocked).** The closed loop stops before the obstacle: the first encounter is
  solved with both candidates, and warm starts afterwards. This is an outcome of the NLP,
  not a designed braking planner. The single-cycle variant at 2 m/s and 1.1 m from the
  obstacle is classified as `:infeasible_geometry`.

**Strict buffer** (`:separation`):
- B: rejected at cycle 2, with a planned dip to 0.169 m between samples.
- C: rejected at cycle 15.
- D: 2.58 ×3 (it never enters the buffer).

**N = 20** (one run each):
- A: 2.39 laps.
- F: completes 2.39 laps.
- B, C and D stop at cycles 101, 130 and 30 (`:segment_assignment`, budget).
- p99 90–139 ms, and 2–6 cycles above 100 ms per obstacle run.

## 8. Unresolved: reassignment oscillation (E3)

At the R −1 → R 2 boundary (s = 43.133 m, lap 2), with the obstacle at s = 8.20 (just
before the boundary), the replayed failing cycle shows one knot alternating between
s = 43.015 m and s = 43.219 m for every number of reassignments from 1 to 8:
- assigned to the R 2 m arc, it is optimised back onto the R −1 m arc;
- assigned to the R −1 m arc, it is optimised onto the R 2 m arc.

The bounded reassignment has no cycle detection, and the obstacle constraint keeps the
knot near the boundary. Possible fixes, not implemented, need approval:
- detect a repeated assignment and pin that knot with a single membership bound;
- assign the boundary knot by the midpoint of its two candidate positions.

## 9. Runtime

All timings come from one Windows laptop, Julia 1.11.5, one thread, with everything in
`solve!` included. N = 10, primary runs:

| scenario | median / p95 / p99 / max [ms] | > 80 / > 100 ms (of ≈750) | encounter cycles: median / max [ms] | ordinary median [ms] |
|:--|:-:|:-:|:-:|:-:|
| A no obstacle | 10.9 / 23.4 / 29.3 / 51.1 | 0 / 0 | – | 10.9 |
| B | 17.8 / 41.8 / 60.1 / 87.2 | 3 / 0 | 67.7 / 87.2 | 17.7 |
| C | 11.9 / 36.8 / 51.3 / 184.5 | 3 / 2 | 25.5 / 184.5 | 11.8 |
| D | 16.1 / 37.7 / 52.5 / 98.6 | 2 / 0 | 53.0 / 98.6 | 15.8 |
| F | 21.2 / 48.3 / 83.7 / 152.0 | 8 / 5 | 101.3 / 118.0 | 21.1 |
| G1 | 20.9 / 47.7 / 73.7 / 136.9 | 7 / 4 | 89.6 / 136.9 | 20.7 |
| G2 | 19.7 / 46.5 / 59.5 / 123.9 | 2 / 2 | 36.7 / 123.9 | 19.6 |

**Breakdown of the 54 cycles above 80 ms** in the primary set:
- 47 were encounter cycles: two candidate solves, often with reassignment solves, 2–4 MB
  allocated;
- 5 were dominated by garbage collection (55–193 ms of GC);
- 2 were ordinary cycles with two solves, at 81 and 96 ms.

**Machine variability.** In the same process, the no-budget reruns of D and E1–E4 had
medians of 10.3–11.1 ms, against 16–23 ms for the same scenarios in the primary set. In
earlier runs of this benchmark during the milestone, the no-obstacle baseline varied
between 10.9 and 14.1 ms; it was 6.1 ms in Milestone 2B.4.
- Budget-driven stops depend on this, which is why they move between runs.
- The no-budget runs give the algorithmic outcome.

**Compilation:** the first construction and solve take 2.8 s / 7.3 s. There is no
real-time guarantee.

## 10. Limitations and next steps

- **Scope:** one stationary circular obstacle and a rear-axle disk proxy, on one track (the
  tests also cover the 3A straight track and a hairpin geometry).
- **No certificate between constrained points:** collisions are detected by the dense
  evaluation, and buffer intrusions are reported.
- **Segment-boundary obstacles** need reassignments.
  - The 80 ms budget often skips them, rejecting the cycle.
  - E3 shows a reassignment oscillation that also fails without the budget.

  The most useful next step is cycle detection in the reassignment (§8).
- **Encounter cost:** encounter cycles (two candidates) often exceed 80 ms. One option is
  to solve one candidate, and the second only on failure or when the obstacle is near the
  centreline.
- **N = 20** is not real-time capable here.
- **Heuristics:** the relevance window, the passage test and `:dynamically_infeasible`
  are heuristics. Ipopt finds local solutions.
- **Side retention** is by initialisation only; one retry switched sides (G1).
- **Braking:** there is no braking planner. Braking happened in G3, but was not designed.
