# Segment-reassignment reliability and encounter computation (Milestone 3B.1, experimental)

**Status: experimental, opt-in. The defaults are unchanged.** Results are in
`benchmarks/results/reassignment_reliability.md`. The vehicle, track, objective, collision
constraints, tolerances and the 3B scenarios are unchanged (N = 10, `dt = 0.1 s`, 5 cm
margin, obstacle radius 0.10 m, ego radius 0.05 m, buffer 0.03 m, 3 interior obstacle
samples).

**Summary**

- **E3 diagnosed and resolved, without a budget.**
  - The cause is a period-two oscillation in **one knot** (knot 10).
  - Cycle detection stops it after 3 NLP solves instead of 9.
  - Branch resolution finds a valid solution: knot 10 bounded to the R −1 m arc. The other
    branch (R 2 m arc) ends in an Ipopt failure.
  - E3 then completes 2.60 laps. Without the new options it stops at cycle 118.
- **E3 is not resolved within the timing budgets.** Detection plus branch resolution needs
  5 NLP solves (≈ 95–265 ms measured), so the 80 ms correction budget (or the 90 ms cycle
  budget) skips it. E3 stays at cycle 118 (one run at cycle 64) in every budgeted run.
- **Sequential encounters halve the encounter-cycle time without changing outcomes.**
  Without a budget, every scenario chose the same side with the same objective, and gave
  the same laps and clearances as the two-candidate strategy. Encounter-cycle medians:
  - B: 35.8 → 17.9 ms;
  - D: 40.5 → 19.3 ms;
  - E2: 55.1 → 26.5 ms;
  - F: 39.9 → 20.4 ms.
- **The optional 90 ms cycle budget trades failures, it does not remove them.** It skips
  the warm-start retry that rescues the R = 1 m right-bend case C at cycle 205, so C fails
  in every run with it, and completes in every run without it.
- **No regressions:** with the options off, all 3B scenarios behave as before. No variant
  produced a collision or a physical-track violation.

## 1. Why reassignment cycles occur

**Background.** In the Cartesian-endpoint NLP each knot uses the analytic geometry of its
assigned segment (2B.4). After a solve, any knot outside its segment is reassigned from the
optimised progress, and the problem is solved again, up to `schedule_corrections` times.

**What goes wrong.** This is a fixed-point iteration on a discrete assignment, and nothing
guarantees convergence. Near a boundary, the knot's optimal position can depend on which
side's geometry it is given:
- with the before-boundary geometry, the optimiser moves the knot past the boundary;
- with the after-boundary geometry, it moves the knot back.

The assignment then alternates.

## 2. E3 diagnosis

**State.** Obstacle at s = 8.20 m, failing second-lap cycle 118, state
`[40.8162, −0.2797, −0.1613, 2.3058]`. The R −1 → R 2 boundary is at s = 43.133 m. The
replay uses the left-side guess and `solve_once!` with 8 reassignments.

| solve | assignment of knots 2…11 | knot outside | its progress | objective | Ipopt it. | knot clearance above D |
|--:|:--|:-:|:-:|:-:|:-:|:-:|
| 1 | all 14 (R −1) | 11 | 43.213 | −4.1623 | 26 | +0.013 |
| 2 | knot 11 → 15 | 10 | 43.219 | −4.2492 | 41 | +0.031 |
| 3 | knot 10 → 15 | 10 | 43.015 | −4.2645 | 46 | +0.013 |
| 4–9 | alternates between solves 2 and 3 | 10 | 43.219 / 43.015 | −4.2492 / −4.2645 | 36 / 46 | +0.031 / +0.013 |

- **One knot.** Only knot 10 changes after solve 2. Assigned to the R −1 arc it ends at
  43.219, past the boundary; assigned to the R 2 arc it ends at 43.015, before it.
- **Every solve converged** (dynamics residual ≤ 8.5e-9). Each solution was rejected only
  because a knot lay outside the segment its equations used.

## 3. Cycle detector

With `reassignment_cycle_detection = true`, `solve_once!` keeps the full lap-aware
assignment vector of every solve in the call. Before each reassignment, the proposed
vector is compared with all earlier ones (`detect_assignment_cycle`).

- **An exact repeat is a cycle.** The reassignment depends only on the optimised progress,
  so revisiting a configuration means the iteration is cycling.
- **What is recorded:** the cycle length (2 for A, B, A; longer cycles within the
  correction limit are also detected) and the knots that differ within the cycle.
- **Then the ordinary reassignment stops.**
- **Trace.** Every solve also records, in `result.reassignment.trace`:
  - the assignment vector, the changed and out-of-segment knots with their progress;
  - the termination status, iterations, residuals and objective;
  - the obstacle clearance and the solver time.
- **E3:** the cycle is detected after 3 solves, with length 2 at knot 10.

## 4. Local branch resolution

With `branch_resolution = true` (and detection on), `resolve_branch!` handles a single
knot alternating between two adjacent segments. For each of the two segments:

1. Take the cycle member that had the knot in that segment: its full assignment
   (installed exactly with `install_cartesian_ids!`) and its solution as the warm start.
2. Bound only that knot's progress to the segment's interval `[s0, s1]`. The knot is not
   fixed to the boundary, and no other knot is bounded.
3. Solve, if the correction budget and the cycle budget allow it (estimate: 1.25 × the
   longest solve so far).
4. Read the solution while the bound is in place, then restore the bound. The bound is
   restored in a `finally` block, so it also happens on errors.
5. Run the full post-solve checks: segment assignments, exact dynamics, physical-track
   re-simulation, inputs and limits, and corridor diagnostics. In the obstacle controller,
   the dense collision evaluation follows.

The valid candidate with the lower objective is accepted. If neither is valid, the cycle
fails explicitly (`:segment_assignment_invalid`), with each candidate's status recorded.

**Correction (Milestone 4A).**
- *The defect:* in 3B.1, the obstacle acceptance rule (constrained-point margin, then the
  dense collision evaluation) was applied only *after* the selection. A lower-objective
  candidate that collides between constrained points could therefore be selected over a
  collision-valid one, and the cycle was then rejected (`:collision_detected`) instead of
  using the valid branch.
- *The fix:* `select_branch_candidate` first discards the candidates that fail
  `obstacle_rejection` (the rule shared with the obstacle wrapper), then takes the lowest
  objective among the rest. The discarded candidates are listed in the message.
- *Scope:* without an obstacle, the selection is unchanged.
- *Test:* `test/test_linear_mpc.jl`, "branch selection respects the obstacle acceptance
  rule". An obstacle is placed between two knots of the lower-objective candidate.
- *Results:* the recorded 3B.1 benchmark results were not re-run. In E3, the only branch
  case, a single candidate was valid, so the selection there is the same.
Several oscillating knots, or non-adjacent segments, are reported and not searched.

**E3, branch candidates:**

| knot 10 bounded to | interval [m] | result | objective |
|:--|:-:|:--|:-:|
| segment 14 (R −1) | [39.991, 43.133] | **valid**: dynamics residual 4.3e-7, knot clearance at `D`, dense check no contact | −4.1828 |
| segment 15 (R 2) | [43.133, 46.274] | Ipopt failure (no acceptable point) | (−3.8701 at stop) |

- **Selected:** segment 14. The cycle is accepted after 5 NLP solves.
- **In closed loop without a budget,** E3 then completes 2.60 laps (variants B and D).
  Closest plant approach 0.1816 m, against 0.15 m for contact; no violations.

**When membership bounds work and when they fail.** Bounding the knot does not make the
problem feasible. It only removes the alternation:
- Here, bounding knot 10 to the R −1 arc is compatible with the dynamics and the obstacle
  constraint. The knot settles at the boundary from the before-boundary side.
- Bounding it to the R 2 arc forces the vehicle further along at that knot, and Ipopt
  found no acceptable point.

Branch resolution therefore has to solve and validate both choices, and may find neither.

## 5. Sequential encounters

With `encounter_strategy = :sequential`, both side guesses are built at an encounter but
only the higher-ranked one is solved. The ranking prefers, in order:
1. a guess whose rollout succeeded;
2. the larger knot clearance above `D`;
3. the larger track clearance;
4. left.

The other side is solved only if the first is rejected; the fallback is tested. The side is
then retained as before.

- **Same outcomes without a budget.** In all 11 scenarios the first-encounter side and
  objective were identical to the two-candidate strategy. Laps, clearances and side
  switches were the same too.
- **Not an optimality claim.** This is an empirical finding for these scenarios. The
  sequential strategy does not compare the two local optima.

## 6. Budgets and timing methodology

The following are now distinguished:
- **per-attempt correction budget** (`correction_time_budget`, 80 ms, unchanged): limits
  reassignments within one `solve_once!`;
- **cycle budget** (`cycle_budget`, optional): limits the **optional** solves of the whole
  cycle:
  - second encounter candidate;
  - warm-start retry;
  - reassignments;
  - branch candidates.

  A solve starts only if the time used plus 1.25 × the longest solve so far fits. This is
  best effort: a single Ipopt solve can exceed it.
- **control period:** 100 ms.

`timing.total` covers the whole `solve!` call. `result.obstacle.phase_times` splits it into
relevance, guesses, attempts (solves, reassignments, validation) and remaining
bookkeeping. The benchmark also records per-cycle allocations and GC time (`@timed`).

## 7. Benchmark results

Four variants:
- A: original (3B);
- B: cycle detection + branch resolution;
- C: sequential encounters;
- D: both.

Three phases:
1. no restrictive budget, one run;
2. 80 ms correction budget only, 3 runs;
3. 80 ms correction budget plus 90 ms cycle budget, 3 runs.

All phases ran 250 cycles per run.

**Outcomes** (laps per run, or the stop cycle):

| scenario | phase 1: A / B / C / D | phase 2 (80 ms): A / B / C / D | phase 3 (80 + 90 ms): A / B / C / D |
|:--|:--|:--|:--|
| B R 1 m left | all 2.46 | 2.46 ×3; ×3; ×3; one run stops c86 (D) | 2.46 ×3 all |
| C R 1 m right | all 2.53 | 2.53 ×3 all | **all variants stop at c205** (A: c193, c205, c205) |
| D R 2 m | all 2.58 | 2.58 ×3 all | 2.58 ×3 all |
| E1 s 1.95 | all 2.54 | A: one stop c120; others 2.54 ×3 | 2.54 ×3 all |
| E2 s 2.10 | all 2.52 | 2.52 ×3 all | 2.52 ×3 all |
| **E3 s 8.20** | **A, C stop c118; B, D 2.60** | all stop c118 (B one run c64) | all stop c118 |
| E4 s 8.40 | all 2.52 | 2.52 ×3 all | 2.52 ×3 all |
| F lap boundary | all 2.58 | 2.58 ×3 all | 2.58 ×3 all |
| G1 +0.08 | all 2.48 | B: one stop c181; others 2.48 ×3 | 2.48 ×3 all |
| G2 −0.08 | all 2.46 | 2.46 ×3 all | A: stops c230, c188; others 2.46 ×3 |
| A no obstacle | all 2.58 | all 2.58 ×3 | all 2.58 ×3 |

**Stops with budgets.** Every stop is `:segment_assignment_invalid` after the correction
budget skipped a reassignment, except phase 3 C at cycle 205:
- there, the warm solve ends `LOCALLY_INFEASIBLE`, and the warm-start retry from the
  remembered side is skipped by the cycle budget;
- without the cycle budget the retry succeeds (phase 2: C completes in all runs).

**E3 under budgets.** Branch resolution was reached in some runs (phase 2: 2–3 cycles
detected) but could not finish within the budget.

**Clearances.** Over all variants and phases:
- the closest plant approach is 0.1759–0.2942 m, i.e. at least 2.6 cm away from contact
  (0.15 m) and at most 0.41 cm inside the buffer (`D` = 0.18 m);
- the physical track clearance is at least 4.4 cm;
- there are no plant violations of the track, inputs or lateral acceleration.

**Computation** (whole `solve!` cycle; phase 1; medians and maxima [ms]):

| scenario | encounter cycles, A (both) → C (sequential) | ordinary median | max A / B / C / D |
|:--|:-:|:-:|:-:|
| B | 35.8 → 17.9 | 10.2–10.4 | 37.0 / 39.8 / 30.0 / 105.6 |
| D | 40.5 → 19.3 | 9.9–10.3 | 50.6 / 51.0 / 114.9 / 24.8 |
| E2 | 55.1 → 26.5 | 10.3–10.4 | 81.8 / 104.0 / 63.1 / 61.6 |
| E3 | 20.2 → 9.8 (single encounter cycles) | 9.9–10.1 | 432.4 / 263.4 / 442.4 / 104.6 |
| F | 39.9 → 20.4 | 9.9–10.4 | 41.7 / 42.6 / 35.4 / 103.3 |
| A no obstacle | – | 6.1–6.3 | 26.2 / 14.2 / 14.6 / 14.5 |

- **Means.** Mean NLP solves per cycle: 1.11–1.19 with two candidates and 1.10–1.16
  sequential. The exception is the failing E3 runs, at 1.31 and 1.29.
- **E3 maxima.** A and C show 432–442 ms: 18 solves in the failing cycle. B and D reach 263
  and 105 ms with detection plus branch resolution.
- **Cycles above 100 ms** were rare (0–4 per 750 cycles in phases 2–3). Outside the E3 and
  C-at-205 cycles, the rows that contain one also show a higher total GC share (1.7–4.5 %
  against 0.4–0.8 %). That is consistent with an isolated GC pause of about 100 ms, but is
  not attributed per cycle in this benchmark.
- **Ipopt** accounts for 92–98 % of the total computation time.

## 8. Unresolved

1. **E3 within 100 ms.** The resolution costs 3 detection solves plus 2 branch solves.
   Remedies, not implemented:
   - detect a likely cycle earlier, e.g. a knot crossing back over the boundary it just
     left;
   - solve the branch candidates in a ranked order and stop at the first valid one;
   - carry the resolved branch into the next cycles' warm start.
2. **Cycle budget versus warm-start retry.** The cycle budget can block the retry that
   rescues a cycle (C at 205). A budget-aware alternative, e.g. retrying instead of a
   reassignment, was not studied.
3. **Isolated cycles above 100 ms** remain; most coincide with GC pauses.

There is no real-time guarantee.
