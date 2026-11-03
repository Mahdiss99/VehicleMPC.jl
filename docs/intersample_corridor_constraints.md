# Between-knot corridor constraints (Milestone 2B.5, experimental)

**Status: experimental, opt-in, not the default.** The setting is
`NMPCConfig(prediction_model=:cartesian_endpoint_exact, interior_samples=1 or 3, ...)`.
`interior_samples = 0` is the Milestone 2B.4 controller, unchanged.

**Summary of the evidence** (`benchmarks/results/intersample_corridor.md`;
`quarter_track()`, 3 × 250 cycles, strict acceptance with tolerance 1e-6 m):

- **Interior constraints shrink the corridor violation between knots, but do not remove
  it.** Largest predicted excursion beyond the 5 cm planning corridor, over all accepted
  plans:

  | interior samples | N = 10 | N = 20 |
  |:--|:-:|:-:|
  | none (2B.4) | 36.3 mm | 42.4 mm |
  | midpoint | 9.1 mm | 10.0 mm |
  | three | 2.3 mm | 2.4 mm (without the time budget) |

- **Strict-corridor acceptance therefore fails at the first or fourth cycle** in every
  configuration, horizon and margin. The violation is between the constrained sample
  times, and the analytic check confirms the dense-sampling minima.
- **The physical track edge was never crossed.**
  - No plant violation occurred in any run.
  - With the 5 cm margin, accepted plans kept at least 7.6 mm (no interior samples),
    40 mm (midpoint) and 47.7 mm (three samples, N = 10) from the track edge.
- **Three samples cost real time.** Each N = 20 solve takes about 35 ms. Within the 80 ms
  budget only one reassignment fits, so the three-sample controller at N = 20 stopped at
  cycle 4 in every run. Without the budget it completes the laps, but with 70 (5 cm) and
  10 (10 cm) of 250 cycles above 80 ms.

## 1. Why knot constraints are not enough

The exact endpoint formulation (2B.4) constrains `|ey| ≤ E = width/2 − margin` only at
the knots. Between two knots the rear axle follows a circular arc of curvature
`c = tan(delta)/L` (§2), while the corridor follows the track's own curvature. The two
can separate by roughly the difference of the sagittas,
`|c − kappa_track|·(v·dt)²/8` per interval: about 2–3 cm at 2–3 m/s across a curvature
jump. This is exactly the 2B.4 observation (up to 4.1 cm outside the corridor between
knots, with exact knots).

## 2. Exact intermediate position

For an interval starting at knot `k` with pose `(X_k, Y_k, psi_k)` (the knot's Frenet
variables mapped with its assigned segment frame), speed `v_k` and input
`(delta_k, a_k)`, the exact position after `tau = alpha·dt` is

    d_tau = v_k tau + a_k tau²/2,   phi_tau = tan(delta_k) d_tau / L,
    p_tau = (X_k, Y_k) + d_tau sinc_un(phi_tau/2) (cos, sin)(psi_k + phi_tau/2).

`interior_point` is exactly the position part of `exact_bicycle_increment` over `tau`,
with the same `sinc_un` operator and analytic derivatives. In the NLP it is a smooth
function of the existing variables `X[:, k]` and `U[:, k]`. There are no new decision
variables, no interpolation and no Euler or RK4 steps.

**Sample times:**
- `interior_samples = 1`: `alpha = 0.5`, i.e. `t_k + 0.05 s`;
- `interior_samples = 3`: `alpha = 0.25, 0.5, 0.75`, i.e. `t_k + 0.025, 0.05, 0.075 s`;

on every interval `k = 1 … N`.

## 3. Straight segments

Each straight segment has a start point `q = (x0, y0)` and left unit normal
`n = (−sin psi0, cos psi0)`. The constraint is

    -E ≤ n·(p_tau − q) ≤ E.

`n·(p − q)` is the exact lateral offset `ey` **if** the orthogonal projection of `p` lies
on that finite segment; otherwise it is the distance to the infinite line (§5).

## 4. Curved segments (annulus)

Each curved segment has curvature `kappa ≠ 0`, `R = 1/|kappa|`, and centre
`O = (x0, y0) + (1/kappa)(−sin psi0, cos psi0)`. The centre lies on the left normal for
`kappa > 0` and on the right normal for `kappa < 0`. The constraint is

    (R − E)² ≤ |p_tau − O|² ≤ (R + E)².

In the NLP both sides are divided by `2R`, so that near the arc the margins are about
metric: `(r² − (R−E)²)/(2R) ≈ r − (R − E)`.

- **When it holds:** the annulus is exactly `|ey| ≤ E` **if** the projection of `p` lies
  on the finite arc. Satisfying the full-circle annulus alone proves nothing about the arc
  (§5).
- **Requirement on the corridor.** It requires `0 < E < R`; the constructor checks this
  and throws otherwise. The 2A Frenet-safety condition already implies it:
  `1 − |kappa|·E ≥ 2 eps_geometry > 0`.

## 5. Persistent formulation and segment membership

**One persistent form.** Every sample has two inequalities
`sel·curved + (1 − sel)·straight ≥ 0`:

    lower = sel·(r² − (R−E)²)/(2R) + (1 − sel)·(n·(p − q) + E)
    upper = sel·((R+E)² − r²)/(2R) + (1 − sel)·(E − n·(p − q))

Eight Parameters per sample (`iframe`) hold `sel ∈ {0, 1}`, `q`, `n`, `1/(2R)`,
`(R−E)²` and `(R+E)²`. The inactive form receives finite dummy constants.

- **Model sizes.** The model is built once per mode and horizon. At N = 20 it has 120
  decision variables in every mode, and 160, 200 and 280 constraints for 0, 1 and 3
  interior samples.
- **Excluded by design.** No segment look-up is evaluated by Ipopt, and there are no
  integer variables.

**Assignment before the solve** (`interior_assignment`):
1. Evaluate the nominal trajectory's exact interior point, with knot `k` in its assigned
   frame.
2. Project it independently onto the actual track (`sample_projection`). The projection
   searches every segment and keeps candidates with `|ey|` below
   `min(width, 0.99·R_min)` within 1 m of the hint, the linearly interpolated nominal
   progress.
3. Assign the lap-aware segment that contains the projection, with half-open boundaries
   (the entering segment).
4. A missing or ambiguous projection falls back to the segment of the hint, and is
   counted.

**Validation after the solve** (`interior_assignment_check`):
1. Rebuild each interior point from the optimised variables, exactly as the NLP does.
2. Project it independently.
3. Require a unique projection inside the assigned segment, within `boundary_knot_tol`.

Missing, ambiguous and out-of-segment projections are counted separately. Any of them
gives `:interior_assignment_invalid`, even when the inequality is satisfied, and the
existing bounded reassignment re-solves with knots and samples reassigned from the
optimised trajectory. The tests construct a point inside the full-circle annulus but
before the arc, and check that it is not accepted.

**Status precedence:** `:reference_violation` > `:segment_assignment_invalid` (knots) >
`:interior_assignment_invalid` > `:corridor_violation` > `:success`. Solver failures and
invalid solutions are reported before any of these, as before.

## 6. Independent validation

**Event-aware re-simulation** (`reference_check`, 5 samples per interval): unchanged.

**Dense exact check** (`corridor_check`, `dense_check_samples` per interval).
1. Propagate the plan's inputs from the measured state with the exact map.
2. Sample every interval at `j·dt/M`, plus the final knot.
3. Project each sample with `sample_projection`.

It reports:
- the minimum physical and corridor clearance, and the time and stage of the worst one;
- the number of violating samples and the largest excursion;
- projection failures;
- the NLP interior points versus the exact reconstruction.

With `strict_corridor = true` it rejects any corridor excursion beyond `acceptance_tol`
(1e-6 m), as does the re-simulation. The benchmark runs it at 20 samples per interval in
every cycle. Its cost (median 0.3–1.0 ms) is included in the timings.

**Resolution study** (every accepted plan of run 1). The minimum corridor clearance
changes by at most 0.09 mm from 20 to 50 samples, and by at most 0.01 mm from 50 to 100.

**Analytic continuous-time clearance** (`analytic_corridor_clearance`; validation only,
**not** in the optimisation).

*Derivation.* In each interval the vehicle point is `p(sigma)` on a circle (or line) of
curvature `c`, with heading `psi0 + c·sigma`. Where the track projection is unique,
`ey(sigma)` is piecewise smooth. Each piece is the signed distance to a straight line, or
`±(R − |p − O|)` for an arc, and pieces end where the point crosses the normal line at a
segment boundary. The extremes of `|ey|` therefore lie among:
1. the interval ends;
2. points where the vehicle heading is parallel to a straight segment
   (`psi = psi_i + mπ`);
3. points where the vehicle-circle centre, the point and an arc centre are collinear
   (`psi = angle(C_v − O) ± π/2 + mπ`);
4. boundary crossings: `sin(psi − psi_b) = −(c·t_b·(p0 − b) + sin(psi_b − psi0))`.

All four are in closed form. Every candidate of every segment is evaluated with the true
projection; extra candidates are harmless.

*Scope of validity.* It requires a unique, continuous projection along the arc (`|ey|`
below the smallest radius of curvature, and no other part of the track within the search
window), and it is computed in `Float64`.

*Measured.* Over about 6 700–13 800 candidates per setting there were no projection
failures. The analytic minimum was never above the 100-sample dense minimum, and at most
3.8e-6 m below it. So dense sampling at 20 or more samples per interval is converged for
this track, and the analytic check confirms that the violations are real.

## 7. Results (quarter track, 3 × 250 cycles)

Strict acceptance was used throughout, with ≤ 8 reassignments within 80 ms. The dense
check ran at 20 samples per interval in every cycle.

**With strict corridor acceptance (A, B, C):**

| | N = 10, 5 cm | N = 10, 10 cm | N = 20, 5 cm | N = 20, 10 cm |
|:--|:--|:--|:--|:--|
| A: no interior samples | c1, excursion 15.2 mm | c1, 16.4 mm | c4, 25.4 mm | c4, 27.0 mm |
| B: midpoint | c1, 4.3 mm | c1, 2.8 mm | c4, 4.0 mm | c4, 2.0 mm |
| C: three samples | c1, 0.8 mm | c1, 1.5 mm | c4, interior assignment\* | c4, knot assignment\* |

- \*The next reassignment was skipped because the 80 ms budget would have been exceeded
  (§8). At N = 20 / 5 cm the same plan was also 2.4 mm outside the corridor.
- All three runs of each configuration are identical.
- Every excursion is between constrained samples. For example, C at N = 10 has its worst
  point at t = 0.865 s in interval 9, whose samples are at 0.825, 0.85 and 0.875 s.

**Without strict acceptance (A0 = 2B.4, B0, C0), to measure the full-lap behaviour.**
Each cell gives laps per run, the largest predicted corridor excursion over accepted
plans, and the minimum plant corridor clearance.

| | N = 10, 5 cm | N = 10, 10 cm | N = 20, 5 cm | N = 20, 10 cm |
|:--|:--|:--|:--|:--|
| A0 | 2.58 ×3; 36.3 mm; −3.7 mm | 2.50 ×3; 36.8 mm; −3.1 mm | 2.39 ×3; 42.4 mm; +78.8 mm | 2.34 ×3; 28.2 mm; +66.3 mm |
| B0 | 2.58 ×3; 9.1 mm; −0.7 mm | 2.50 ×3; 7.3 mm; −0.6 mm | 2.39, 2.39, stop c104†; 10.0 mm; +80.6 mm | 2.34 ×3; 7.9 mm; +66.2 mm |
| C0 | 2.58 ×3; 2.3 mm; −0.2 mm | 2.50 ×3; 2.1 mm; −0.1 mm | stop c4 ×3 (budget) | stop c4 ×3 (budget) |

- †Cycle 104 had a 114 ms GC pause, so the next reassignment was skipped.
- **Without the time budget** (one run each), all nine A0/B0/C0 settings complete. C0 then
  reaches a 2.4 mm (N = 20 / 5 cm) and 2.0 mm (N = 20 / 10 cm) excursion.
- **Plant and edge.** There were no plant violations in any run. With N = 10 the executed
  plant left the corridor by up to 3.7 mm (A0), 0.7 mm (B0) and 0.2 mm (C0).
- **Accuracy of the interior points.** At the solution they agree with the exact
  reconstruction to ≤ 1.6e-8 m, and the interior inequalities hold to ≥ −1.0e-8.
- **Other tracks** (N = 20, 5 cm, strict, 100 cycles):
  - straight track and R = 1 m circle: every configuration completed, with no violation
    (exact geometry, no curvature jump);
  - straight → arc → straight: A and B accepted all cycles until the horizon reached the
    open track's end (`:scheduling_failure` at cycle 19, as designed). C was rejected at
    cycle 1: the next reassignment exceeded the budget.

## 8. Runtime

All timings come from one Windows laptop, Julia 1.11.5, one thread, with everything in
`solve!` included. The table gives warmed medians [ms]:

| | N = 10 | N = 20 |
|:--|:-:|:-:|
| A0 (2B.4) | 6.1–6.3 | 10.1 |
| B0 midpoint | 9.6–10.0 | 16.8–17.1 |
| C0 three samples | 16.8–17.6 | 36–38 (cycles before stopping) |

- **Three samples at N = 20** take about 35 ms per solve, and 1.5 solves per cycle on
  average.
  - Without the budget: median 51.8 ms (5 cm) and 35.7 ms (10 cm). Cycles above 80 ms:
    70 and 10 of 250; above 100 ms: 21 and 3.
  - This threatens the 10 Hz requirement. With the budget it cannot complete a lap.
- **Midpoint at N = 20** stays below 80 ms except in garbage-collection cycles.
- **Outliers.** All 5 cycles above 100 ms in the main grid coincided with a measured GC
  pause of 89–127 ms (allocations 0.7–2.4 MB).
  - Three of them fell inside `optimize!`.
  - Two fell inside the validation step: A0 cycle 214 at 110.8 ms, and B0 cycle 222 at
    98.8 ms, which also counts as one of the 5.
  - Allocation-driven GC pauses of about 100 ms are therefore a measured, recurring cause
    of deadline misses.
- **Validation cost:** median 0.3–1.0 ms per cycle (event-aware re-simulation, exact-map
  check, interior check and dense check).

## 9. What continuous-time certification would still require

1. **Constraints that hold for the whole interval**, not at samples. The analytic
   candidates (§6) give a closed-form maximum of `|ey|` per interval and segment piece.
   Imposing it in the NLP would need care: the candidate set changes with the variables,
   and the min/max structure is non-smooth. Smooth over-approximations are an option,
   e.g. a sagitta bound `|c − kappa|·d²/8` subtracted from the sample margins.
   - Such tightening must be derived and documented as a constraint, not as a change of
     the margin, and it needs approval.
2. **A proof that the projection is unique** along the whole vehicle arc: `|ey| < R_min`,
   and no other part of the track within reach.
3. **Floating-point and solver tolerances.** Ipopt satisfies the inequalities to about
   1e-8, and the acceptance tolerance is 1e-6 m. A certificate needs an explicit
   tolerance budget.
4. **Recursive feasibility**, and robustness to the GC-induced deadline misses (§8).
   Neither is addressed.

## 10. Assessment

- **Midpoint vs three samples.**
  - The midpoint removes about 75 % of the between-knot excursion, at roughly 1.6× the
    solve time. It is practical at both horizons, apart from GC outliers.
  - Three samples remove about 94 % at roughly 2.8× the time (N = 10) and 3.5× (N = 20).
    At N = 20 that cost breaks the 80 ms correction budget.
- **Neither satisfies the original 5 cm corridor in continuous time.** The remaining
  excursions are 9–10 mm (midpoint) and about 2 mm (three samples), so strict acceptance
  rejects every configuration.
- **The physical edge was respected** in every plan and run.
- **Default unchanged.** Promoting either mode is not supported by these results.
