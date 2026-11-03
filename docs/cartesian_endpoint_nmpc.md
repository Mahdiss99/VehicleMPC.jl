# Cartesian-endpoint NMPC (Milestone 2B.4, experimental)

*Update (2B.5):* the between-knot corridor excursion (§7) is addressed, partially, by the
experimental interior-sample constraints; see `docs/intersample_corridor_constraints.md`.

**Status: experimental, not the default.** The default `prediction_model = :frenet` (with
`curvature_model = :stagewise_average`) is unchanged. The new mode is selected with
`NMPCConfig(prediction_model=:cartesian_endpoint_exact, schedule_corrections=8)`.

**Summary of the evidence** (`benchmarks/results/cartesian_endpoint_nmpc.md`;
`quarter_track()`, 3 × 250 cycles per configuration):

- **The prediction error from curvature averaging is gone.** Optimised knots agree with
  the event-aware reference simulation of their own inputs to ≤ 1.2e-6 m over the
  horizon. The Frenet formulations, including the exact segment-pair one, showed 1–4 cm.
- **With reassignments, the original 5 cm margin works.** The Cartesian-endpoint
  controller completed all four combinations of N = 10 / 20 and margin 5 cm / 10 cm, in
  every run, with every cycle accepted. At 5 cm it covered 2.58 laps (N = 10) and
  2.39 laps (N = 20) per run. No earlier formulation completed a lap with the 5 cm margin.
- **One solve per cycle is not enough.**
  - Without membership bounds, the first cycle already moves a knot out of its assigned
    segment.
  - With membership bounds:
    - N = 10 became infeasible at cycles 32–34;
    - N = 20 / 5 cm was rejected at cycle 202, with a solver residual of 1.02e-6 against
      the 1e-6 acceptance tolerance;
    - N = 20 / 10 cm completed every run.
- **Between knots, the planning corridor is still not respected.**
  - Accepted plans leave the tightened corridor between knots by up to 4.1 cm (5 cm
    margin). The physical track edge was not crossed in any accepted plan or by the plant.
  - The opt-in strict-corridor mode therefore rejects within the first four cycles. Exact
    knot dynamics do not constrain the path between knots (§7).

## 1. The exact zero-order-hold bicycle map

The rear-axle kinematic bicycle in Cartesian coordinates is

    Xdot = v cos(psi),   Ydot = v sin(psi),   psidot = v tan(delta)/L,   vdot = a.

Hold `delta` and `a` constant on `[0, dt]` and define the signed arc length of the rear
axle and the path curvature:

    sigma(t) = ∫_0^t v = v0 t + a t²/2,      c = tan(delta)/L.

Then `psi(t) = psi0 + c·sigma(t)`, because `psidot = c·sigmadot`. Every right-hand side
is a function of `sigma` alone times `sigmadot`, so by the chain rule

    X(dt) - X0 = ∫_0^d cos(psi0 + c σ) dσ,   d = sigma(dt) = v0 dt + a dt²/2,

and the same holds for `Y` with `sin`. Integrating gives the closed form

    ΔX = d·sinc_un(phi/2)·cos(psi0 + phi/2),   ΔY = d·sinc_un(phi/2)·sin(psi0 + phi/2),
    Δpsi = phi = c·d,   Δv = a·dt,   sinc_un(z) = sin(z)/z.

This is the chord of a circular arc of curvature `c` and length `d`. The path of the
vehicle over one interval is that arc whatever the acceleration, which only changes how
fast the arc is traversed.

- **What is exact.** The map is the exact solution of the ODE for constant `delta` and
  `a`. It needs no condition on the sign of `v`, because the substitution `σ = sigma(t)`
  holds for any `sigma`.
- **Agreement with the forward-driving Frenet plant** additionally requires `v(t) ≥ 0`.
  The NLP bounds `v ≥ v_min = 0` at the knots, and `v` is linear in time, so this holds on
  every interval.
- **What is not exact.** The map does **not** hold for inputs that vary within an
  interval, and it says nothing about a dynamic tyre model.
- **Prior use.** This closed form is the standard solution of the kinematic bicycle
  under constant inputs. No novelty is claimed.

`exact_bicycle_increment` (`src/models/exact_discretization.jl`) is the single
implementation, generic in the number type. `exact_cartesian_step` and
`simulate_exact_cartesian` wrap it for `Float64`.

### Evaluating `sin(z)/z`

Julia's `Base.sinc` is the *normalised* `sin(πz)/(πz)`, so the code defines its own
`sinc_un(z) = sin(z)/z`, together with its first and second derivatives:

- **`|z| < 0.5`:** the Taylor series through `z^16`, by Horner's rule. The first omitted
  term is below 1e-19 for the value and both derivatives.
- **Otherwise:** the closed forms. Their cancellation costs at most about 25 ulp at the
  switch point.
- **Checks:** against 256-bit references, the errors are ≤ 2e-16 (value), ≤ 5e-16 (first
  derivative) and ≤ 1e-15 (second derivative). ForwardDiff agrees with the analytic
  derivatives to ≤ 2e-15, and the switch at 0.5 is continuous to ≤ 1e-14.

In the NLP, `sinc_un` is a registered univariate operator with these analytic
derivatives, so Ipopt's Hessian is exact at zero steering as well. Zero steering reduces
to the straight-line update bit for bit. In this vehicle `|phi/2| ≤ 0.49`, so the series
branch is used almost always.

## 2. Endpoint equalities with Frenet decision variables

The decision variables are unchanged (`X = [s, ey, epsi, v]` at N+1 knots,
`U = [delta, a]` on N intervals). Each knot's Cartesian pose is

    p(s, ey) = p_c(s) + ey·n(s),   psi(s, epsi) = psi_c(s) + epsi    (n = left normal; psi unwrapped),

and stage `k` imposes the four equalities

    p(X_{k+1}) - p(X_k) = (ΔX, ΔY)(psi(X_k), v_k, U_k)
    psi(X_{k+1}) - psi(X_k) = phi_k
    v_{k+1} - v_k = a_k dt.

- **No integration of progress.** `s` is never integrated; it is determined implicitly
  through the geometry.
- **Well-defined mapping.** `(s, ey) ↦ p` is locally invertible where `1 - kappa·ey > 0`.
  The constructor's Frenet-safety check guarantees this on the knot corridor.
- **What disappears.** No curvature is averaged, so there is no curvature schedule, no
  stage points and no RK4 error. Per stage there are four equalities; the RK4 Frenet
  formulation needs 13 (and 9 stage-point variables).
- **Other constraints.** The objective, weights, bounds, `|epsi| ≤ epsi_max`, the
  lateral-acceleration inequality and the input-rate penalties are unchanged.

## 3. Smooth track geometry in assigned segments

Ipopt never evaluates the discontinuous segment look-up. Instead each knot `k` is
assigned one segment, and its centreline is written with that segment's analytic formula
(`segment_local_pose`, the same `sinc_un` form):

    psi_c = psi0 + kappa·(s - s0),
    p_c = (x0, y0) + (s - s0)·sinc_un(kappa (s - s0)/2)·(cos, sin)(psi0 + kappa (s - s0)/2).

- **Parameters.** The constants `(s0, x0, y0, psi0, kappa)` of every knot are JuMP
  Parameters (`ctrl.cframe`, 5×(N+1)), rewritten every cycle without a rebuild.
- **Curvature cases.** Straight segments (`kappa = 0`) and both curvature signs use the
  same formula, without branching or division.
- **Unwrapped progress and heading.** Segment ids are lap-aware (`lap·n + i`). `s0` is the
  unwrapped start progress, `(x0, y0)` the periodic start position, and
  `psi0 = psi_i + lap·total_turning`. The heading is therefore continuous across laps and
  needs no angle wrapping.
- **Checks.** Against `centerline_pose` and `frenet_to_cartesian` over three tracks and
  2.5 laps, the formulas agree to ≤ 2e-14. At every boundary the two adjacent frames give
  the same pose.

## 4. Segment-assignment validity

The assigned formulas equal the track only for `s ∈ [s0, s1]`; outside, they extrapolate
the segment's circle or line. The solution is therefore a solution of the true geometry
**only if every optimised knot stays in its assigned segment**.

**Assignment.** The nominal progress comes from an event-aware rollout of the initial
guess on the actual track:
- warm start: the shifted previous inputs;
- cold start: a quasi-steady profile.

Each nominal knot is assigned the segment containing it. A knot exactly on a boundary
belongs to the segment it **enters** (half-open), which is where a forward-driving vehicle
goes next. If the rollout reaches an end of an open track, the cycle is a
`:scheduling_failure` and the solver is not called.

**Post-solve check** (`cartesian_consistency`).
- Every optimised knot `k ≥ 2` must lie in `[s0 - tol, s1 + tol]`, with
  `tol = boundary_knot_tol = 1e-6 m`. Within `tol` the extrapolation error is below about
  `|Δkappa|·tol·(|ey| + tol)`, i.e. under 1e-6 m.
- Any knot outside gives `:segment_assignment_invalid`. Such a solution is never accepted
  in this mode, and there is no qualified acceptance.
- A closed single-segment track (a circle) is exempt, because the continuation *is* the
  track.

**Corrections** (`schedule_corrections > 0`, within `correction_time_budget`): the
segments are reassigned from the optimised progress, and the problem is re-solved from
that solution.

**Membership bounds** (`segment_membership = true`, optional) instead bound every
progress knot to its segment. That makes the identity hold by construction, but it can
make the problem infeasible (§6).

**Exact-map reconstruction** (`exact_map_check`, in `result.exact`). For every candidate,
the knots are mapped with the *true* geometry, and the inputs are propagated from the
measured state with the exact map. This gives:
- position, heading and speed errors over the horizon, and at the first step;
- the Frenet progress and lateral mismatches (by projection);
- the one-step knot residual;
- `geometry_residual`, the difference between the assigned-segment formula and the true
  geometry at the optimised knots.

The existing event-aware reference check runs as before.

**Diagnostics by failure type:**

| what happened | where it shows |
|:--|:--|
| exact dynamics | `exact.knot_residual`, `max_dynamics_residual` (the NLP's own equalities, recomputed in `Float64`) |
| correct assignment | `schedule.configuration_changes == 0`, `exact.geometry_residual` ≈ 1e-15 |
| invalid assignment | `:segment_assignment_invalid` |
| numerical optimizer failure | `:solver_failure`, `:invalid_solution` |
| physical infeasibility | `:reference_violation`, `reference.min_boundary_distance < 0` |
| planning-corridor violation | `reference.min_corridor_distance < 0`; with `strict_corridor = true`, `:corridor_violation` |
| deadline overrun | `deadline_missed(result, 0.1)` (`timing.total`); not a status |

## 5. Validation of the exact map (before any NLP)

Results are in the benchmark results file and the tests.

**One step of 0.1 s against a 256-bit RK4 reference** (4000 steps). Eleven cases:
straight, left, right, accelerating and braking under constant steering, zero steering,
1e-9 rad steering, start from rest, a 40 rad unwrapped heading, `δ = 25°` at 5 m/s, and
braking to rest at `dt`.
- The exact map agrees to ≤ 4.4e-16 m and ≤ 2.8e-17 rad, i.e. to rounding.
- The existing `integrate_cartesian` (RK4, 1 ms steps) agrees to ≤ 1.9e-12 m.

**Against the event-aware Frenet plant** (RK4, 1 ms steps, located boundary crossings),
mapped to Cartesian coordinates:
- across the straight → R 1 m transition, the R 1 m → R −1 m S-bend and the lap boundary;
- over a 60-step second lap with braking, and from rest.

Agreement is ≤ 7.2e-12 m and ≤ 1.1e-12 rad. Over 20 steps the plant's error falls with
its step size: 3.7e-8 m at 10 ms, 5.1e-12 m at 1 ms. The remaining difference is
therefore the plant's integration error, not the map's.

**Gradients.** ForwardDiff Jacobians of the exact step agree with central differences to
≤ 1e-8.

## 6. Quarter-track experiments

The configurations are:

| | configuration |
|:--|:--|
| A | stagewise average curvature, ≤ 8 corrections, reject (2B.2) |
| B | segment-pair D2 (2B.3B) |
| C | Cartesian, 1 solve |
| D | Cartesian, ≤ 8 reassignments |
| E | Cartesian with membership bounds, 1 solve |
| F | D with `strict_corridor` |

Every controller is primed (one solve, then reset) before its run (§8). All runs were
deterministic: the three runs of each configuration gave the same outcome in every
setting.

**Laps per run** (3 runs each; "stop" = cycle and reason):

| | N = 10, 5 cm | N = 10, 10 cm | N = 20, 5 cm | N = 20, 10 cm |
|:--|:--|:--|:--|:--|
| A stagewise | stop c42 (stale) | **2.46** | stop c37 (budget; c38 without) | stop c37 (budget; c38 without) |
| B segment-pair D2 | stop c97 (edge −1.5 mm) | **2.48** | stop c6 (edge −5.4 mm) | **2.35** |
| C Cartesian, 1 solve | stop c1 (segment) | stop c1 (segment) | stop c1 (segment) | stop c1 (segment) |
| **D Cartesian + reassignments** | **2.58** | **2.50** | **2.39** | **2.34** |
| E Cartesian + membership | stop c32 (infeasible) | stop c34 (infeasible) | stop c202 (residual 1.02e-6) | **2.35** |
| F = D + strict corridor | stop c1 | stop c1 | stop c4 | stop c4 |

- **Determinism.** The rerun without the correction time budget reproduces every outcome
  except A at N = 20. There, A stops at cycle 38 instead of 37; it is limited by stale
  schedules either way.
- **Reproduction of earlier results.** The A and B results equal the 2B.3B benchmark
  where the settings coincide: laps, stop cycles, solve counts and residuals. The one
  exception is run 1 of A at N = 20 / 10 cm, which stopped at cycle 1 in 2B.3B. There the
  controllers were not primed, so evaluator construction probably counted against the
  correction budget (its message reports the budget; not re-measured).

**Prediction accuracy** (accepted cycles, maximum over all runs):

| | exact-map error: first step / horizon [m] | event-aware reference: horizon [m] | NLP residual / geometry residual |
|:--|:-:|:-:|:-:|
| A stagewise | 1.5e-2 to 1.7e-2 / 2.5e-2 to 4.1e-2 | 2.5e-2 to 4.1e-2 | 4.7e-8 / – |
| B segment-pair D2 | 5.9e-8 to 2.0e-2 / 9.9e-3 to 4.1e-2 | 9.9e-3 to 4.1e-2 | 4.6e-8 / – |
| D Cartesian | ≤ 1.3e-10 / ≤ 1.1e-8 | ≤ 1.2e-6 | 4.7e-8 / ≤ 8.0e-15 |
| E Cartesian + membership | ≤ 1.5e-10 / ≤ 8.4e-7 | ≤ 9.6e-7 | 9.4e-7 / ≤ 7.6e-15 |

- **Where the residual error comes from.** The ≤ 1.2e-6 m reference error is the RK4 error
  of the reference itself at 5 samples per interval. The exact-map reconstruction agrees
  to ≤ 1.1e-8 m, which is the solver's equality tolerance accumulated over the horizon.
- **Assignments.** No accepted Cartesian cycle had a segment change. Reassignments were
  needed in 11–22 % of the D cycles, with at most 3 solves per cycle.
- **Single transition** (straight → arc, cold start, nine speed/phase cases):
  - Cartesian with reassignments: success in 9 of 9, with 2 solves each; exact-map error
    ≤ 1.4e-10 m, reference error ≤ 9.6e-9 m.
  - With one solve: rejected in 9 of 9 (segment change).
  - Segment-pair: success in 9 of 9 with 0.3–4.9 mm error.
  - Stagewise: stale in 9 of 9.

## 7. Clearances: physical edge vs planning corridor

All values are minima over accepted cycles and runs [m].

| | margin | plant: track / corridor | predicted: track / corridor |
|:--|:-:|:-:|:-:|
| D Cartesian, N = 10 | 0.05 | 0.046 / −0.004 | 0.015 / −0.035 |
| D Cartesian, N = 20 | 0.05 | 0.129 / 0.079 | 0.009 / −0.041 |
| D Cartesian, N = 10 | 0.10 | 0.097 / −0.003 | 0.064 / −0.036 |
| D Cartesian, N = 20 | 0.10 | 0.166 / 0.066 | 0.073 / −0.027 |
| B segment-pair, N = 20 | 0.10 | 0.153 / 0.053 | 0.049 / −0.051 |

- **The physical edge holds.** There were no plant violations in any run of any
  configuration, and no accepted Cartesian plan crossed the physical track edge in its
  re-simulation.
- **The planning corridor does not.** The executed plant left the corridor by up to 4 mm
  with N = 10. Predicted plans left it **between knots** by up to 4.1 cm.
- **Sampling sensitivity.** Re-simulating each accepted plan at 5, 20 and 100 samples per
  interval changes the minimum predicted track clearance by at most 1.4 mm between 5 and
  20 samples, and by ≤ 0.1 mm between 20 and 100. Sampling therefore shows convergence;
  a formal continuous-time guarantee is not claimed.
- **Strict corridor.** F is cleanly rejected (`:corridor_violation`, physical check
  passed) within 1–4 cycles, because no corridor constraint exists between knots.
- **A possible refinement** (not implemented, needs approval). Between knots the path is a
  circular arc (§1), so intermediate-point or sagitta constraints can be written exactly.

## 8. Computational performance

All timings come from one Windows laptop, Julia 1.11.5, one thread, with all computation
included.

**D Cartesian, warmed up** (750 cycles per setting):

| | NLP solves mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms |
|:--|:-:|:-:|:-:|
| N = 10, 5 cm | 1.13 / 2 | 6.1 / 12.8 / 14.4 / 19.9 | 0 / 0 |
| N = 10, 10 cm | 1.11 / 2 | 6.1 / 12.5 / 14.7 / 25.8 | 0 / 0 |
| N = 20, 5 cm | 1.22 / 3 | 9.8 / 20.7 / 22.8 / **110.0** | 1 / 1 |
| N = 20, 10 cm | 1.18 / 2 | 9.9 / 20.5 / 22.3 / 26.3 | 0 / 0 |

- **Comparison with the Frenet formulations.** At N = 20 / 10 cm, segment-pair D2 took a
  median of 16.1 ms and a maximum of 67.8 ms; stagewise A at N = 10 / 10 cm took a median
  of 15.0 ms. The smaller NLP has no stage points.
- **Model construction.** Building the model takes 2–9 ms. The first solve on a new
  model builds JuMP's NLP evaluator: 8–24 ms for the Cartesian model versus 24–73 ms for
  stagewise (excluded from the timings above; reported as "priming").
- **Compilation.** The first construction and solve in a process, compilation included,
  took 2.7 s / 5.9 s; the Cartesian controller then added 0.24 s / 0.98 s.
- **Priming does not change results.** A primed run and an unprimed run (no time budget)
  were checked to give identical closed loops.

**Outliers** (all cycles above 80 ms, recorded with `@timed`):
- **D, N = 20 / 5 cm, cycle 249 (110 ms):**
  - 99 ms of it was garbage collection inside `optimize!` (Ipopt reported 104 ms);
  - the solve itself was ordinary: 15 iterations, 0.5 MB allocated.
- **B, run 1, cycle 5 (135 ms):** no GC and only 18 ms in `optimize!`, but 7.8 MB allocated
  against a median of 0.2 MB. This suggests first-time compilation of a code path. That is
  not verified.

**Machine variability.** An earlier run of this benchmark on the same machine, about an
hour before, was about 1.9× slower throughout. There, the same segment-pair configuration
did an identical computation (same solve counts) with a median of 17.8 ms instead of
9.2 ms, and Ipopt's own solve time had doubled as well. Absolute timings therefore depend
on the machine's state. The correction-budget decisions do too, which made A stop at
cycles 4–5 in that run.

There is no real-time guarantee.

## 9. Advantages, limitations, risks

**Compared with the Frenet curvature formulations**
- The prediction is exact at the knots for piecewise-constant inputs. There is no
  curvature averaging, no intra-stage error at curvature jumps, and no RK4 error.
- The NLP is smaller, and the median solve time fell by about 40 % at N = 20.
- Track geometry enters only through Parameters. Lap wrapping and unwrapped heading need
  no special cases.

**Limitations and risks**
- **Assignment validity.** Validity relies on knots staying in their assigned segments.
  One solve per cycle fails immediately on this track. Reassignment needs up to 3 solves,
  and there is no proof that it converges: a knot could alternate between two segments
  until the 8-correction limit.
- **Membership bounds** avoid reassignment but caused local infeasibility at N = 10.
  E's residual failure (1.02e-6 > 1e-6) was Ipopt stopping at its default tolerances with
  an active membership bound; the acceptance tolerance was not changed.
- **Between knots.** Knot-exact dynamics do not keep the vehicle inside the planning
  corridor between knots (up to 4.1 cm outside). The physical-edge clearance measured here
  (≥ 9 mm predicted) is empirical, not guaranteed.
- **Local solutions.** The equalities are non-convex (trigonometric in heading and
  progress), so Ipopt finds local solutions only. Ipopt's tolerance limits the equality
  residuals to about 5e-8, and in one case about 1e-6.
- **Scope.** Exactness is for the kinematic bicycle with zero-order-hold inputs. A
  dynamic tyre model would need its own discretisation.
- **Timing.** The single-thread timings vary by about 2× with the machine's state.
  Garbage-collection pauses of about 100 ms occurred.

## 10. Recommendation (for review)

1. **Keep the default unchanged**, as instructed.
2. **The strongest candidate so far is the Cartesian-endpoint mode with bounded
   reassignments (D).** It was the only configuration that completed all four
   horizon/margin combinations, including the original 5 cm margin. It was also the
   fastest, and its predictions are exact at the knots.
3. **Before it could replace the default:**
   - between-knot corridor handling (§7);
   - a decision on the 1e-6 acceptance tolerance versus Ipopt's tolerances;
   - tests on more tracks.
