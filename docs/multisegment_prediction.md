# Multi-segment track dynamics and curvature prediction (Milestone 2B.1)

This is a numerical validation milestone. It adds an accurate reference simulation for
piecewise constant-curvature tracks and measures how much error each curvature
approximation introduces into a discrete-time prediction. **The NMPC is unchanged**: it
still accepts only single-segment tracks and rejects multi-segment ones.

Code:
- `src/simulation/piecewise_track.jl`: event-aware integrator and Cartesian reference model.
- `src/tracks/curvature_schedule.jl`: average curvature and stagewise schedules.
- `src/analysis/curvature_prediction.jl`: prediction methods A, B and D, and error measures.

Reproduce the numbers with `julia --project=. benchmarks/multisegment_prediction.jl`; use
`--project=examples` to also produce the figures. Full tables are in
`benchmarks/results/multisegment_prediction.md`.

## 1. Event-aware integration

The Frenet model is `x' = f(x, u, kappa(s))`, where `kappa(s)` is piecewise constant
in progress with jumps at the segment boundaries `b_j`. The centreline is C¹ (position and
tangent are continuous), so the Frenet coordinates `(s, ey, epsi)` of a moving vehicle are
continuous across a boundary. Only the right-hand side jumps. When `sdot ≠ 0` at the
boundary (a transversal crossing), the exact solution is the concatenation of the smooth
flows of the neighbouring segments, joined at the crossing time `t*`, where `s(t*) = b`.

`integrate_on_track(model, track, x, u, dt; max_step)` implements this:

1. **Segment tracking.** The current segment `(lap, i)` with its unwrapped interval
   `[lo, hi)` is tracked explicitly. After the first look-up it is never re-derived from
   `s`.
2. **Trial step.** One RK4 step of length `h ≤ max_step` is taken with `kappa_i`.
3. **Crossing detection.** If the trial step leaves `[lo, hi)`, the crossing time is the
   root of `g(tau) = s(Φ_tau(x)) − b` on `[0, h]`. Here `Φ_tau` is the RK4 step map of
   length `tau`, which is smooth in `tau` and has the same local accuracy as the trial
   step.
4. **Root search.** The root is found by a bracketed Illinois/regula-falsi search with
   bisection fallback (`locate_crossing`). The bracket keeps `g(a) < 0 ≤ g(b)`, and the
   returned point is the right end, so the state is on the boundary or past it, never
   before it.
5. **Continuation.** Integration continues in the neighbouring segment for the remaining
   time. Several crossings per call are handled the same way, in either direction.
6. **Lap boundaries.** On a closed track the last segment is followed by segment 1 of the
   next lap. `s` is never reset.
7. **Open-track ends.** Reaching an end of an open track returns `:end_of_track` or
   `:start_of_track` at the boundary state. There is no extrapolation.
8. **Safeguards.**
   - Explicit segment tracking means a state on a boundary (up to round-off) cannot
     re-trigger the same event.
   - A stationary vehicle on a boundary produces no events.
   - `max_events` caps the work.
   - Failed root searches return `:event_failure`, and domain errors return
     `:domain_error`. Nothing is clamped.

**Accuracy.**
- **Order of convergence.** The end-state error against an independent fine reference falls
  by a factor of more than 10 each time `max_step` is halved through events, consistent
  with RK4's 4th order (tested). Driving straight along the tangent shows clean 4th-order
  convergence: 1.3e-7 → 2.1e-10 → 2.1e-14 m for `max_step` = 0.1, 0.01, 0.001.
- **Cross-check with the Cartesian model.** Over all 290 experiment cases (up to 6
  crossings per 2 s horizon), the event-aware Frenet solution and an independent Cartesian
  RK4 solution (`Xdot = v cos ψ`, `Ydot = v sin ψ`, `ψdot = v tan δ / L`, no track
  information) agree to **2.5e-11** in position and heading. After back-projecting the
  Cartesian trajectory into Frenet coordinates with progress hints, `s`, `ey` and `epsi`
  agree to **7.3e-11**.

## 2. Why start-of-step curvature is inaccurate

Write the absolute heading as `psi = psi_c(s) + epsi`. Along any Frenet trajectory that
uses a model curvature `kappa_m(t)`:

```text
d/dt (psi_c(s) + epsi) = kappa(s)·sdot + v·tan(delta)/L − kappa_m·sdot
```

The true vehicle has `psidot = v·tan(delta)/L`, and the inputs and speed are identical in
both. So the predicted absolute heading error is **exactly**

```text
e_psi(t) = ∫ (kappa(s(τ)) − kappa_m(τ)) · sdot(τ) dτ        (along the predicted trajectory)
```

For method A (`kappa_m = kappa(s_k)` on stage k), a stage that crosses a jump `Δkappa`
after a fraction `θ` of the step gives:

- heading error `e_psi = Δkappa·(s_{k+1} − b) ≈ Δkappa·v·dt·(1−θ)`, which is **first
  order in dt**;
- position error `≈ Δkappa·(v·dt·(1−θ))²/2`.

RK4 does not reduce either: the error comes from the wrong right-hand side, not from
integration. Measured worst one-step values at `dt = 0.1`, `v = 4` (`Δkappa = 2`,
`θ = 0.1`): heading 0.63 rad, position 0.13 m.

## 3. Stagewise average curvature

For a stage whose nominal progress runs from `s_k` to `s_{k+1}`:

```text
kappa_avg_k = (psi_c(s_{k+1}) − psi_c(s_k)) / (s_{k+1} − s_k) = (1/Δs) ∫ kappa(σ) dσ
```

This uses the unwrapped, continuous centreline heading, so it holds exactly inside one
segment, across one or several boundaries, and across lap boundaries.
`average_curvature` handles `|Δs| < 1e-6 m` by the one-sided limit in the direction of
travel, and throws for `Δs = 0` exactly at a jump, where the limit is ambiguous.
`curvature_schedule` builds the stagewise vector meant for the NMPC's existing
`kappa[k]` `Parameter`s; the controller does not use it yet.

From the identity in §2, the heading error accumulated over stage `k` is

```text
∫_{s_k}^{s_{k+1,pred}} kappa ds − kappa_avg_k·(s_{k+1,pred} − s_k)
  = ∫_{s_{k+1,nom}}^{s_{k+1,pred}} (kappa − kappa_avg_k) ds
```

It **vanishes whenever the predicted progress equals the nominal progress used for the
schedule** (a *self-consistent* schedule). This is confirmed numerically: over 277
converged self-consistent cases the worst heading error at any knot is 5.3e-10 rad.

The position error inside a crossing stage does not vanish. The model turns uniformly,
while the true vehicle switches curvature mid-step. For a jump at fraction `θ`, the
lateral error is about `Δkappa·(v·dt)²·θ(1−θ)/2 ≤ Δkappa·(v·dt)²/8`. Measured worst
one-step position error: 4.0e-2 m at `v = 4`, `dt = 0.1`, `Δkappa = 2`, which is exactly
the bound.

## 4. Limitations of the stagewise average

- **Intrinsic error per crossing.** The lateral error `O(Δkappa·(v·dt)²)` per crossing
  stage persists as an offset over the rest of the horizon. It shrinks with `dt²`, not with
  the integrator order.
- **Sensitivity to the nominal.** The schedule must come from a progress trajectory that
  matches the prediction. With a constant-speed nominal that is 10 % off, horizon position
  errors reach 1.2–1.5 m at `v = 3` (table in §5), because the predicted turns are shifted
  in time.
- **Fixed point may not converge.** Iterating the schedule to self-consistency converged
  in 277 of 290 cases. The median was 1 rollout. The 13 non-converged cases are perturbed
  steering inputs at higher speeds, and they produce most of the remaining outliers (up to
  1.1 m).
- **Short segments.** Segments shorter than `v·dt` (on the lab track, 0.37 m arcs against
  0.4 m per step at 4 m/s) put several jumps into one stage. The heading stays exact, but
  the averaged turn is a poor description of the path.

## 5. Numerical results

**Setup.**
- **Cases.** 5 transitions: straight→curve, curve→curve (κ +1→−1), curve→straight,
  lap boundary (κ 0→+1), and the legacy lab track with segments of 0.37–1.95 m and
  κ = ±1.43.
- **Grid.** 5 speeds (0.5–4 m/s), `dt` = 0.1 and 0.05 s, first crossing at 10 %, 50 % or
  90 % of the first interval, and 2 input families: centreline feed-forward along a
  constant-speed plan, and the same plus a 0.08 rad sinusoidal steering perturbation.
- **Totals.** 300 cases, of which 290 have a valid reference. Ten are excluded because the
  *reference* vehicle itself leaves the model domain (perturbed steering at 4 m/s on
  1 m-radius arcs) or the open track.
- **Prediction.** 2 s horizon, RK4 with one step per stage (as in the NMPC), identical
  initial state `[s0, 0, 0, v]` and inputs for every method.
- **Reference.** Event-aware RK4 with `max_step = dt/100`.

**Methods.**
- **A:** start-of-step curvature.
- **B:** stagewise average curvature, with the schedule from the nominal plan, after one
  refinement, *self-consistent* (fixed point), or from the reference's own endpoints
  ("oracle").
- **D:** event-split stages, with split times from the nominal plan (linear
  interpolation), self-consistent, or exact (from the reference).

### One-step vs horizon errors, feed-forward inputs (150 cases)

| method | one-step position max [m] | one-step heading max [rad] | horizon position median / max [m] | horizon heading max [rad] | horizon `ey` max [m] | horizon `s` max [m] | model-domain failures |
|:--|--:|--:|:-:|--:|--:|--:|--:|
| A start-of-step | 1.3e-1 | 6.3e-1 | 1.3e-1 / 3.3 | 6.9e-1 | 2.8 | 2.2 | 1 |
| B average, self-consistent | 4.0e-2 | 1.4e-12 | 2.2e-3 / 7.6e-2 | 1.7e-12 | 7.5e-2 | 6.7e-2 | 0 |
| B average, reference endpoints | 4.1e-2 | 4.0e-3 | 2.5e-3 / 1.3e-1 | 4.4e-2 | 9.0e-2 | 1.1e-1 | 0 |
| D split, nominal times | 1.1e-3 | 5.2e-3 | 2.4e-4 / 6.9e-2 | 3.7e-2 | 5.1e-2 | 6.9e-2 | 0 |
| D split, self-consistent | 1.6e-3 | 8.0e-3 | 5.6e-5 / 2.1e-2 | 1.2e-2 | 1.6e-2 | 2.0e-2 | 0 |
| D split, exact times | 2.1e-5 | 9.2e-6 | 1.2e-8 / 8.0e-5 | 6.2e-5 | 5.1e-5 | 6.1e-5 | 0 |

With feed-forward inputs the nominal plan is already self-consistent for B, so B's
"nominal", "1 refinement" and "self-consistent" rows are identical.

A **model-domain failure** means the rollout threw, or a predicted knot lies where
`1 − kappa(s)·ey ≤ eps_geometry` for the *actual* track, or beyond the end of an open
track. During development, one prediction with a wrong curvature schedule placed the
vehicle at `ey = −1.1 m` on a κ = −1 arc. The frame is invalid there, but the prediction's
own curvature did not detect it. These failures are only visible when predictions are
checked against the true geometry.

### Perturbed steering inputs (140 cases)

| method | horizon position median / max [m] | model-domain failures |
|:--|:-:|--:|
| A start-of-step | 1.3e-1 / 3.6 | 2 |
| B average, nominal plan | 2.1e-1 / 3.7 | 2 |
| B average, self-consistent | 1.7e-3 / 3.7 (converged cases: max 7.6e-2 overall) | 2 |
| B average, reference endpoints | 2.2e-3 / 2.1e-1 | 0 |
| D split, nominal times | 2.3e-1 / 3.8 | 3 |
| D split, self-consistent | 3.3e-3 / 1.8 | 3 |
| D split, exact times | 9.6e-7 / 2.2e-3 | 0 |

### Dependence on speed and `dt` (feed-forward inputs, max horizon position error [m])

| method | dt | v = 0.5 | 1 | 2 | 3 | 4 |
|:--|--:|--:|--:|--:|--:|--:|
| A start-of-step | 0.1 | 8.4e-2 | 2.9e-1 | 1.3 | 1.2 | 3.3 |
| A start-of-step | 0.05 | 4.3e-2 | 1.5e-1 | 4.1e-1 | 8.4e-1 | 1.5 |
| B self-consistent | 0.1 | 7.2e-4 | 2.8e-3 | 1.5e-2 | 2.6e-2 | 7.6e-2 |
| B self-consistent | 0.05 | 1.6e-4 | 7.2e-4 | 3.1e-3 | 9.8e-3 | 1.8e-2 |
| D self-consistent | 0.1 | 1.5e-5 | 2.1e-4 | 2.5e-3 | 1.3e-2 | 2.1e-2 |
| D self-consistent | 0.05 | 1.9e-6 | 3.0e-5 | 5.1e-4 | 1.7e-3 | 3.1e-3 |

For A, halving `dt` roughly halves the error (first order). For B it shrinks by about
4× (second order: `Δkappa·(v·dt)²`).

### Sensitivity to the nominal (v = 3 m/s, dt = 0.1, crossing at 0.5·dt; horizon position error [m])

| transition | nominal speed / actual | B average | D split |
|:--|--:|--:|--:|
| straight→curve | 0.9 / 1.0 / 1.1 | 1.4 / 1.4e-2 / 1.2 | 1.4 / 7.0e-4 / 1.2 |
| curve→curve (+1→−1) | 0.9 / 1.0 / 1.1 | 1.5 / 2.4e-2 / 1.3 | 1.5 / 4.4e-3 / 1.2 |

### Track-boundary violations

These count knots with `|ey| > 0.35 m` whose violation status differs from the reference,
over all 290 cases:

| method | mismatched knots |
|:--|--:|
| A | 515 |
| B, nominal | 291 |
| B, self-consistent | 11 |
| B, reference endpoints | 9 |
| D, nominal | 327 |
| D, self-consistent | 68 |
| D, exact times | 0 |

Figures:
- [errors along one horizon (κ +1→−1, v = 3 m/s, perturbed steering)](figures/multiseg_horizon_example.png)
- [one-step heading error vs speed and dt](figures/multiseg_speed_dt.png)

## 6. Cost of the reference integrator

Julia 1.11.5, one thread, mean wall-clock time:

| operation | µs |
|:--|--:|
| one RK4 step, constant curvature | 0.19 |
| `integrate_on_track`, 0.1 s with one crossing, `max_step` = 0.1 / 0.01 / 0.001 | 11 / 16 / 102 |
| 2 s reference (`simulate_on_track`, `max_step` = 0.001) | 1 900 |
| `curvature_schedule`, N = 20 | 2.4 |
| 20-stage rollout: A / B / D | 12 / 13 / 32 |

The event-aware integrator costs much more than a plain RK4 step. Each crossing needs a
root search of about 10–20 RK4 evaluations, plus bookkeeping. It is meant for the plant
simulation and for validation, not for use inside the optimizer.

## 7. When the average-curvature approximation performs poorly

- **High speed and large jumps.** The `Δkappa·(v·dt)²/8` lateral error per crossing is
  4 cm at `v = 4`, `dt = 0.1` and `Δkappa = 2`. That is comparable to the NMPC's 5 cm
  track margin.
- **Inconsistent schedules.** A schedule from a nominal that does not match the predicted
  progress (warm start after a disturbance, speed changes, large lateral motion) produces
  metre-level horizon errors.
- **No fixed-point convergence.** In the 13 non-converged cases (perturbed steering, higher
  speeds) the self-consistent iteration does not settle.
- **Short segments.** Segments shorter than one step contain several jumps per stage.

## 8. Projection and geometry notes

- **Back-projection.** Projecting the Cartesian reference with `cartesian_to_frenet(...;
  s_hint)` succeeded at every knot of all 290 cases. Hints are required: without them a
  point can be ambiguous wherever two parts of a track lie within `max_lateral` of each
  other.
- **Lab track.** The legacy lab track is not closed (5.5 cm gap; see `legacy_audit.md`),
  so it is open here, and long horizons hit its end (4 of the 10 excluded cases).
- **Tight arcs.** On the 1 m-radius arcs at 4 m/s, a 0.08 rad steering perturbation is
  enough to drive the true vehicle to `1 − kappa·ey ≤ 0.1` (6 excluded cases). These are
  physical limits of the scenario, not numerical failures.
- **Quarter track.** Its R = 1 m left/right S-bend has `Δkappa = 2`, the largest jump
  tested.

## 9. Recommendation for the JuMP predictor (for review)

**Recommended: method B, stagewise average curvature in the existing `kappa[k]`
Parameters, from a self-consistent schedule.** Specifically:

1. **Nominal.** Take the nominal progress from the warm-start trajectory: the shifted
   previous solution rolled out from the measured state, as `solve!` already does.
2. **Self-consistency before the solve.** Make the schedule self-consistent outside the
   NLP by iterating `curvature_schedule` → rollout until the progress stops changing.
   This needs median 1 rollout (~13 µs each), and a fixed small cap.
3. **Consistency check after the solve.** Compare the solution's predicted progress with
   the schedule's nominal. If they differ by more than a tolerance, the curvature
   parameters were stale. Either recompute and re-solve once, or reject or flag the
   solution, depending on the time budget.
4. **Margin.** Keep the track margin above the intrinsic bound `Δkappa·(v·dt)²/8` for the
   largest jump and speed in use, or use `dt = 0.05` on tracks with large jumps.

Reasons:

- **Accuracy.**
  - Heading is exact for a self-consistent schedule.
  - The horizon position error is ≤ 2.6 cm up to 3 m/s and 7.6 cm at 4 m/s (`dt = 0.1`),
    and 1.8 cm at 4 m/s with `dt = 0.05`. Method A's errors are metres.
- **Same NLP.** The NLP structure, size and smoothness are unchanged: only Parameter
  values change.
- **Why not D (event-split stages).** D is more accurate *only with accurate split
  times*. With nominal or self-consistent split times its horizon accuracy is within about
  3–4× of B in the feed-forward family (worst 2.1 cm vs 7.6 cm). It was also less robust: one converged
  self-consistent case reached 0.41 m. In JuMP the split times would be Parameters
  (sub-step durations `tau`, `dt − tau`). The stage that crosses changes between solves,
  so every stage would need the extra lifted RK4 sub-step.
- **Possible later refinement.** Split **only the first stage**. Its crossing time is
  known accurately from the measurement, and D's one-step position error is 30× smaller
  than B's (1.1e-3 vs 4.0e-2 m). The cost would be about 1/N extra.

This recommendation is for review. Nothing has been implemented in the controller.

## 10. Computational implications

These are estimates, not measurements of a modified controller.

- **B.** No change to the NLP. Extra work per control cycle:
  - schedule: ~2 µs;
  - fixed-point rollouts: ~13 µs each, median 1;
  - post-solve check: microseconds.

  This is negligible next to the measured 15–40 ms median Ipopt solves (Milestone 2A). A
  re-solve when the schedule is stale would double the time of that cycle, so its frequency
  must be measured in closed loop.
- **D, every stage split.** Each stage would carry two lifted RK4 sub-steps, roughly
  doubling the equality constraints (260 → ~520 for N = 20). Ipopt's per-iteration time
  has scaled with the constraint count in 2A (Euler, 80 constraints: ~1.05 ms per
  iteration; RK4, 260 constraints: ~2.0 ms). The expected cost is therefore ~2× per
  iteration, i.e. a median of roughly 60–75 ms on the 2A benchmark machine. That would put
  the 80 ms p99 target at risk.
- **D, first stage only.** About +5 % (1 of 20 stages).
- **Event-aware integrator.** About 2 ms per 2 s of simulated time at `max_step = 0.001`.
  It should serve as the closed-loop plant for multi-segment tracks.
