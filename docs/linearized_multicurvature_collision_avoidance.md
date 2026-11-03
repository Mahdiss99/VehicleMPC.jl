# Linearized MPC on multi-curvature tracks with a stationary obstacle (Milestone 4BC)

**Status: experimental.** It is implemented in the existing `LinearMPCController` (JuMP +
OSQP) as the **local track model**. The straight track model of Milestone 4A is unchanged and
remains the default on a single straight segment without an obstacle; its closed-loop
results are bit-identical to Milestone 4A. The nonlinear MPC is unchanged.

Results: `benchmarks/results/linear_multicurvature_obstacle.md` (benchmark
`benchmarks/linear_multicurvature_obstacle.jl`). Figures:
`docs/figures/linear_multicurvature_trajectory.png` and
`docs/figures/linear_multicurvature_timing.png`.

**Scope.**
- One stationary circular obstacle.
- No moving obstacles, multiple obstacles or interaction.
- No continuous-time collision certificate.
- Measured runtimes are not a hard real-time guarantee.

```julia
using VehicleMPC
vehicle = KinematicBicycle(reference_test_vehicle())
track = quarter_track()
obstacle = obstacle_on_track(track, 3.5, 0.0; radius=0.10)
ctrl = LinearMPCController(vehicle, track, LinearMPCConfig(horizon=10, ego_radius=0.05, obstacle_buffer=0.03); obstacle)
cl = simulate_closed_loop(ctrl, vehicle, [0.0, 0.0, 0.0, 0.5], 250)
```

**Summary** (250 cycles per run, N = 10, dt = 0.1 s; obstacle radius 0.10 m, ego radius
0.05 m, buffer 0.03 m, so contact at 0.15 m and D = 0.18 m; one Windows laptop):

- **Quarter-track obstacle scenarios (Milestone 3B).** With M = 3 interior samples,
  **30 of 30 runs** (10 scenarios × 3) completed 250 cycles with every cycle accepted.
  - The bends: R = 1 m left, R = 1 m right (S-bend), R = 2 m, both offsets.
  - Both sides of the two curvature transitions, and the lap boundary.
  - The Ipopt NMPC (Milestone 3B.1 options) completed 9 of 10. E3 stops at cycle 118,
    as in 3B.1.
- **Safety on the plant** (LMPC):
  - no contact; minimum clearance above contact 0.026 m;
  - no physical-track or operating-constraint violation;
  - no accepted plan failed the independent validation.
  - Between constrained points the buffer was entered by at most 3.7 mm. This is the same
    contact-based acceptance as the NMPC, whose plans entered it by up to 4.1 mm.
- **Computation** (LMPC, N = 10, M = 3, stage C):
  - median 4.7–5.4 ms per cycle; p99 13.6–19.5 ms;
  - about 2 QP solves per cycle (at most 5); OSQP median 350–400 iterations per QP;
  - three cycles in 7 500 exceeded 100 ms (105, 125 and 137 ms). Each coincided with a
    100–128 ms garbage-collection pause;
  - the NMPC: median 9.9–10.8 ms, p99 22.6–46.4 ms, maximum 318 ms.
- **Progress:** the LMPC is more conservative than the NMPC.
  - 2.22–2.32 laps in 250 cycles against 2.46–2.58 (7–10 % less).
  - On the bends its plans have a higher original objective than fresh Ipopt plans at the
    same states (section 6).
- **Quarter track without an obstacle:** 2.34 laps (NMPC 2.58), RMS ey 0.135 m (NMPC
  0.202 m), median 3.7 ms (NMPC 6.2 ms).
- **Straight-track obstacle (Milestone 3A geometry):** the same progress as the NMPC
  (122.9 m), minimum clearance above contact 0.030 m, median 1.1 ms.
- **Interior samples matter.** With M = 0, 4 of 10 quarter-track scenarios failed:
  - three when the dense evaluation found contact between knots, which the validation
    correctly rejected;
  - one when the plant entered the buffer, after which the measured state violated D.
- **N = 20** (secondary): 10 of 11 configurations completed. R 2 m (D) stopped at cycle 68,
  when a cycle that included a 125 ms GC pause exhausted the 80 ms budget. Medians were
  9.2–12.6 ms.

## 1. Formulation

### State, input and dynamics

As in Milestone 4A:
- state `z = [px, py, psi, v]` (rear axle, Cartesian, unwrapped heading);
- input `u = [delta, a]`;
- dynamics: the exact zero-order-hold bicycle map `z⁺ = F(z, u)`, linearised at a nominal
  trajectory:

  ```
  z_{k+1} = A_k z_k + B_k u_k + c_k,   A_k = ∂F/∂z,  B_k = ∂F/∂u (ForwardDiff),  c_k = F(z̄_k, ū_k) − A_k z̄_k − B_k ū_k
  ```

### Local track reference (`track_reference`, `trajectory_references`)

**Projection.** Each nominal state `z̄_k` is projected onto the actual piecewise
constant-curvature track. The existing segment projection is used, unwrapped to the lap
nearest a progress hint:
- for knot 1, the hint is the measured progress;
- for knot `k + 1`, it is `s̄_k + |p̄_{k+1} − p̄_k| cos ēψ_k / (1 − κ_k ēy_k)`.

**Rejection, never a placeholder.** A `TrackReferenceError` is raised, and the attempt ends
with `:invalid_reference`, in these cases:
- there is no candidate within `max_projection_jump = 1 m` of the hint;
- two distinct candidates exist (ambiguous). Coinciding projections on both sides of a
  segment boundary count as one;
- `1 − κ ēy < min_frenet_factor = 0.2`. The factor is never divided by a near-zero value.

**Reference quantities.** For each knot: unwrapped progress `s̄`, lateral error `ēy`,
heading error `ēψ` (wrapped), the centreline point and unwrapped heading, unit tangent `t`,
left normal `n`, and signed curvature `κ`.

**Curvature convention (one-sided).** `κ` is the curvature of the segment containing `s̄`
under the half-open convention `[s_i, s_{i+1})`. Exactly at a boundary it is therefore the
downstream segment's curvature. Within a QP the reference is fixed. It is recomputed at
every successive linearisation, so a knot that moves across a boundary gets the other
segment's curvature at the next QP.

### Affine track coordinates

From `dp = (1 − κ e_y) t ds + n de_y`:

```
ŝ_k  = s̄_k + tᵀ(p_k − p̄_k) / (1 − κ_k ēy_k)
êy_k = ēy_k + nᵀ(p_k − p̄_k)
êψ_k = ēψ_k + (ψ_k − ψ̄_k) − κ_k (ŝ_k − s̄_k)
```

**Validation.** The coordinates are exact at the nominal point. Their gradients agree with
central finite differences of the true projection to 1e-6, for both curvature signs and
straight segments, away from segment transitions (tests). The error is quadratic in the
offset.

**In the QP.** The coordinates are auxiliary variables `S`, `EY`, `EP` (knots 2…N+1), tied
to the states by equality rows whose coefficients and right-hand sides are updated per QP.
The objective therefore never changes and stays convex.

### QP (`build_local_qp!`)

**Objective.** The NMPC objective (`NMPCConfig` weights and normalisation) in the affine
coordinates, plus the slack penalty:

```
J = −ws (S_N − s0)/(v_max N dt) + Σ_k [qy (EY_k/E)² + qpsi (EP_k/epsi_max)²]
    + Σ_k [rdelta (δ_k/δ_max)² + ra (a_k/a_ref)² + rd_delta (Δδ_k/δ_max)² + rd_a (Δa_k/a_ref)²]
    + slack_weight Σ σ²
```

Its Hessian is positive semidefinite (test).

**Constraints**, all linear:

| constraint | form |
|---|---|
| dynamics | linearised exact map; measured initial state and previous input fixed |
| track coordinates | the three equality rows above |
| corridor | `|EY| ≤ E − mismatch_tol − 0.005 + σ` (`E = width/2 − margin`) |
| heading | `|EP| ≤ epsi_max − 0.01 + σ` |
| lateral acceleration | linearised `v² tan δ / L`, at both interval end points: `|ay_lin| ≤ ay_max − 0.05 + σ` |
| speed, inputs | `v ∈ [v_min, v_max − 0.001]`, actuator box |
| trust region | intersected with the bounds above (section 2) |
| obstacle | supporting halfspaces (section 1.5), each `+ σ` |

**Slacks: which constraints are softened, which remain hard.**

| constraint | treatment |
|---|---|
| corridor `|EY|`, heading `|EP|`, linearised lateral acceleration, obstacle halfspaces | **softened**: `+ σ`, `σ ≥ 0`, penalty `slack_weight Σ σ²` (`slack_weight = 1e4`) |
| linearised dynamics, fixed measured state / previous input / progress, track-coordinate definitions | **hard** (equalities) |
| trust region, actuator box, speed bounds | **hard** (variable bounds) |

**Why the slacks help.**
- Successive linearisation often linearises at a trajectory that violates a constraint:
  the rollout of a rejected candidate, or a warm start that runs wide into a bend.
- With hard affine constraints and a small trust region, such a QP was often infeasible
  and ended the cycle (observed during development).
- With the slacks, the QP still returns a candidate. Its exact rollout is checked, and can
  move the next linearisation point towards feasibility.

**What the slacks do not guarantee.**
- *Feasibility.* The slacks do **not** make every QP feasible. A QP is guaranteed feasible
  only when the nominal trajectory satisfies the hard bounds (it is then itself a feasible
  point). The inputs satisfy the actuator box by construction; the speeds normally lie in
  `[v_min, v_max − 0.001]`. OSQP may still report infeasibility, or stop at its iteration
  limit.
- *Physical feasibility.* A softened QP solution is not necessarily physically feasible.
  The slack may be nonzero, the quadratic penalty is inexact (a violation of about
  λ/(2·slack_weight) for a multiplier λ), and the affine constraints are approximations
  anyway. The nonlinear rollout validation (section 3) is therefore mandatory: acceptance
  never relies on the QP's constraints or on the slack.

**What happens to each QP result:**
- *OSQP `OPTIMAL`:* the solution is a candidate.
- *`ALMOST_OPTIMAL` or the iteration limit (`local_osqp_max_iter = 1500`):* OSQP's last
  iterate is read directly, because OSQP's MOI wrapper reports no primal solution then. If
  it is finite, it is a candidate like any other.
- *Infeasibility or another status:* the attempt ends with `:qp_failure`.
- *Every candidate:* its inputs are first clipped to the actuator box (OSQP satisfies
  bounds only to its tolerance), then rolled out exactly and validated.
- *If no candidate of any attempt passes validation* within `max_qp_solves` and the
  80 ms budget, the cycle is rejected (`accepted = false`, `u = nothing`, an explicit
  status and `obstacle.failure_cause`). The warm start is reset, and no control is
  returned; `simulate_closed_loop` then stops with `:controller_failure`.

**Back-offs (`local_backoff`).** The QP-side tightenings (5 mm corridor, 0.01 rad heading,
0.05 m/s² lateral acceleration, 5 mm obstacle, 0.001 m/s speed) absorb three small errors
on short steps:
- the second-order error of the affine constraints;
- the inexact quadratic penalty, which permits a violation of about λ/(2·slack_weight);
- OSQP's tolerance.

The validation checks the untightened limits. These tightenings are one of the
differences from the NMPC, which imposes the constraints exactly.

### Obstacle halfspaces (`install_track_rows!`, `obstacle_direction`)

**Halfspace.** For the required separation `D = r_ego + r_obs + d_b` and a nominal point
`p̄`, the constraint is

```
nᵀ(p − o) ≥ D + mismatch_tol + 0.005,     n = (p̄ − o)/|p̄ − o|
```

Every point of the halfspace lies outside the disk of radius `D` around `o` (test). It is
therefore a conservative, convex inner approximation of the collision-free set at that
point.

**Degenerate direction.** If `|p̄ − o| < obstacle_min_direction = 1 mm`, the direction is
the track's left normal at the nominal reference, signed by the passing side (`:right`
gives `−n`). It is never the normalisation of a zero vector. Degenerate halfspaces are
counted in the result (`obstacle.degenerate_halfspaces`); a constraint is never dropped.

**Constrained points.**
- *Knots `k ≥ 2`:* the position is a decision variable.
- *Interior samples* at `τ_j = j/(M+1)·dt` (`obstacle_samples = M`, default 3): the exact
  partial-step position is linearised at the nominal state and input,

  ```
  p_k(τ) ≈ p̄_k(τ) + J_{z,k,τ}(z_k − z̄_k) + J_{u,k,τ}(u_k − ū_k)        (interior_jacobian, ForwardDiff)
  ```

  so each interior halfspace is one affine row in `(z_k, u_k)`. The Jacobian agrees with
  finite differences to 1e-7 (test).

No between-knot track-corridor constraints are added (by decision; see Milestone 2B.5).

### Persistent JuMP/OSQP updates

**What is updated in place, per QP:**
- the dynamics coefficients and right-hand sides;
- the lateral-acceleration rows;
- the track-coordinate rows (including the `κ` coefficient on `S`, which changes sign
  between the bends);
- the obstacle halfspace coefficients (directions rotate as the nominal trajectory passes
  the obstacle) and right-hand sides;
- the fixed measured state, previous input and measured progress;
- the trust-region bounds and the primal start.

**Sparsity pattern.** OSQP fixes the sparsity pattern of its constraint matrix at the first
solve (found in Milestone 4A). Every position that can ever be nonzero is therefore
created with a nonzero placeholder:
- all `A`/`B` entries;
- `t`, `n` and `κ` in the track rows;
- all six `(z_k, u_k)` coefficients of each interior halfspace;
- two position coefficients of each knot halfspace.

Every bounded variable has placeholder bounds. A one-iteration solve then sets OSQP up
before any coefficient can become exactly zero.

**Checks.** The tests run 45 consecutive cycles across straight → R +1 → R −1 → R 2 m
segments with one model object, and verify the installed coefficients. The benchmark runs
thousands of cycles without a model rebuild.

`update_mode = :rebuild` (a new model per QP) remains available for comparison. Its cost is
included in the measured cycle time.

## 2. Successive linearisation (`sqp_attempt!`)

Up to `max_qp_solves = 5` QPs per attempt. All attempts of a cycle share
`sqp_time_budget = 80 ms`.

**Each QP:**
1. project the nominal trajectory (references);
2. linearise the dynamics, lateral acceleration, track coordinates and halfspaces;
3. set the trust region and solve with OSQP;
4. project the inputs onto the actuator box;
5. roll them out exactly and validate them;
6. compute the mismatch between the QP prediction and the rollout (knots and the affine
   interior samples).

**Acceptance:** mismatch ≤ `mismatch_tol` (1 mm, 1 mrad, 1 mm/s) and no violation.

**Trust region.** It has componentwise radii (as in 4A), scaled by `ρ`. The local model
uses the `:trust_region` step policy, whose rules were each prompted by a measured failure
mode:

| rule | reason |
|---|---|
| A rejected candidate with a large mismatch and a violating rollout is not automatically the next linearisation point. The point with the smaller relative constraint excess is used: the current nominal (the step is retracted) or the rollout. | A long step through an infeasible region otherwise became the linearisation point (lateral acceleration over the limit by 1.3 m/s²), and the next QP was infeasible. |
| `ρ` shrinks by `clamp(0.9√(tol/mismatch), 0.1, 0.5)` after a mismatch rejection, and by 0.5 after a rejection for violations alone. | Without the second rule, a QP that rides an affine constraint (lateral acceleration) failed validation by the same second-order error at every iteration. |
| `ρ` is carried across cycles: each cycle starts at `min(1, 2·ρ_accepted)` unless its nominal violates a constraint by more than 2 % (relative), in which case it starts at 1. | With `ρ = 1` at every cycle, the objective's linear progress term pushed the first QP to the trust-region boundary in almost every cycle (mismatch ≈ 0.1 m), so most cycles needed 3 QPs. |
| Inexact OSQP results (`ALMOST_OPTIMAL`, iteration limit at `local_osqp_max_iter = 1500`, `eps = 1e-5`) are still evaluated as candidates; OSQP's last iterate is used when its wrapper reports no primal solution. | Acceptance comes from the nonlinear validation, never from the solver status. |

The track-linearisation error (affine `ŝ`, `êy`, `êψ` against the true coordinates of the
QP prediction) is reported in `result.track_error`. It is a diagnostic, not an acceptance
criterion.

## 3. Nonlinear validation (`validate_plan_on_track`)

It is independent of the QP: it uses only the inputs, and is the same function for the LMPC
and the NMPC plans.

**Rollout.** The exact Cartesian rollout from the measured state. Every knot and 10 samples
per interval are projected onto the actual track (lap-unwrapped, chained hints).

**Checks:**
- the planning corridor and heading error at the knots;
- the physical track at every sample;
- a valid projection everywhere (a plan that leaves the track has none);
- the speed, against the plant tolerance `acceptance_tol`;
- the inputs;
- the exact lateral acceleration at both interval ends.

**Obstacle:**
- the distance at the constrained points (knots and interior samples) must be at least
  `D`;
- the dense evaluation (50 samples per interval) must have no contact
  (`distance < r_ego + r_obs`);
- separation violations between constrained points (inside the buffer, without contact)
  are counted and reported. They are rejected only with `obstacle_acceptance = :separation`,
  as in the NMPC.

This is a finite sampling, not a continuous-time certificate.

## 4. Initialisation and passing side

**Cold start.**
- *Steering:* pure pursuit towards the actual centreline point at `s + ld`. The look-ahead
  is limited by the curvature within 2 m. On a constant-curvature arc, from the centreline,
  this gives `δ = atan(Lκ)`, the curvature feedforward; off the centreline it adds the
  feedback.
- *Speed:* capped over the next `v + 0.5` m by `sqrt(0.9 ay_max/|κ|)`, with braking room.
- *Rollout:* exact, with chained projections across straight-to-curve, opposite-sign and
  lap-boundary transitions.

**Warm start.** The shifted previous inputs plus a pursuit tail. If their rollout violates
a constraint by more than 2 %, the cold nominal is also evaluated, and the one with the
smaller excess is used (`default_nominal_local`).

This matters because a plan that is valid over its own horizon can end too fast for the
next bend: the MPC has no terminal constraint. On an R = 2 m circle the shifted plan's last
knot was then 0.17–0.25 m outside the corridor in every cycle.

**Obstacle encounters** (`solve_local!`). The relevance, association and gap heuristics are
the NMPC's (`obstacle_associations`, `obstacle_frenet`, `passage_sides`). They now accept
either controller; the NMPC behaviour is unchanged.

At a new encounter, `side_guess` (also shared) builds the left and right passing guesses.
Their inputs are re-rolled with the exact Cartesian map, so each nominal trajectory is
dynamically consistent and its halfspaces separate it from the obstacle. Then:
- `encounter_strategy = :sequential` (default): the higher-ranked side is solved first; the
  other only if it is rejected and the budget allows. This chooses a feasible side; it does
  not compare the two local optima. The ranking is the NMPC's: guess rollout succeeded,
  then larger clearance, then larger track clearance, then left;
- `:both`: both sides are solved, and the lower nonlinear objective is kept.

**Later cycles.** The warm start keeps the side. If it is rejected, one attempt is made from
the remembered side's guess (budget permitting). Ipopt is never called.

**Failure classes** (`obstacle.failure_cause`): `:infeasible_geometry` (no passage by the gap
heuristic), `:collision`, `:obstacle_constraint`, `:qp_infeasible`, `:invalid_reference`,
`:qp_failure`, `:not_converged`, `:budget`, `:initial_collision`.

## 5. Experiments

Full tables: `benchmarks/results/linear_multicurvature_obstacle.md`. Figures:
`docs/figures/linear_multicurvature_trajectory.png` (lap 1 of both controllers for B, C, E3
and F) and `docs/figures/linear_multicurvature_timing.png` (computation-time distributions,
and the per-cycle breakdown for B).

**Setup.** One Windows laptop, Julia 1.11.5.
- **Runs.** 250 cycles per run, after a warm-up of every code path in the same process.
  Three runs of each primary LMPC configuration; one NMPC run.
- **Plant.** Event-aware, 10 substeps per period.
- **NMPC.** Cartesian-endpoint model, ≤ 8 reassignments within the 80 ms correction budget,
  M = 3, with the Milestone 3B.1 cycle detection, branch resolution and sequential
  encounters.
- **Common to both.** The same vehicle, track, obstacle, start state, horizon, period,
  objective weights and limits. Both controllers' plans pass through the same
  `validate_plan_on_track`.

### Stage A: quarter track, no obstacle (N = 10)

| | LMPC (3 runs) | NMPC |
|---|---|---|
| outcome | 2.34 laps each | 2.58 laps |
| first lap [s] | 10.6 | 9.8 |
| RMS / max abs ey (plant) [m] | 0.135 / 0.296 | 0.202 / 0.304 |
| first-step prediction error, max [mm] | 0.037 | 1e-5 |
| prediction vs exact rollout, max [mm] | 0.999 (bounded by the 1 mm tolerance) | 1e-5 |
| cycle median / p99 / max [ms] | 3.7 / 11.8 / 20.8 | 6.2 / 14.6 / 23.2 |
| solves per cycle (mean / max) | 1.95 / 3 QPs | 1.13 / 2 NLPs |

The LMPC stays inside the planning corridor (|ey| ≤ 0.296 m). The NMPC leaves it between
knots by up to 4 mm (known since Milestone 2B.5; the physical edge is 0.35 m).

### Stage B: straight track, obstacle at (8, 0) (Milestone 3A geometry)

| | LMPC M = 3 (3 runs) | LMPC M = 0 | NMPC M = 3 | NMPC M = 0 |
|---|---|---|---|---|
| distance in 250 cycles [m] | 122.9 | 122.9 | 122.9 | 122.9 |
| min clearance above contact / above D [m] | 0.0299 / −0.0001 | 0.0299 / −0.0001 | 0.0243 / −0.0057 | 0.0243 / −0.0057 |
| median / p99 / max [ms] | 1.1 / 8.0 / 14.2 | 0.9 / 6.8 / 9.3 | 5.8 / 15.0 / 45.7 | 4.1 / 14.2 / 33.4 |

The halfspaces were updated every cycle while passing: the directions rotate with the
nominal points. A −0.1 mm buffer clearance is within the validation tolerance.

### Stage C: quarter-track obstacle scenarios (N = 10)

| scenario | LMPC M = 3: laps (3 runs) | min clearance contact / D [m] | median / p99 [ms] | NMPC: laps | NMPC clearance contact / D [m] | LMPC M = 0 |
|---|---|---|---|---|---|---|
| B R 1 m left, s = 3.5 | 2.26 ×3 | 0.034 / 0.004 | 5.0 / 19.5 | 2.46 | 0.030 / −0.001 | 2.26 laps |
| C R 1 m right, s = 6.6 | 2.22 ×3 | 0.033 / 0.003 | 4.7 / 19.2 | 2.53 | 0.030 / −0.001 | 2.24 laps |
| D R 2 m, s = 9.7 | 2.28 ×3 | 0.035 / 0.005 | 4.7 / 16.4 | 2.58 | 0.144 / 0.114 | 2.27 laps |
| E1 before straight→R 1, s = 1.95 | 2.32, 2.30, 2.32 | 0.030 / −0.000 | 5.2 / 19.2 | 2.54 | 0.026 / −0.004 | **stop c8: collision** |
| E2 after straight→R 1, s = 2.10 | 2.32 ×3 | 0.052 / 0.022 | 5.0 / 15.3 | 2.52 | 0.029 / −0.001 | 2.33 laps |
| E3 before R −1→R 2, s = 8.20 | 2.30 ×3 | 0.029 / −0.001 | 5.4 / 19.5 | **stop c118: segment assignment** | – | **stop c139: initial collision** |
| E4 after R −1→R 2, s = 8.40 | 2.32 ×3 | 0.032 / 0.002 | 5.0 / 13.6 | 2.52 | 0.027 / −0.003 | 2.32 laps |
| F lap boundary, s = L − 0.3 | 2.32 ×3 | 0.026 / −0.004 | 4.9 / 16.9 | 2.58 | 0.036 / 0.006 | **stop c204: collision** |
| G1 offset +0.08 on R 1 left | 2.29 ×3 | 0.034 / 0.004 | 5.0 / 17.0 | 2.48 | 0.029 / −0.001 | **stop c220: collision** |
| G2 offset −0.08 on R 1 right | 2.23 ×3 | 0.034 / 0.004 | 5.1 / 19.1 | 2.46 | 0.030 / −0.000 | 2.23 laps |

**Every scenario, both controllers:** no plant violation, and no accepted plan failed the
shared validation.

**Passing sides.** The LMPC passed C and E3 on the other side from the NMPC (trajectory
figure). With the sequential strategy the first feasible side is kept; the two local optima
are not compared.

**Failure causes:**
- **E3 (NMPC):** a knot outside its assigned segment whose correction was skipped by the
  80 ms budget; the 3B.1 finding.
- **E1, F, G1 (LMPC, M = 0):** with constraints at the knots only, every candidate of the
  cycle had contact in the dense evaluation (minimum distance 0.084–0.149 m < 0.15 m), so
  the cycle was rejected, as intended: the collision-tunnelling case of Milestone 3A.
- **E3 (LMPC, M = 0):** between knots the plant passed up to 2.7 cm inside D, without
  contact. A measured state at a sampling instant then lay 0.1 mm inside D and was rejected
  (`:initial_collision`).

### Stage D: N = 20 (secondary)

Ten of the 11 configurations completed:
- quarter track without an obstacle: 2.23 laps;
- straight obstacle: 122.9 m;
- nine quarter scenarios: 2.10–2.22 laps.

Medians were 9.2–12.6 ms on the quarter track. **D (R 2 m)** stopped at cycle 68 with
`:not_converged`: that cycle took 130.7 ms, including a 125 ms garbage-collection pause, so
the 80 ms budget allowed no further QP. The longer horizon was slower and made less
progress than N = 10. It was not tuned further (N = 10 is the primary target).

### Computation-time breakdown (LMPC, median per cycle, stage C, M = 3)

| part | time [ms] |
|---|---|
| nominal trajectory | 0.21–0.23 |
| track projections (references and track-error diagnostics) | 0.12–0.13 |
| linearisation (ForwardDiff) | 0.04 |
| JuMP/OSQP coefficient updates | 0.66–0.71 |
| OSQP solves | 2.90–3.44 |
| exact rollout + validation (incl. 50-sample collision check) | 0.47–0.51 |
| **total** | **4.7–5.4** |

**Build and compilation.**
- After warm-up, a build takes 2.8 ms (N = 10, obstacle) or 4.5 ms (N = 20).
- The first build and solve in a fresh Julia process take 6.7 s and 5.7 s (compilation).

**Garbage collection.** Garbage-collection time per 250-cycle run was 118–265 ms. Single
pauses of 100–128 ms occurred in 3 of the 30 primary runs, and produced the only cycles
above 100 ms. These are runtime-environment effects measured on one machine, not a hard
real-time property.

## 6. Differences from the NMPC caused by the approximation

- **QP-side tightenings.**
  - *Corridor (5 mm + 1 mm).* The LMPC's plant trajectories stay within |ey| ≤ 0.300 m
    against the NMPC's 0.303–0.306 m.
  - *Obstacle (5 mm + 1 mm), lateral acceleration (0.05 m/s²), heading (0.01 rad), speed
    (1 mm/s).* The NMPC imposes the exact constraints, at its knots only.
- **Track and constraint linearisation.**
  - Track coordinates, lateral acceleration and collision halfspaces are first-order models
    within a trust region.
  - A halfspace is a conservative inner approximation of the collision-free set.
  - All are rechecked exactly on the rollout.
  - The track-linearisation error of accepted plans is reported per cycle
    (`track_error`).
- **Progress.**
  - The SQP stops at the first candidate that passes validation, with a step limited by
    the trust region; it does not iterate to a local optimum.
  - In closed loop the LMPC made 7–10 % less progress on the quarter track than the NMPC.
  - At the same states, its in-loop plans had an original objective 0.26–1.56 higher
    (worse) than freshly computed Ipopt plans on the bends. That is roughly 5–35 % of a
    typical objective value of −3 to −9.
  - On the straight track the difference is ≤ 0.015 (the plans coincide to 0.01 in the
    inputs).
  - These are the original nonlinear objective on each plan's exact rollout. The QP's
    surrogate objective is not compared with Ipopt's.
- **Prediction.**
  - The LMPC's predictions differ from the exact rollout by up to the 1 mm acceptance
    tolerance; the NMPC's exact-map predictions by 1e-5 mm.
  - First-step errors against the plant were below 0.06 mm for both.
- **Passing side.** The sequential strategy keeps the first feasible side. In C and E3 the
  LMPC chose the opposite side from the NMPC.

## 7. Failure modes and limitations

- **Local method, no guarantees.**
  - Acceptance guarantees, to the stated tolerances, the true constraints at the knots, at
    the validation samples (10 per interval for the track, 50 for the obstacle), and the
    separation at the constrained points.
  - It is **not** a continuous-time collision certificate. Between constrained points the
    buffer can be entered: by ≤ 3.7 mm (M = 3), and by ≤ 2.7 cm with M = 0.
  - It is not a recursive-feasibility guarantee: there is no terminal constraint.
  - It is not globally optimal avoidance. The passing side is chosen heuristically
    (sequential strategy) or by comparing two local optima (`:both`).
- **Measured state inside the buffer.** As in the NMPC, a measured state inside D (even by
  0.1 mm, without contact) is rejected as `:initial_collision`. This ended the M = 0 run of
  E3.
- **Explicit failures, no fallback.** A cycle that is not accepted within 5 QPs per attempt
  and the 80 ms budget is rejected with its cause, and the closed loop stops.
  - *QP difficulty.* On curved tracks OSQP needs a median of 350–400 iterations per QP
    (about 50 on the straight track), and QPs often reach the 1 500-iteration cap. The
    lateral-acceleration rows near the limit are nearly parallel. The iterate is then
    validated like any candidate.
  - *Timing.* One measured failure (N = 20, scenario D) was caused by a garbage-collection
    pause.
- **Not optimal.** Progress is 7–10 % lower than the NMPC on the quarter track (section 6).
- **Tuning.** Trust-region memory (×2 growth, 2 % excess threshold), back-offs and slack
  weight were chosen from the measured behaviour described in section 2. They are not
  optimised, and were tested only on the reference vehicle, the quarter track and the 3A
  straight track.
- **Projection.**
  - The local reference needs an unambiguous projection within 1 m of the progress hint,
    with `1 − κ ey ≥ 0.2`. Tracks whose parts come within the corridor width of each other
    at nearby progress are rejected rather than handled.
  - Exactly at a curvature discontinuity, the downstream curvature is used.
- **Scope.**
  - One stationary circular obstacle and a disk ego proxy. No moving or multiple
    obstacles, no interaction.
  - No between-knot track-corridor constraints (by decision).
  - The straight 3A geometry was run on a 140 m track so that 250 cycles fit.
- **Timing.** One Windows laptop. The first build/solve in a process includes compilation
  (seconds). Timings vary between runs (about 2× observed on this machine) and include
  garbage-collection pauses. **No hard real-time guarantee** is claimed.
