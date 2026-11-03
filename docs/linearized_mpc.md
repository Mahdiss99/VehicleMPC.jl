# Linearized vehicle MPC with JuMP and OSQP (Milestone 4A)

`LinearMPCController` is a successive-linearisation MPC. Each control cycle solves one or
more convex quadratic programs (QPs) with OSQP, and accepts a plan only after an independent
nonlinear validation. It is a separate controller: the nonlinear MPC (`NMPCController`),
its defaults, objective, vehicle equations and track geometry are unchanged.

**Scope (Milestone 4A).**
- This document describes the **straight track model**: a single straight segment, no
  obstacle.
- Milestone 4BC added the local track model for curved, multi-segment tracks and one
  stationary obstacle, in the same controller; see
  `docs/linearized_multicurvature_collision_avoidance.md`.
- With `track_model = :auto` (the default), the straight model is still used on a straight
  track without an obstacle. Its results are unchanged: the Milestone 4A closed loops
  reproduce bit for bit.
- Measured runtimes are measurements on one machine, **not a real-time guarantee**.

```julia
using VehicleMPC
vehicle = KinematicBicycle(reference_test_vehicle())
track = straight_track(400.0; width=1.0)
ctrl = LinearMPCController(vehicle, track, LinearMPCConfig(horizon=10, dt=0.1))   # optimizer=OSQP.Optimizer
r = solve!(ctrl, [0.0, 0.3, -0.2, 2.0]; previous_input=(0.0, 0.0))   # Frenet [s, ey, epsi, v]
r.accepted && println(r.u)
cl = simulate_closed_loop(ctrl, vehicle, [0.0, 0.3, -0.2, 2.0], 250)
```

## Model

**State, input and dynamics.**
- State `z = [px, py, psi, v]`: the rear-axle position, heading and speed in the track's
  Cartesian frame. Input `u = [delta, a]`.
- Dynamics: the exact zero-order-hold kinematic bicycle map `z⁺ = F(z, u)`
  (`exact_cartesian_step`, Milestone 2B.4), the same map as the Cartesian-endpoint NMPC.

**Linearisation.** At a nominal pair `(z̄_k, ū_k)`:

```
z_{k+1} = A_k z_k + B_k u_k + c_k,   A_k = ∂F/∂z,  B_k = ∂F/∂u,  c_k = F(z̄_k, ū_k) − A_k z̄_k − B_k ū_k
```

- `A_k` and `B_k` are computed by ForwardDiff on the exact map (`linearize_exact_map`).
- The tests compare them with central finite differences (agreement to 1e-7), verify the
  affine identity `A z̄ + B ū + c = F(z̄, ū)` to 1e-12, and check that the model error is
  quadratic in the perturbation.

**Track coordinates.** On a straight track with origin `o`, unit tangent `t`, left normal
`n` and heading `psi0`, the track coordinates are affine in `z`:
- `s = t·(p − o)`;
- `ey = n·(p − o)`;
- `epsi = psi − psi0`.

The corridor, the heading bound and the objective are therefore exact (not approximated) in
the QP.

## QP

Variables `Z` (4×(N+1)), `U` (2×N), and `uprev` (2, fixed to the previous input).

**Objective.** The same normalised progress objective and weights as the NMPC (`NMPCConfig`
defaults), with `E = width/2 − track_margin`, `s_scale = v_max N dt` and
`a_ref = max(−a_min, a_max)`:

```
J = −ws (s_N − s_0)/s_scale + Σ_{k=2}^{N+1} [qy (ey_k/E)² + qpsi (epsi_k/epsi_max)²]
    + Σ_{k=1}^{N} [rdelta (delta_k/delta_max)² + ra (a_k/a_ref)²
                   + rd_delta ((delta_k − delta_{k−1})/delta_max)² + rd_a ((a_k − a_{k−1})/a_ref)²]
```

with `(delta_0, a_0)` the previous input. It is a convex quadratic: the tests assemble its
Hessian and check that it is positive semidefinite. `linear_plan_objective` evaluates the
same function, including constants, on any trajectory. Evaluated on the NMPC's exact rollout,
it reproduces the NMPC's objective to 1e-6 (test).

**Constraints (all linear).**
- **Dynamics:** `Z[:, k+1] − A_k Z[:, k] − B_k U[:, k] = c_k`.
- **Measured initial state:** `Z[:, 1]` fixed to the measured state, converted from the
  Frenet measurement with `frenet_state_to_cartesian`.
- **Corridor:** `|n·(p_k − o)| ≤ E − mismatch_tol[1]` at `k = 2…N+1`. The corridor is backed
  off by the accepted position mismatch, so that an accepted prediction inside the corridor
  implies a rollout inside the planning corridor.
- **Bounds** (variable bounds):
  - `v ∈ [v_min, v_max]`;
  - `|delta| ≤ delta_max`;
  - `a ∈ [a_min, a_max]`;
  - `|psi − psi0| ≤ epsi_max`;
  - each intersected with the trust region (see below).
- **Lateral acceleration, affine approximation.** `ay = v² tan(delta)/L` is linearised at
  the nominal `(v̄, δ̄)` (`linearize_lateral_acceleration`):

  ```
  ay ≈ ḡ + g_v (v − v̄) + g_δ (δ − δ̄),   g_v = 2 v̄ tan δ̄ / L,   g_δ = v̄² (1 + tan² δ̄)/L
  ```

  - It is imposed as two inequalities `−ay_max ≤ ay_lin ≤ ay_max` at both ends of each
    interval (`v_k` and `v_{k+1}` with `δ_k`), as in the NMPC.
  - The approximation is exact only at the nominal point. The exact `|v² tan δ / L| ≤ ay_max`
    is checked by the nonlinear validation. A candidate that violates it is rejected, and
    the next QP is linearised at that candidate.

## Persistent model and coefficient updates

The JuMP model is built once (`build_qp!`), and `build_time` is recorded separately from the
solve time.

**Placeholder pattern.** At construction, every dynamics coefficient that can be nonzero
(all `A_k`, `B_k` entries) and both lateral-acceleration coefficients are present with
nonzero placeholder values. Every bounded variable has placeholder bounds.

**OSQP setup.** OSQP is then set up once with a one-iteration solve. OSQP fixes the sparsity
pattern of its constraint matrix at setup, and JuMP re-copies the whole model if a
constraint is added later. With the full pattern present, the per-cycle changes are all
in-place modifications (`set_normalized_coefficient`, `set_normalized_rhs`, `fix`,
`set_lower_bound`/`set_upper_bound`, `set_start_value`). This includes exact zeros such as
`∂px/∂py`.

**Per QP:**
- update the `A_k`, `B_k` coefficients and the `c_k` right-hand sides (`4N` rows);
- update the lateral-acceleration coefficients and right-hand sides (`4N` rows);
- update the measured state and the previous input (fixed variables);
- update the trust-region bounds;
- update the primal start.

Nothing is frozen via `value()` of an old model; the coefficients always come from the
current linearisation. `update_mode = :rebuild` builds a new model for every QP instead.
It exists only to measure the cost of reconstruction.

## Initialisation

- **Cold:** `nominal_inputs` applies pure-pursuit steering towards the centreline and
  `a = (v_max − v)/dt`, both clipped to the input and lateral-acceleration limits, along an
  exact rollout from the measured state. The nominal trajectory is dynamically consistent.
  The NMPC is not used.
- **Warm (`warm_start = true`, default):** `shifted_inputs` shifts the previous accepted
  inputs by one stage, re-rolls them with the exact map from the *new measured* state, and
  re-linearises.
  - The last stage is the pure-pursuit input at the end of the shifted rollout
    (`warm_tail = :pursuit`), or a repeat of the previous last input (`:repeat`).
  - After a rejected cycle the warm start is reset, and the next cycle starts cold.
- **OSQP primal start (`primal_start = :nominal`):** the nominal trajectory itself, which is
  dynamically consistent with the new measurement. With `:none`, OSQP starts from its
  previous iterate.

## Trust region and successive linearisation

**Trust region.** Componentwise physical radii around the nominal trajectory, scaled by
`ρ` (`ρ = 1` at the start of every cycle):
- `trust_region_z = (0.5 m, 0.5 m, 0.35 rad, 1.5 m/s)` for `(px, py, psi, v)`;
- `trust_region_u = (0.15 rad, 3 m/s²)` for `(delta, a)`.

They are imposed as variable bounds intersected with the physical bounds.

**Algorithm** (`solve!`, at most `max_qp_solves = 5` QPs):

1. Build the nominal trajectory (cold or warm), rolled out exactly from the measured state.
2. Linearise at the nominal trajectory, update the QP, and solve it with OSQP.
3. If OSQP does not return `OPTIMAL` with a feasible primal point, stop with `:qp_failure`.
   No control is returned.
4. Project the candidate inputs onto the actuator box. OSQP meets bounds only to its
   tolerance; violations of about 1e-6 were observed.
5. Run the independent nonlinear validation on the projected inputs (below), and compute the
   mismatch between the QP prediction and the exact rollout: `(max position distance, max
   |Δpsi|, max |Δv|)` over the knots.
6. **Accept** only if the mismatch is within `mismatch_tol = (1e-3 m, 1e-3 rad, 1e-3 m/s)` and
   the validation reports no violation. OSQP's success alone never accepts a candidate.
7. Otherwise:
   - the next nominal trajectory is the candidate's exact rollout (re-linearisation);
   - if the normalised mismatch `m = max(mismatch ./ tol)` exceeds 1, `ρ` is multiplied by
     `clamp(0.9 √(1/m), 0.1, 0.5)`. The linearisation error is quadratic in the step, so
     this aims the next step at the tolerance;
   - a rejection for violations alone keeps `ρ`.
8. **Time budget:** a further QP starts only if the elapsed cycle time plus the mean QP solve
   time so far fits within `sqp_time_budget = 80 ms`.
9. If no candidate was accepted, return `:not_converged` with the reason, and no control.

**Diagnostics** (`LinearMPCResult`):
- `status`, `accepted`, `u`;
- the QP prediction `Z`, `U`, the last nominal trajectory `Zbar`, `Ubar`, and the exact
  rollout `Zroll`;
- `mismatch` and `mismatch_history`;
- `violations`, `max_abs_ey`, `max_abs_ay`;
- per QP: `qp_status`, `qp_iterations`, `primal_residual`, `dual_residual`, `trust_scale`;
- `qp_solves`;
- the QP `objective` and the `nonlinear_objective` (the same function on the rollout);
- `timing` (`total`, `nominal`, `linearize`, `update`, `solve`, `validate`);
- `message`.

## Independent nonlinear validation

`validate_plan(vehicle, track, limits, z0, U)` is independent of the QP. It uses only the
inputs, and it is the same function used for the NMPC plans in the benchmark.

It rolls the inputs out with the exact map from `z0`, then checks the following, with
tolerance `validation_tol = 1e-4`:
- the planning corridor `|ey| ≤ width/2 − margin` at the knots;
- the physical track `|ey| ≤ width/2` along the rollout (`validation_substeps = 10` exact
  samples per interval);
- `v ∈ [v_min, v_max]` at the knots;
- the input limits;
- `|epsi| ≤ epsi_max`;
- the exact `|v² tan δ/L| ≤ ay_max` at both ends of each interval.

The closed loop additionally records plant operating-constraint violations at every plant
substep (`simulate_closed_loop`, unchanged).

## Results

Full tables: `benchmarks/results/linear_mpc.md` (`julia --project=. benchmarks/linear_mpc.jl`,
a few minutes, mostly compilation). Setup:
- 250 closed-loop cycles per configuration, one run each;
- straight track 400 m, width 1.0 m, reference vehicle, dt = 0.1 s;
- zero-order-hold RK4 plant;
- scenarios: **A** nominal progression from 1 m/s; **B** recovery from ey = 0.35 m,
  epsi = −0.25 rad at 2 m/s; **C** near the speed limit (4.9 m/s, ey = 0.1 m).

The baseline (**F**) is the Cartesian-endpoint Ipopt NMPC with the same horizon, dt,
weights and limits. Every accepted NMPC plan was passed through the same `validate_plan`.

**Outcome.**
- Every configuration completed 250/250 cycles with every cycle accepted:
  - LMPC warm, cold, rebuild, warm with a repeated tail, and warm without a primal start;
  - N = 10 and N = 20;
  - the NMPC.
- No accepted plan failed the nonlinear validation, and there were no plant
  operating-constraint violations.
- The largest accepted mismatch was 0.997 × tolerance (i.e. < 1 mm).

**Full-cycle timing, LMPC warm (default) vs NMPC, in ms:**

| N | scenario | LMPC median / p95 / p99 / max | NMPC median / p95 / p99 / max |
|---:|---|---|---|
| 10 | A | 0.7 / 1.6 / 3.3 / 3.8 | 3.7 / 7.7 / 11.4 / 13.6 |
| 10 | B | 0.4 / 1.7 / 1.9 / 17.8 | 3.2 / 3.8 / 4.2 / 10.1 |
| 10 | C | 0.4 / 1.6 / 1.8 / 2.0 | 3.5 / 4.2 / 4.8 / 14.0 |
| 20 | A | 0.8 / 1.3 / 2.8 / 15.9 | 5.9 / 7.1 / 11.0 / 14.6 |
| 20 | B | 0.8 / 1.1 / 3.1 / 12.0 | 6.0 / 7.0 / 10.7 / 14.5 |
| 20 | C | 0.8 / 1.1 / 2.5 / 10.3 | 5.9 / 6.8 / 10.1 / 14.6 |

- No LMPC persistent-mode cycle exceeded 80 ms.
- One `:rebuild` cycle took 103 ms.
- OSQP needed a median of 50 iterations per QP; the maximum was 700 for N = 10 and 475 for
  N = 20.

**Tracking.** The closed-loop trajectories match the NMPC's closely:

| scenario, N = 10 | RMS ey: LMPC / NMPC | max abs ay: LMPC / NMPC |
|---|---|---|
| B | 0.0355 / 0.0355 m | 1.64 / 1.66 m/s² |
| C | 0.0125 / 0.0127 m | 2.86 / 2.72 m/s² |

Both reach and hold v_max = 5 m/s, and both covered the same distance (123.4–125.0 m).

**Objective vs Ipopt** (same state, both controllers fresh, N = 10):
- the LMPC plan's nonlinear objective was within 6e-4 of the NMPC objective (relative
  6e-5);
- it was never better beyond solver tolerances;
- first controls differed by at most 1.6e-3.

**Update overhead (D).** Median per-cycle linearisation plus in-place updates: 0.2–0.35 ms.
With `update_mode = :rebuild`, the median cycle grows from 0.4–0.7 ms to 4.6–8.2 ms.

**Build time.**
- The first build in a Julia process takes 13.8 s and the first solve 3.7 s, including
  compilation.
- Afterwards, a build takes 4.6 ms (N = 10) or 5.9 ms (N = 20).

**Cold vs warm start (E).** Both work.
- The cold pure-pursuit nominal needed slightly fewer QP solves than the warm shift in
  B and C at N = 10. The mean QP solves were:

  | start | B | C |
  |---|---:|---:|
  | cold | 1.06 | 1.00 |
  | warm, pursuit tail (default) | 1.25 | 1.24 |
  | warm, repeated tail | 1.77 | 1.84 |

- At N = 20 the two are equal (1.00–1.01).
- Warm cycles needing re-linearisation took 4 QPs (21 and 20 of 250 cycles). The time is
  still small (p99 ≤ 3.1 ms).
- The explicit OSQP primal start (the dynamically consistent nominal) reduced the median
  OSQP iterations from 175–200 to 50.
- The shifted previous QP prediction, tried during development, was worse (median 350).
  It is not offered.

## Limitations

- **Straight tracks only** for this model. Curved tracks and a stationary obstacle use the
  local track model (Milestone 4BC). Moving obstacles are not implemented.
- **Local method.**
  - Acceptance guarantees, to the stated tolerances, that the plan's exact rollout
    satisfies the constraints at the knots and at the validation samples. It is not a
    continuous-time guarantee.
  - Acceptance is also not a guarantee of optimality. The plan is the solution of the last
    convexified QP, typically close to the NMPC solution (see the objective comparison).
- **Explicit failures.** A cycle that does not converge within `max_qp_solves` or the time
  budget fails explicitly. The closed loop then stops (`:controller_failure`); there is no
  fallback input.
- **Timing.**
  - All timings are from one Windows laptop and vary between runs (about 2× observed).
  - They are not real-time guarantees.
  - The first build and solve in a Julia process include compilation (seconds).
