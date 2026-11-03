# Public controller API

The package provides `DynamicNMPCController` (six-state dynamic bicycle, JuMP + Ipopt),
`NMPCController` (kinematic bicycle, JuMP + Ipopt), and `LinearMPCController` (successive
linearization of the kinematic bicycle, JuMP + OSQP). Their measurement coordinates differ.
Public names have REPL docstrings, for example `?DynamicNMPCConfig`.

## Dynamic bicycle controller

The measured state is the Cartesian centre-of-mass state `z = [X, Y, psi, vx, vy, r]`;
the input is `u = [delta, a]`. The following is an offline example with no retry time
budget. This does not establish real-time performance.

```julia
using VehicleMPC
parameters = showcase_dynamic_parameters()
vehicle = DynamicBicycle(parameters)
track = quarter_track()
Xr, Yr, psi = frenet_to_cartesian(track, 0.0, 0.0, 0.0)
X, Y, psi = cg_pose_from_rear_axle(parameters, Xr, Yr, psi)
z0 = [X, Y, psi, 1.0, 0.0, 0.0]
obstacles = [obstacle_on_track(track, 3.6, 0.06; radius=0.10)]
config = DynamicNMPCConfig(time_budget=Inf)
ctrl = DynamicNMPCController(vehicle, track, config; obstacles)
result = solve!(ctrl, z0; s_hint=parameters.lr)
if result.accepted
    input = result.u
else
    error(result.message)  # No rejected control is applied.
end
```

The dynamic controller accepts a fixed set of stationary circular obstacles at
construction. Its state uses the centre of mass; the four-state controllers below use
the rear axle. `simulate_dynamic_closed_loop` integrates a `DynamicBicycle` plant;
`simulate_closed_loop` integrates a `KinematicBicycle` plant. Both accept an optional empty
`cycle_times=Float64[]` vector for timing complete `solve!` calls. Plant integration,
export and rendering are excluded from these timing samples.

See [the dynamic model and MPC formulation](dynamic_bicycle_mpc.md) for parameter units,
acceptance checks, model validity and the racing showcases. Sections 1 onward below
describe the two kinematic controllers.

The mathematical formulations are in:
- `docs/nmpc_formulation.md` and `docs/cartesian_endpoint_nmpc.md` (nonlinear);
- `docs/linearized_mpc.md` and `docs/linearized_multicurvature_collision_avoidance.md`
  (linear).

## 1. Vehicle, track and state conventions

```julia
using VehicleMPC
vehicle = KinematicBicycle(reference_test_vehicle())   # L = 0.25 m, |δ| ≤ 25°, a ∈ [−5, 5] m/s², v ∈ [0, 5] m/s
track = quarter_track()                                # or straight_track(len; width), circular_track(R; width), Track([...])
```

**Model.** A kinematic bicycle referenced at the rear axle; see
`docs/model_conventions.md` for signs, frames, units and domain handling.

**Measured state, both controllers: Frenet `x = [s, ey, epsi, v]`.**
- `s`: progress along the centreline [m], **unwrapped** on closed tracks (it keeps
  increasing over laps);
- `ey`: lateral offset along the left normal [m];
- `epsi`: heading error [rad];
- `v`: speed [m/s].

**Input, both controllers: `u = [delta, a]`** (steering angle [rad], acceleration
[m/s²]).

**Internal state representation.**
- `NMPCController` predicts in Frenet coordinates (`prediction_model = :frenet`, the
  default). With `prediction_model = :cartesian_endpoint_exact`, it links the knots with
  the exact Cartesian map through segment-local formulas, but still reports `X` in Frenet
  coordinates.
- `LinearMPCController` works in Cartesian coordinates `z = [px, py, psi, v]` (rear axle,
  unwrapped heading). It converts the measurement internally, and reports its predictions
  `Z` in Cartesian coordinates.

**Conversions** with the existing track utilities:

```julia
z = frenet_state_to_cartesian(track, x)                 # [s, ey, epsi, v] -> [px, py, psi, v]
X, Y, psi = frenet_to_cartesian(track, s, ey, epsi)     # pose only
s, ey, epsi = cartesian_to_frenet(track, X, Y, psi; s_hint=s_prev)   # unique projection or nearest to the hint
ref = track_reference(track, X, Y, psi; s_hint=s_prev)  # (s, ey, epsi, kappa, t, n, ...) with explicit ambiguity checks
```

`cartesian_to_frenet` without a hint requires a unique projection. Pass `s_hint` on closed
tracks to obtain unwrapped progress near the previous value.

## 2. Construction and configuration

```julia
ctrl = NMPCController(vehicle, track, NMPCConfig(; kwargs...); obstacle=nothing)       # optimizer = Ipopt.Optimizer
ctrl = LinearMPCController(vehicle, track, LinearMPCConfig(; kwargs...); obstacle=nothing)   # optimizer = OSQP.Optimizer
```

**Shared settings.** Both configurations share the objective weights (`ws, qy, qpsi,
rdelta, ra, rd_delta, rd_a`), `dt = 0.1`, `track_margin = 0.05`, `epsi_max = 60°`,
`ay_max = 4.0` and the obstacle fields (`ego_radius`, `obstacle_buffer`,
`obstacle_samples`, `obstacle_check_samples`, `obstacle_acceptance`, `encounter_strategy`).
Defaults are documented in the docstrings.

**Differing defaults** (not unified, for backwards compatibility):

| setting | `NMPCConfig` | `LinearMPCConfig` |
|---|---|---|
| `horizon` | 20 | 10 |
| `obstacle_samples` | 0 | 3 |
| `ego_radius`, `obstacle_buffer` | 0.0 | 0.0 |
| `encounter_strategy` | `:both` | `:sequential` |
| acceptance tolerance | `acceptance_tol = 1e-6` (knots) | `validation_tol = 1e-4` (validation); `acceptance_tol = 1e-6` (plant recorder and speed) |
| time budget per cycle | `correction_time_budget = 0.08`, `cycle_budget = Inf` | `sqp_time_budget = 0.08` |

**NMPC configurations used in the documented experiments:**
- straight and circular tracks: `NMPCConfig()`;
- the quarter track and obstacles: `prediction_model = :cartesian_endpoint_exact,
  schedule_corrections = 8`. Add `reassignment_cycle_detection = true,
  branch_resolution = true, encounter_strategy = :sequential` for the Milestone 3B.1
  options.

See `examples/nmpc_obstacle.jl`.

**`LinearMPCConfig().track_model = :auto`:**
- `:straight` (the Milestone 4A QP) on a single straight segment without an obstacle;
- `:local` (curved tracks and/or an obstacle) otherwise.

**Obstacles.** One stationary disk:

```julia
obstacle = CircleObstacle(x=..., y=..., radius=...)            # Cartesian centre
obstacle = obstacle_on_track(track, s, ey; radius=0.10)        # placed with track coordinates
set_obstacle!(ctrl, new_obstacle)                              # controllers built with an obstacle only
required_separation(obstacle, ego_radius, buffer)               # D = r_ego + r_obs + d_b
```

The ego vehicle is a disk of radius `ego_radius` centred at the rear axle: a collision
proxy, not a vehicle-body model. **Contact** means a centre distance below
`ego_radius + radius`. The **required separation** `D` adds the buffer.

## 3. `solve!`

```julia
r = solve!(ctrl, x; previous_input=(0.0, 0.0))
```

- `x`: the measured Frenet state (section 1).
- `previous_input`: the input applied over the previous period; it enters the rate
  penalties. Pass the control you actually applied (or `(0, 0)` at the start).
- `r.accepted`: whether the cycle produced an acceptable plan.
- `r.u`: the first control `[delta, a]`, or `nothing` if the cycle was rejected. **Apply
  `r.u` only when `r.accepted` is true.**
- **Warm starts.** The controller keeps its own warm start. A rejected cycle resets it, so
  the next cycle starts cold. `reset_warm_start!(ctrl)` resets it explicitly (for example
  after a jump in the measured state).

### Solver success is not acceptance

A cycle is accepted only after checks that are independent of the optimiser's status:

| | nonlinear MPC | linear MPC |
|---|---|---|
| solver status | `r.termination_status`, `r.primal_status` (Ipopt via MOI; `LOCALLY_SOLVED` is normal) | `r.qp_status` (one per QP; `OPTIMAL`, `ALMOST_OPTIMAL`, `ITERATION_LIMIT`, …) |
| independent checks | knot residuals, curvature-schedule / segment-assignment consistency, event-aware re-simulation on the actual track (`r.reference`), exact-map check, obstacle collision check (`r.obstacle`) | exact Cartesian rollout of the (box-clipped) inputs; projection on the actual track; corridor, track, heading, speed, input, lateral-acceleration and obstacle checks (`r.violations`); prediction mismatch ≤ `mismatch_tol` (`r.mismatch`) |
| accepted when | `r.status ∈ (:success, :success_stale_schedule)` | `r.status === :success` |

**Consequences:**
- An `OPTIMAL`/`LOCALLY_SOLVED` solve can be **rejected**: for example a segment-assignment
  error or a collision between knots (NMPC), or a mismatch or violation on the exact
  rollout (linear MPC).
- The linear MPC may also **accept** a candidate from an inexact OSQP result
  (`ALMOST_OPTIMAL`, iteration limit) when its exact rollout passes every check.
- When nothing is accepted, `r.u === nothing` and `r.status` names the reason. There is
  no fallback control.

### Status values

**Common:**
- `:success`;
- `:invalid_measurement` (state or previous input outside the model domain; no solve);
- `:initial_collision` (the measured disk is inside D; no solve).

**Nonlinear MPC:**
- `:solver_failure`, `:invalid_solution`;
- `:schedule_inconsistent`, `:reference_violation`;
- `:segment_assignment_invalid`, `:interior_assignment_invalid`;
- `:corridor_violation`;
- `:obstacle_constraint_violation`, `:collision_detected`;
- `:success_stale_schedule` (accepted).

See `?NMPCResult`.

**Linear MPC:**
- `:qp_failure` (OSQP returned infeasibility or another unusable status);
- `:not_converged` (no candidate passed validation within `max_qp_solves` and the time
  budget);
- `:invalid_reference` (no valid, unambiguous track projection of the nominal trajectory).

See `?LinearMPCResult`. With an obstacle, `r.obstacle.failure_cause` classifies a rejected
cycle for both controllers. The values differ slightly between them (see the docstrings).

## 4. Result types

**`NMPCResult`** (`?NMPCResult`):
- outcome: `status`, `accepted`, `u`, `message`;
- prediction: `X` (Frenet, 4×(N+1); `X[:, 1]` is the measurement), `U` (2×N);
- solver: `objective`, `iterations`, `nlp_solves` (NLP solves in the cycle);
- checks: `max_dynamics_residual`, `max_constraint_violation`, `warm_started`;
- `timing` (`NMPCTiming`);
- diagnostic reports: `schedule`, `reference`, `exact`, `corridor`, `obstacle`
  (`ObstacleReport`), `reassignment`.

**`LinearMPCResult`** (`?LinearMPCResult`):
- outcome: `status`, `accepted`, `u`, `message`;
- prediction: `Z` (Cartesian QP prediction), `U`;
- nominal and rollout: `Zbar`, `Ubar` (last linearisation point), `Zroll` (exact rollout
  of `U`), `rollout_frenet` (its true track coordinates);
- checks:
  - `mismatch` (position, heading, speed difference between `Z` and `Zroll`) and
    `mismatch_history`;
  - `violations` (the failed checks, as strings);
  - `max_abs_ey`, `max_abs_ay`, `track_clearance`;
  - `track_error` (affine vs true track coordinates, local model);
  - `references` (nominal `s̄, ēy, ēψ, κ`);
- solver: `qp_status`, `qp_iterations`, `primal_residual`, `dual_residual`,
  `trust_scale` (all per QP), `qp_solves`;
- objective:
  - `objective` is the QP objective, a surrogate that includes slack penalties;
  - `nonlinear_objective` is the original objective on the exact rollout. Use it to
    compare plans;
- `timing` (see below);
- `track_model`;
- `obstacle`: a NamedTuple with `present`, `relevant`, `side`, `attempts`, `guesses`,
  `required`, `collision_distance`, `min_distance`, `physical_clearance`,
  `buffer_clearance`, `constrained_clearance`, `collisions`, `separation_violations`,
  `degenerate_halfspaces`, `failure_cause` and `association`.

### Timing diagnostics

All times are in seconds. `timing.total` is the controller's internal measurement,
including attempts, retries, rollouts and validation. It excludes construction of the
returned result; older dynamic-controller revisions also excluded final bookkeeping.
For complete-call latency, pass an empty `cycle_times=Float64[]` vector to the relevant
closed-loop simulator. Fresh showcase exports use these external measurements, excluding
plant integration, export and rendering.

| field | `NMPCTiming` | linear `timing` NamedTuple |
|---|---|---|
| controller-reported cycle | `total` | `total` |
| solver | `solver` (Ipopt-reported), `optimize` (wall time of `optimize!`) | `solve` (all OSQP solves) |
| model updates | `update`, `warm_start` | `update` (JuMP/OSQP coefficient updates), `linearize` (Jacobians) |
| checks | `validation`, `schedule`, `reference` | `validate` (rollouts and checks), `reference` (track projections) |
| initialisation | – | `nominal`, `guesses` (passing-side guesses) |

`deadline_missed(r, 0.1)` (NMPC) or `r.timing.total > 0.1` tests a deadline. A missed
deadline is reported, not enforced: the time budgets only decide whether *further*
optional solves start. Neither controller guarantees hard real-time execution.

### Constraint-violation reporting

- **Planned trajectories:**
  - NMPC: `r.max_constraint_violation`, `r.reference` (re-simulation on the actual track),
    `r.corridor` and `r.obstacle`;
  - linear MPC: `r.violations`, `r.track_clearance` and `r.obstacle`.
- **Plant:** `simulate_closed_loop` records every operating-constraint violation of the
  simulated plant (track edge, speed and input limits, tolerance `config.acceptance_tol`)
  in `cl.violations`. `check_operating_constraints(vehicle, track, x, u)` checks one state.
- **Shared validation:** `validate_plan_on_track(vehicle, track, limits, z0, U; s0,
  obstacle, D, Dc, alphas)` applies the linear MPC's nonlinear validation to any input
  sequence, including NMPC plans. It was used for the controller comparison.

## 5. Closed-loop simulation

```julia
cl = simulate_closed_loop(ctrl, vehicle, x0, n_steps; initial_input=(0.0, 0.0))
```

**Each period:**
1. measure the plant state;
2. `solve!` with the previously applied input;
3. stop with `cl.status = :controller_failure` if the cycle is not accepted;
4. otherwise apply `r.u` over `dt` and integrate the plant. The plant is event-aware on
   multi-segment tracks: it resolves segment-boundary crossings and never uses the
   controller's model.

**`ClosedLoopResult` fields:**
- `t`, `states` (Frenet, unwrapped), `poses` (Cartesian), `inputs`;
- `controller` (the per-cycle results);
- `fine_t`, `fine_states` (plant substeps);
- `violations`, `crossings`, `status`, `message`.

**Reproducibility.**
- *Budgeted configurations.* Their outcomes can depend on the machine's speed, because
  the time budgets decide whether further solves start. Set `sqp_time_budget = Inf`
  (linear) or a large `correction_time_budget` (nonlinear) for machine-independent
  outcomes.
- *Under CPU load.* Even then, OSQP's default automatic adaptive-ρ interval can make the
  linear MPC's iterates timing-dependent. A fixed interval
  (`osqp_settings = ["adaptive_rho_interval" => 25]`) avoids this; it is not the default
  (see `docs/release_readiness.md`).

**First cycles.** The first call of each code path compiles (seconds). Under a per-cycle
time budget, a fresh Julia session can therefore fail its first cycles. Warm up with a
discarded run of the same configuration; the standalone examples do this in
`run_example`.

## 6. Minimal examples

```julia
using VehicleMPC
vehicle = KinematicBicycle(reference_test_vehicle())

# Linear MPC on the quarter track with an obstacle
track = quarter_track()
obstacle = obstacle_on_track(track, 3.5, 0.0; radius=0.10)
lmpc = LinearMPCController(vehicle, track, LinearMPCConfig(horizon=10, ego_radius=0.05, obstacle_buffer=0.03); obstacle)
cl = simulate_closed_loop(lmpc, vehicle, [0.0, 0.0, 0.0, 0.5], 250)

# Nonlinear MPC, the same scenario
cfg = NMPCConfig(prediction_model=:cartesian_endpoint_exact, horizon=10, schedule_corrections=8,
    ego_radius=0.05, obstacle_buffer=0.03, obstacle_samples=3)
nmpc = NMPCController(vehicle, track, cfg; obstacle)
r = solve!(nmpc, [0.0, 0.0, 0.0, 0.5])
r.accepted && println(r.u)
```

Complete, runnable versions with reporting are in `examples/` (see the README).

## 7. Dynamic bicycle and its NMPC (final milestone)

A third controller, for a different vehicle model. It does not replace the two above, and
its interface differs from them in the measured state.

```julia
using VehicleMPC
p = showcase_dynamic_parameters()          # synthetic; legacy_dynamic_parameters() cannot race here (docs/dynamic_bicycle_mpc.md)
vehicle = DynamicBicycle(p)
track = quarter_track()
obstacles = [obstacle_on_track(track, 25.4, 0.10; radius=0.10), obstacle_on_track(track, 27.2, -0.10; radius=0.10)]
ctrl = DynamicNMPCController(vehicle, track, DynamicNMPCConfig(ego_radius=0.05, obstacle_buffer=0.03); obstacles)

Xg, Yg, psi = cg_pose_from_rear_axle(p, 0.0, 0.0, 0.0)  # rear axle at the origin → CoM
z = [Xg, Yg, psi, 1.0, 0.0, 0.0]                        # [X, Y, ψ, vx, vy, r] of the CoM
r = solve!(ctrl, z; s_hint=p.lr)                        # s_hint: progress of the CoM (unwrapped)
r.accepted && println(r.u)                              # [δ, a]; r.u === nothing when rejected

cl = simulate_dynamic_closed_loop(DynamicNMPCController(vehicle, track; obstacles), vehicle, z, 250; s0=p.lr)
```

**Measured state.** The Cartesian CoM state `z = [X, Y, ψ, vx, vy, r]`, not a Frenet
state. Its track coordinates are computed by projection near `s_hint`, and later near the
previous cycle's progress. A state with `vx ≤ 0`, non-finite entries, or no valid
projection is rejected as `:invalid_measurement`.

**Model functions.**

| function | returns |
|:--|:--|
| `dynamic_rhs(p, z, u)` | the time derivative of the state |
| `slip_angles`, `tyre_forces` | `(α_f, α_r)`, `(F_yf, F_yr)` |
| `dynamic_lateral_acceleration` | `a_y` |
| `dynamic_step(model, z, u, dt; substeps=4)` | the MPC's RK4 step |
| `integrate_dynamic(model, z, u, dt; h=1e-3)` | the plant / validation integration |
| `check_dynamic_domain` | a `DomainError` outside the model domain |
| `dynamic_operating_violations` | violated operating limits |
| `steady_state_cornering`, `max_cornering_speed`, `lateral_mode_rates` | feasibility analysis |

**Result** (`DynamicNMPCResult`). The same acceptance contract as the other controllers:
`status`, `accepted`, and `u` (`nothing` unless accepted).
- **Status values:** `:success`, `:invalid_measurement`, `:initial_collision`,
  `:solver_failure`, `:segment_assignment_invalid`, `:validation_failed`.
- **Plan:** `Z` (CoM states), `frenet`, `U`, `path` (every RK4 sub-step).
- **Checks:** `validation` (independent fine rollout; see the docstring), `violations`.
- **Obstacles:** `active_obstacles`, `sides`.
- **Diagnostics:** `attempts`, `timing = (total, setup, guesses, solve, validation)`.

**Closed loop** (`DynamicClosedLoopResult`): `t`, `states`, `frenet`, `inputs`,
`controller`, `fine_t`, `fine_states`, `fine_frenet`, `violations`, `status`, `message`.
- The status is `:success`, `:controller_failure`, `:plant_domain_error` or
  `:plant_left_track`.
- `succeeded(cl)` is true for `:success` only.

**Obstacles.** `obstacles` takes any number of `CircleObstacle`s. `buffers` optionally
gives one buffer per obstacle; the default is `obstacle_buffer` for all.
`required_separation(ctrl, i)` returns `D_i`.
