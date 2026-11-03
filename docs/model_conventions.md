# Model and geometry conventions

This page is the reference for every sign, unit and frame used by `VehicleMPC`.
The dynamics (`src/models/kinematic_bicycle.jl`) and the geometry
(`src/tracks/track_geometry.jl`) share these conventions, and the tests check that they
agree (`test/test_kinematic_dynamics.jl`, testset "Frenet dynamics agree with the
Cartesian model").

## Units

SI throughout: metres, seconds, radians, m/s, m/s². Steering angles are radians in code;
degrees appear only in comments.

## Cartesian frame

Right-handed inertial frame `(X, Y)`. Heading `psi` is measured counter-clockwise from the
`+X` axis.

## Track centreline and Frenet frame

The centreline is a sequence of constant-curvature segments parameterised by arc length
`s`.

| quantity    | definition                                                                  |
|:------------|:----------------------------------------------------------------------------|
| `s`         | arc length along the centreline from the start pose [m]                     |
| `psi_c(s)`  | centreline tangent heading; `d psi_c / ds = kappa(s)`                        |
| `kappa(s)`  | signed curvature [1/m]: **`> 0` turns left (CCW)**, `< 0` turns right         |
| `n(s)`      | **left** unit normal `(-sin psi_c, cos psi_c)`                              |
| `ey`        | lateral offset along `n`: **`> 0` is left of the centreline** [m]            |
| `epsi`      | heading error `psi - psi_c(s)`: **`> 0` means rotated CCW from the tangent** |

Frenet → Cartesian:

```text
X   = x_c(s) - ey*sin(psi_c(s))
Y   = y_c(s) + ey*cos(psi_c(s))
psi = psi_c(s) + epsi
```

With these signs the centre of curvature of a left turn (`kappa > 0`) lies at
`ey = 1/kappa`, so the distance from a point to the centre is `(1 - kappa*ey)/|kappa|`.
The Frenet frame is valid only where the **metric factor `1 - kappa*ey > 0`**.

These are the same conventions the legacy `Environment.jl` uses for
`build_track_keypoint` and `local_to_global` (radius `> 0` ⇒ centre on the left, `ey`
along `psi + π/2`). The test "regression against legacy Environment.jl" checks that the
new geometry reproduces the legacy forward map to 1e-12.

## Vehicle model

Kinematic bicycle referenced to the **rear axle** (no side slip at the reference point).

```text
state  x = [s, ey, epsi, v]
input  u = [delta, a]

sdot    = v*cos(epsi) / (1 - kappa*ey)
eydot   = v*sin(epsi)
epsidot = v*tan(delta)/L - kappa*sdot
vdot    = a
```

- `L` is the wheelbase, `v` the rear-axle speed, `delta` the front steering angle
  (**`> 0` steers left**), `a` the longitudinal acceleration.
- The yaw rate is `psidot = v*tan(delta)/L`, so `epsidot = psidot - psi_c'(s)*sdot`.
- Steady state on a circle of curvature `kappa`: `delta = atan(kappa*L)` gives `eydot = 0`,
  `epsidot = 0` at `ey = epsi = 0`.

### Curvature is an external input

`kappa` is passed to `continuous_dynamics`, `discrete_dynamics` and the Jacobian functions
as an explicit argument. The Jacobians hold `kappa` fixed, so `∂f/∂s = 0` and nothing is
differentiated through the piecewise-constant segment lookup. A simulator or controller
looks up `kappa = curvature(track, s)` and passes it in.

### Domain of validity

`check_domain` (called by every model function) throws a `DomainError` unless:

| condition                                   | reason                                            |
|:--------------------------------------------|:--------------------------------------------------|
| all of `x`, `u`, `kappa` finite             | no silent NaN propagation                         |
| `1 - kappa*ey > eps_geometry` (default 0.1) | stays away from the Frenet singularity; 0.1 means no closer to the centre of curvature than 10 % of the local radius |
| `v ≥ 0`                                     | forward-driving model only                        |
| `|delta| ≤ π/2 - 0.1`                       | bounded away from the `tan` singularity           |

Actuator limits (`delta_max`, `a_min`, `a_max`) are a property of the vehicle, not of the
equations. They are checked by `within_input_limits`, and `simulate_open_loop` stops with
status `:input_limit` when an input violates them. Nothing is ever clamped.

### Operating constraints vs model domain

Two kinds of conditions are kept apart:

| kind | examples | how it is reported |
|:-----|:---------|:-------------------|
| **Model domain** (the maths is invalid) | `1 - kappa*ey ≤ eps_geometry`, `v < 0`, steering near `±π/2`, NaN/Inf | `check_domain` throws `DomainError` |
| **Operating constraints** (physically or operationally unacceptable, but the model is valid) | `|ey| > width/2`, `v > v_max`, `v < v_min`, input outside `delta_max`/`a_min`/`a_max` | `check_operating_constraints` returns a `Vector{ConstraintViolation}` |

`check_operating_constraints(model, track, x, u=nothing)` lists every violated limit. Each
entry has a symbol, the offending value, the bound it crossed and a readable message. The
function never throws for a violation and never modifies or clamps `x` or `u`. The track
boundary is checked only when the track has a positive width. `trajectory_violations`
applies the check to a whole `SimulationResult`.

The open-loop simulator does **not** stop at the track boundary or at `v_max`. The
open-loop example leaves the 0.7 m corridor on purpose: it shows that the system has to
be controlled. Enforcing these limits is the controller's job.

## Discretisation

`discrete_dynamics(model, x, u, kappa, dt, integrator)` with `dt > 0`:

- `ForwardEuler()` (default): `x⁺ = x + dt*f(x, u, kappa)`.
- `RK4()`: classical Runge–Kutta, `u` and `kappa` constant over the step.

In both cases curvature is a zero-order hold over the step. A step that crosses a segment
boundary uses the curvature at the start of the step.

`discrete_jacobians` returns `A = ∂f_d/∂x`, `B = ∂f_d/∂u`. For forward Euler they are
analytical (`A = I + dt*Ac`, `B = dt*Bc`). For other integrators they come from
ForwardDiff (`discrete_jacobians_ad`). Adding an integrator means adding a
`discrete_dynamics` method; the model and the controller API stay the same.

## Progress: unwrapped state vs wrapped geometry

The state always carries **continuous, unwrapped** progress `s`. It is never wrapped
inside the dynamics or the simulator, so lap time and progress objectives stay continuous.

Geometry look-ups wrap only internally:

- `split_progress(track, s) -> (lap, s_wrapped)` with `s_wrapped ∈ [0, L)` on a closed
  track.
- On a closed track `centerline_heading(track, s + L) = centerline_heading(track, s) +
  total_turning(track)`, so heading is continuous in unwrapped `s`.
- On an **open** track, progress outside `[0, L]` throws a `DomainError`. There is no
  extrapolation.
- A track is `closed` only if its end pose matches its start pose. The constructor checks
  this.

Segments are half-open `[s_i, s_{i+1})`: at a boundary the curvature of the following
segment applies (same as legacy `get_curvature`).

## Cartesian → Frenet

`cartesian_to_frenet(track, X, Y, psi; s_hint, max_lateral)` searches every segment for an
orthogonal projection with `|ey| ≤ max_lateral` (default: half the track width):

- No projection: `ArgumentError`. No `(0, 0, 0)` placeholder is ever returned.
- Several distinct projections and no `s_hint`: `ArgumentError` (ambiguous).
- With `s_hint`: the projection closest in progress is returned. On a closed track `s` is
  unwrapped to the lap nearest the hint.
- `epsi` is wrapped to `(-π, π]`.

## Reference test vehicle

`reference_test_vehicle()` is a **test configuration, not a calibrated vehicle**:
`L = 0.25 m`, `|delta| ≤ 25°`, `a ∈ [-5, 5] m/s²`, `v ∈ [0, 5] m/s`. See the docstring and
`docs/legacy_audit.md` for where the values come from. The legacy parameter sets are kept
separately in `LEGACY_PARAMETER_SETS` and must not be mixed.

## Open numerical issues for the optimizer (Milestone 2)

These two issues were written down at the end of Milestone 1.

**Status after Milestone 2A.** Issue A is addressed for single-segment tracks, along the
lines of the proposed structure: a non-throwing `kinematic_rhs` kernel, domain limits as
variable bounds (including lifted RK4 stage points) and validation after the solve. See
`docs/nmpc_formulation.md`. Issue B is still **open**: the NMPC accepts only
constant-curvature tracks, so no curvature transition occurs inside the horizon.

**Status after Milestone 2B.1.** Issue B has been analysed but not resolved in the
controller.

- An event-aware reference integrator (`integrate_on_track`) and stagewise average
  curvature utilities (`average_curvature`, `curvature_schedule`) exist.
- The errors of strategies 1–3 have been measured, and a predictor has been recommended
  for review: `docs/multisegment_prediction.md`.
- The NMPC still rejects multi-segment tracks.

### A. Domain checks during optimization

`check_domain` throws a `DomainError` for invalid Frenet states (`1 - kappa*ey ≤
eps_geometry`), negative speed, steering near `±π/2` and non-finite values. That is the
right behaviour for simulation, but not inside an optimizer:

- Ipopt evaluates the model at trial points that are not yet feasible. Constraint
  functions may be violated at these points. Variable bounds are kept: interior-point
  iterates stay inside them, relaxed by `bound_relax_factor`, 1e-8 by default.
- A Julia exception thrown inside a JuMP operator or a model callback aborts the solve. It
  does not reject the trial point.
- Ipopt can recover from an evaluation failure (a non-finite value or a failed callback)
  by shortening the line-search step. Returning `Inf`/`NaN` on purpose is a fallback, not a
  formulation: the singular point stays inside the feasible set and derivatives near it
  blow up.

Proposed structure:

1. **Smooth model evaluation.** A non-throwing `f(x, u, kappa)` kernel with the same
   equations, used by the optimizer and AD. The current `continuous_dynamics` becomes
   `check_domain` + kernel. The simulator keeps the checked version.
2. **Domain as variable bounds, with margins.** When `kappa_k` is a fixed parameter per
   stage, `1 - kappa_k*ey_k ≥ eps_geometry` is linear in `ey_k`, so it becomes the bound
   `ey_k ≤ (1 - eps_geometry)/kappa_k` (`≥` for `kappa_k < 0`). Speed and steering get
   bounds `0 ≤ v_k` and `|delta_k| ≤ delta_max`; `delta_max` is already at least 0.1 rad
   below `π/2`. Because these are variable bounds, Ipopt never evaluates outside them.
   Normally the track-boundary bound `|ey| ≤ width/2` is the tighter one, and track
   construction can check `width/2 < (1 - eps_geometry)/max|kappa|`. If the curvature
   depends on the decision variables (section B), the Frenet condition becomes a general
   nonlinear constraint and needs separate care.
3. **Validation after the solve.** Check the solution with `check_domain` and
   `check_operating_constraints` before applying it. Report violations as a solver-failure
   status and do not apply that solution.

An option to evaluate: an input transformation that removes the `tan` singularity, for
example optimising the path curvature `c = tan(delta)/L`, bounded by `tan(delta_max)/L`,
instead of `delta`.

Only the knot points `x_k` are constrained. The trajectory between knots is not checked.

### B. Track curvature at segment boundaries

The tracks are built from constant-curvature pieces. Position and heading are continuous,
but the curvature `kappa(s)` jumps at every segment boundary (no clothoid transitions).

**Zero-order-hold error.** `discrete_dynamics` holds `kappa` at its start-of-step value. In
a step that crosses a boundary, the wrong curvature is applied over the distance `Δs`
travelled past the boundary. The heading-error update `epsidot = … - kappa*sdot` then picks
up an error of about `|Δkappa|·Δs`, at most `|Δkappa|·v·dt`, and the lateral error picks up
about `½·|Δkappa|·Δs²`. Measured for one step on the quarter track (straight → radius 1 m,
`Δkappa = 1`, starting 0.05 m before the boundary, `delta = 0`), against an RK4 reference
with 1e-5 s substeps:

| v [m/s] | dt [s] | Δs past boundary [m] | epsi error [rad] | ey error [m] |
|:-------:|:------:|:--------------------:|:----------------:|:------------:|
| 2 | 0.05 | 0.05 | 0.050 | 0.0013 |
| 2 | 0.10 | 0.15 | 0.149 | 0.011  |
| 4 | 0.10 | 0.35 | 0.337 | 0.060  |

RK4 gives the same error as Euler: holding the curvature constant limits the error, not the
integrator. Over a whole trajectory the error depends on where the time grid falls
relative to the boundary, and it does not decrease monotonically with `dt`. The ±0.35 m
track half-width makes errors of this size significant for MPC predictions.

**Why not differentiate through the segment look-up.** `kappa(s)` is piecewise constant:
its derivative is zero almost everywhere and undefined at the jumps (a Dirac impulse in
the distributional sense). ForwardDiff through `curvature(track, s)` therefore returns
`∂kappa/∂s = 0`. The solver cannot see that moving `s` shifts the start of a turn. The
look-up also uses discrete operations (`searchsortedlast`, `mod` wrapping) whose derivatives
are undefined at the boundaries. Ipopt and SQP-type methods assume twice continuously
differentiable functions for their Newton steps, line search and convergence guarantees.
Discontinuous dynamics break those assumptions and can stall or trap the iterations.

**Strategies for Milestone 2** (to be chosen together; the track geometry is not changed
without approval):

1. **Curvature as a stage-wise parameter from the previous solution (time domain).**
   Predict `s_k` from the shifted previous solution and pass a fixed `kappa_k` to the
   solver, then recompute and re-solve a few times if needed. To reduce the
   zero-order-hold error, use the interval-averaged curvature
   `kappa̅_k = (psi_c(s_{k+1}) - psi_c(s_k)) / (s_{k+1} - s_k)`. It comes from the
   continuous centreline heading and gives the exact heading change of the centreline over
   the predicted interval. The NLP stays smooth because `kappa_k` is a constant. This is
   the natural choice for the receding-horizon time-domain MPC; the cost is the iteration
   on the curvature prediction.
2. **Spatial reformulation (progress as the independent variable).** Integrate in `s` with
   time as a state: `dt/ds = (1 - kappa*ey)/(v*cos(epsi))`, which requires
   `v*cos(epsi) > 0`. Place the grid so every segment boundary is a grid point. The
   curvature is then exactly constant on every interval and the discontinuity disappears.
   Lap time becomes a terminal state, which suits minimum-time objectives. It is a larger
   change of formulation and needs a positive-progress constraint.
3. **A smooth (C²) curvature representation, for example clothoid transitions, a smoothing
   spline or a sigmoid blend at each jump.** `kappa(s)` becomes differentiable in the
   decision variable `s`. This changes the reference geometry, so it needs approval, and
   the smoothing length trades fidelity against conditioning.

Separately, the simulator (the "plant") can remove its own error by splitting a step at
the boundary crossing. Such event-aligned integration can be added without changing the
model or the controller API.
