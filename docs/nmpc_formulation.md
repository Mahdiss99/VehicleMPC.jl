# Nonlinear MPC formulation (Milestone 2A)

**Status: functional prototype.** The controller solves the problem below in closed loop
on straight and constant-curvature circular tracks. The 10 Hz timing target was met in the
benchmarks on one Windows laptop (no solve above 100 ms in 6 000 measured solves; details
below). That is a measurement on one machine, not a real-time guarantee. Nothing here is
validated on the multi-segment legacy tracks. There is no collision avoidance.

Code: `src/controllers/nmpc_config.jl`, `src/controllers/nonlinear_mpc.jl`,
`src/simulation/closed_loop.jl`. Conventions (signs, units, Frenet frame) are in
`docs/model_conventions.md`.

## 1. Model

The Milestone 1 rear-axle kinematic bicycle, unchanged. State `x = [s, ey, epsi, v]`,
input `u = [delta, a]`:

```text
sdot    = v cos(epsi) / (1 - kappa ey)
eydot   = v sin(epsi)
epsidot = v tan(delta) / L - kappa sdot
vdot    = a
```

The equations exist once, in `kinematic_rhs(L, ey, epsi, v, delta, a, kappa)`. This
function never throws and has no branches.

- The checked `continuous_dynamics` (used by simulation and validation) is
  `check_domain` + `kinematic_rhs`.
- The NMPC traces the same `kinematic_rhs` into JuMP nonlinear expressions. JuMP then
  differentiates them with its own exact AD (first and second derivatives).
- No throwing check is ever evaluated inside Ipopt's callbacks.

## 2. Optimal control problem

Direct multiple shooting with `N` intervals of length `dt`. The decision variables are
`X[:, 2:N+1]` and `U[:, 1:N]`, plus the RK4 stage points (section 3). `X[:, 1]` is not a
decision variable: it is a JuMP `Parameter` holding the measured state, so
`X[:,1] = x_measured` holds exactly at every Ipopt iterate.

### Objective

All terms are normalised so that the weights are dimensionless:

```text
J = - ws (s[N+1] - s[1]) / (v_max N dt)
    + Σ_{k=2}^{N+1} [ qy (ey[k]/ey_max)² + qpsi (epsi[k]/epsi_max)² ]
    + Σ_{k=1}^{N}   [ rdelta (delta[k]/delta_max)² + ra (a[k]/a_ref)²
                      + rd_delta ((delta[k] - delta[k-1])/delta_max)² + rd_a ((a[k] - a[k-1])/a_ref)² ]
```

- `ey_max = width/2 − track_margin` and `a_ref = max(|a_min|, a_max)`.
- `delta[0]` and `a[0]` are the previously applied input, a `Parameter` updated on every
  solve.
- The state terms start at `k = 2` because `X[:,1]` is fixed.
- `v_max·N·dt` is the largest distance the horizon could cover, so the progress term is at
  most `ws`.

This is a **progress-maximising surrogate**, not an exact minimum-time problem. Progress is
measured at the end of a fixed time horizon, and nothing guarantees the closed loop is
time-optimal.

### Constraints

| constraint | form | where |
|:--|:--|:--|
| initial state | `X[:,1] = x_measured` (Parameters) | exact |
| dynamics | `X[:,k+1] = F(X[:,k], U[:,k], kappa_k)` with `F` = RK4 or forward Euler | `k = 1…N` |
| track corridor | `-ey_max ≤ ey_k ≤ ey_max` (variable bounds) | knots `k = 2…N+1` |
| heading | `-epsi_max ≤ epsi_k ≤ epsi_max`, `epsi_max = 60°` | knots `k = 2…N+1` |
| speed | `v_min ≤ v_k ≤ v_max` | knots `k = 2…N+1` |
| steering, acceleration | `|delta_k| ≤ delta_max`, `a_min ≤ a_k ≤ a_max` | `k = 1…N` |
| lateral acceleration | `v² tan(delta_k)/L ≤ ay_max` and `≥ -ay_max`, at **both** `v = v_k` and `v = v_{k+1}` | `k = 1…N` |
| Frenet validity of RK4 stages | `1 - kappa ey ≥ 2·eps_geometry` and `v ≥ 0` (variable bounds) | stage points |

- **Heading bound.** `|epsi| ≤ 60°` keeps `cos(epsi) ≥ 0.5`, so `sdot > 0` whenever
  `v > 0` (forward progress).
- **Lateral acceleration.** `|v² tan(delta)/L|` is written as two smooth inequalities
  instead of using `abs`. `ay_max` (default 4 m/s²) is a controller tuning parameter. It is
  **not** a calibrated tyre-friction limit.
- **Frenet validity at the knots.** The constructor checks that the admissible corridor
  satisfies `1 - kappa·ey ≥ 2·eps_geometry`, and throws otherwise. For the R = 1 m,
  0.7 m-wide circle the smallest value is 0.70.

## 3. Keeping every model evaluation inside the domain

Ipopt evaluates trial points that are not yet feasible for the general constraints, but its
interior-point iterates always respect **variable bounds**. Bounds are relaxed only by
`bound_relax_factor = 1e-8`. The formulation therefore places every quantity that could
make the model singular under variable bounds:

- `X[:,1]` is a Parameter, and the measured state is checked with `check_domain` *before*
  the solve. An invalid measurement returns `:invalid_measurement` without calling the
  solver.
- Knot `ey`, `epsi`, `v` and the inputs are variable bounds. `|delta| ≤ delta_max` stays
  more than 0.1 rad away from the `tan` singularity.
- **Lifted RK4.** The intermediate RK4 points
  `x + dt/2·k1`, `x + dt/2·k2` and `x + dt·k3` (their `ey`, `epsi`, `v` components; `s` does
  not enter `f`) are extra decision variables. They are tied to the stages by equality
  constraints and carry the Frenet-safe bound
  `ey ≤ (1 - 2·eps_geometry)/kappa` (for `kappa > 0`; mirrored for `kappa < 0`) and `v ≥ 0`.
  Every evaluation of `f` therefore happens at a point whose denominator is at least
  `2·eps_geometry = 0.2` (up to the 1e-8 relaxation), at trial points as well as at
  solutions. Lifting also keeps each constraint expression small; nesting four RK4 stages
  symbolically would make the expression trees grow multiplicatively.
- No value is clamped and no singular expression is replaced by a constant.

## 4. Four kinds of validity

| notion | meaning | how it is established |
|:--|:--|:--|
| optimisation feasibility | Ipopt found a locally optimal, primal-feasible point | `termination_status ∈ {LOCALLY_SOLVED, ALMOST_LOCALLY_SOLVED}` and `primal_status = FEASIBLE_POINT` |
| model-domain validity | every predicted step is inside the model's domain | `validate_prediction` recomputes every step with the **checked** `discrete_dynamics` (which throws on domain violations) |
| operating constraints at the knots | bounds and inequalities hold at the knots | `validate_prediction` measures the largest violation independently of the solver |
| between-knot validity | the continuous-time trajectory between knots respects the limits | `between_knot_diagnostics` (fine RK4 of the prediction); the closed-loop plant is checked at every substep |

A solve is **accepted** only if all of the following hold:
- the termination and primal status are acceptable;
- every value is finite;
- `X[:,1]` equals the measurement;
- the dynamics residual and the constraint violation are at most `acceptance_tol = 1e-6`;
- the first input is strictly within the actuator limits.

`ALMOST_LOCALLY_SOLVED` is accepted only after this independent validation, and the
result's message says so. A rejected solve returns `u = nothing` and clears the warm-start
memory. The closed-loop runner then stops with `:controller_failure`. There is **no
fallback input**.

Ipopt is run with `honor_original_bounds = "yes"`. Without it, Ipopt 3.14 can return inputs
up to about 1e-8 outside their bounds (relaxed bounds), which the strict actuator check
rejected during development.

**What the knot constraints guarantee between knots, and what they do not.**

- **Speed and lateral acceleration hold in continuous time.** Over an interval `a` is
  constant, so `v(t)` is linear: the speed bounds at both ends hold throughout. With `delta`
  constant, `|v(t)² tan(delta)/L|` is convex in `t` and so is largest at an end point.
  Imposing the `ay` bound at both `v_k` and `v_{k+1}` therefore enforces it on the whole
  interval for this model. With only the start point constrained, the predicted `ay`
  exceeded `ay_max` by up to 1.3 m/s² between knots in development. That version was
  replaced.
- **`ey` and `epsi` are constrained only at the knots.** The continuous path can bulge past
  the knot bound. A test constructs a case in which both knots are inside the corridor and
  the path between them reaches `ey = 0.379 m` on a ±0.35 m track. The `track_margin`
  (0.05 m) is the buffer. In the progress-dominant circle example, the plant reaches
  `|ey| = 0.3002 m` (knot bound 0.30 m, track edge 0.35 m). Bounding this exactly would
  need, for example, sampled intermediate constraints. That is not done here.

## 5. Persistent model and parameter updates

The JuMP model is built once per `NMPCController`. Each `solve!` only:

1. writes the measured state and the previous input into JuMP `Parameter`s
   (`set_parameter_value`);
2. sets the initial guess with `set_start_value`;
3. calls `optimize!`.

The curvature on each interval is also a `Parameter` (`kappa[1:N]`). In Milestone 2A it is
the constant curvature of the single-segment track, and there is no public API to change it
yet.

Parameters are supported natively by Ipopt.jl (tested with JuMP 1.32.1, Ipopt.jl 1.16.0,
Ipopt 3.14.19). Three tests confirm that updates reach the existing nonlinear equations:

- a persistent model re-solved for a new state gives the same solution (to 1e-9) as a model
  built from scratch;
- changing the previous input changes the first optimal input;
- a direct evaluation of every dynamics constraint at a known point gives a residual of 0
  to 1e-12.

No `value(...)` call is used to build constraints.

## 6. Initial guesses

- **Cold start.** Every stage gets `delta = atan(kappa·L)` (the curvature feed-forward,
  clipped to the steering limit) and `a = 0`. The checked prediction model is rolled out
  from the measured state.
- **Warm start** (default, after an accepted solve). The previous inputs are shifted one
  step, with the last input repeated. The checked prediction model is rolled out from the
  **new measured state**, so the guess satisfies the dynamics exactly. If the rollout
  leaves the model domain, the remaining columns come from the shifted previous trajectory.
- Start values (not states) are projected into the variable bounds. The RK4 stage-point
  guesses are computed from the same rollout.
- Only primal values are passed. Ipopt's dual warm start (`warm_start_init_point`) is
  **not** enabled.

## 7. Baseline configuration

| parameter | value | interpretation |
|:--|--:|:--|
| `horizon`, `dt` | 20, 0.1 s | 2 s look-ahead at 10 Hz |
| `integrator` | RK4 | prediction error ~1e-6 per step vs a fine reference (Euler: ~2e-2, see §8) |
| `ws` | 10 | covering the maximum possible horizon distance `v_max·N·dt` is worth 10 |
| `qy`, `qpsi` | 0.1, 0.1 | a whole horizon at the corridor edge (or at `epsi_max`) costs `N·0.1 = 2`, i.e. 20 % of the maximum progress reward |
| `rdelta`, `ra` | 0.01, 0.01 | weak regularisation of input magnitude |
| `rd_delta`, `rd_a` | 1.0, 0.1 | smoothness: a full-range steering step `delta_max` costs 1 |
| `track_margin` | 0.05 m | buffer for between-knot excursions (§4) |
| `epsi_max` | 60° | keeps progress forward |
| `ay_max` | 4.0 m/s² | tuning value, not a tyre limit; on R = 1 m it sets `v ≈ √(ay_max·R) = 2 m/s` |
| Ipopt | `tol = 1e-8`, `max_iter = 200`, no wall-time limit, MUMPS, `honor_original_bounds = yes` | |
| `acceptance_tol` | 1e-6 | independent validation threshold |

The reference test vehicle is unchanged: `L = 0.25 m`, `|delta| ≤ 25°`, `a ∈ [-5, 5] m/s²`,
`v ∈ [0, 5] m/s`. The baseline `qy` dominates the gain from cutting the inside of a 1 m
circle, so the vehicle stays near the centreline. The circle example also runs a
progress-dominant variant (`qy = qpsi = 0`) in which the corridor bound becomes active. The
weights are tuning choices to be reviewed, not derived values.

## 8. Results (Milestone 2A)

### Closed-loop examples

Reproduce with `julia --project=. examples/nmpc_straight.jl` and
`julia --project=. examples/nmpc_circle.jl`.

| scenario | periods | accepted | outcome |
|:--|--:|--:|:--|
| A: straight, start `[0, 0.2, 0.1, 0.5]` | 100 | 100 | `v` → `v_max = 5.0`, `ey`, `epsi` → 0, max plant \|ey\| 0.215 m, 0 violations |
| B: circle R = 1, baseline | 70 | 70 | 2.2 laps, `v` → 1.97 m/s, `|ay|` active at 4.0, max \|ey\| 0.032 m, 0 violations |
| B: circle R = 1, progress-dominant | 70 | 70 | 2.55 laps, rides the inner bound `ey = 0.30` at `v = 1.673 = √(4·0.7)`, plant max \|ey\| 0.3002 m, 0 violations |

Across all three runs the largest dynamics residual is ≤ 5e-8 and the largest knot
constraint violation is 0.

Closed-loop plots (regenerate with `julia --project=examples examples/plot_nmpc.jl`). The
`solve!` time panels come from that plotting run, not from the benchmark:

- [straight track](figures/nmpc_straight.png)
- [circle, baseline](figures/nmpc_circle_baseline.png)
- [circle, progress-dominant](figures/nmpc_circle_progress_dominant.png)

### Prediction accuracy (circle, 70 periods, vs the 10-substep RK4 plant)

| prediction integrator | one-step error | worst error over the horizon (fine re-simulation) | between-knot violations in predictions |
|:--|--:|--:|--:|
| RK4 (baseline) | 2.7e-7 | 1.3e-5 | 0 |
| RK4 (progress-dominant) | 1.5e-6 | 5.3e-5 | 0 |
| Euler (baseline) | 2.3e-2 | 8.9e-2 | 0 |
| Euler (progress-dominant) | 2.2e-2 | 1.7e-1 | 14 |

### Runtime

The full table is in `benchmarks/results/nmpc_runtime.md` (run
`julia --project=. benchmarks/nmpc_runtime.jl`). It covers 8 scenarios, each with 3
interleaved rounds of 250 consecutive closed-loop solves (750 per scenario). The machine is
a Windows laptop with Julia 1.11.5 and one Julia thread.

| configuration | median | p99 | max | > 80 ms | > 100 ms |
|:--|--:|--:|--:|--:|--:|
| circle, RK4, N = 20, warm (baseline) | 38.4 | 56.7 | 66.9 | 0 | 0 |
| circle, RK4, N = 20, cold | 30.4 | 52.8 | 93.7 | 1 (0.1 %) | 0 |
| circle, Euler, N = 20, warm | 20.6 | 31.3 | 40.7 | 0 | 0 |
| circle, RK4, N = 10, warm | 18.4 | 29.4 | 39.3 | 0 | 0 |
| circle, RK4, N = 30, warm | 49.7 | 78.8 | 88.3 | 8 (1.1 %) | 0 |
| straight, RK4, N = 20, warm | 16.3 | 30.6 | 52.9 | 0 | 0 |

All 6 000 solves were accepted. Start-up: first model construction ≈ 4 s, first solve
≈ 8.6 s (compilation), second solve ≈ 0.27 s. After that, the parameter update, initial
guess and validation together take < 0.5 ms per solve.

Findings:

- **Where the time goes.** `optimize!` wall time and Ipopt's own solve time agree to within
  about 0.1 ms, so JuMP's per-solve overhead is negligible. Ipopt's timing statistics for a
  representative RK4 N = 20 solve (326 variables, 14 iterations) give about 17 ms inside the
  interior-point algorithm, mostly the primal-dual system solve, and about 6 ms in function
  and derivative evaluation. Windows timer resolution is about 1 ms. The **bottleneck is
  Ipopt's per-iteration cost** (≈ 2 ms per iteration for RK4 N = 20, ≈ 1 ms for Euler).
  The model code and the controller's Julia code are not the bottleneck.
- **Primal warm start does not help Ipopt here.** On the circle it costs about 3 more
  iterations than the cold start (18.1 vs 15.1 for RK4); on the straight it is neutral. A
  likely cause: the shifted solution lies on active bounds (`ay`, corridor), Ipopt pushes
  the start point into the interior anyway (`bound_push`), and it restarts with
  `mu_init = 0.1`. Primal warm start stays the default for continuity. Dual warm starting
  was not tried (see §9).
- **Run-to-run variability is large on this machine.** The same baseline configuration had
  a median of 14 ms in an earlier session and 38 ms in this benchmark. That is why the
  benchmark interleaves rounds and reports the per-round range. Absolute numbers should be
  re-measured on the target hardware.
- **Euler vs RK4.** Euler halves the per-iteration cost (80 instead of 260 equality
  constraints), but at `dt = 0.1` its predictions are 4–5 orders of magnitude less
  accurate, and its predictions show between-knot violations.

## 9. Limitations and open issues (input for Milestone 2B)

1. **Tracks.** Only single-segment tracks (constant curvature) at this milestone.
   *Update (2B.2):* multi-segment tracks are now accepted with a stagewise
   average-curvature schedule; see `docs/multisegment_nmpc.md` for that extension and its
   limitations.
2. **No terminal ingredients.** There is no terminal set or cost, so recursive feasibility
   and stability are not guaranteed. Feasibility is observed in these scenarios, not
   proven.
3. **Between-knot `ey`/`epsi`** rely on the margin (§4).
4. **Timing.** These are wall-clock measurements on one machine with no real-time
   guarantee. Before relying on 10 Hz, re-measure on the target platform under load.
   Candidate speed-ups, none of them tried:
   - Ipopt dual warm start (`warm_start_init_point = yes` with a smaller `mu_init`);
   - a faster sparse linear solver (HSL MA27/MA57, which needs a licence);
   - Euler with a smaller `dt`, or a shorter horizon;
   - an SQP / real-time-iteration scheme.
5. **No fallback input** on solver failure. The closed loop stops. Any fallback would
   provide no safety guarantee.
6. **Kinematic model.** It has no tyre or slip dynamics. `ay_max` only keeps the vehicle
   in a regime where the kinematic model is plausible, and it is not calibrated.
7. **Progress surrogate.** The objective is not a minimum-time formulation.
