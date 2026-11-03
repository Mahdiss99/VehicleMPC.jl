# Legacy code audit (Milestone 1)

Scope: every `.jl` file in the repository at commit `6c70d74`. No legacy file was modified
or moved. Each claim marked *(verified)* was reproduced by running the legacy code under
Julia 1.11.5 with ForwardDiff. Claims without that mark come from reading the code.

## 1. Confirmed formulation errors

### Kinematic bicycle (`New_model_Dynamic.jl`, `NonlinearKinematicBicycleGame`)

1. **Lateral dynamics use acceleration instead of speed.**
   `eyd = u[2]*sin(x[3])` (lines 143, 156), and the same in `MPC with JuMP/main.jl`
   (`Dynamics2`). It should be `v*sin(epsi)`. With `a = 0`, `ey` never changes whatever the
   heading. At `epsi = 0.3`, `v = 2` the legacy `eydot` is `0` instead of `0.591`
   *(verified)*.
2. **Heading-error equation has the wrong sign.**
   The legacy code has `eψd = κ·ṡ − v·cosβ·tanδ/L`, the negative of `ψ̇ − κ·ṡ`. But
   `local_to_global` builds the pose as `ψ = ψ_track + eψ`, so the dynamics and the geometry
   disagree. Positive (left) steering gives `ε̇ψ = −0.067 rad/s` at `v = 1`, `δ = 0.2`, so
   the vehicle turns right on the map *(verified)*. With `δ = atan(κL)` on a circle the
   legacy `ε̇ψ = 0.1 ≠ 0`, so steady-state cornering is not an equilibrium *(verified)*.
3. **Mixed reference points.** The yaw rate uses the centre-of-mass slip angle `β`
   (`v·cosβ·tanδ/(lf+lr)`), but `ṡ` and `ėy` use `epsi` without `β`. This is neither a
   rear-axle model nor a consistent CoM model. The new model uses the rear axle (`β = 0`).
4. The legacy kinematic Jacobian matches automatic differentiation of the legacy discrete
   map to 1.6e-17 *(verified)*. It linearises the wrong equations faithfully, so it cannot
   be kept.

### Geometry (`Environment.jl`)

5. **`global_to_local` is unreliable.** The loop runs `i` to `size(track_key_pts, 1)` and
   reads `track_key_pts[i+1]`, so any point it cannot place throws a `BoundsError`
   *(verified, off-track point)*. On segments spanning more than π it fails to place even
   valid points: `compute_angle` returns values in `(−π, π]`. A centreline point at ¾ lap
   of the legacy circular track throws `BoundsError` *(verified)*. When the loop does not
   throw, a point it cannot place returns a silent `(0, 0, 0)`.
6. **The legacy lab track is not closed.** `Lab_trackoption` ends 5.5 cm from its start
   *(verified)*. Wrapping it with `mod(s, L)` makes the position jump at the start/finish
   line.

### MPC scripts (deferred, recorded for later milestones)

7. `MPC with JuMP/main_dynamic.jl` builds its dynamics constraints from `value.(A)` and the
   other parameter values, i.e. fixed numbers. The later `set_value(A…)` calls never reach
   the constraints, so the QP stays linearised about the initial guess for the whole run.
8. `MPC with JuMP/OSQP_comparison.jl` linearises every horizon step about
   `LQ_predicted_states[iteration][1]`, the first state of the horizon, while taking the
   curvature from the k-th state.
9. `MPC with JuMP/main.jl` has these problems:
   - Its constraint between steps `k−1` and `k` uses `κ[k]` (index shift).
   - It allows steering up to ±90°, which reaches the `tan` singularity.
   - Its objective `−x'QN x` with `QN[1,1] = 50` maximises `s²`, which is non-convex.
10. `Controller.jl` cannot run as written:
    - `Linearized_collision_coeff` uses the undefined `ego_Start`.
    - `buildCollisionConstrLTI` uses the undefined `i`, `cur_coord`, `nx`, `x0`, `Cd`,
      `track_key_pts`.
    - `buildEqConstrLTV` uses the undefined `x0` and a hard-coded state size `6`.

## 2. Numerical weaknesses

- **No domain checks anywhere.** Nothing guards `1 − κ·ey → 0`, negative speed or
  `tan(δ)` near ±90°. Nothing detects NaN.
- **Dynamic bicycle regularisation.** The dynamics divide by `max(|vx|, 1e-10)`, but the
  Jacobian uses `vx + 1e-10` and `vx² + 1e-10`, so the two disagree for `vx < 0`. At
  `vx = 0` Jacobian entries reach 3.3e9 *(verified)*. Away from `vx = 0` the DBM Jacobian
  matches AD to 1.8e-11 *(verified)*.
- `wrap_angle` corrects only one multiple of 2π. `local_to_global` avoids `sign(0)` by
  adding `1e-20` (or `1e-10` in `Track_generator.jl`).
- Progress is wrapped with `mod` inside every geometry call, and closure is never checked.
  On an open track this silently produces wrong positions.
- `main_dynamic.jl` recomputes `linear_Matrices` for the whole horizon once per matrix
  entry, which is O(N²·nx²) work per step.
- The sanity scripts integrate with forward Euler at `dt = 0.1` and compare against
  linearisations at the same `dt`. Euler error and linearisation error are never separated.

## 3. Software issues

- **Absolute paths:**
  - `main.jl` and `Simple_MPC.jl`: `E:/UC Davis/Lab/Database/Julia`.
  - Every script under `MPC with JuMP/`, plus `Vehicle_model_sanity_check.jl` and both
    `Linearization_Sanity_*.jl`: `C:/Users/mahdi/Lab Repository/Julia/…`. That is a
    different checkout from this repository.
  - `OSQP_comparison.jl` includes a `visualization.jl` that is not in the repository.
- **Script-level side effects:** `Track_generator.jl` calls `plot_track` (PyPlot) when
  included. Every script plots and runs simulations at top level.
- **Broken module usage:**
  - `main.jl` calls `using .environment`, but that module is commented out.
  - `Simple_MPC.jl` depends on `RobotDynamics` and a commented-out module.
  - The inner constructor of `Simple_MPC.jl`'s `Player` passes its fields in the wrong
    order.
  - `Linearization_Sanity_check_traj.jl` pushes to `global_lin_traj` before defining it.
- **Hard-coded constants:**
  - The lap length `11.14` in `Simulator.jl` (the rectangular track length 11.1416).
  - The state size `6` in `buildEqConstrLTV`.
  - The scale factors `κ/10` and `κ/100` in the plots.
- **No `Project.toml`:** dependencies are implicit. They include PyPlot, PyCall, Plots,
  OSQP, JuMP, Ipopt, NLsolve, Parameters, StaticArrays and RobotDynamics. All structs are
  `mutable` and untyped (`@with_kw`), and the scripts rely on globals.

## 4. Parameter discrepancies

| source                                      | L = lf + lr | steering   | accel. [m/s²]     | speed [m/s] | other |
|:--------------------------------------------|:-----------:|:----------:|:-----------------:|:-----------:|:------|
| `NonlinearKinematicBicycleGame` defaults, `main.jl` | 3.0 m | ±14°       | ±5 (p1), ±10 (p2) | —           | |
| `MPC with JuMP/main.jl`                     | 0.2 m       | ±90°       | ±5                | [0, 5]      | |
| `NonlinearDynamicBicycleGame` defaults, `OSQP_comparison.jl` | 0.25 m | ±30° | ±10   | [0, 5]      | m = 1.98 kg, Iz = 0.024, cf = cr = 1.25 |

The full-scale 3 m kinematic wheelbase was used on tracks with 0.5–1 m radii and 0.5 m
width, so the geometry was dimensionally inconsistent. The cornering stiffness 1.25 has no
stated unit. All three sets are preserved verbatim in `LEGACY_PARAMETER_SETS` and are
**not** used by the new tests. The new `reference_test_vehicle()` takes `L = 0.25 m` from
the small-scale dynamic model (the scale the tracks were built for) and the
acceleration/speed bounds from the JuMP/OSQP scripts. It sets `δmax = 25°` so that
`atan(L/R) = 14.0°` is feasible for the tightest legacy radius `R = 1 m`. It is labelled a
test configuration, not a calibrated vehicle.

## 5. Reusable legacy components

- **Track construction** (`build_track_keypoint`, `local_to_global`, `get_curvature`): the
  forward map uses correct, consistent conventions (κ > 0 left, `ey` left, `ψ = ψ_c + eψ`).
  It was re-implemented in `src/tracks/track_geometry.jl`, and the regression test
  reproduces it to 1e-12 for the quarter, rectangular, lab and circular tracks.
- **Track definitions** (`Quarter_trackoption`, `Circular_trackoption`,
  `Rectangular_trackoption`, `Lab_trackoption`): copied verbatim into
  `LEGACY_TRACK_SPECS`. The quarter, circular and rectangular tracks close to within 3e-15 m.
  `Track_generator.jl` describes the same kind of rounded rectangle with hand-written key
  points and was not ported.
- **Frenet kinematics of the dynamic bicycle** (`ṡ`, `ėy`, `ε̇ψ = ω − κ·ṡ`): consistent
  with the geometry conventions. A candidate for a later milestone (see section 6).
- The `linear_Matrices` form `x⁺ ≈ A x + B u + C` with `C = f_d(x̄, ū) − A x̄ − B ū`, and the
  OSQP sparse QP layout in `Controller.jl` and `OSQP_comparison.jl`.

## 6. Deferred to later milestones

- The dynamic bicycle model (tyre parameters, low-speed regularisation, Jacobians).
- Every MPC script: JuMP/Ipopt NMPC, OSQP LTV-MPC, the JuMP parameter-update bug (item 7),
  and the linearisation-point bug (item 8).
- Collision-avoidance constraints (`Controller.jl`), multi-player simulation
  (`Simulator.jl`, `main.jl`), animation and plotting.
- Cartesian → Frenet initialisation for self-overlapping tracks beyond the `s_hint`
  mechanism.
- RK4 or higher-order integration in progress `s`, with curvature varying inside a step.

## 7. Final milestone: removal of the superseded scripts

Sections 1–6 describe the scripts as they were at commit `6c70d74`. In the final milestone
every legacy script except `Environment.jl` was removed from the working tree. They remain
in the Git history, unchanged, at commit `d4483c2` (and earlier), so this audit can still be
checked against them.

Criteria for removal, all checked before removing (`grep` over the package, tests,
examples, benchmarks, CI and documentation):
- not loaded by the package, the tests, the examples, the benchmarks or CI;
- unable to run in this repository (absolute paths into other checkouts, undefined names,
  missing or commented-out modules; section 3);
- superseded by tested code in `src/`;
- any values worth keeping already preserved: the three parameter sets verbatim in
  `LEGACY_PARAMETER_SETS` (their `source` fields name the original files), the track
  definitions verbatim in `LEGACY_TRACK_SPECS`.

| file | lines | why it cannot stay as a runnable script | superseded by |
|:--|--:|:--|:--|
| `New_model_Dynamic.jl` | 348 | wrong lateral and heading equations (items 1–3); `1e-10` regulariser with an inconsistent Jacobian (section 2) | `src/models/kinematic_bicycle.jl`; `src/models/dynamic_bicycle.jl` (operating domain `vx ≥ vx_min`, no regulariser); parameters in `LEGACY_PARAMETER_SETS`, `legacy_dynamic_parameters()` |
| `Controller.jl` | 188 | undefined names (item 10) | `src/controllers/linear_mpc.jl` (OSQP), obstacle constraints in `src/controllers/` |
| `Simulator.jl` | 58 | hard-coded lap length 11.14; depends on `Controller.jl` | `src/simulation/` |
| `main.jl` | 105 | absolute path `E:/UC Davis/…`; `using .environment` of a commented-out module | `examples/`, `examples/showcase/` |
| `Simple_MPC.jl` | 207 | absolute path; `RobotDynamics`; wrong constructor field order | `src/controllers/nonlinear_mpc.jl` |
| `Track_generator.jl` | 146 | plots (PyPlot) when included; hand-written key points not ported (section 5) | `src/tracks/track_geometry.jl` |
| `Plot_track.jl` | 25 | PyPlot-only plotting of a legacy track | `examples/plot_*.jl`, `examples/showcase/render_animation.py` |
| `Linearization_Sanity_Check.jl` | 93 | absolute path; Euler at `dt = 0.1` mixes discretisation and linearisation error (section 2) | `test/test_jacobians.jl`, `test/test_linear_mpc.jl` |
| `Linearization_Sanity_check_traj.jl` | 102 | as above; uses `global_lin_traj` before defining it | as above |
| `Vehicle_model_sanity_check.jl` | 52 | absolute path; legacy model | `test/test_kinematic_dynamics.jl`, `test/test_dynamic_bicycle.jl` |
| `MPC with JuMP/main.jl` | 132 | absolute path; index shift, ±90° steering, non-convex objective (item 9) | `src/controllers/nonlinear_mpc.jl` |
| `MPC with JuMP/main_dynamic.jl` | 467 | absolute path; constraints frozen at `value.(A)` (item 7) | `src/controllers/dynamic_nmpc.jl` |
| `MPC with JuMP/OSQP_comparison.jl` | 193 | absolute path; missing `visualization.jl`; linearisation-point bug (item 8) | `src/controllers/linear_mpc.jl`, `benchmarks/linear_mpc.jl` |
| `MPC with JuMP/simple_point_mass.jl` | 107 | absolute path; point-mass toy unrelated to the package | — |

**Kept:** `Environment.jl` (408 lines). The test "regression against legacy Environment.jl"
in `test/test_track_geometry.jl` loads it (with `Parameters`, a test-only dependency) as the
reference for the track geometry, so it is a test fixture. It is not loaded by the package.

The deferred items of section 6 were addressed later:
- the dynamic bicycle model and its MPC in the final milestone
  (`docs/dynamic_bicycle_mpc.md`);
- the OSQP LTV-MPC, without the bugs of items 7 and 8, in Milestone 4
  (`docs/linearized_mpc.md`);
- collision avoidance in Milestones 3–4 and the final milestone;
- animation in the final milestone (`examples/showcase/`).
