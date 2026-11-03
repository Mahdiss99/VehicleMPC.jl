"""
    NMPCConfig(; kwargs...)

Configuration of [`NMPCController`](@ref). All fields are keyword arguments with defaults;
the defaults are the baseline documented in `docs/nmpc_formulation.md`.

**Discretisation**
- `horizon = 20`: number of control intervals `N`.
- `dt = 0.1`: control and prediction period [s].
- `integrator = RK4()`: `RK4()` or `ForwardEuler()` for the prediction model.

**Objective weights** (dimensionless; every term is normalised, see below)
- `ws = 10.0`: reward on progress `(s[N+1] - s[1]) / (v_max·N·dt)`.
- `qy = 0.1`, `qpsi = 0.1`: penalties on `(ey/ey_max)²` and `(epsi/epsi_max)²` at
  `k = 2 … N+1`.
- `rdelta = 0.01`, `ra = 0.01`: penalties on `(delta/delta_max)²` and `(a/a_ref)²` with
  `a_ref = max(|a_min|, a_max)`.
- `rd_delta = 1.0`, `rd_a = 0.1`: penalties on the input increments
  `((delta[k]-delta[k-1])/delta_max)²` and `((a[k]-a[k-1])/a_ref)²`; for `k = 1` the
  previously applied input is used.

**Constraints**
- `track_margin = 0.05`: `|ey| ≤ track.width/2 - track_margin` at the predicted knots [m].
- `epsi_max = deg2rad(60)`: `|epsi| ≤ epsi_max < π/2`, which keeps `cos(epsi) ≥ 0.5`, so
  progress is forward while `v > 0`.
- `ay_max = 4.0`: kinematic lateral-acceleration limit `|v²·tan(delta)/L| ≤ ay_max`
  [m/s²]; `nothing` disables it. This is a controller tuning parameter, **not** a
  calibrated tyre-friction limit.
- Speed and input bounds come from the vehicle's `VehicleParameters`.

**Solver**
- `warm_start = true`: reuse the shifted previous solution as the initial guess.
- `max_iter = 200`, `tol = 1e-8`: Ipopt iteration limit and convergence tolerance.
- `max_wall_time = nothing`: optional Ipopt wall-clock limit [s].
- `acceptance_tol = 1e-6`: maximum dynamics residual and constraint violation for a
  solution to be accepted (checked independently of the solver).
- `optimizer_attributes = Pair{String,Any}[]`: extra raw Ipopt options.

**Multi-segment tracks: curvature schedule** (Milestone 2B.2; see `docs/multisegment_nmpc.md`)
- `schedule_tol = 1e-6`: fixed-point tolerance [m] on the largest change of the nominal
  progress between two schedule iterations.
- `schedule_max_iter = 20`: iteration cap of the fixed point. A schedule that has not
  converged is never used; the controller tries one cold initialisation, and otherwise
  returns `:scheduling_failure`.
- `max_stage_curvature_residual = 5e-3`: post-solve limit [rad] on
  `|psi_c(s*_{k+1}) - psi_c(s*_k) - kappa_k·(s*_{k+1} - s*_k)|`. This is the heading error
  the averaged curvature introduces on stage `k` along the *optimised* progress.
- `max_cumulative_curvature_residual = 1e-2`: post-solve limit [rad] on the largest partial
  sum of the stage residuals, i.e. the worst predicted heading error at any knot.
- `max_progress_mismatch = Inf`: optional limit [m] on `max_k |s*_k - s_nominal_k|`
  (reported always; not used for acceptance by default).
- `stale_schedule_policy = :reject`: `:reject` returns `:schedule_inconsistent` for a
  solution that violates the residual limits; `:accept_if_reference_valid` instead accepts
  it with the qualified status `:success_stale_schedule`, but only if the independent
  reference check passes.
- `schedule_corrections = 0`: optional extra NLP solves (each with a schedule recomputed
  from the previous solution) when the schedule is stale. The default is one solve per
  control cycle.
- `correction_time_budget = 0.08`: a correction is started only if the time used so far
  plus the duration of the previous `optimize!` stays within this budget [s].

**Curvature model** (Milestone 2B.3A; see `docs/progress_aware_curvature.md`)
- `curvature_model = :stagewise_average` (default): fixed per-stage curvature Parameters
  from the schedule (Milestone 2B.2).
- `curvature_model = :progress_linearized` (**experimental**): per-stage effective curvature
  variables `kappa_eff[k] = kbar_k + A_k (s_k - sbar_k) + B_k (s_{k+1} - sbar_{k+1})`,
  hard-bounded to the track's curvature range, with the first-order sensitivities of the
  stagewise average curvature as Parameters.
- `linearization_min_interval = 1e-3`: nominal stage progress [m] below which the local
  model is constant (`A = B = 0`).
- `linearization_trust_region = Inf`: optional trust region (fraction `ρ` of the nominal
  stage length). With a finite `ρ`, every knot that bounds a stage with non-zero
  sensitivities is constrained by `|s_k - sbar_k| ≤ ρ·|Δsbar|`, which keeps the linear
  model inside its region of validity. `Inf` (default) adds no constraint. This is an
  additional constraint on the NLP and is reported as such.
- `curvature_model = :segment_pair_exact` (**experimental**, Milestone 2B.3B; see
  `docs/segment_pair_curvature.md`): per-stage effective curvature with
  `kappa_eff·(s_{k+1} - s_k) = Δpsibar + kappa_j (s_{k+1} - sbar_{k+1}) - kappa_i (s_k - sbar_k)`.
  This is exact while every optimised knot stays in its assigned segment.
- `segment_membership = false`: segment-pair mode only. When `true`, every predicted
  progress knot is bounded to its assigned segment (variable bounds updated each cycle),
  which keeps the identity exact by construction (approach B).
- `segment_change_policy = :residual`: segment-pair mode only.
  - `:residual`: an optimised knot outside its assigned segment is reported, and
    acceptance is decided by the consistency residuals;
  - `:reject`: any such knot makes the schedule inconsistent, which triggers a correction
    solve if one is enabled.
- `boundary_knot_tol = 1e-6`: a nominal knot within this distance [m] of a segment
  boundary is treated as lying on it. The one-sided derivatives are then averaged and the
  knot is reported.

**Prediction model** (Milestone 2B.4; see `docs/cartesian_endpoint_nmpc.md`)
- `prediction_model = :frenet` (default): the Frenet dynamics integrated with
  `integrator`, together with the curvature model above.
- `prediction_model = :cartesian_endpoint_exact` (**experimental**): exact
  zero-order-hold Cartesian bicycle steps between knots.
  - The knot poses use the analytic formulas of each knot's assigned segment.
  - `integrator` and the curvature-model options are not used, and `curvature_model`
    must keep its default.
  - `schedule_corrections` and `correction_time_budget` bound the segment
    reassignments.
  - `segment_membership = true` bounds every progress knot to its assigned segment.
  - A knot outside its assigned segment (beyond `boundary_knot_tol`) is always rejected
    (`segment_change_policy` does not apply).

**Independent validation on the actual track**
- `reference_check = true`: re-simulate the optimised inputs with the event-aware
  integrator (actual piecewise curvature) and reject any hard constraint violation.
- `reference_substeps = 5`: samples per control interval for that check (step `dt/5`).
- `strict_corridor = false`: when `true`, a solution whose re-simulation leaves the
  tightened planning corridor `|ey| ≤ width/2 - track_margin` at any sample (beyond
  `acceptance_tol`) is rejected with `:corridor_violation`. The default keeps the earlier
  acceptance rule, in which only the physical track edge is a hard limit in the
  re-simulation. This rule applies to every prediction model.
- `dense_check_samples = 0`: when positive, every candidate is also checked with the
  dense exact-map corridor check (`corridor_check`) at this many samples per interval.
  It is reported in `result.corridor`; with `strict_corridor`, a corridor violation it
  finds rejects the candidate.

**Stationary-obstacle avoidance** (Milestone 3A, experimental; only for a controller built
with `obstacle = CircleObstacle(...)`; see `docs/stationary_collision_avoidance.md`)
- `ego_radius = 0.0`: radius [m] of the disk that represents the ego vehicle in collision
  checks, centred at the rear-axle reference point. This is a collision proxy, not the
  vehicle body.
- `obstacle_buffer = 0.0`: safety buffer [m]. The required centre separation is
  `D = ego_radius + obstacle.radius + obstacle_buffer`.
- `obstacle_samples = 0`: additional collision constraints at
  `j/(obstacle_samples + 1)` of every interval, using the exact intermediate position (0:
  knots only).
- `obstacle_check_samples = 50`: samples per interval of the independent dense collision
  evaluation of every candidate.
- `obstacle_ramp = 1.5`, `obstacle_ramp_min = 0.3`: lateral ramp lengths [m] of the
  passing-side initial guesses (Milestone 3B). The approach ramp is
  `clamp(distance to the obstacle - D, obstacle_ramp_min, obstacle_ramp)`, and the
  departure ramp is `obstacle_ramp`.
- `obstacle_acceptance = :collision`: in the dense evaluation, `:collision` rejects any
  overlap of the ego disk and the obstacle (distance `< ego_radius + radius`), and reports
  buffer intrusions; `:separation` rejects any distance `< D`.

**Segment-reassignment reliability and cycle budget** (Milestone 3B.1, experimental; see
`docs/segment_reassignment_reliability.md`)
- `reassignment_cycle_detection = false`: in the Cartesian-endpoint mode, stop the
  reassignment when a proposed lap-aware assignment vector repeats an earlier one in the
  same solve (a cycle), and record it.
- `branch_resolution = false`: with cycle detection, resolve a single-knot cycle between
  two adjacent segments. Two candidate NLPs are solved, each with that knot temporarily
  bounded to one of the segments, and the valid one with the lower objective is kept.
- `encounter_strategy = :both`: obstacle encounters solve both passing-side candidates
  (`:both`, the Milestone 3A/3B behaviour), or the higher-ranked one first and the other
  only if it is rejected (`:sequential`).
- `cycle_budget = Inf`: total budget [s] of one control cycle for **optional** solves:
  - the second passing-side candidate;
  - the warm-start retry;
  - reassignments;
  - branch candidates.

  An optional solve starts only if the time used so far in the cycle plus
  `1.25 ×` the longest solve so far in this cycle fits. This is a best-effort guard, not a
  real-time guarantee.

**Between-knot corridor constraints** (Milestone 2B.5, Cartesian-endpoint mode only; see
`docs/intersample_corridor_constraints.md`)
- `interior_samples = 0`: `0` (none; the Milestone 2B.4 formulation), `1` (midpoint,
  `alpha = 0.5`) or `3` (`alpha = 0.25, 0.5, 0.75`). At every interior sample of every
  interval the exact vehicle position must lie in the planning corridor of its assigned
  segment: a strip for straight segments, an annulus for curved ones.
"""
Base.@kwdef struct NMPCConfig
    horizon::Int = 20
    dt::Float64 = 0.1
    integrator::AbstractIntegrator = RK4()
    ws::Float64 = 10.0
    qy::Float64 = 0.1
    qpsi::Float64 = 0.1
    rdelta::Float64 = 0.01
    ra::Float64 = 0.01
    rd_delta::Float64 = 1.0
    rd_a::Float64 = 0.1
    track_margin::Float64 = 0.05
    epsi_max::Float64 = deg2rad(60)
    ay_max::Union{Nothing,Float64} = 4.0
    warm_start::Bool = true
    max_iter::Int = 200
    tol::Float64 = 1e-8
    max_wall_time::Union{Nothing,Float64} = nothing
    acceptance_tol::Float64 = 1e-6
    optimizer_attributes::Vector{Pair{String,Any}} = Pair{String,Any}[]
    schedule_tol::Float64 = 1e-6
    schedule_max_iter::Int = 20
    max_stage_curvature_residual::Float64 = 5e-3
    max_cumulative_curvature_residual::Float64 = 1e-2
    max_progress_mismatch::Float64 = Inf
    stale_schedule_policy::Symbol = :reject
    schedule_corrections::Int = 0
    correction_time_budget::Float64 = 0.08
    reference_check::Bool = true
    reference_substeps::Int = 5
    curvature_model::Symbol = :stagewise_average
    linearization_min_interval::Float64 = 1e-3
    linearization_trust_region::Float64 = Inf
    boundary_knot_tol::Float64 = 1e-6
    segment_membership::Bool = false
    segment_change_policy::Symbol = :residual
    prediction_model::Symbol = :frenet
    strict_corridor::Bool = false
    dense_check_samples::Int = 0
    interior_samples::Int = 0
    ego_radius::Float64 = 0.0
    obstacle_buffer::Float64 = 0.0
    obstacle_samples::Int = 0
    obstacle_check_samples::Int = 50
    obstacle_acceptance::Symbol = :collision
    obstacle_ramp::Float64 = 1.5
    obstacle_ramp_min::Float64 = 0.3
    reassignment_cycle_detection::Bool = false
    branch_resolution::Bool = false
    encounter_strategy::Symbol = :both
    cycle_budget::Float64 = Inf
end

function validate_config(c::NMPCConfig)
    c.horizon >= 1 || throw(ArgumentError("horizon must be ≥ 1, got $(c.horizon)"))
    check_timestep(c.dt)
    c.integrator isa Union{RK4,ForwardEuler} ||
        throw(ArgumentError("NMPC supports RK4() and ForwardEuler(), got $(c.integrator)"))
    for f in (:ws, :qy, :qpsi, :rdelta, :ra, :rd_delta, :rd_a, :track_margin)
        v = getfield(c, f)
        (isfinite(v) && v >= 0) || throw(ArgumentError("$f must be finite and nonnegative, got $v"))
    end
    0 < c.epsi_max < π / 2 || throw(ArgumentError("epsi_max must lie in (0, π/2), got $(c.epsi_max)"))
    c.ay_max === nothing || (isfinite(c.ay_max) && c.ay_max > 0) ||
        throw(ArgumentError("ay_max must be positive and finite or nothing, got $(c.ay_max)"))
    c.max_iter >= 1 || throw(ArgumentError("max_iter must be ≥ 1"))
    c.tol > 0 && c.acceptance_tol > 0 || throw(ArgumentError("tolerances must be positive"))
    c.schedule_tol > 0 || throw(ArgumentError("schedule_tol must be positive"))
    c.schedule_max_iter >= 1 || throw(ArgumentError("schedule_max_iter must be ≥ 1"))
    for f in (:max_stage_curvature_residual, :max_cumulative_curvature_residual, :max_progress_mismatch)
        getfield(c, f) > 0 || throw(ArgumentError("$f must be positive"))
    end
    c.stale_schedule_policy in (:reject, :accept_if_reference_valid) ||
        throw(ArgumentError("stale_schedule_policy must be :reject or :accept_if_reference_valid"))
    c.schedule_corrections >= 0 || throw(ArgumentError("schedule_corrections must be ≥ 0"))
    c.correction_time_budget > 0 || throw(ArgumentError("correction_time_budget must be positive"))
    c.reference_substeps >= 1 || throw(ArgumentError("reference_substeps must be ≥ 1"))
    c.curvature_model in (:stagewise_average, :progress_linearized, :segment_pair_exact) ||
        throw(ArgumentError("curvature_model must be :stagewise_average, :progress_linearized or :segment_pair_exact, got $(c.curvature_model)"))
    c.segment_change_policy in (:residual, :reject) ||
        throw(ArgumentError("segment_change_policy must be :residual or :reject, got $(c.segment_change_policy)"))
    c.linearization_trust_region > 0 || throw(ArgumentError("linearization_trust_region must be positive (Inf disables it)"))
    c.linearization_min_interval > 0 && c.boundary_knot_tol > 0 ||
        throw(ArgumentError("linearization_min_interval and boundary_knot_tol must be positive"))
    c.prediction_model in (:frenet, :cartesian_endpoint_exact) ||
        throw(ArgumentError("prediction_model must be :frenet or :cartesian_endpoint_exact, got $(c.prediction_model)"))
    c.prediction_model === :cartesian_endpoint_exact && c.curvature_model !== :stagewise_average &&
        throw(ArgumentError("prediction_model = :cartesian_endpoint_exact uses no curvature model; leave curvature_model at its default"))
    c.dense_check_samples >= 0 || throw(ArgumentError("dense_check_samples must be ≥ 0"))
    c.interior_samples in (0, 1, 3) || throw(ArgumentError("interior_samples must be 0, 1 or 3, got $(c.interior_samples)"))
    (c.ego_radius >= 0 && c.obstacle_buffer >= 0 && isfinite(c.ego_radius + c.obstacle_buffer)) ||
        throw(ArgumentError("ego_radius and obstacle_buffer must be finite and nonnegative"))
    c.obstacle_samples >= 0 && c.obstacle_check_samples >= 1 ||
        throw(ArgumentError("obstacle_samples must be ≥ 0 and obstacle_check_samples ≥ 1"))
    0 < c.obstacle_ramp_min <= c.obstacle_ramp < Inf ||
        throw(ArgumentError("need 0 < obstacle_ramp_min ≤ obstacle_ramp < Inf"))
    c.encounter_strategy in (:both, :sequential) ||
        throw(ArgumentError("encounter_strategy must be :both or :sequential, got $(c.encounter_strategy)"))
    c.cycle_budget > 0 || throw(ArgumentError("cycle_budget must be positive (Inf disables it)"))
    !c.branch_resolution || c.reassignment_cycle_detection ||
        throw(ArgumentError("branch_resolution requires reassignment_cycle_detection = true"))
    c.obstacle_acceptance in (:collision, :separation) ||
        throw(ArgumentError("obstacle_acceptance must be :collision or :separation, got $(c.obstacle_acceptance)"))
    c.interior_samples == 0 || c.prediction_model === :cartesian_endpoint_exact ||
        throw(ArgumentError("interior_samples > 0 requires prediction_model = :cartesian_endpoint_exact"))
    return c
end
