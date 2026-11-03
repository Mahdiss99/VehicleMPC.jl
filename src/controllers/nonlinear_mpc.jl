# Nonlinear MPC (direct multiple shooting) for the kinematic bicycle, built once with JuMP
# and re-solved with Ipopt. See docs/nmpc_formulation.md for the full formulation.

"""
Wall-clock breakdown of one [`solve!`](@ref) call, in seconds.

- `update`: writing the measured state and previous input into the JuMP parameters.
- `warm_start`: building the initial guess and calling `set_start_value`.
- `optimize`: wall time of `JuMP.optimize!`, including the JuMP → Ipopt hand-over.
- `solver`: solve time reported by Ipopt (`NaN` if unavailable).
- `validation`: extracting the solution and validating it independently (knot residuals).
- `total`: the whole `solve!` call, including everything below.
- `schedule`: building and refining the curvature schedule (all attempts); in the
  Cartesian-endpoint mode, the initial guess and the segment assignments.
- `reference`: the independent event-aware re-simulation on the actual track and, in the
  Cartesian-endpoint mode, the exact-map reconstruction ([`ExactMapReport`](@ref)).

`optimize` and `solver` add up over all NLP solves of the call.
"""
struct NMPCTiming
    update::Float64
    warm_start::Float64
    optimize::Float64
    solver::Float64
    validation::Float64
    total::Float64
    schedule::Float64
    reference::Float64
end

"""
Curvature-schedule diagnostics of one control cycle (see `docs/multisegment_nmpc.md`).

Pre-solve (fixed point):
- `method`: `:constant` (single-segment track: the exact segment curvature, no averaging),
  `:warm`, `:cold`, or `:cold_fallback` (the warm fixed point did not converge and one
  cold initialisation was tried);
- `iterations`, `converged`: fixed-point iterations and convergence;
- `max_progress_change`: last fixed-point update `max_k |Δs_k|` [m];
- `kappas`, `s_nominal`: stagewise curvature passed to the NLP and its nominal progress.

Post-solve (along the optimised progress `s*`):
- `stage_residual`: `max_k |psi_c(s*_{k+1}) - psi_c(s*_k) - kappa_k·(s*_{k+1} - s*_k)|` [rad];
- `cumulative_residual`: largest |partial sum| of those residuals over the knots [rad];
- `progress_mismatch`: `max_k |s*_k - s_nominal_k|` [m];
- `crossing_stages`: number of stages whose optimised progress crosses a segment boundary;
- `consistent`: all post-solve limits met (`false` if not evaluated);
- `message`: details of a failure.

Curvature model (Milestone 2B.3A):
- `curvature_model`: `:stagewise_average`, `:progress_linearized`, or `:constant`
  (single-segment track);
- `model_curvature_error`: `max_k |kappa_k - Kexact_k(s*)|`. This compares the curvature
  the NLP actually used on each stage (Parameter, or optimised `kappa_eff`) with the exact
  average curvature over the optimised progress [1/m];
- `configuration_changes`: stages whose optimised end points lie in different segments
  than the nominal end points, i.e. where the local curvature model's assumed segment
  configuration changed;
- `boundary_knots`, `short_intervals`: nominal knots on a segment boundary (averaged
  one-sided derivatives), and stages using the constant local model (linearized mode).
- `saturated_stages`: stages whose optimised `kappa_eff` lies on its curvature bound while
  the nominal `kbar` does not. This indicates that the linear model was pushed outside
  its range (linearized mode).
- `trust_region_knots`, `trust_region_active`: knots with a finite trust region, and knots
  where it is active at the solution (linearized mode with a finite
  `linearization_trust_region`).
- `min_progress_increment`: smallest optimised stage progress `min_k (s*_{k+1} - s*_k)` [m].
  A negative or near-zero value flags backward or stalled predicted progress.

In the segment-pair mode, `configuration_changes` counts stages with an optimised knot
outside its assigned segment interval, i.e. where the exact identity no longer holds.

In the Cartesian-endpoint mode (Milestone 2B.4, `prediction_model = :cartesian_endpoint_exact`):
- `method`: `:warm`, `:cold` or `:constant` (single-segment cold start), with a
  `_corrected` suffix after a reassignment;
- `kappas`: the nominal stage curvatures (reporting only; the NLP uses no curvature
  schedule);
- `stage_residual`, `cumulative_residual`: the exact-map heading residual on the **true**
  track geometry, `psi(X*_{k+1}) - psi(X*_k) - phi_k`, and its largest partial sum [rad];
- `configuration_changes`: optimised knots outside their assigned segment (beyond
  `boundary_knot_tol`). Any such knot makes `consistent = false`.
"""
struct ScheduleReport
    method::Symbol
    iterations::Int
    converged::Bool
    max_progress_change::Float64
    kappas::Vector{Float64}
    s_nominal::Vector{Float64}
    stage_residual::Float64
    cumulative_residual::Float64
    progress_mismatch::Float64
    crossing_stages::Int
    consistent::Bool
    message::String
    curvature_model::Symbol
    model_curvature_error::Float64
    configuration_changes::Int
    boundary_knots::Int
    short_intervals::Int
    saturated_stages::Int
    trust_region_knots::Int
    trust_region_active::Int
    min_progress_increment::Float64
end

"""
Independent validation of the optimised inputs on the actual track.

The inputs are re-simulated from the measured state with `simulate_on_track` (actual
piecewise curvature, resolved boundary crossings) at `reference_substeps` samples per
interval.

- `checked`: whether the check ran;
- `status`: the simulation status (`:success`, `:domain_error`, `:end_of_track`, ...);
- `violations`: `t => ConstraintViolation` at the samples. These cover the actual track
  edge (no margin), speed, inputs, `|epsi| <= epsi_max` and `|ay| <= ay_max`, with
  tolerance `acceptance_tol`;
- `max_position_error`, `max_heading_error`, `max_progress_error`, `max_ey_error`:
  predicted knots vs the reference at the knots [m, rad, m, m];
- `max_abs_ey`, `max_abs_ay`: extremes over all samples;
- `crossings`: segment-boundary crossings in the reference;
- `passed`: no violation and a successful simulation;
- `first_step_position_error`: Cartesian position error of the first predicted knot `X[:,2]`
  against the reference [m];
- `min_boundary_distance`: `width/2 - max |ey|` over the reference samples [m] (negative
  means the track edge was crossed).
- `min_corridor_distance`: `ey_max - max |ey|` with the tightened planning corridor
  `ey_max = width/2 - track_margin` [m]. Negative values mean the corridor (not
  necessarily the track) was left between or at knots. This is reported separately and is
  not a rejection criterion.
"""
struct ReferenceReport
    checked::Bool
    status::Symbol
    violations::Vector{Pair{Float64,ConstraintViolation}}
    max_position_error::Float64
    max_heading_error::Float64
    max_progress_error::Float64
    max_ey_error::Float64
    max_abs_ey::Float64
    max_abs_ay::Float64
    crossings::Int
    passed::Bool
    message::String
    first_step_position_error::Float64
    min_boundary_distance::Float64
    min_corridor_distance::Float64
end

"""
Independent check of a prediction with the validated exact Cartesian map (Milestone 2B.4).

The optimised knots are mapped to Cartesian poses with the **true** track geometry
(`frenet_state_to_cartesian`, segment look-up in `Float64`). The inputs are propagated from
the measured state with [`exact_cartesian_step`](@ref). The two are then compared.

- `checked`, `status`: `:success`, `:not_run`, `:off_track` (a knot is outside the
  Frenet-valid region or an open track), or `:projection_failed` (a reconstructed pose
  has no centreline projection, so the Frenet mismatches are `NaN`);
- `max_position_error`, `max_heading_error`, `max_speed_error`: reconstruction vs knots
  over the horizon [m, rad, m/s];
- `max_progress_error`, `max_ey_error`: the reconstruction projected back to Frenet
  coordinates (hint: the knot's progress) vs the knots [m];
- `first_step_position_error`: position error at `X[:, 2]` [m];
- `knot_residual`: largest one-step residual `|z(X_{k+1}) - exact_step(z(X_k), U_k)|`
  (position, heading, speed) with the true geometry;
- `geometry_residual`: largest `|pose_assigned(X_k) - pose_true(X_k)|` over the knots,
  i.e. how far the segment-local formulas used by the NLP are from the true track at the
  optimised knots (`NaN` without an assignment);
- `message`: details.
"""
struct ExactMapReport
    checked::Bool
    status::Symbol
    max_position_error::Float64
    max_heading_error::Float64
    max_speed_error::Float64
    max_progress_error::Float64
    max_ey_error::Float64
    first_step_position_error::Float64
    knot_residual::Float64
    geometry_residual::Float64
    message::String
end

const EXACT_NOT_RUN = ExactMapReport(false, :not_run, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, "")

"""
Between-knot corridor diagnostics (Milestone 2B.5). The dense part comes from
[`corridor_check`](@ref): exact Cartesian reconstruction and true track projection at
`dense_samples` samples per interval. The interior part comes from
[`interior_assignment_check`](@ref).

- `dense_samples`: samples per interval (0 = dense check not run);
- `min_track_clearance`, `min_corridor_clearance`: `width/2 - |ey|` and `ey_max - |ey|`,
  minimised over the samples [m];
- `worst_time`, `worst_stage`: where the corridor clearance is smallest;
- `violations`, `max_violation`: samples outside the corridor beyond `acceptance_tol`, and
  the largest excess [m];
- `projection_failures`: samples without a unique track projection (they are excluded from
  the minima, and are counted);
- `interior_samples`, `interior_failures`, `interior_min_margin`: the NLP's interior sample
  count, interior samples with an invalid segment assignment, and the smallest corridor
  inequality value at those samples;
- `interior_prediction_error`: NLP interior points vs the exact reconstruction at the same
  times [m];
- `message`: details.
"""
struct CorridorReport
    dense_samples::Int
    min_track_clearance::Float64
    min_corridor_clearance::Float64
    worst_time::Float64
    worst_stage::Int
    violations::Int
    max_violation::Float64
    projection_failures::Int
    interior_samples::Int
    interior_failures::Int
    interior_min_margin::Float64
    interior_prediction_error::Float64
    message::String
end

const CORRIDOR_NOT_RUN = CorridorReport(0, NaN, NaN, NaN, 0, 0, NaN, 0, 0, 0, NaN, NaN, "")

"""
Segment-reassignment diagnostics of one `solve_once!` call in the Cartesian-endpoint mode
(Milestone 3B.1).

**`trace`**: one entry per NLP solve, `(; solve, ids, changed, outside, outside_s, status,
termination, iterations, dyn_res, viol, objective, solver_time, obstacle_clearance, branch)`:
- `ids`: the lap-aware segment id of every knot `1 … N+1` used by that solve;
- `changed`: knots whose id differs from the previous solve's;
- `outside`, `outside_s`: knots outside their assigned segment, and their optimised
  progress;
- `obstacle_clearance`: smallest knot distance to the obstacle minus `D` (`NaN` without an
  obstacle);
- `branch`: `0`, or the segment id that a branch-resolution solve bounded its knot to.

**Cycle detection and branch resolution:**
- `cycles`: number of repeated assignment configurations detected (0 or 1);
- `cycle_length`: solves between the two equal configurations;
- `cycle_knots`: knots whose segment differs within the cycle;
- `branch`: branch-resolution candidates `(; knot, segment, s0, s1, status, objective,
  solves, seconds)`;
- `branch_selected`: the selected segment id (0 if none);
- `message`: details.
"""
struct ReassignmentReport
    trace::Vector{Any}
    cycles::Int
    cycle_length::Int
    cycle_knots::Vector{Int}
    branch::Vector{Any}
    branch_selected::Int
    message::String
end

const REASSIGNMENT_NONE = ReassignmentReport(Any[], 0, 0, Int[], Any[], 0, "")

empty_schedule_report(method=:none, msg=""; model=:none) =
    ScheduleReport(method, 0, false, NaN, Float64[], Float64[], NaN, NaN, NaN, 0, false, msg, model, NaN, 0, 0, 0, 0, 0, 0, NaN)
const REFERENCE_NOT_RUN = ReferenceReport(false, :not_run, Pair{Float64,ConstraintViolation}[], NaN, NaN, NaN, NaN, NaN, NaN, 0, false, "", NaN, NaN, NaN)

"Pre-solve schedule report (post-solve fields not evaluated)."
presolve_schedule_report(sched, converged, msg, model) =
    ScheduleReport(sched.method, sched.iterations, converged, sched.change, collect(Float64, sched.kappas),
        collect(Float64, sched.s_nominal), NaN, NaN, NaN, 0, false, msg, model, NaN, 0, 0, 0, 0, 0, 0, NaN)

"""
Result of one [`solve!`](@ref) call.

- `status`:
  - `:success`: the solution was accepted;
  - `:invalid_measurement`: the measured state or previous input is outside the model
    domain, and the solver was not called;
  - `:solver_failure`: Ipopt did not report a locally optimal, primal-feasible point;
  - `:invalid_solution`: Ipopt reported success, but the independent knot validation
    failed (dynamics residual, bounds, finiteness);
  - `:scheduling_failure`: no self-consistent curvature schedule was found (fixed point not
    converged, or the horizon leaves an open track), so the solver was not called;
  - `:schedule_inconsistent`: the solution is valid for the curvature schedule it was
    computed with, but that schedule is stale along the optimised progress (post-solve
    residual limits exceeded);
  - `:reference_violation`: the re-simulation on the actual track violates a hard
    constraint or leaves the model domain;
  - `:success_stale_schedule`: only with `stale_schedule_policy = :accept_if_reference_valid`.
    The schedule is stale, but the reference check passed. This is a qualified acceptance.
    It is never used in the Cartesian-endpoint mode;
  - `:segment_assignment_invalid` (Cartesian-endpoint mode): an optimised knot left the
    segment whose formulas the NLP used, so the solution is not a solution of the true
    geometry;
  - `:interior_assignment_invalid` (Cartesian-endpoint mode with `interior_samples > 0`):
    an interior corridor sample does not project uniquely into the segment whose corridor
    inequality the NLP used;
  - `:corridor_violation`: only with `strict_corridor = true`. The re-simulation (or the
    dense check) stays on the physical track, but leaves the tightened planning
    corridor.
- `accepted`: `true` for `:success` and `:success_stale_schedule`. Only then is `u` a
  control to apply.
- `termination_status`, `primal_status`: as reported by JuMP/MOI.
- `X` (4×(N+1)), `U` (2×N): predicted states and inputs (`NaN` when unavailable).
  `X[:, 1]` is the measured state.
- `u`: first control action `U[:, 1]`; `nothing` unless accepted.
- `objective`, `iterations`: objective value and Ipopt iteration count (`-1` if unknown).
- `timing`: [`NMPCTiming`](@ref).
- `max_dynamics_residual`: `max |X[:,k+1] - f_d(X[:,k], U[:,k])|`, recomputed with the
  checked `discrete_dynamics`.
- `max_constraint_violation`: largest violation of any bound or inequality at the knots.
- `warm_started`: whether the initial guess came from the previous solution.
- `message`: reason for rejection, or notes (empty on a clean success).
- `schedule`: [`ScheduleReport`](@ref); `reference`: [`ReferenceReport`](@ref).
- `nlp_solves`: number of Ipopt solves in this control cycle (0 if the solver was not
  called).
- `exact`: [`ExactMapReport`](@ref) (Cartesian-endpoint mode; `:not_run` otherwise).
- `corridor`: [`CorridorReport`](@ref) (interior samples and/or dense check;
  `dense_samples == 0` when the dense check did not run).
- `obstacle`: [`ObstacleReport`](@ref) (controllers built with an obstacle;
  `present == false` otherwise).
- `reassignment`: [`ReassignmentReport`](@ref) (Cartesian-endpoint mode; empty
  otherwise).

Statuses specific to a controller with an obstacle (Milestone 3A):
- `:initial_collision`: the measured state already violates the required separation; the
  solver is not called;
- `:obstacle_constraint_violation`: the solution violates an imposed collision constraint
  beyond `acceptance_tol`;
- `:collision_detected`: the dense evaluation of the plan finds the ego disk overlapping
  the obstacle (with `obstacle_acceptance = :separation`, any violation of the required
  separation).

A missed deadline is not a status. Check `timing.total`, or use
[`deadline_missed`](@ref).
"""
struct NMPCResult
    status::Symbol
    accepted::Bool
    termination_status::MOI.TerminationStatusCode
    primal_status::MOI.ResultStatusCode
    X::Matrix{Float64}
    U::Matrix{Float64}
    u::Union{Nothing,SVector{2,Float64}}
    objective::Float64
    iterations::Int
    timing::NMPCTiming
    max_dynamics_residual::Float64
    max_constraint_violation::Float64
    warm_started::Bool
    message::String
    schedule::ScheduleReport
    reference::ReferenceReport
    nlp_solves::Int
    exact::ExactMapReport
    corridor::CorridorReport
    obstacle::ObstacleReport
    reassignment::ReassignmentReport
end

"Whether the complete online computation of `r` took longer than `deadline` seconds."
deadline_missed(r::NMPCResult, deadline::Real=0.1) = r.timing.total > deadline

"""
    NMPCController(vehicle, track, config=NMPCConfig(); optimizer=Ipopt.Optimizer, obstacle=nothing)

Build the nonlinear MPC problem once. Later [`solve!`](@ref) calls only update parameters
(measured state, previous input, stagewise curvature) and initial guesses.

**Tracks.**
- Single-segment tracks use their exact constant curvature.
- Multi-segment tracks (Milestone 2B.2) use a stagewise average-curvature schedule. It is
  computed outside Ipopt before every solve and passed through the `kappa[k]` Parameters.
  See `docs/multisegment_nmpc.md`.

**Frenet safety.** With `[kappa_min, kappa_max]` the range of segment curvatures (every
stagewise average lies in it), the constructor requires `1 - kappa·ey ≥ 2·eps_geometry`:
- at the corridor `|ey| ≤ width/2 - track_margin`, for both curvature extremes;
- for the RK4 stage-point bounds, which are the intersection of the Frenet-safe half-spaces
  of both extremes.

The condition is affine in `kappa`, so it then holds for every legal curvature value.
An `ArgumentError` is thrown when no such region exists.

**Cartesian-endpoint mode** (`prediction_model = :cartesian_endpoint_exact`, experimental,
Milestone 2B.4; see `docs/cartesian_endpoint_nmpc.md`).
- The RK4 Frenet dynamics are replaced by four equalities per stage. They equate the
  Cartesian pose and speed of knot `k+1` with the exact zero-order-hold bicycle step from
  knot `k` ([`exact_bicycle_increment`](@ref)).
- Knot poses come from segment-local centreline formulas ([`frenet_pose_local`](@ref)),
  whose segment constants are Parameters (`cframe`), updated every cycle.
- There are no stage points and no curvature schedule in the dynamics. `sin(z)/z` enters
  as the registered operator `sinc_un` with analytic first and second derivatives.
- With `interior_samples = 1` or `3` (Milestone 2B.5; see
  `docs/intersample_corridor_constraints.md`), two corridor inequalities are added at
  each interior sample `alpha·dt` of every interval. The exact interior point is a function
  of the knot variables and inputs; the segment constants are Parameters (`iframe`). This
  requires `ey_max < 1/|kappa|` on every curved segment, which is checked.
"""
mutable struct NMPCController
    vehicle::KinematicBicycle{Float64}
    track::Track{Float64}
    config::NMPCConfig
    model::JuMP.Model
    X::Matrix{VariableRef}                  # 4×(N+1); column 1 are Parameters (measured state)
    U::Matrix{VariableRef}                  # 2×N
    P::Array{VariableRef,3}                 # (ey, epsi, v) × RK4 stage point (2,3,4) × N; empty for Euler
    u_prev::Vector{VariableRef}             # Parameters: previously applied input
    kappa::Vector{VariableRef}              # Parameters: curvature on each interval
    dynamics::Vector{ConstraintRef}
    ey_max::Float64                         # knot bound |ey| ≤ ey_max
    stage_ey_bounds::Tuple{Float64,Float64} # Frenet-safe bounds on RK4 stage points
    kappa_range::Tuple{Float64,Float64}     # legal curvature-parameter values [kappa_min, kappa_max]
    kappa_eff::Vector{VariableRef}          # :progress_linearized only: effective curvature per stage
    lin_kbar::Vector{VariableRef}           # Parameters: nominal average curvature (linearized mode)
    lin_A::Vector{VariableRef}              # Parameters: ∂kbar/∂s_k
    lin_B::Vector{VariableRef}              # Parameters: ∂kbar/∂s_{k+1}
    lin_sbar::Vector{VariableRef}           # Parameters: nominal progress at the N+1 knots
    lin_tr::Vector{VariableRef}             # Parameters: trust-region radius per knot (1e6 = inactive)
    sp_w::Vector{VariableRef}               # segment-pair Parameters: 1 = bilinear crossing stage, 0 = kappa_eff = kc
    sp_dpsi::Vector{VariableRef}            #   nominal heading increment over the stage
    sp_ki::Vector{VariableRef}              #   curvature of the start knot's assigned segment
    sp_kj::Vector{VariableRef}              #   curvature of the end knot's assigned segment
    sp_kc::Vector{VariableRef}              #   constant curvature for w = 0 stages
    cframe::Matrix{VariableRef}             # Cartesian-endpoint Parameters, 5×(N+1): assigned segment (s0, x0, y0, psi0, kappa) per knot; 0×0 otherwise
    alphas::Vector{Float64}                 # interior sample fractions of each interval (empty: no interior constraints)
    iframe::Matrix{VariableRef}             # interior corridor Parameters, 8×(N·length(alphas)); see corridor_parameters
    interior::Vector{ConstraintRef}         # interior corridor inequalities (2 per sample)
    obstacle::Union{Nothing,CircleObstacle} # stationary obstacle (Milestone 3A), or nothing
    obs_par::Vector{VariableRef}            # Parameters (ox, oy, D) of the collision constraints
    obs_alphas::Vector{Float64}             # interior obstacle-sample fractions of each interval
    obs_cons::Vector{ConstraintRef}         # collision constraints: knots 2…N+1, then interior samples
    obstacle_side::Symbol                   # :none (not yet chosen), :left, :right
    obstacle_encounter::Float64             # unwrapped progress of the association whose side was chosen (NaN: none)
    last_X::Union{Nothing,Matrix{Float64}}
    last_U::Union{Nothing,Matrix{Float64}}
    build_time::Float64
end

"Lateral acceleration of the kinematic bicycle, `v²·tan(delta)/L` [m/s²]."
lateral_acceleration(vehicle::KinematicBicycle, v, delta) = v^2 * tan(delta) / vehicle.params.wheelbase

function NMPCController(vehicle::KinematicBicycle, track::Track, config::NMPCConfig=NMPCConfig();
    optimizer=Ipopt.Optimizer, obstacle::Union{Nothing,CircleObstacle}=nothing)
    validate_config(config)
    obstacle === nothing || config.prediction_model === :cartesian_endpoint_exact ||
        throw(ArgumentError("obstacle avoidance requires prediction_model = :cartesian_endpoint_exact"))
    kmin, kmax = extrema(seg.curvature for seg in track.segments)
    kappa = track.segments[1].curvature                 # initial parameter value
    p, N, dt = vehicle.params, config.horizon, config.dt
    L = p.wheelbase

    ey_max = track.width / 2 - config.track_margin
    ey_max > 0 || throw(ArgumentError("track.width/2 - track_margin must be positive, got $ey_max"))
    metric_min = 2 * vehicle.eps_geometry
    for k in (kmin, kmax), ey in (-ey_max, ey_max)      # affine in kappa and ey: corners suffice
        1 - k * ey >= metric_min || throw(ArgumentError(
            "admissible region |ey| ≤ $ey_max reaches the Frenet safety margin for kappa = $k: " *
            "1 - kappa*ey = $(1 - k * ey) < $metric_min"))
    end
    # RK4 stage points need only model validity, not the track boundary: intersect the
    # half-spaces 1 - kappa*ey ≥ metric_min for kappa = kmin and kappa = kmax.
    stage_ey = (kmin < 0 ? (1 - metric_min) / kmin : -Inf, kmax > 0 ? (1 - metric_min) / kmax : Inf)

    t_build = @elapsed begin
        m = Model(optimizer)
        set_silent(m)
        set_attribute(m, "sb", "yes")
        # Ipopt relaxes bounds internally (bound_relax_factor); project the final point back
        # so that returned inputs satisfy the actuator limits exactly.
        set_attribute(m, "honor_original_bounds", "yes")
        set_attribute(m, "max_iter", config.max_iter)
        set_attribute(m, "tol", config.tol)
        config.max_wall_time === nothing || set_attribute(m, "max_wall_time", config.max_wall_time)
        for (k, v) in config.optimizer_attributes
            set_attribute(m, k, v)
        end

        @variable(m, x0[1:4] in Parameter(0.0))
        @variable(m, u_prev[1:2] in Parameter(0.0))
        @variable(m, kap[1:N] in Parameter(kappa))
        @variable(m, Xf[1:4, 1:N])
        @variable(m, U[1:2, 1:N])
        X = hcat(x0, Xf)

        # Experimental progress-linearized curvature (Milestone 2B.3A). On a single-segment
        # track the average curvature is constant (A = B = 0), so the fixed-Parameter
        # formulation is used unchanged.
        cartesian = config.prediction_model === :cartesian_endpoint_exact
        linearized = config.curvature_model === :progress_linearized && kmin < kmax && !cartesian
        segpair = config.curvature_model === :segment_pair_exact && kmin < kmax && !cartesian
        sp_w = sp_dpsi = sp_ki = sp_kj = sp_kc = VariableRef[]
        cframe = Matrix{VariableRef}(undef, 0, 0)
        alphas = cartesian ? interior_alphas(config.interior_samples) : Float64[]
        obs_par, obs_cons = VariableRef[], ConstraintRef[]
        obs_alphas = obstacle === nothing ? Float64[] : [j / (config.obstacle_samples + 1) for j in 1:config.obstacle_samples]
        iframe = Matrix{VariableRef}(undef, 0, 0)
        icons = ConstraintRef[]
        if segpair
            # Experimental exact segment-pair curvature (Milestone 2B.3B). A single persistent
            # equality per stage; the Parameter w selects the bilinear crossing form (w = 1)
            # or the constant form kappa_eff = kc (w = 0) without rebuilding the model.
            @variable(m, lin_sbar[1:N+1] in Parameter(0.0))
            @variable(m, sp_w[1:N] in Parameter(0.0))
            @variable(m, sp_dpsi[1:N] in Parameter(0.0))
            @variable(m, sp_ki[1:N] in Parameter(0.0))
            @variable(m, sp_kj[1:N] in Parameter(0.0))
            @variable(m, sp_kc[1:N] in Parameter(kappa))
            @variable(m, kmin <= kappa_eff[1:N] <= kmax)          # hard physical range: Frenet bounds hold
            for k in 1:N
                @constraint(m, sp_w[k] * (kappa_eff[k] * (X[1, k+1] - X[1, k]) - sp_dpsi[k] -
                                          sp_kj[k] * (X[1, k+1] - lin_sbar[k+1]) + sp_ki[k] * (X[1, k] - lin_sbar[k])) +
                               (1 - sp_w[k]) * (kappa_eff[k] - sp_kc[k]) == 0)
            end
            lin_kbar = lin_A = lin_B = lin_tr = VariableRef[]
            kcur = kappa_eff
        elseif linearized
            @variable(m, lin_kbar[1:N] in Parameter(kappa))
            @variable(m, lin_A[1:N] in Parameter(0.0))
            @variable(m, lin_B[1:N] in Parameter(0.0))
            @variable(m, lin_sbar[1:N+1] in Parameter(0.0))
            @variable(m, kmin <= kappa_eff[1:N] <= kmax)          # hard physical range: Frenet bounds hold
            for k in 1:N
                @constraint(m, kappa_eff[k] == lin_kbar[k] + lin_A[k] * (X[1, k] - lin_sbar[k]) +
                                               lin_B[k] * (X[1, k+1] - lin_sbar[k+1]))
            end
            if isfinite(config.linearization_trust_region)
                @variable(m, lin_tr[1:N+1] in Parameter(1e6))
                for k in 2:N+1
                    @constraint(m, X[1, k] - lin_sbar[k] <= lin_tr[k])
                    @constraint(m, X[1, k] - lin_sbar[k] >= -lin_tr[k])
                end
            else
                lin_tr = VariableRef[]
            end
            kcur = kappa_eff
        else
            lin_kbar = lin_A = lin_B = lin_sbar = kappa_eff = lin_tr = VariableRef[]
            kcur = kap
        end

        for k in 1:N
            set_lower_bound(Xf[2, k], -ey_max); set_upper_bound(Xf[2, k], ey_max)
            set_lower_bound(Xf[3, k], -config.epsi_max); set_upper_bound(Xf[3, k], config.epsi_max)
            set_lower_bound(Xf[4, k], p.v_min); set_upper_bound(Xf[4, k], p.v_max)
            set_lower_bound(U[1, k], -p.delta_max); set_upper_bound(U[1, k], p.delta_max)
            set_lower_bound(U[2, k], p.a_min); set_upper_bound(U[2, k], p.a_max)
        end

        dyn = ConstraintRef[]
        rhs(ey, epsi, v, k) = kinematic_rhs(L, ey, epsi, v, U[1, k], U[2, k], kcur[k])
        if cartesian
            # Exact Cartesian endpoint dynamics (Milestone 2B.4). Row r of cframe holds, per
            # knot, the assigned segment's s0, x0, y0, psi0 and kappa.
            P = Array{VariableRef,3}(undef, 3, 0, N)
            @operator(m, op_sinc, 1, sinc_un, sinc_un_d1, sinc_un_d2)
            @variable(m, cf[1:5, 1:N+1] in Parameter(0.0))
            cframe = cf
            pose = k -> frenet_pose_local((s0=cf[1, k], x0=cf[2, k], y0=cf[3, k], psi0=cf[4, k], kappa=cf[5, k]),
                X[1, k], X[2, k], X[3, k], op_sinc)
            for k in 1:N
                pk, pn = pose(k), pose(k + 1)
                inc = exact_bicycle_increment(L, pk[3], X[4, k], U[1, k], U[2, k], dt, op_sinc)
                for i in 1:3
                    push!(dyn, @constraint(m, pn[i] - pk[i] - inc[i] == 0))
                end
                push!(dyn, @constraint(m, X[4, k+1] - X[4, k] - inc[4] == 0))
            end
            if obstacle !== nothing
                # Collision constraints (Milestone 3A): the ego reference point outside the
                # disk of radius D around the obstacle, at the knots 2…N+1 and, optionally,
                # at interior samples. The obstacle is three Parameters (ox, oy, D).
                @variable(m, op[1:3] in Parameter(0.0))
                obs_par = op
                set_parameter_value(op[1], obstacle.x)
                set_parameter_value(op[2], obstacle.y)
                set_parameter_value(op[3], required_separation(obstacle, config.ego_radius, config.obstacle_buffer))
                for k in 2:N+1
                    pk = pose(k)
                    push!(obs_cons, @constraint(m, obstacle_margin(pk[1], pk[2], op[1], op[2], op[3]) >= 0))
                end
                for k in 1:N, a in obs_alphas
                    px, py = interior_point(L, pose(k), X[4, k], U[1, k], U[2, k], a * dt, op_sinc)
                    push!(obs_cons, @constraint(m, obstacle_margin(px, py, op[1], op[2], op[3]) >= 0))
                end
            end
            if !isempty(alphas)
                # Between-knot corridor (Milestone 2B.5): two persistent inequalities per
                # interior sample; the Parameter row 1 selects the straight or curved form.
                for seg in track.segments
                    seg.curvature == 0 || ey_max < 1 / abs(seg.curvature) || throw(ArgumentError(
                        "interior corridor constraints need ey_max = $ey_max < R = $(1 / abs(seg.curvature))"))
                end
                @variable(m, ip[1:8, 1:N*length(alphas)] in Parameter(0.0))
                iframe = ip
                col = 0
                for k in 1:N
                    pk = pose(k)
                    for a in alphas
                        col += 1
                        px, py = interior_point(L, pk, X[4, k], U[1, k], U[2, k], a * dt, op_sinc)
                        lo, hi = corridor_margins(Tuple(ip[:, col]), px, py, ey_max)
                        push!(icons, @constraint(m, lo >= 0))
                        push!(icons, @constraint(m, hi >= 0))
                    end
                end
            end
        elseif config.integrator isa ForwardEuler
            P = Array{VariableRef,3}(undef, 3, 0, N)
            for k in 1:N
                f = rhs(X[2, k], X[3, k], X[4, k], k)
                for i in 1:4
                    push!(dyn, @constraint(m, X[i, k+1] == X[i, k] + dt * f[i]))
                end
            end
        else
            # Lifted RK4: the stage points are decision variables with Frenet-safe bounds, so
            # every evaluation of the dynamics happens inside the model domain.
            @variable(m, P[1:3, 1:3, 1:N])
            for k in 1:N, j in 1:3
                isfinite(stage_ey[1]) && set_lower_bound(P[1, j, k], stage_ey[1])
                isfinite(stage_ey[2]) && set_upper_bound(P[1, j, k], stage_ey[2])
                set_lower_bound(P[3, j, k], 0.0)
            end
            for k in 1:N
                xk = (X[2, k], X[3, k], X[4, k])
                f1 = rhs(xk..., k)
                for i in 1:3
                    push!(dyn, @constraint(m, P[i, 1, k] == xk[i] + dt / 2 * f1[i+1]))
                end
                f2 = rhs(P[1, 1, k], P[2, 1, k], P[3, 1, k], k)
                for i in 1:3
                    push!(dyn, @constraint(m, P[i, 2, k] == xk[i] + dt / 2 * f2[i+1]))
                end
                f3 = rhs(P[1, 2, k], P[2, 2, k], P[3, 2, k], k)
                for i in 1:3
                    push!(dyn, @constraint(m, P[i, 3, k] == xk[i] + dt * f3[i+1]))
                end
                f4 = rhs(P[1, 3, k], P[2, 3, k], P[3, 3, k], k)
                for i in 1:4
                    push!(dyn, @constraint(m, X[i, k+1] == X[i, k] + dt / 6 * (f1[i] + 2 * f2[i] + 2 * f3[i] + f4[i])))
                end
            end
        end

        if config.ay_max !== nothing
            # On interval k, delta and a are constant, so v(t) is linear and v(t)² is convex:
            # |v²·tan(delta)/L| is largest at an end point. Imposing the bound at both v_k and
            # v_{k+1} therefore enforces it for the whole interval (continuous time).
            for k in 1:N, v in (X[4, k], X[4, k+1])
                ay = v^2 * tan(U[1, k]) / L                 # smooth; |ay| ≤ ay_max as two inequalities
                @constraint(m, ay <= config.ay_max)
                @constraint(m, ay >= -config.ay_max)
            end
        end

        s_scale = p.v_max * N * dt
        a_ref = max(-p.a_min, p.a_max)
        J = -config.ws * (X[1, N+1] - X[1, 1]) / s_scale
        for k in 2:N+1
            J += config.qy * (X[2, k] / ey_max)^2 + config.qpsi * (X[3, k] / config.epsi_max)^2
        end
        for k in 1:N
            dprev, aprev = k == 1 ? (u_prev[1], u_prev[2]) : (U[1, k-1], U[2, k-1])
            J += config.rdelta * (U[1, k] / p.delta_max)^2 + config.ra * (U[2, k] / a_ref)^2 +
                 config.rd_delta * ((U[1, k] - dprev) / p.delta_max)^2 + config.rd_a * ((U[2, k] - aprev) / a_ref)^2
        end
        @objective(m, Min, J)
    end

    return NMPCController(vehicle, track, config, m, X, U, P, u_prev, kap, dyn, ey_max, stage_ey, (kmin, kmax),
        kappa_eff, lin_kbar, lin_A, lin_B, lin_sbar, lin_tr, sp_w, sp_dpsi, sp_ki, sp_kj, sp_kc, cframe, alphas, iframe, icons,
        obstacle, obs_par, obs_alphas, obs_cons, :none, NaN, nothing, nothing, t_build)
end

horizon(ctrl::NMPCController) = ctrl.config.horizon

"Prediction model: `:frenet` (RK4/Euler Frenet dynamics with a curvature model) or `:cartesian_endpoint_exact`."
prediction_model(ctrl::NMPCController) = isempty(ctrl.cframe) ? :frenet : :cartesian_endpoint_exact

"""
Curvature model in effect (`:constant` on single-segment tracks, where every Frenet mode
reduces to it; `:none` in the Cartesian-endpoint mode, which uses no curvature model).
"""
curvature_model(ctrl::NMPCController) = !isempty(ctrl.cframe) ? :none : isempty(ctrl.kappa_eff) ?
    (num_segments(ctrl.track) == 1 ? :constant : :stagewise_average) :
    (isempty(ctrl.sp_w) ? :progress_linearized : :segment_pair_exact)
"Curvature values currently used on the N intervals."
curvature_values(ctrl::NMPCController) = parameter_value.(ctrl.kappa)

"Forget the stored solution, so the next solve uses a cold start."
reset_warm_start!(ctrl::NMPCController) = (ctrl.last_X = nothing; ctrl.last_U = nothing; ctrl)

"""
    solve!(controller, measured_state; previous_input=(0.0, 0.0)) -> NMPCResult

One control cycle:

1. Check the measured state against the model domain with the **actual** track curvature
   at its position.
2. Build the initial guess and the stagewise curvature schedule (fixed point; see
   [`curvature_fixed_point`](@ref)). If no self-consistent schedule exists, return
   `:scheduling_failure` without solving. Then check the measured state against the first
   stage's curvature.
3. Write the measured state, previous input and curvature schedule into the JuMP
   Parameters, and set the primal start values.
4. Solve with Ipopt and validate the knots independently (dynamics residual, bounds).
5. Post-solve schedule consistency along the optimised progress.
6. Re-simulate the optimised inputs on the actual track (event-aware) and check every hard
   constraint.
7. Optionally (`schedule_corrections > 0`, within `correction_time_budget`), when the
   schedule is stale, re-solve once per correction with a schedule rebuilt (fixed point)
   from the previous solution's inputs and progress.

The first control is returned in `result.u` only when the result is accepted. A rejected
cycle clears the warm-start memory. Every step is included in `result.timing.total`.

**Cartesian-endpoint mode.**
- Step 2 builds the initial guess by an event-aware rollout and assigns every nominal knot
  its segment, without a curvature fixed point.
- Step 5 checks every optimised knot against its assigned segment, and step 6 adds the
  exact-map reconstruction ([`ExactMapReport`](@ref)).
- A knot outside its segment gives `:segment_assignment_invalid`. Each correction re-solves
  with the segments reassigned from the optimised progress, warm-started from that
  solution. There is no qualified acceptance.

**Strict corridor** (`strict_corridor = true`, any mode): a solution whose re-simulation
leaves the tightened planning corridor (`reference.min_corridor_distance <
-acceptance_tol`) is rejected with `:corridor_violation`. With `dense_check_samples > 0`,
the dense exact check (`corridor.min_corridor_clearance < -acceptance_tol`) is applied
as well.

**Interior samples** (Cartesian-endpoint mode, `interior_samples > 0`).
- Every interior sample's segment is assigned from the nominal trajectory and validated
  after the solve ([`interior_assignment_check`](@ref)).
- An invalid assignment gives `:interior_assignment_invalid`, and is corrected by the same
  bounded reassignment as the knots.
"""
function solve!(ctrl::NMPCController, measured_state::AbstractVector; previous_input=(0.0, 0.0))
    ctrl.obstacle === nothing || return solve_with_obstacle!(ctrl, measured_state; previous_input)
    return solve_once!(ctrl, measured_state; previous_input)
end

"""
    solve_once!(controller, x; previous_input, initial=nothing) -> NMPCResult

One control cycle as documented for [`solve!`](@ref). With `initial` (a schedule-like named
tuple with `Xg`, `Ug`, `s_nominal`, `kappas`; Cartesian-endpoint mode), that initial guess
replaces the warm/cold guess. Used by the obstacle-avoidance candidates.

`cycle_start` (a `time_ns()` value) is the start of the enclosing control cycle. With a
finite `cycle_budget`, it is used to skip optional reassignment and branch solves that
would not fit.

**Reassignment cycles** (Cartesian-endpoint mode, Milestone 3B.1):
- With `reassignment_cycle_detection = true`, every proposed reassignment is compared with
  all assignment vectors already solved in this call. An exact repeat is a cycle: it is
  recorded, and the ordinary reassignment stops.
- With `branch_resolution = true` in addition, a cycle in a single knot between two
  adjacent segments is resolved locally (see [`ReassignmentReport`](@ref) and
  `docs/segment_reassignment_reliability.md`). One NLP is solved per segment, with that
  knot temporarily bounded to the segment and the rest of the cycle's assignment kept.
  The valid candidate with the lower objective is accepted. The bound is always restored.
"""
function solve_once!(ctrl::NMPCController, measured_state::AbstractVector; previous_input=(0.0, 0.0), initial=nothing,
    cycle_start=nothing)
    t0 = time_ns()
    N, c = horizon(ctrl), ctrl.config
    x = SVector{4,Float64}(measured_state...)
    uprev = SVector{2,Float64}(previous_input...)
    t_sched = 0.0

    fail(status, msg, sched; t_update=0.0, t_warm=0.0) = begin
        reset_warm_start!(ctrl)
        NMPCResult(status, false, MOI.OPTIMIZE_NOT_CALLED, MOI.NO_SOLUTION, fill(NaN, 4, N + 1), fill(NaN, 2, N), nothing,
            NaN, -1, NMPCTiming(t_update, t_warm, 0.0, NaN, 0.0, (time_ns() - t0) / 1e9, t_sched, 0.0), NaN, NaN, false,
            msg, sched, REFERENCE_NOT_RUN, 0, EXACT_NOT_RUN, CORRIDOR_NOT_RUN, OBSTACLE_NONE, REASSIGNMENT_NONE)
    end

    cartesian = prediction_model(ctrl) === :cartesian_endpoint_exact
    rmodel = cartesian ? :cartesian_endpoint_exact : curvature_model(ctrl)
    problem = measurement_problem(ctrl, x, uprev)
    problem === nothing || return fail(:invalid_measurement, problem, empty_schedule_report(; model=rmodel))
    # Linearized mode: X[:,1] is a Parameter evaluated with kappa_eff[1], a variable anywhere in
    # [kappa_min, kappa_max]. Both extremes must be valid (affine in kappa), which does not
    # depend on the schedule, so this is checked before scheduling.
    for kchk in (isempty(ctrl.kappa_eff) ? () : ctrl.kappa_range)
        problem = measurement_problem(ctrl, x, uprev, kchk)
        problem === nothing || return fail(:invalid_measurement,
            problem * " (with the first-stage curvature $(kchk) that the NLP may use)", empty_schedule_report(; model=rmodel))
    end

    # Initial guess and curvature schedule.
    t1 = time_ns()
    warm = c.warm_start && ctrl.last_U !== nothing && initial === nothing
    sched = initial !== nothing ? initial : cartesian ? cartesian_initial_guess(ctrl, x, warm) : initial_schedule(ctrl, x, warm)
    t_sched += (time_ns() - t1) / 1e9
    if !sched.converged
        return fail(:scheduling_failure, sched.message, presolve_schedule_report(sched, false, sched.message, rmodel))
    end
    # Stagewise mode: X[:,1] is a Parameter evaluated with the fixed first-stage curvature.
    if isempty(ctrl.kappa_eff) && !cartesian
        problem = measurement_problem(ctrl, x, uprev, sched.kappas[1])
        problem === nothing || return fail(:invalid_measurement,
            problem * " (with the first-stage curvature $(sched.kappas[1]) that the NLP uses)",
            presolve_schedule_report(sched, true, "", rmodel))
    end

    t2 = time_ns()
    for i in 1:4
        set_parameter_value(ctrl.X[i, 1], x[i])
    end
    for i in 1:2
        set_parameter_value(ctrl.u_prev[i], uprev[i])
    end
    t3s = time_ns()
    sched = install_schedule!(ctrl, sched)
    t_sched += (time_ns() - t3s) / 1e9
    t3 = time_ns()
    apply_start_values!(ctrl, sched.Xg, sched.Ug, sched.kappas)
    t4 = time_ns()

    t_opt, t_ipopt, t_val, t_ref = 0.0, 0.0, 0.0, 0.0
    nlp = 0
    local core, sreport, rreport, status, msgs, ereport, dreport
    # Reassignment diagnostics (Milestone 3B.1).
    tc_start = cycle_start === nothing ? t0 : cycle_start
    trace, hist, branch, rmsgs = Any[], Any[], Any[], String[]
    ncycles, cyclen, cycknots, branch_sel = 0, 0, Int[], 0
    max_solve = 0.0
    branch_bound = 0                                     # segment id bounding the branch knot in the current solve (0: none)
    while true
        ta = time_ns()
        optimize!(ctrl.model)
        tb = time_ns()
        nlp += 1
        t_opt += (tb - ta) / 1e9
        max_solve = max(max_solve, (tb - ta) / 1e9)
        core = extract_core(ctrl, x, sched.kappas; frames=cartesian ? sched.asg.frames : nothing)
        isnan(core.solver_time) || (t_ipopt += core.solver_time)
        tc = time_ns()
        t_val += (tc - tb) / 1e9
        td = time_ns()
        pc = post_solve_checks(ctrl, x, sched, core)
        t_ref += (time_ns() - td) / 1e9
        sreport, rreport, ereport, dreport, status, msgs = pc.sreport, pc.rreport, pc.ereport, pc.dreport, pc.status, pc.msgs
        stale = pc.stale
        if cartesian
            push!(trace, reassignment_entry(ctrl, x, sched, core, status, nlp, isempty(trace) ? nothing : trace[end].ids, branch_bound))
            push!(hist, (; ids=copy(sched.asg.seg), X=core.X, U=core.U, s=core.X[1, :], status, objective=core.obj))
        end
        core.status === :success || break                # solver failure / invalid knots: no correction
        # Optional bounded correction: only when the schedule is stale. A reference violation
        # that comes with a stale schedule is also eligible, since the stale curvature is its
        # likely cause. Solver failures and invalid knots are never corrected.
        stale || break
        can_correct = nlp - 1 < c.schedule_corrections
        if can_correct
            elapsed = (time_ns() - t0) / 1e9
            if elapsed + (tb - ta) / 1e9 > c.correction_time_budget
                push!(msgs, "schedule correction skipped: time budget $(c.correction_time_budget) s would be exceeded")
                can_correct = false
            elseif !cycle_budget_allows(c, tc_start, max_solve)
                push!(msgs, "schedule correction skipped: cycle budget $(c.cycle_budget) s would be exceeded")
                can_correct = false
            end
        end
        if !can_correct
            # Qualified acceptance (opt-in) only after corrections are exhausted, and only with
            # a passed reference check on the actual track.
            if status === :schedule_inconsistent && c.stale_schedule_policy === :accept_if_reference_valid && c.reference_check
                status = :success_stale_schedule
            end
            break
        end
        tc0 = time_ns()
        if cartesian
            # Reassign the segments from the optimised progress; warm-start from that solution.
            corr = cartesian_reassignment(ctrl, sched, core)
            t_sched += (time_ns() - tc0) / 1e9
            if !corr.converged
                push!(msgs, "segment reassignment failed: " * corr.message)
                break
            end
            if c.reassignment_cycle_detection
                newids = cartesian_assignment(ctrl.track, corr.s_nominal; knot_tol=c.boundary_knot_tol).seg
                cyc = detect_assignment_cycle([h.ids for h in hist], newids)
                if cyc !== nothing
                    ncycles, cyclen, cycknots = 1, cyc.length, cyc.knots
                    m = "reassignment cycle of length $(cyc.length) detected at knot(s) $(cyc.knots)"
                    push!(rmsgs, m)
                    push!(msgs, m)
                    if c.branch_resolution
                        br = resolve_branch!(ctrl, x, sched, hist, cyc, tc_start, t0, max_solve)
                        append!(branch, br.attempts)
                        append!(trace, br.trace)
                        nlp += br.solves
                        t_opt += br.t_opt
                        isnan(br.t_ipopt) || (t_ipopt += br.t_ipopt)
                        push!(rmsgs, br.message)
                        push!(msgs, br.message)
                        if br.chosen !== nothing
                            core, sreport, rreport, ereport, dreport, status = br.chosen.core, br.chosen.sreport, br.chosen.rreport,
                            br.chosen.ereport, br.chosen.dreport, br.chosen.status
                            sched = br.chosen.sched
                            branch_sel = br.chosen.segment
                            msgs = filter(s -> !startswith(s, "invalid segment assignment"), msgs)
                        end
                    end
                    break
                end
            end
            sched = install_schedule!(ctrl, corr)
            apply_start_values!(ctrl, sched.Xg, sched.Ug, sched.kappas)
            continue
        end
        corr = curvature_fixed_point(ctrl, x, core.U, core.X[1, :])
        t_sched += (time_ns() - tc0) / 1e9
        if !corr.converged
            push!(msgs, "schedule correction did not converge: " * corr.message)
            if status === :schedule_inconsistent && c.stale_schedule_policy === :accept_if_reference_valid && c.reference_check
                status = :success_stale_schedule
            end
            break
        end
        sched = (; sched..., kappas=corr.kappas, Xg=corr.Xg, Ug=core.U, s_nominal=corr.s_nominal,
            iterations=sched.iterations + corr.iterations, change=corr.change, method=Symbol(sched.method, :_corrected))
        sched = install_schedule!(ctrl, sched)
        apply_start_values!(ctrl, sched.Xg, sched.Ug, sched.kappas)
    end

    accepted = status === :success || status === :success_stale_schedule
    if accepted
        ctrl.last_X, ctrl.last_U = core.X, core.U
    else
        reset_warm_start!(ctrl)
    end
    timing = NMPCTiming((t3 - t2) / 1e9, (t4 - t3) / 1e9, t_opt, nlp > 0 ? t_ipopt : NaN, t_val,
        (time_ns() - t0) / 1e9, t_sched, t_ref)
    return NMPCResult(status, accepted, core.ts, core.ps, core.X, core.U,
        accepted ? SVector{2}(core.U[1, 1], core.U[2, 1]) : nothing, core.obj, core.iterations, timing,
        core.dyn_res, core.viol, warm, join(msgs, "; "), sreport, rreport, nlp, ereport, dreport, OBSTACLE_NONE,
        cartesian ? ReassignmentReport(trace, ncycles, cyclen, cycknots, branch, branch_sel, join(rmsgs, "; ")) : REASSIGNMENT_NONE)
end

"""
    post_solve_checks(controller, x, sched, core) -> NamedTuple

The post-solve classification of one NLP solve (shared by the ordinary loop and the
branch-resolution candidates):
- the schedule or segment-assignment consistency;
- the interior-sample assignment;
- the event-aware reference check, the exact-map check, and the dense corridor check;
- the final status, by precedence:
  1. reference violation;
  2. invalid knot assignment (or stale schedule);
  3. invalid interior assignment;
  4. corridor violation (strict);
  5. success.

Returns `(; sreport, rreport, ereport, dreport, status, msgs, stale)`. `stale` marks a
solution that a reassignment (or schedule correction) could fix.
"""
function post_solve_checks(ctrl::NMPCController, x, sched, core)
    c = ctrl.config
    cartesian = !isempty(ctrl.cframe)
    status, msgs = core.status, copy(core.messages)
    sreport = cartesian ? cartesian_consistency(ctrl, sched, core, status === :success) :
              schedule_consistency(ctrl, sched, core, status === :success)
    rreport, ereport, dreport = REFERENCE_NOT_RUN, EXACT_NOT_RUN, CORRIDOR_NOT_RUN
    core.status === :success || return (; sreport, rreport, ereport, dreport, status, msgs, stale=false)
    iasg = cartesian && !isempty(ctrl.alphas) ? interior_assignment_check(ctrl, sched, core) : nothing
    c.reference_check && (rreport = reference_check(ctrl, x, core.X, core.U))
    cartesian && (ereport = exact_map_check(ctrl, x, core.X, core.U; frames=sched.asg.frames))
    if c.dense_check_samples > 0
        dreport = corridor_check(ctrl, x, core.X, core.U; samples=c.dense_check_samples, interior=iasg)
    elseif iasg !== nothing
        dreport = CorridorReport(0, NaN, NaN, NaN, 0, 0, NaN, 0, interior_fields(ctrl, iasg)..., NaN, "")
    end
    stale_knots = !sreport.consistent
    stale_interior = iasg !== nothing && iasg.failures > 0
    ref_ok = !c.reference_check || rreport.passed
    corridor_ok = !(c.strict_corridor && rreport.checked && !(rreport.min_corridor_distance >= -c.acceptance_tol)) &&
                  !(c.strict_corridor && dreport.dense_samples > 0 && !(dreport.min_corridor_clearance >= -c.acceptance_tol))
    # A hard-constraint failure dominates a stale schedule (or an invalid knot/interior
    # segment assignment), which dominates a corridor violation.
    status = !ref_ok ? :reference_violation :
             stale_knots ? (cartesian ? :segment_assignment_invalid : :schedule_inconsistent) :
             stale_interior ? :interior_assignment_invalid : !corridor_ok ? :corridor_violation : :success
    stale_knots && push!(msgs, (cartesian ? "invalid segment assignment: " : "stale curvature schedule: ") * sreport.message)
    stale_interior && push!(msgs, "invalid interior segment assignment: " * iasg.message)
    ref_ok || push!(msgs, "reference check on the actual track failed: " * rreport.message)
    corridor_ok || push!(msgs, "plan leaves the planning corridor by $(fmt(-minimum(filter(!isnan, [rreport.min_corridor_distance, dreport.min_corridor_clearance]); init=0.0)))" *
        " m (strict corridor; event-aware re-simulation $(fmt(rreport.min_corridor_distance)) m, dense check $(fmt(dreport.min_corridor_clearance)) m" *
        (dreport.dense_samples > 0 ? " at t = $(fmt(dreport.worst_time)) s, stage $(dreport.worst_stage))" : ")"))
    return (; sreport, rreport, ereport, dreport, status, msgs, stale=stale_knots || stale_interior)
end

"""
Check the measured state and previous input against the model domain, using the actual
track curvature at the measured progress (or `kappa`, when given). Returns `nothing` or a
message.
"""
function measurement_problem(ctrl, x, uprev, kappa=nothing)
    try
        k = kappa === nothing ? curvature(ctrl.track, x[1]) : kappa
        check_domain(ctrl.vehicle, x, uprev, k)
    catch err
        err isa DomainError || rethrow()
        return "measured state or previous input outside the model domain: " * sprint(showerror, err)
    end
    return nothing
end

"""
Roll the checked prediction model forward from `x`. If a step leaves the model domain, the
remaining columns are taken from `fallback` (when given) or hold the last valid state.
"""
function rollout_guess(ctrl, x, Ug, kappas; fallback=nothing)
    N, dt = horizon(ctrl), ctrl.config.dt
    Xg = Matrix{Float64}(undef, 4, N + 1)
    Xg[:, 1] = x
    valid = true
    for k in 1:N
        if valid
            try
                Xg[:, k+1] = discrete_dynamics(ctrl.vehicle, Xg[:, k], Ug[:, k], kappas[k], dt, ctrl.config.integrator)
            catch err
                err isa DomainError || rethrow()
                valid = false
            end
        end
        valid || (Xg[:, k+1] = fallback === nothing ? Xg[:, k] : fallback[:, k+1])
    end
    return Xg
end

"""
Cold start: steer with the curvature feed-forward `atan(kappa_k*L)` (clipped to the steering
limit), keep the acceleration at zero, and roll out the prediction model from the measured
state with the stagewise curvature `kappas` (default: the current Parameter values).
"""
function cold_start_guess(ctrl::NMPCController, x, kappas=curvature_values(ctrl))
    N, p = horizon(ctrl), ctrl.vehicle.params
    Ug = zeros(2, N)
    for k in 1:N
        Ug[1, k] = clamp(atan(kappas[k] * p.wheelbase), -p.delta_max, p.delta_max)
    end
    return rollout_guess(ctrl, x, Ug, kappas), Ug
end

"""
Warm start: shift the previous input sequence one step (repeating the last input), roll the
prediction model out from the new measured state with `kappas` (default: the current
Parameter values), and fall back to the shifted previous state trajectory (with the
measured first column) if the rollout leaves the model domain.
"""
function warm_start_guess(ctrl::NMPCController, x, kappas=curvature_values(ctrl))
    N = horizon(ctrl)
    Ug = hcat(ctrl.last_U[:, 2:N], ctrl.last_U[:, N])
    shifted = hcat(ctrl.last_X[:, 2:N+1], ctrl.last_X[:, N+1])
    return rollout_guess(ctrl, x, Ug, kappas; fallback=shifted), Ug
end

"Project an initial guess into a variable's bounds (start values only; never states)."
function bounded_start(v::VariableRef, value::Float64)
    isfinite(value) || (value = 0.0)
    has_lower_bound(v) && (value = max(value, lower_bound(v)))
    has_upper_bound(v) && (value = min(value, upper_bound(v)))
    return value
end

function apply_start_values!(ctrl::NMPCController, Xg, Ug, kappas)
    N, dt, L = horizon(ctrl), ctrl.config.dt, ctrl.vehicle.params.wheelbase
    for k in 2:N+1, i in 1:4
        set_start_value(ctrl.X[i, k], bounded_start(ctrl.X[i, k], Xg[i, k]))
    end
    for k in 1:N, i in 1:2
        set_start_value(ctrl.U[i, k], bounded_start(ctrl.U[i, k], Ug[i, k]))
    end
    for k in eachindex(ctrl.kappa_eff)
        set_start_value(ctrl.kappa_eff[k], bounded_start(ctrl.kappa_eff[k], Float64(kappas[k])))
    end
    isempty(ctrl.P) && return nothing
    for k in 1:N
        rhs = z -> kinematic_rhs(L, z[1], z[2], z[3], Ug[1, k], Ug[2, k], kappas[k])
        z1 = (Xg[2, k], Xg[3, k], Xg[4, k])
        z2 = z1 .+ dt / 2 .* rhs(z1)[2:4]
        z3 = z1 .+ dt / 2 .* rhs(z2)[2:4]
        z4 = z1 .+ dt .* rhs(z3)[2:4]
        for (j, z) in enumerate((z2, z3, z4)), i in 1:3
            set_start_value(ctrl.P[i, j, k], bounded_start(ctrl.P[i, j, k], all(isfinite, z) ? z[i] : z1[i]))
        end
    end
    return nothing
end

const ACCEPTED_TERMINATION = (MOI.LOCALLY_SOLVED, MOI.ALMOST_LOCALLY_SOLVED)

"""
Extract the Ipopt result and validate the knots independently (status, finiteness,
dynamics residual with the curvature schedule used by the NLP, bounds, actuator limits).
Returns a named tuple; `status` is `:success`, `:solver_failure` or `:invalid_solution`.
"""
function extract_core(ctrl::NMPCController, x, kappas; frames=nothing)
    m, N = ctrl.model, horizon(ctrl)
    ts, ps = termination_status(m), primal_status(m)
    iters = try
        Int(MOI.get(m, MOI.BarrierIterations()))
    catch
        -1
    end
    solver_time = try
        solve_time(m)
    catch
        NaN
    end
    if has_values(m)
        X, U = value.(ctrl.X), value.(ctrl.U)
        obj = objective_value(m)
        # The curvature the NLP actually used on each stage.
        kappas = isempty(ctrl.kappa_eff) ? collect(Float64, kappas) : value.(ctrl.kappa_eff)
    else
        X, U, obj = fill(NaN, 4, N + 1), fill(NaN, 2, N), NaN
        kappas = collect(Float64, kappas)
    end
    status, msg = :success, String[]
    dyn_res, viol = NaN, NaN
    if !(ts in ACCEPTED_TERMINATION && ps == MOI.FEASIBLE_POINT)
        status = :solver_failure
        push!(msg, "solver returned termination status $ts with primal status $ps")
    elseif !(all(isfinite, X) && all(isfinite, U))
        status = :invalid_solution
        push!(msg, "solution contains non-finite values")
    else
        dyn_res, viol, problems = validate_prediction(ctrl, X, U, x, kappas; frames)
        if !isempty(problems) || dyn_res > ctrl.config.acceptance_tol || viol > ctrl.config.acceptance_tol
            status = :invalid_solution
            append!(msg, problems)
            dyn_res > ctrl.config.acceptance_tol && push!(msg, "dynamics residual $dyn_res exceeds acceptance_tol")
            viol > ctrl.config.acceptance_tol && push!(msg, "constraint violation $viol exceeds acceptance_tol")
        elseif !within_input_limits(ctrl.vehicle.params, U[:, 1])
            status = :invalid_solution
            push!(msg, "first input $(U[:, 1]) outside actuator limits")
        end
        ts == MOI.ALMOST_LOCALLY_SOLVED && push!(msg, "accepted after independent validation although Ipopt reported ALMOST_LOCALLY_SOLVED")
    end
    return (; status, messages=msg, ts, ps, X, U, obj, iterations=iters, solver_time, dyn_res, viol, kappas_used=kappas)
end

"""
    validate_prediction(ctrl, X, U, x_measured, kappas; frames=nothing) -> (dynamics_residual, max_violation, problems)

Check a predicted trajectory independently of the solver: recompute every step with the
checked `discrete_dynamics`, and measure the violation of all bounds and the
lateral-acceleration inequality at the knots.

With `frames` (Cartesian-endpoint mode: the assigned segment frame of every knot), the
residual is that of the NLP's own equalities, recomputed in `Float64`:
`|pose(X_{k+1}) - pose(X_k) - exact increment|` (position, heading) and the speed
equation, with the segment-local poses.
"""
function validate_prediction(ctrl::NMPCController, X, U, x, kappas; frames=nothing)
    c, p, N = ctrl.config, ctrl.vehicle.params, horizon(ctrl)
    problems = String[]
    X[:, 1] == x || push!(problems, "X[:,1] differs from the measured state")
    dyn = 0.0
    for k in 1:N
        if frames !== nothing
            dyn = max(dyn, cartesian_stage_residual(ctrl, X, U, frames, k))
            continue
        end
        try
            xn = discrete_dynamics(ctrl.vehicle, X[:, k], U[:, k], kappas[k], c.dt, c.integrator)
            dyn = max(dyn, maximum(abs, xn - X[:, k+1]))
        catch err
            err isa DomainError || rethrow()
            push!(problems, "step $k leaves the model domain: " * sprint(showerror, err))
            dyn = Inf
        end
    end
    excess(v, lo, hi) = max(lo - v, v - hi, 0.0)
    viol = 0.0
    for k in 2:N+1
        viol = max(viol, excess(X[2, k], -ctrl.ey_max, ctrl.ey_max), excess(X[3, k], -c.epsi_max, c.epsi_max),
            excess(X[4, k], p.v_min, p.v_max))
    end
    for k in 1:N
        viol = max(viol, excess(U[1, k], -p.delta_max, p.delta_max), excess(U[2, k], p.a_min, p.a_max))
        if c.ay_max !== nothing
            for v in (X[4, k], X[4, k+1])
                viol = max(viol, abs(lateral_acceleration(ctrl.vehicle, v, U[1, k])) - c.ay_max)
            end
        end
    end
    return dyn, viol, problems
end

"""
    between_knot_diagnostics(ctrl, X, U; substeps=20)

Independent finer-step check of a prediction. Starting from `X[:, 1]`, apply `U`
piecewise constant and integrate with RK4 at step `dt/substeps`. Every fine sample is
checked against the operating constraints (actual track width, no margin) and the
lateral-acceleration limit, allowing a numerical tolerance `tol` (default
`config.acceptance_tol`). The prediction's own integration error is measured at the
knots.

Returns a named tuple `(max_knot_error, max_abs_ey, max_abs_ay, violations, status)`:
- `max_knot_error`: maximum absolute difference between the fine solution and `X` at the
  knots, over all four states;
- `violations`: `t => ConstraintViolation` pairs at fine samples, with
  `:lateral_acceleration` added for the `ay_max` limit;
- `status`: `:success`, or `:domain_error` if the fine solution leaves the model domain.
"""
function between_knot_diagnostics(ctrl::NMPCController, X::AbstractMatrix, U::AbstractMatrix; substeps::Integer=20,
    tol::Real=ctrl.config.acceptance_tol)
    substeps >= 1 || throw(ArgumentError("substeps must be ≥ 1"))
    c, N = ctrl.config, horizon(ctrl)
    kappas = curvature_values(ctrl)
    h = c.dt / substeps
    x = SVector{4,Float64}(X[:, 1])
    knot_err, max_ey, max_ay = 0.0, abs(x[2]), 0.0
    violations = Pair{Float64,ConstraintViolation}[]
    for k in 1:N
        u = SVector{2,Float64}(U[:, k])
        for j in 1:substeps
            ay = lateral_acceleration(ctrl.vehicle, x[4], u[1])
            max_ay = max(max_ay, abs(ay))
            t = ((k - 1) * substeps + j - 1) * h
            for v in check_operating_constraints(ctrl.vehicle, ctrl.track, x, u; tol)
                push!(violations, t => v)
            end
            if c.ay_max !== nothing && abs(ay) > c.ay_max + tol
                push!(violations, t => ConstraintViolation(:lateral_acceleration, ay, copysign(c.ay_max, ay),
                    "lateral acceleration $(fmt(ay)) m/s² exceeds ay_max = $(fmt(c.ay_max)) m/s²"))
            end
            try
                x = discrete_dynamics(ctrl.vehicle, x, u, kappas[k], h, RK4())
            catch err
                err isa DomainError || rethrow()
                return (max_knot_error=Inf, max_abs_ey=max_ey, max_abs_ay=max_ay, violations=violations, status=:domain_error)
            end
            max_ey = max(max_ey, abs(x[2]))
        end
        knot_err = max(knot_err, maximum(abs, x - X[:, k+1]))
    end
    for v in check_operating_constraints(ctrl.vehicle, ctrl.track, x; tol)
        push!(violations, N * c.dt => v)
    end
    return (max_knot_error=knot_err, max_abs_ey=max_ey, max_abs_ay=max_ay, violations=violations, status=:success)
end

between_knot_diagnostics(ctrl::NMPCController, r::NMPCResult; kwargs...) = between_knot_diagnostics(ctrl, r.X, r.U; kwargs...)
