# Linearized (successive-linearization) vehicle MPC with JuMP and OSQP (Milestone 4A).
# The straight track model is here (see docs/linearized_mpc.md); the local track model for
# curved tracks and a stationary obstacle is in linear_mpc_tracks.jl (Milestone 4BC).

"""
    LinearMPCConfig(; kwargs...)

Configuration of [`LinearMPCController`](@ref). The objective weights and normalisation,
the margin, the heading and lateral-acceleration limits, and the vehicle limits are the
same as the nonlinear MPC baseline ([`NMPCConfig`](@ref) defaults).

**Discretisation and objective**
- `horizon = 10`, `dt = 0.1`;
- `ws`, `qy`, `qpsi`, `rdelta`, `ra`, `rd_delta`, `rd_a`: as in `NMPCConfig`;
- `track_margin = 0.05`, `epsi_max = deg2rad(60)`, `ay_max = 4.0` (`nothing` disables it).

**Trust region and successive linearisation**
- `trust_region_z = (0.5, 0.5, 0.35, 1.5)`: componentwise radius around the nominal states
  `(px [m], py [m], psi [rad], v [m/s])`;
- `trust_region_u = (0.15, 3.0)`: radius around the nominal inputs `(delta [rad],
  a [m/s²])`;
- `trust_region_shrink = 0.5`: largest factor applied to the trust region after a
  candidate is rejected for its mismatch `m > tol` (normalised by `mismatch_tol`). The
  linearisation error is quadratic in the step, so the factor is
  `clamp(0.9 sqrt(tol/m), min_trust_scale_factor, trust_region_shrink)`;
- `min_trust_scale_factor = 0.1`: smallest factor of one such reduction;
- `max_qp_solves = 5`: maximum number of QP solves per control cycle;
- `sqp_time_budget = 0.08`: a further QP is started only if the time used plus the last
  QP iteration's duration fits [s].

**Acceptance (nonlinear validation)**
- `mismatch_tol = (1e-3, 1e-3, 1e-3)`: maximum difference between the QP prediction and
  the exact nonlinear rollout of its inputs, `(position [m], heading [rad], speed [m/s])`;
- `validation_tol = 1e-4`: tolerance for the nonlinear constraint checks (planning
  corridor at the knots, physical track along the rollout, speed, inputs, heading and
  lateral acceleration);
- `validation_substeps = 10`: samples per interval for the physical-track check;
- `acceptance_tol = 1e-6`: tolerance used by the closed-loop plant-violation recorder (as
  in `NMPCConfig`).

**Solver**
- `warm_start = true`: the nominal trajectory of a cycle is the rollout of the previous
  accepted inputs shifted by one stage ([`shifted_inputs`](@ref)); otherwise (and after a
  rejected cycle) it is the cold-start rollout ([`nominal_inputs`](@ref));
- `warm_tail = :pursuit`: the last input of the shifted sequence, [`pursuit_input`](@ref)
  at the end of the shifted rollout, or `:repeat` (the previous last input);
- `primal_start = :nominal`: the OSQP primal start of every QP is its nominal trajectory
  (dynamically consistent), or `:none` (OSQP's own previous iterate);
- `update_mode = :persistent`: update the QP coefficients in place, or `:rebuild` (build a
  new JuMP model for every QP, for comparison);
- `eps_abs = 1e-6`, `eps_rel = 1e-6`, `osqp_max_iter = 20000`, `polish = true`: OSQP
  settings (tighter than OSQP's defaults of 1e-3).
- `osqp_settings = []`: further OSQP settings (`name => value`), applied when the QP is
  built (Milestone 4BC).

**Track model** (Milestone 4BC; see `docs/linearized_multicurvature_collision_avoidance.md`)
- `track_model = :auto`: `:straight` (the Milestone 4A QP, unchanged) on a single straight
  segment without an obstacle, otherwise `:local`. The local model uses affine track
  coordinates at the nominal references, on any track. `:straight` and `:local` can be
  forced; `:straight` requires a straight track and no obstacle;
- `min_frenet_factor = 0.2`: a nominal reference with `1 - kappa·ey` below this is rejected
  (`:invalid_reference`) instead of being linearised;
- `max_projection_jump = 1.0`: largest progress distance [m] between a projection and its
  hint (see [`track_reference`](@ref)).

**Stationary obstacle** (`LinearMPCController(...; obstacle)`; same meaning as in
`NMPCConfig`)
- `ego_radius = 0.0`, `obstacle_buffer = 0.0`: `D = ego_radius + radius + buffer`;
- `obstacle_samples = 3`: interior halfspace samples per interval, at
  `tau_j = j/(M+1)·dt` (`M = 0` imposes the knots only);
- `obstacle_check_samples = 50`: samples per interval of the dense collision validation;
- `obstacle_acceptance = :collision`: reject contact (`:separation` also rejects buffer
  violations);
- `obstacle_ramp = 1.5`, `obstacle_ramp_min = 0.3`: passing-guess ramps ([`side_guess`](@ref));
- `obstacle_min_direction = 1e-3`: a nominal point closer than this to the obstacle centre
  [m] uses the passing-side track normal as its halfspace direction;
- `encounter_strategy = :sequential`: passing sides tried in ranked order, or `:both`
  (both solved, the lower objective kept).

**Rejected steps** (`step_policy`)
- `:rollout`: always re-linearise at the rejected candidate's exact rollout (Milestone 4A);
- `:trust_region`: when the candidate's mismatch exceeds the tolerance *and* its rollout
  violates a constraint, the next linearisation point is the one with the smaller relative
  constraint excess (`excess` of [`validate_plan_on_track`](@ref)): the current nominal
  (the step is retracted and only the trust region shrinks) or the candidate's rollout.
  Otherwise re-linearise at the rollout. A rejection for violations alone (mismatch within
  tolerance) also multiplies the trust region by `trust_region_shrink`. The second-order
  error of the affine constraints (for example the lateral acceleration) then decreases;
  without the shrink it would persist at the same step size;
- `:auto` (default): `:rollout` for the straight track model (unchanged from 4A), and
  `:trust_region` for the local model;
- `slack_penalty = :quadratic`, `slack_weight = 1e4`: penalty `slack_weight·Σ slack²` on the
  slacks of the local model's approximated inequalities (see [`build_local_qp!`](@ref);
  `:l1` gives `slack_weight·Σ slack`, `:none` hard constraints);
- `trust_region_growth = 2.0`: with `:trust_region`, each cycle starts from the last
  accepted trust-region scale times this factor (at most 1), unless the cycle's nominal
  trajectory violates a constraint by more than `trust_memory_excess` (relative excess,
  see [`validate_plan_on_track`](@ref); default 0.02), in which case it starts from 1. A
  rejected cycle resets the scale to 1;
- `local_backoff = (corridor=0.005, heading=0.01, ay=0.05, obstacle=0.005, speed=1e-3)`:
  tightening of the local model's QP constraints [m, rad, m/s², m, m/s on `v_max`], in
  addition to the mismatch back-off.
  The nonlinear validation checks the untightened limits. The margin absorbs the
  second-order error of the affine constraints on short steps, the inexact quadratic slack
  penalty and OSQP's tolerance, so an active constraint does not fail validation by a
  fraction of a millimetre. Plans are correspondingly slightly conservative;
- `local_eps = 1e-5`, `local_osqp_max_iter = 1500`: OSQP tolerance and iteration limit of
  the local model. With `:trust_region`, an inexact OSQP result (`ALMOST_OPTIMAL`, the
  iteration limit) with finite values is still evaluated as a candidate: acceptance is
  decided by the nonlinear validation and the mismatch, never by the solver status.
"""
Base.@kwdef struct LinearMPCConfig
    horizon::Int = 10
    dt::Float64 = 0.1
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
    trust_region_z::NTuple{4,Float64} = (0.5, 0.5, 0.35, 1.5)
    trust_region_u::NTuple{2,Float64} = (0.15, 3.0)
    trust_region_shrink::Float64 = 0.5
    min_trust_scale_factor::Float64 = 0.1
    max_qp_solves::Int = 5
    sqp_time_budget::Float64 = 0.08
    mismatch_tol::NTuple{3,Float64} = (1e-3, 1e-3, 1e-3)
    validation_tol::Float64 = 1e-4
    validation_substeps::Int = 10
    acceptance_tol::Float64 = 1e-6
    warm_start::Bool = true
    warm_tail::Symbol = :pursuit
    primal_start::Symbol = :nominal
    update_mode::Symbol = :persistent
    eps_abs::Float64 = 1e-6
    eps_rel::Float64 = 1e-6
    osqp_max_iter::Int = 20000
    polish::Bool = true
    track_model::Symbol = :auto
    min_frenet_factor::Float64 = 0.2
    max_projection_jump::Float64 = 1.0
    ego_radius::Float64 = 0.0
    obstacle_buffer::Float64 = 0.0
    obstacle_samples::Int = 3
    obstacle_check_samples::Int = 50
    obstacle_acceptance::Symbol = :collision
    obstacle_ramp::Float64 = 1.5
    obstacle_ramp_min::Float64 = 0.3
    obstacle_min_direction::Float64 = 1e-3
    encounter_strategy::Symbol = :sequential
    step_policy::Symbol = :auto
    slack_weight::Float64 = 1e4
    slack_penalty::Symbol = :quadratic
    osqp_settings::Vector{Pair{String,Any}} = Pair{String,Any}[]
    local_eps::Float64 = 1e-5
    local_osqp_max_iter::Int = 1500
    trust_region_growth::Float64 = 2.0
    trust_memory_excess::Float64 = 0.02
    local_backoff::NamedTuple{(:corridor, :heading, :ay, :obstacle, :speed),NTuple{5,Float64}} = (corridor=0.005, heading=0.01, ay=0.05, obstacle=0.005, speed=1e-3)
end

function validate_config(c::LinearMPCConfig)
    c.horizon >= 1 || throw(ArgumentError("horizon must be ≥ 1"))
    check_timestep(c.dt)
    0 < c.epsi_max < π / 2 || throw(ArgumentError("epsi_max must lie in (0, π/2)"))
    all(>(0), c.trust_region_z) && all(>(0), c.trust_region_u) || throw(ArgumentError("trust-region radii must be positive"))
    0 < c.trust_region_shrink < 1 || throw(ArgumentError("trust_region_shrink must lie in (0, 1)"))
    0 < c.min_trust_scale_factor <= c.trust_region_shrink ||
        throw(ArgumentError("min_trust_scale_factor must lie in (0, trust_region_shrink]"))
    c.max_qp_solves >= 1 || throw(ArgumentError("max_qp_solves must be ≥ 1"))
    all(>(0), c.mismatch_tol) && c.validation_tol > 0 || throw(ArgumentError("tolerances must be positive"))
    c.update_mode in (:persistent, :rebuild) || throw(ArgumentError("update_mode must be :persistent or :rebuild"))
    c.warm_tail in (:pursuit, :repeat) || throw(ArgumentError("warm_tail must be :pursuit or :repeat"))
    c.primal_start in (:nominal, :none) || throw(ArgumentError("primal_start must be :nominal or :none"))
    c.track_model in (:auto, :straight, :local) || throw(ArgumentError("track_model must be :auto, :straight or :local"))
    0 < c.min_frenet_factor < 1 || throw(ArgumentError("min_frenet_factor must lie in (0, 1)"))
    c.max_projection_jump > 0 || throw(ArgumentError("max_projection_jump must be positive"))
    c.ego_radius >= 0 && c.obstacle_buffer >= 0 || throw(ArgumentError("ego_radius and obstacle_buffer must be nonnegative"))
    c.obstacle_samples >= 0 && c.obstacle_check_samples >= 1 || throw(ArgumentError("obstacle_samples ≥ 0 and obstacle_check_samples ≥ 1 required"))
    c.obstacle_acceptance in (:collision, :separation) || throw(ArgumentError("obstacle_acceptance must be :collision or :separation"))
    c.encounter_strategy in (:sequential, :both) || throw(ArgumentError("encounter_strategy must be :sequential or :both"))
    c.slack_weight > 0 || throw(ArgumentError("slack_weight must be positive"))
    all(>=(0), values(c.local_backoff)) || throw(ArgumentError("local_backoff entries must be nonnegative"))
    c.slack_penalty in (:l1, :quadratic, :none) || throw(ArgumentError("slack_penalty must be :l1, :quadratic or :none"))
    c.local_eps > 0 && c.local_osqp_max_iter >= 1 && c.trust_region_growth >= 1 ||
        throw(ArgumentError("local_eps > 0, local_osqp_max_iter ≥ 1 and trust_region_growth ≥ 1 required"))
    c.step_policy in (:auto, :rollout, :trust_region) || throw(ArgumentError("step_policy must be :auto, :rollout or :trust_region"))
    c.obstacle_min_direction > 0 && c.obstacle_ramp > 0 && c.obstacle_ramp_min > 0 || throw(ArgumentError("obstacle distances must be positive"))
    return c
end

"""
Result of one [`solve!`](@ref) of a [`LinearMPCController`](@ref).

- `status`: `:success`, `:invalid_measurement`, `:qp_failure` (OSQP did not return an
  optimal point), `:not_converged` (no candidate passed the nonlinear validation within
  `max_qp_solves` or the time budget), and in the local track model `:invalid_reference`
  (no valid, unambiguous track reference of the nominal trajectory) or
  `:initial_collision` (the measured disk violates the required separation);
- `accepted`, `u`: whether the first control may be applied, and that control;
- `Z`, `U`: the accepted (or last) QP prediction, Cartesian `[px, py, psi, v]` (4×(N+1))
  and inputs (2×N);
- `Zbar`, `Ubar`: the nominal trajectory of the last linearisation;
- `Zroll`: the exact nonlinear rollout of `U` from the measured state;
- `mismatch`: `(position [m], heading [rad], speed [m/s])` between `Z` and `Zroll`;
- `violations`: the failed nonlinear checks of the last candidate (strings);
- `max_abs_ey`, `max_abs_ay`: extremes of the nonlinear rollout;
- `qp_status`, `qp_iterations`, `primal_residual`, `dual_residual`: per QP solve (all
  attempts of the cycle);
- `trust_scale`: the trust-region scale factor used by each QP;
- `mismatch_history`: the mismatch of each QP candidate;
- `qp_solves`: number of QP solves in this cycle;
- `objective`: the QP objective (including constants) of the last QP;
- `nonlinear_objective`: the original objective evaluated on the nonlinear rollout (true
  track coordinates);
- `timing`: `(; total, nominal, linearize, update, solve, validate, reference, guesses)`
  [s] for the whole cycle (`reference`: track projections of the nominal trajectories and
  the track-linearisation diagnostics; `guesses`: passing-side guesses);
- `message`;
- `track_model`: `:straight` or `:local`;
- `track_error`: `(progress, lateral, heading)` difference between the QP's affine track
  coordinates and the true coordinates of its prediction (local model; NaN otherwise);
- `references`: 4×(N+1) nominal `(s̄, ēy, ēψ, κ)` of the last linearisation (local
  model; NaN otherwise);
- `rollout_frenet`: 3×(N+1) true `(s, ey, epsi)` of the exact rollout;
- `track_clearance`: smallest physical track clearance `width/2 - |ey|` along the rollout;
- `obstacle`: obstacle diagnostics `(; present, relevant, side, attempts, guesses,
  required, collision_distance, min_distance, physical_clearance, buffer_clearance,
  constrained_clearance, collisions, separation_violations, degenerate_halfspaces,
  failure_cause, association)`. `attempts` holds `(label, status, QP solves, seconds)`
  per attempt; `guesses` holds `(side, clearance above D, track clearance)`.
"""
struct LinearMPCResult
    status::Symbol
    accepted::Bool
    u::Union{Nothing,SVector{2,Float64}}
    Z::Matrix{Float64}
    U::Matrix{Float64}
    Zbar::Matrix{Float64}
    Ubar::Matrix{Float64}
    Zroll::Matrix{Float64}
    mismatch::NTuple{3,Float64}
    violations::Vector{String}
    max_abs_ey::Float64
    max_abs_ay::Float64
    qp_status::Vector{Symbol}
    qp_iterations::Vector{Int}
    primal_residual::Vector{Float64}
    dual_residual::Vector{Float64}
    trust_scale::Vector{Float64}
    mismatch_history::Vector{NTuple{3,Float64}}
    qp_solves::Int
    objective::Float64
    nonlinear_objective::Float64
    timing::NamedTuple{(:total, :nominal, :linearize, :update, :solve, :validate, :reference, :guesses),NTuple{8,Float64}}
    message::String
    track_model::Symbol
    track_error::NTuple{3,Float64}
    references::Matrix{Float64}
    rollout_frenet::Matrix{Float64}
    track_clearance::Float64
    obstacle::NamedTuple
end

"""
    LinearMPCController(vehicle, track, config=LinearMPCConfig(); optimizer=OSQP.Optimizer, obstacle=nothing)

Successive-linearisation MPC as a convex QP solved by OSQP.
- On a straight track without an obstacle, it uses the straight track model of
  Milestone 4A (see `docs/linearized_mpc.md`).
- On any other track, or with one stationary `CircleObstacle`, it uses the local track
  model with obstacle halfspaces (Milestone 4BC; see
  `docs/linearized_multicurvature_collision_avoidance.md` and `config.track_model`).

**Model.**
- State `z = [px, py, psi, v]` (rear axle, Cartesian) and input `u = [delta, a]`.
- Dynamics: the exact zero-order-hold bicycle map `F` ([`exact_cartesian_step`](@ref)),
  linearised at a nominal trajectory: `z[k+1] = A_k z[k] + B_k u[k] + c_k`.

**Track frame.** The track frame (origin, tangent `t`, left normal `n`, heading `psi0`)
gives `s = t·(p - o)`, `ey = n·(p - o)` and `epsi = psi - psi0`, all affine in `z`.

**Persistence.** The JuMP model is built once. Per QP, the dynamics and
lateral-acceleration coefficients, their right-hand sides, the measured state, the
previous input, and the trust-region bounds are updated in place.
"""
mutable struct LinearMPCController
    vehicle::KinematicBicycle{Float64}
    track::Track{Float64}
    config::LinearMPCConfig
    optimizer::Any
    model::JuMP.Model
    Z::Matrix{VariableRef}
    U::Matrix{VariableRef}
    uprev::Vector{VariableRef}
    dyn::Matrix{ConstraintRef}
    ay_hi::Matrix{ConstraintRef}            # N×2: endpoint v_k (1) and v_{k+1} (2), upper side
    ay_lo::Matrix{ConstraintRef}
    frame::NamedTuple
    ey_max::Float64
    last_U::Union{Nothing,Matrix{Float64}}
    last_Z::Union{Nothing,Matrix{Float64}}
    build_time::Float64
    track_model::Symbol
    obstacle::Union{Nothing,CircleObstacle}
    obs_alphas::Vector{Float64}
    S::Vector{VariableRef}                  # local model: affine track coordinates of knots 2…N+1
    EY::Vector{VariableRef}
    EP::Vector{VariableRef}
    s0var::Union{Nothing,VariableRef}
    slack::Vector{VariableRef}
    con_s::Vector{ConstraintRef}
    con_ey::Vector{ConstraintRef}
    con_ep::Vector{ConstraintRef}
    obs::Matrix{ConstraintRef}              # (1 + M)×N obstacle halfspaces: knot k+1, then interior samples of interval k
    lin_cache::Any
    obstacle_side::Symbol
    obstacle_encounter::Float64
    trust_memory::Float64
end

horizon(ctrl::LinearMPCController) = ctrl.config.horizon
reset_warm_start!(ctrl::LinearMPCController) = (ctrl.last_U = nothing; ctrl.last_Z = nothing; ctrl.trust_memory = 1.0; ctrl)

"Straight-track frame `(; o, t, n, psi0, s0)` of a single straight segment."
function straight_frame(track::Track)
    (num_segments(track) == 1 && track.segments[1].curvature == 0) ||
        throw(ArgumentError("LinearMPCController supports straight (single straight-segment) tracks only"))
    x0, y0, psi0 = track.poses[1]
    return (; o=(x0, y0), t=(cos(psi0), sin(psi0)), n=(-sin(psi0), cos(psi0)), psi0, s0=0.0)
end

function LinearMPCController(vehicle::KinematicBicycle, track::Track, config::LinearMPCConfig=LinearMPCConfig();
    optimizer=OSQP.Optimizer, obstacle::Union{Nothing,CircleObstacle}=nothing)
    validate_config(config)
    straight = num_segments(track) == 1 && track.segments[1].curvature == 0
    tm = config.track_model === :auto ? (straight && obstacle === nothing ? :straight : :local) : config.track_model
    tm === :straight && obstacle !== nothing &&
        throw(ArgumentError("the straight track model has no obstacle constraints; use track_model = :auto or :local"))
    frame = tm === :straight ? straight_frame(track) : (;)
    ey_max = track.width / 2 - config.track_margin
    ey_max > config.mismatch_tol[1] || throw(ArgumentError("track.width/2 - track_margin must exceed the position mismatch tolerance"))
    obstacle === nothing || required_separation(obstacle, config.ego_radius, config.obstacle_buffer)
    M = config.obstacle_samples
    ctrl = LinearMPCController(vehicle, track, config, optimizer, Model(), Matrix{VariableRef}(undef, 0, 0),
        Matrix{VariableRef}(undef, 0, 0), VariableRef[], Matrix{ConstraintRef}(undef, 0, 0), Matrix{ConstraintRef}(undef, 0, 0),
        Matrix{ConstraintRef}(undef, 0, 0), frame, ey_max, nothing, nothing, 0.0, tm, obstacle, [j / (M + 1) for j in 1:M],
        VariableRef[], VariableRef[], VariableRef[], nothing, VariableRef[], ConstraintRef[], ConstraintRef[], ConstraintRef[],
        Matrix{ConstraintRef}(undef, 0, 0), nothing, :none, NaN, 1.0)
    ctrl.build_time = @elapsed (tm === :straight ? build_qp!(ctrl) : build_local_qp!(ctrl))
    return ctrl
end

"""
Build the QP: variables, linear constraints with placeholder dynamics and
lateral-acceleration coefficients, and the convex quadratic objective.
"""
function build_qp!(ctrl::LinearMPCController)
    c, p, N, fr, E = ctrl.config, ctrl.vehicle.params, ctrl.config.horizon, ctrl.frame, ctrl.ey_max
    m = Model(ctrl.optimizer)
    set_silent(m)
    if ctrl.optimizer === OSQP.Optimizer
        set_attribute(m, "eps_abs", c.eps_abs)
        set_attribute(m, "eps_rel", c.eps_rel)
        set_attribute(m, "max_iter", c.osqp_max_iter)
        set_attribute(m, "polish", c.polish)
        for (k, v) in c.osqp_settings
            set_attribute(m, k, v)
        end
    end
    # Placeholder bounds: the trust-region bounds are later modified in place (adding a bound
    # after setup would make JuMP copy the whole model into OSQP again).
    @variable(m, -1e3 <= Z[1:4, 1:N+1] <= 1e3)
    @variable(m, -1e3 <= U[1:2, 1:N] <= 1e3)
    @variable(m, uprev[1:2])
    for i in 1:4
        fix(Z[i, 1], 0.0; force=true)
    end
    fix.(uprev, 0.0)
    # Placeholder dynamics: every A/B entry present, so that later coefficient updates
    # never change the sparsity pattern.
    dyn = Matrix{ConstraintRef}(undef, 4, N)
    for k in 1:N, i in 1:4
        dyn[i, k] = @constraint(m, Z[i, k+1] - sum(Z[j, k] for j in 1:4) - sum(U[j, k] for j in 1:2) == 0)
    end
    # Planning corridor at the knots (fixed coefficients): -E ≤ n·(p - o) ≤ E.
    for k in 2:N+1
        e = fr.n[1] * (Z[1, k] - fr.o[1]) + fr.n[2] * (Z[2, k] - fr.o[2])
        # Backed off by the accepted position mismatch, so that an accepted QP prediction
        # within the corridor implies a nonlinear rollout within the planning corridor.
        @constraint(m, e <= E - c.mismatch_tol[1])
        @constraint(m, e >= -(E - c.mismatch_tol[1]))
    end
    # Linearised lateral acceleration, both interval end points (coefficients updated).
    ay_hi = Matrix{ConstraintRef}(undef, N, 2)
    ay_lo = Matrix{ConstraintRef}(undef, N, 2)
    if c.ay_max !== nothing
        for k in 1:N, (j, kv) in enumerate((k, k + 1))
            ay_hi[k, j] = @constraint(m, Z[4, kv] + U[1, k] <= c.ay_max)
            ay_lo[k, j] = @constraint(m, Z[4, kv] + U[1, k] >= -c.ay_max)
        end
    end
    # Objective (as the nonlinear MPC, in the affine track coordinates).
    s_scale = p.v_max * N * c.dt
    a_ref = max(-p.a_min, p.a_max)
    sN = fr.t[1] * (Z[1, N+1] - Z[1, 1]) + fr.t[2] * (Z[2, N+1] - Z[2, 1])
    J = -c.ws * sN / s_scale
    for k in 2:N+1
        ey = fr.n[1] * (Z[1, k] - fr.o[1]) + fr.n[2] * (Z[2, k] - fr.o[2])
        J += c.qy * (ey / E)^2 + c.qpsi * ((Z[3, k] - fr.psi0) / c.epsi_max)^2
    end
    for k in 1:N
        dprev, aprev = k == 1 ? (uprev[1], uprev[2]) : (U[1, k-1], U[2, k-1])
        J += c.rdelta * (U[1, k] / p.delta_max)^2 + c.ra * (U[2, k] / a_ref)^2 +
             c.rd_delta * ((U[1, k] - dprev) / p.delta_max)^2 + c.rd_a * ((U[2, k] - aprev) / a_ref)^2
    end
    @objective(m, Min, J)
    # Set OSQP up now, while every placeholder coefficient is nonzero: OSQP fixes its
    # sparsity pattern at the first solve, and later updates (including exact zeros, e.g.
    # ∂px/∂py) then modify stored entries in place. One iteration suffices for the setup.
    if ctrl.optimizer === OSQP.Optimizer
        set_attribute(m, "max_iter", 1)
        optimize!(m)
        set_attribute(m, "max_iter", c.osqp_max_iter)
    end
    ctrl.model, ctrl.Z, ctrl.U, ctrl.uprev, ctrl.dyn, ctrl.ay_hi, ctrl.ay_lo = m, Z, U, uprev, dyn, ay_hi, ay_lo
    return ctrl
end

"""
    linearize_exact_map(vehicle, z, u, dt) -> (A, B, c)

Exact discrete-time linearisation of the Cartesian bicycle map `F(z, u)` at `(z, u)`:
`A = ∂F/∂z` (4×4), `B = ∂F/∂u` (4×2) by ForwardDiff on [`exact_cartesian_step`](@ref), and
`c = F(z, u) - A z - B u`, so that `F(z, u) = A z + B u + c` holds exactly at the point.
"""
function linearize_exact_map(veh::KinematicBicycle, z::AbstractVector, u::AbstractVector, dt::Real)
    f = w -> exact_cartesian_step(veh, w[1:4], w[5:6], dt)
    w = vcat(collect(Float64, z), collect(Float64, u))
    J = ForwardDiff.jacobian(f, w)
    A, B = J[:, 1:4], J[:, 5:6]
    Fz = collect(exact_cartesian_step(veh, z, u, dt))
    return A, B, Fz - A * w[1:4] - B * w[5:6]
end

"""
    linearize_lateral_acceleration(L, v, delta) -> (g, gv, gdelta)

First-order model of `ay(v, delta) = v² tan(delta)/L` at `(v, delta)`:
`ay ≈ g + gv (v' - v) + gdelta (delta' - delta)`, with `gv = 2 v tan(delta)/L` and
`gdelta = v² (1 + tan²(delta))/L`. It is exact at the point only. The QP uses it as two
linear inequalities, and the nonlinear check validates the true value.
"""
function linearize_lateral_acceleration(L, v, delta)
    t = tan(delta)
    return v^2 * t / L, 2v * t / L, v^2 * (1 + t^2) / L
end

"Exact Cartesian rollout of the inputs `U` (2×N) from `z0`."
function exact_rollout(veh::KinematicBicycle, z0::AbstractVector, U::AbstractMatrix, dt::Real)
    N = size(U, 2)
    Z = zeros(4, N + 1)
    Z[:, 1] = z0
    for k in 1:N
        Z[:, k+1] = exact_cartesian_step(veh, Z[:, k], U[:, k], dt)
    end
    return Z
end

"Track coordinates `(s, ey, epsi)` of a Cartesian state on the controller's straight frame."
function track_coordinates(fr, z)
    dx, dy = z[1] - fr.o[1], z[2] - fr.o[2]
    return (fr.t[1] * dx + fr.t[2] * dy + fr.s0, fr.n[1] * dx + fr.n[2] * dy, z[3] - fr.psi0)
end

"""
    linear_plan_objective(ctrl, Z, U, uprev) -> Float64

The MPC objective evaluated on a Cartesian trajectory `Z` with inputs `U`. It is the same
function as the QP objective, including constants. Applied to the exact rollout, it gives
the nonlinear objective of a plan.
"""
function linear_plan_objective(ctrl::LinearMPCController, Z::AbstractMatrix, U::AbstractMatrix, uprev)
    c, p, N, fr, E = ctrl.config, ctrl.vehicle.params, ctrl.config.horizon, ctrl.frame, ctrl.ey_max
    s_scale, a_ref = p.v_max * N * c.dt, max(-p.a_min, p.a_max)
    J = -c.ws * (track_coordinates(fr, Z[:, N+1])[1] - track_coordinates(fr, Z[:, 1])[1]) / s_scale
    for k in 2:N+1
        _, ey, ep = track_coordinates(fr, Z[:, k])
        J += c.qy * (ey / E)^2 + c.qpsi * (ep / c.epsi_max)^2
    end
    for k in 1:N
        dprev, aprev = k == 1 ? (uprev[1], uprev[2]) : (U[1, k-1], U[2, k-1])
        J += c.rdelta * (U[1, k] / p.delta_max)^2 + c.ra * (U[2, k] / a_ref)^2 +
             c.rd_delta * ((U[1, k] - dprev) / p.delta_max)^2 + c.rd_a * ((U[2, k] - aprev) / a_ref)^2
    end
    return J
end

"""
    validate_plan(vehicle, track, limits, z0, U; substeps=10, tol=1e-4) -> NamedTuple

Independent nonlinear validation of an input sequence from the Cartesian state `z0` on a
straight track. It is used for both the QP and the nonlinear MPC plans.
- Propagate with the exact Cartesian map.
- Check, with tolerance `tol`:
  - the planning corridor `|ey| ≤ width/2 - margin` at the knots `k ≥ 2`;
  - the physical track `|ey| ≤ width/2` along the rollout (`substeps` samples per
    interval);
  - the speed at the knots;
  - the input limits;
  - `|epsi| ≤ epsi_max` at the knots;
  - the exact `|v² tan(delta)/L| ≤ ay_max` at both interval end points.

`limits = (; track_margin, epsi_max, ay_max)`. Returns
`(; Zroll, violations, max_abs_ey, max_abs_ay, min_track_clearance, min_corridor_clearance)`.
"""
function validate_plan(veh::KinematicBicycle, track::Track, limits, z0::AbstractVector, U::AbstractMatrix; substeps::Integer=10,
    tol::Real=1e-4)
    p, L, N = veh.params, veh.params.wheelbase, size(U, 2)
    fr = straight_frame(track)
    dt = limits.dt
    W, E = track.width / 2, track.width / 2 - limits.track_margin
    Zr = exact_rollout(veh, z0, U, dt)
    viol = String[]
    maxey, maxay, mintrack = 0.0, 0.0, Inf
    for k in 1:N
        for j in 0:substeps-1
            q = j == 0 ? Zr[:, k] : interior_point(L, Zr[:, k], Zr[4, k], U[1, k], U[2, k], j * dt / substeps)
            ey = fr.n[1] * (q[1] - fr.o[1]) + fr.n[2] * (q[2] - fr.o[2])
            mintrack = min(mintrack, W - abs(ey))
        end
    end
    mincorr = Inf
    for k in 2:N+1
        _, ey, ep = track_coordinates(fr, Zr[:, k])
        maxey = max(maxey, abs(ey))
        mincorr = min(mincorr, E - abs(ey))
        mintrack = min(mintrack, W - abs(ey))
        abs(ey) > E + tol && push!(viol, "knot $k: |ey| = $(fmt(abs(ey))) m > planning corridor $(fmt(E)) m")
        abs(ep) > limits.epsi_max + tol && push!(viol, "knot $k: |epsi| = $(fmt(abs(ep))) rad > $(fmt(limits.epsi_max)) rad")
        (p.v_min - tol <= Zr[4, k] <= p.v_max + tol) || push!(viol, "knot $k: speed $(fmt(Zr[4, k])) m/s outside [$(p.v_min), $(p.v_max)]")
    end
    mintrack < -tol && push!(viol, "physical track edge crossed along the rollout (clearance $(fmt(mintrack)) m)")
    for k in 1:N
        (abs(U[1, k]) <= p.delta_max + tol && p.a_min - tol <= U[2, k] <= p.a_max + tol) ||
            push!(viol, "stage $k: input $(U[:, k]) outside the actuator limits")
        if limits.ay_max !== nothing
            for v in (Zr[4, k], Zr[4, k+1])
                ay = abs(v^2 * tan(U[1, k]) / L)
                maxay = max(maxay, ay)
                ay > limits.ay_max + tol && push!(viol, "stage $k: |ay| = $(fmt(ay)) m/s² > $(limits.ay_max) m/s²")
            end
        end
    end
    return (; Zroll=Zr, violations=viol, max_abs_ey=maxey, max_abs_ay=maxay, min_track_clearance=mintrack, min_corridor_clearance=mincorr)
end

validate_plan(ctrl::LinearMPCController, z0, U) = validate_plan(ctrl.vehicle, ctrl.track,
    (; ctrl.config.dt, ctrl.config.track_margin, ctrl.config.epsi_max, ctrl.config.ay_max), z0, U;
    substeps=ctrl.config.validation_substeps, tol=ctrl.config.validation_tol)

"""
    pursuit_input(controller, z) -> (delta, a)

The nominal-trajectory input at the Cartesian state `z`:
- pure-pursuit steering towards the centreline (look-ahead `clamp(0.5 v, 0.5, 2) m`),
  clipped to the steering limit and to the steering angle at which `|ay| = ay_max` at the
  current speed;
- acceleration `(v_max - v)/dt`, clipped to `[a_min, a_max]`.
"""
function pursuit_input(ctrl::LinearMPCController, z::AbstractVector)
    c, p, fr = ctrl.config, ctrl.vehicle.params, ctrl.frame
    L, v = p.wheelbase, z[4]
    ld = clamp(0.5v, 0.5, 2.0)
    s, _, _ = track_coordinates(fr, z)
    tx = fr.o[1] + fr.t[1] * (s + ld - fr.s0)
    ty = fr.o[2] + fr.t[2] * (s + ld - fr.s0)
    alpha = atan(ty - z[2], tx - z[1]) - z[3]
    dmax = c.ay_max === nothing ? p.delta_max : min(p.delta_max, atan(c.ay_max * L / max(v, 0.1)^2))
    return clamp(atan(2L * sin(alpha) / ld), -dmax, dmax), clamp((p.v_max - v) / c.dt, p.a_min, p.a_max)
end

"""
    nominal_inputs(controller, z0) -> U (2×N)

Cold-start inputs: [`pursuit_input`](@ref) applied along an exact rollout from `z0`, so
the nominal trajectory is dynamically consistent.
"""
function nominal_inputs(ctrl::LinearMPCController, z0::AbstractVector)
    c, N = ctrl.config, ctrl.config.horizon
    U = zeros(2, N)
    z = collect(Float64, z0)
    for k in 1:N
        U[:, k] .= pursuit_input(ctrl, z)
        z = collect(exact_cartesian_step(ctrl.vehicle, z, U[:, k], c.dt))
    end
    return U
end

"""
    shifted_inputs(controller, z0) -> U (2×N)

Warm-start inputs: the previous accepted inputs shifted by one stage. The last stage is
[`pursuit_input`](@ref) at the end of the shifted rollout (`warm_tail = :pursuit`), or
a repeat of the previous last input (`warm_tail = :repeat`).
"""
function shifted_inputs(ctrl::LinearMPCController, z0::AbstractVector)
    c, N, Up = ctrl.config, ctrl.config.horizon, ctrl.last_U
    U = hcat(Up[:, 2:N], Up[:, N])
    if c.warm_tail === :pursuit
        z = exact_rollout(ctrl.vehicle, z0, U[:, 1:N-1], c.dt)[:, N]
        U[:, N] .= pursuit_input(ctrl, z)
    end
    return U
end

"Install the linearisation at `(Zb, Ub)`, the trust-region bounds and the start values."
function install_linearization!(ctrl::LinearMPCController, Zb, Ub, scale, start)
    c, p, N, L = ctrl.config, ctrl.vehicle.params, ctrl.config.horizon, ctrl.vehicle.params.wheelbase
    t_lin = @elapsed lin = [linearize_exact_map(ctrl.vehicle, Zb[:, k], Ub[:, k], c.dt) for k in 1:N]
    aymax = c.ay_max === nothing ? nothing : ctrl.track_model === :local ? c.ay_max - c.local_backoff.ay : c.ay_max
    t_upd = @elapsed begin
        for k in 1:N
            A, B, cc = lin[k]
            for i in 1:4
                con = ctrl.dyn[i, k]
                for j in 1:4
                    set_normalized_coefficient(con, ctrl.Z[j, k], -A[i, j])
                end
                set_normalized_coefficient(con, ctrl.Z[i, k+1], 1.0)
                for j in 1:2
                    set_normalized_coefficient(con, ctrl.U[j, k], -B[i, j])
                end
                set_normalized_rhs(con, cc[i])
            end
            if c.ay_max !== nothing
                for (j, kv) in enumerate((k, k + 1))
                    g, gv, gd = linearize_lateral_acceleration(L, Zb[4, kv], Ub[1, k])
                    off = g - gv * Zb[4, kv] - gd * Ub[1, k]
                    for con in (ctrl.ay_hi[k, j], ctrl.ay_lo[k, j])
                        set_normalized_coefficient(con, ctrl.Z[4, kv], gv)
                        set_normalized_coefficient(con, ctrl.U[1, k], gd)
                    end
                    set_normalized_rhs(ctrl.ay_hi[k, j], aymax - off)
                    set_normalized_rhs(ctrl.ay_lo[k, j], -aymax - off)
                end
            end
        end
        # Trust region ∩ physical bounds.
        rz, ru = scale .* c.trust_region_z, scale .* c.trust_region_u
        # (Local track model: the heading-error bound is on the EP variables instead of psi.)
        psib = ctrl.track_model === :straight ? (ctrl.frame.psi0 - c.epsi_max, ctrl.frame.psi0 + c.epsi_max) : (-Inf, Inf)
        vmax = ctrl.track_model === :local ? p.v_max - c.local_backoff.speed : p.v_max
        physz = ((-Inf, Inf), (-Inf, Inf), psib, (p.v_min, vmax))
        physu = ((-p.delta_max, p.delta_max), (p.a_min, p.a_max))
        setb(v, bar, r, ph) = begin
            lo, hi = max(ph[1], bar - r), min(ph[2], bar + r)
            lo > hi && (lo = hi = clamp(bar, ph[1], ph[2]))
            set_lower_bound(v, lo)
            set_upper_bound(v, hi)
        end
        for k in 2:N+1, i in 1:4
            setb(ctrl.Z[i, k], Zb[i, k], rz[i], physz[i])
        end
        for k in 1:N, i in 1:2
            setb(ctrl.U[i, k], Ub[i, k], ru[i], physu[i])
        end
        if start !== nothing
            for k in 2:N+1, i in 1:4
                set_start_value(ctrl.Z[i, k], start[1][i, k])
            end
            for k in 1:N, i in 1:2
                set_start_value(ctrl.U[i, k], start[2][i, k])
            end
        end
    end
    return t_lin, t_upd
end

"""
    qp_values(controller, vars) -> Array

Primal values of `vars` from the last OSQP solve: the solution when OSQP reports a feasible
point, otherwise OSQP's last iterate (an inexact solve; OSQP's MOI wrapper reports no
primal solution then). The iterate is used only as a candidate, which the nonlinear
validation accepts or rejects.
"""
function qp_values(ctrl::LinearMPCController, vars)
    primal_status(ctrl.model) == MOI.FEASIBLE_POINT && return value.(vars)
    opt = unsafe_backend(ctrl.model)
    return map(v -> opt.results.x[optimizer_index(v).value], vars)
end

"OSQP diagnostics of the last solve: `(iterations, primal residual, dual residual)`."
function osqp_info(m::JuMP.Model)
    try
        info = unsafe_backend(m).results.info
        return Int(info.iter), Float64(info.pri_res), Float64(info.dua_res)
    catch
        return -1, NaN, NaN
    end
end

"""
    solve!(controller::LinearMPCController, measured_state; previous_input=(0, 0)) -> LinearMPCResult

One control cycle of the successive-linearisation MPC. `measured_state` is the Frenet
state `[s, ey, epsi, v]` on the track (unwrapped progress), converted to Cartesian
coordinates.

**Straight track model** (one attempt):
1. **Nominal trajectory.** Warm: the previous inputs shifted by one step. Cold:
   [`nominal_inputs`](@ref). Either way it is rolled out with the exact map from the
   measured state, so it is dynamically consistent.
2. **Successive linearisation** ([`sqp_attempt!`](@ref)).
3. **No acceptable candidate:** `:qp_failure` or `:not_converged`, and no control is
   returned.

**Local track model:** curved tracks and/or a stationary obstacle; one or more attempts,
see [`solve_local!`](@ref).
"""
function solve!(ctrl::LinearMPCController, measured_state::AbstractVector; previous_input=(0.0, 0.0))
    t0 = time_ns()
    c, veh = ctrl.config, ctrl.vehicle
    uprev = SVector{2,Float64}(previous_input...)
    x = SVector{4,Float64}(measured_state...)
    z0 = try
        check_domain(veh, x, uprev, 0.0)
        collect(frenet_state_to_cartesian(ctrl.track, x))
    catch err
        err isa DomainError || rethrow()
        reset_warm_start!(ctrl)
        obrep = ctrl.obstacle === nothing ? LINEAR_OBSTACLE_NONE :
                merge(LINEAR_OBSTACLE_NONE, (; present=true, required=required_separation(ctrl), failure_cause=:invalid_measurement))
        return linear_result(ctrl, t0, :invalid_measurement, "measured state or previous input outside the model domain: " *
                                                              sprint(showerror, err), nothing, uprev, x; obstacle=obrep)
    end
    ctrl.track_model === :local && return solve_local!(ctrl, x, z0, uprev, t0)
    tn = time_ns()
    Ub = c.warm_start && ctrl.last_U !== nothing ? shifted_inputs(ctrl, z0) : nominal_inputs(ctrl, z0)
    t_nomin = (time_ns() - tn) / 1e9
    a = sqp_attempt!(ctrl, x, z0, uprev, Ub, t0)
    if a.status === :success
        ctrl.last_U, ctrl.last_Z = a.Uq, a.Zq
    else
        reset_warm_start!(ctrl)
    end
    return linear_result(ctrl, t0, a.status, a.msg, a, uprev, x; t_nomin)
end

"""
    sqp_attempt!(controller, x, z0, uprev, Ub, t0; side=:none, nominal_excess=nothing) -> NamedTuple

Bounded successive linearisation from the nominal inputs `Ub`, rolled out exactly from
`z0`. For `i = 1 … max_qp_solves`:

1. **Local track model:** project the nominal trajectory onto the actual track
   ([`trajectory_references`](@ref)). An invalid or ambiguous reference ends the attempt
   with `:invalid_reference`.
2. Linearise the dynamics and the lateral acceleration at the nominal trajectory, and (local
   model) update the track-coordinate rows and the obstacle halfspaces
   ([`install_track_rows!`](@ref); `side` resolves degenerate halfspace directions).
3. Set the trust region (scale `ρ`, initially 1) intersected with the physical bounds, and
   solve the QP with OSQP.
4. Project the candidate inputs onto the actuator box (OSQP meets bounds only to its
   tolerance).
5. Validate the inputs with the exact nonlinear rollout. The straight model uses
   [`validate_plan`](@ref); the local model uses [`validate_plan_on_track`](@ref): actual
   track, projections and obstacle.
6. Compute the mismatch between the QP prediction and the rollout. In the local model with
   interior obstacle samples, the position mismatch includes the affine interior positions.
7. **Accept** if OSQP is optimal, the mismatch is within `mismatch_tol`, and there are no
   violations.
8. **Otherwise** the new nominal trajectory is the candidate's rollout (relinearisation).
   - If the mismatch exceeded `mismatch_tol`, `ρ` is reduced (see `trust_region_shrink` in
     [`LinearMPCConfig`](@ref)). A rejection for violations alone keeps `ρ`.
   - The next QP starts only if the cycle's elapsed time plus this attempt's mean QP time
     fits within `sqp_time_budget`.
"""
function sqp_attempt!(ctrl::LinearMPCController, x, z0, uprev, Ub, t0; side::Symbol=:none, nominal_excess=nothing)
    c, N, veh = ctrl.config, ctrl.config.horizon, ctrl.vehicle
    loc = ctrl.track_model === :local
    tn = time_ns()
    Zb = exact_rollout(veh, z0, Ub, c.dt)
    t_nom = (time_ns() - tn) / 1e9
    t_lin, t_upd, t_sol, t_val, t_ref = 0.0, 0.0, 0.0, 0.0, 0.0
    qst, qit, pres, dres, scales = Symbol[], Int[], Float64[], Float64[], Float64[]
    mhist = NTuple{3,Float64}[]
    scale = step_policy(ctrl) === :trust_region ? ctrl.trust_memory : 1.0
    Zq, Uq = fill(NaN, 4, N + 1), fill(NaN, 2, N)
    val, mis, obj = nothing, (NaN, NaN, NaN), NaN
    refs, ndeg, terr = nothing, 0, (NaN, NaN, NaN)
    status, msg = :not_converged, ""
    # nominal_excess: constraint excess of the current nominal (local model; may be passed in)
    if step_policy(ctrl) === :trust_region && loc && scale < 1
        tv = time_ns()                                        # carried-over trust region only from a valid nominal
        nominal_excess === nothing && (nominal_excess = validate_local(ctrl, x[1], z0, Ub).excess)
        nominal_excess > c.trust_memory_excess && (scale = 1.0)
        t_val += (time_ns() - tv) / 1e9
    end
    start = c.primal_start === :nominal ? (Zb, Ub) : nothing
    for it in 1:c.max_qp_solves
        if it > 1 && (time_ns() - t0) / 1e9 + t_sol / (it - 1) > c.sqp_time_budget
            msg = "no further QP: time budget $(c.sqp_time_budget) s would be exceeded"
            break
        end
        if loc
            tr_ = time_ns()
            refs = try
                trajectory_references(ctrl, Zb, x[1])
            catch err
                err isa TrackReferenceError || rethrow()
                status, msg = :invalid_reference, "nominal trajectory of QP $it: " * err.msg
                nothing
            end
            t_ref += (time_ns() - tr_) / 1e9
            refs === nothing && break
        end
        if c.update_mode === :rebuild
            t_upd += @elapsed (loc ? build_local_qp!(ctrl) : build_qp!(ctrl))
        end
        tu = time_ns()
        for i in 1:4
            fix(ctrl.Z[i, 1], z0[i]; force=true)
        end
        fix(ctrl.uprev[1], uprev[1]; force=true)
        fix(ctrl.uprev[2], uprev[2]; force=true)
        loc && fix(ctrl.s0var, x[1]; force=true)
        t_upd += (time_ns() - tu) / 1e9
        tl, tu2 = install_linearization!(ctrl, Zb, Ub, scale, start)
        t_lin += tl
        t_upd += tu2
        if loc
            tu = time_ns()
            ndeg, _ = install_track_rows!(ctrl, refs, Zb, Ub, side)
            if start !== nothing
                for i in 1:N
                    set_start_value(ctrl.S[i], refs[i+1].s)
                    set_start_value(ctrl.EY[i], refs[i+1].ey)
                    set_start_value(ctrl.EP[i], refs[i+1].epsi)
                end
            end
            t_upd += (time_ns() - tu) / 1e9
        end
        ts = time_ns()
        optimize!(ctrl.model)
        t_sol += (time_ns() - ts) / 1e9
        ts_ = termination_status(ctrl.model)
        iters, pr, dr = osqp_info(ctrl.model)
        push!(qst, Symbol(ts_)); push!(qit, iters); push!(pres, pr); push!(dres, dr); push!(scales, scale)
        usable = ts_ in (MOI.OPTIMAL, MOI.LOCALLY_SOLVED) && primal_status(ctrl.model) == MOI.FEASIBLE_POINT
        if !usable && step_policy(ctrl) === :trust_region && ts_ in (MOI.ALMOST_OPTIMAL, MOI.ITERATION_LIMIT, MOI.TIME_LIMIT)
            usable = all(isfinite, qp_values(ctrl, ctrl.Z)) && all(isfinite, qp_values(ctrl, ctrl.U))   # inexact: still a candidate
        end
        if !usable
            status, msg = :qp_failure, "OSQP returned $(ts_) (QP solve $it)"
            break
        end
        Zq, Uq = qp_values(ctrl, ctrl.Z), qp_values(ctrl, ctrl.U)
        # OSQP satisfies bounds only to its tolerance: project the inputs onto the actuator box
        # (the rollout and the mismatch below use the projected inputs).
        pv = veh.params
        Uq[1, :] .= clamp.(Uq[1, :], -pv.delta_max, pv.delta_max)
        Uq[2, :] .= clamp.(Uq[2, :], pv.a_min, pv.a_max)
        obj = try
            objective_value(ctrl.model)
        catch
            NaN
        end
        tv = time_ns()
        val = loc ? validate_local(ctrl, x[1], z0, Uq) : validate_plan(ctrl, z0, Uq)
        d = val.Zroll .- Zq
        mis = (maximum(hypot(d[1, k], d[2, k]) for k in 1:N+1), maximum(abs, d[3, :]), maximum(abs, d[4, :]))
        loc && (mis = (max(mis[1], interior_mismatch(ctrl, Zq, Uq, val.Zroll)), mis[2], mis[3]))
        t_val += (time_ns() - tv) / 1e9
        if loc
            tr_ = time_ns()
            terr = track_linearization_error(ctrl, Zq, x[1])
            t_ref += (time_ns() - tr_) / 1e9
        end
        push!(mhist, mis)
        ok = all(mis .<= c.mismatch_tol) && isempty(val.violations)
        if ok
            status, msg = :success, ""
            break
        end
        msg = "QP solve $it rejected: mismatch $(fmt.(mis))" * (isempty(val.violations) ? "" : "; " * first(val.violations))
        ratio = maximum(mis ./ c.mismatch_tol)
        if ratio > 1
            scale *= clamp(0.9 * sqrt(1 / ratio), c.min_trust_scale_factor, c.trust_region_shrink)
        elseif step_policy(ctrl) === :trust_region
            scale *= c.trust_region_shrink                    # violations from the constraint approximation alone
        end
        keep = false
        if step_policy(ctrl) === :trust_region && ratio > 1 && !isempty(val.violations)
            tv = time_ns()                                    # compare the two linearisation points
            nominal_excess === nothing && (nominal_excess = validate_local(ctrl, x[1], z0, Ub).excess)
            keep = nominal_excess < val.excess
            t_val += (time_ns() - tv) / 1e9
        end
        if keep                                               # retract the step: keep the nominal, smaller region
            c.primal_start === :none || (start = (Zb, Ub))
        else
            Zb, Ub = val.Zroll, Uq                            # relinearise at the candidate's nonlinear rollout
            nominal_excess = loc ? val.excess : nothing
            c.primal_start === :none || (start = (Zq, Uq))
        end
    end
    nlobj = val === nothing ? NaN : loc ? plan_objective_on_track(c, veh, ctrl.ey_max, val.frenet, Uq, uprev; horizon=N, dt=c.dt) :
            linear_plan_objective(ctrl, val.Zroll, Uq, uprev)
    return (; status, msg, Zq, Uq, Zb, Ub, val, mis, obj, nlobj, qst, qit, pres, dres, scales, mhist, t_nom, t_lin, t_upd, t_sol,
        t_val, t_ref, refs, terr, ndeg)
end

"Effective rejected-step policy of a controller (see `LinearMPCConfig`)."
step_policy(ctrl::LinearMPCController) = ctrl.config.step_policy === :auto ?
    (ctrl.track_model === :straight ? :rollout : :trust_region) : ctrl.config.step_policy

"Assemble a [`LinearMPCResult`](@ref) from the chosen attempt `a` (or `nothing`) and all attempts of the cycle."
function linear_result(ctrl::LinearMPCController, t0, status::Symbol, msg::AbstractString, a, uprev, x;
    obstacle::NamedTuple=LINEAR_OBSTACLE_NONE, t_guess=0.0, t_nomin=0.0, all_attempts=(a === nothing ? Any[] : Any[a]))
    N = ctrl.config.horizon
    tsum(f) = sum(getfield(b, f) for b in all_attempts; init=0.0)
    timing = (; total=(time_ns() - t0) / 1e9, nominal=t_nomin + tsum(:t_nom), linearize=tsum(:t_lin), update=tsum(:t_upd),
        solve=tsum(:t_sol), validate=tsum(:t_val), reference=tsum(:t_ref), guesses=Float64(t_guess))
    cat(f, T) = reduce(vcat, [getfield(b, f) for b in all_attempts]; init=T[])
    qst, qit, pres, dres, scales = cat(:qst, Symbol), cat(:qit, Int), cat(:pres, Float64), cat(:dres, Float64), cat(:scales, Float64)
    mhist = cat(:mhist, NTuple{3,Float64})
    nan(r, c) = fill(NaN, r, c)
    if a === nothing
        return LinearMPCResult(status, false, nothing, nan(4, N + 1), nan(2, N), nan(4, N + 1), nan(2, N), nan(4, N + 1),
            (NaN, NaN, NaN), String[], NaN, NaN, qst, qit, pres, dres, scales, mhist, length(qst), NaN, NaN, timing, msg,
            ctrl.track_model, (NaN, NaN, NaN), nan(4, N + 1), nan(3, N + 1), NaN, obstacle)
    end
    accepted = status === :success
    val = a.val
    refm = a.refs === nothing ? nan(4, N + 1) : reduce(hcat, [[r.s, r.ey, r.epsi, r.kappa] for r in a.refs])
    fren = val === nothing ? nan(3, N + 1) : ctrl.track_model === :local ? val.frenet :
           reduce(hcat, [collect(track_coordinates(ctrl.frame, val.Zroll[:, k])) for k in 1:N+1])
    return LinearMPCResult(status, accepted, accepted ? SVector(a.Uq[1, 1], a.Uq[2, 1]) : nothing, a.Zq, a.Uq, a.Zb, a.Ub,
        val === nothing ? nan(4, N + 1) : val.Zroll, a.mis, val === nothing ? String[] : val.violations,
        val === nothing ? NaN : val.max_abs_ey, val === nothing ? NaN : val.max_abs_ay, qst, qit, pres, dres, scales, mhist,
        length(qst), a.obj, a.nlobj, timing, msg, ctrl.track_model, a.terr, refm, fren,
        val === nothing ? NaN : val.min_track_clearance, obstacle)
end
