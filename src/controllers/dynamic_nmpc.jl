# Nonlinear MPC (JuMP + Ipopt) for the six-state dynamic bicycle, with any number of
# stationary circular obstacles imposed simultaneously (final milestone). See
# docs/dynamic_bicycle_mpc.md.

"""
    DynamicNMPCConfig(; kwargs...)

Configuration of [`DynamicNMPCController`](@ref).

**Discretisation:** `horizon = 10`, `dt = 0.1` [s], `substeps = 4` RK4 sub-steps per
interval inside the NLP (`h = 0.025 s`). For [`showcase_dynamic_parameters`](@ref) at
`vx_min = 0.8 m/s`, the stiffest mode rate times `h` is about 1.6, inside RK4's real-axis
stability bound of about 2.78; the plan agrees with the 1 ms reference integration to
about 1e-5 m (measured).

**Objective:** the progress-maximising objective of [`NMPCConfig`](@ref), with the same
weights and normalisation (`ws, qy, qpsi, rdelta, ra, rd_delta, rd_a`; the speed scale is
`vx_max`).

**Constraints:**
- `track_margin = 0.05`, `epsi_max = 60°`;
- `ay_max = 4.0` [m/s²], on the dynamic lateral acceleration `(F_yf cos δ + F_yr)/m` (not
  `v² tan δ / L`).

**Obstacles:**
- `ego_radius = 0.05`: a disk centred at the **centre of mass**;
- `obstacle_buffer = 0.03`;
- `obstacle_check_samples = 50` per interval (dense validation);
- `obstacle_reach_factor = 1.5`: an obstacle is given an active constraint slot if its
  centre is within `reach_factor · vx_max · horizon · dt + D_i` of the CoM;
- `obstacle_ramp = 1.5`, `obstacle_ramp_min = 0.3` [m]: lateral transition lengths of the
  passing guesses;
- `max_side_obstacles = 2`: at most this many newly encountered obstacles have their
  passing sides enumerated (at most 4 candidate sequences).

**Assignment and acceptance:**
- `max_reassignments = 4`, `knot_tol = 1e-6`;
- `acceptance_tol = 1e-6`: NLP residuals;
- `consistency_tol = 5e-3`: plan vs independent fine rollout [m, rad, m/s];
- `validation_tol = 1e-4`: true-geometry checks;
- `handling_tol = 1e-2`: slip angles [rad] and lateral acceleration [m/s²] on the fine
  rollout. The NLP imposes them exactly on its RK4 prediction; the allowance covers the
  prediction/rollout difference (bounded by `consistency_tol` on the states) when a plan
  rides a handling limit;
- `validation_samples = 10` per interval;
- `plant_step = 1e-3` [s]: step of the independent validation integrator.

**Solver and time:**
- `max_iter = 300`, `tol = 1e-8`, `optimizer_attributes` (Ipopt);
- `time_budget = 0.1` [s]: a further initialisation attempt or reassignment starts only if
  the elapsed cycle time plus the longest NLP solve so far fits.
"""
Base.@kwdef struct DynamicNMPCConfig
    horizon::Int = 10
    dt::Float64 = 0.1
    substeps::Int = 4
    ws::Float64 = 10.0
    qy::Float64 = 0.1
    qpsi::Float64 = 0.1
    rdelta::Float64 = 0.01
    ra::Float64 = 0.01
    rd_delta::Float64 = 1.0
    rd_a::Float64 = 0.1
    track_margin::Float64 = 0.05
    epsi_max::Float64 = deg2rad(60)
    ay_max::Float64 = 4.0
    ego_radius::Float64 = 0.05
    obstacle_buffer::Float64 = 0.03
    obstacle_check_samples::Int = 50
    obstacle_reach_factor::Float64 = 1.5
    obstacle_ramp::Float64 = 1.5
    obstacle_ramp_min::Float64 = 0.3
    max_side_obstacles::Int = 2
    max_reassignments::Int = 4
    knot_tol::Float64 = 1e-6
    acceptance_tol::Float64 = 1e-6
    consistency_tol::Float64 = 5e-3
    validation_tol::Float64 = 1e-4
    handling_tol::Float64 = 1e-2
    validation_samples::Int = 10
    plant_step::Float64 = 1e-3
    max_iter::Int = 300
    tol::Float64 = 1e-8
    time_budget::Float64 = 0.1
    optimizer_attributes::Vector{Pair{String,Any}} = Pair{String,Any}[]
end

function validate_config(c::DynamicNMPCConfig)
    c.horizon >= 1 && c.substeps >= 1 || throw(ArgumentError("horizon ≥ 1 and substeps ≥ 1 required"))
    check_timestep(c.dt)
    0 < c.epsi_max < π / 2 || throw(ArgumentError("epsi_max must lie in (0, π/2)"))
    c.ay_max > 0 && c.ego_radius >= 0 && c.obstacle_buffer >= 0 || throw(ArgumentError("ay_max > 0, ego_radius ≥ 0, obstacle_buffer ≥ 0 required"))
    c.obstacle_check_samples >= 1 && c.validation_samples >= 1 && c.max_reassignments >= 0 || throw(ArgumentError("sample counts ≥ 1, max_reassignments ≥ 0"))
    c.plant_step > 0 && c.plant_step <= c.dt / c.substeps || throw(ArgumentError("plant_step must be positive and finer than the MPC sub-step"))
    c.max_side_obstacles >= 1 || throw(ArgumentError("max_side_obstacles ≥ 1 required"))
    return c
end

"""
Result of one [`solve!`](@ref) of a [`DynamicNMPCController`](@ref).

**Outcome.**
- `status`:
  - `:success`;
  - `:invalid_measurement` (non-finite, `vx` outside the model domain, or no valid track
    projection);
  - `:initial_collision` (the measured CoM disk is inside some `D_i`);
  - `:solver_failure` (Ipopt did not report a locally optimal point in any attempt);
  - `:segment_assignment_invalid` (a knot stayed outside its assigned segment after the
    allowed reassignments);
  - `:validation_failed` (the plan failed an independent check; see `violations`).
- `accepted`, `u`: only an accepted cycle has a control `u = [δ, a]`.
- `termination_status`: Ipopt's status of the last NLP solve (a `Symbol`);
  `iterations`: Ipopt iterations summed over the cycle; `nlp_solves`.

**Plan** (the accepted, or last, attempt):
- `Z` (6×(N+1)): Cartesian CoM states;
- `frenet` (3×(N+1)): `(s, ey, eψ)` of the CoM;
- `U` (2×N);
- `path` (2×(N·substeps+1)): the predicted CoM positions at every RK4 sub-step, from the
  optimiser's solution (for drawing the planned horizon).

**Validation**, on an independent fine rollout of `U` from the measurement
(`validation` NamedTuple):
- `mismatch` (position, heading, velocity) and `dynamics_residual`;
- `min_track_clearance` and `max_abs_ey`;
- `max_slip`, `max_ay`;
- `min_obstacle_distance`, `min_contact_clearance`, `min_buffer_clearance`;
- `collisions`, `buffer_violations`, `plan_obstacle_margin`.

`violations` lists the failed checks.

**Obstacles:** `active_obstacles` are the indices with a constraint slot this cycle;
`sides` holds the passing side per obstacle index (`:left`/`:right`, relative to the track
direction): the side on which the accepted plan's predicted path makes its closest approach
to the obstacle. While the plan ends before the obstacle, it is the side of the initial
guess that led to the accepted plan.

**Diagnostics:**
- `attempts`: `(label, status, nlp_solves, seconds)` for every initialisation attempt;
- `timing`: `(; total, setup, guesses, solve, validation)` [s], where `total` is the whole
  call;
- `message`.
"""
struct DynamicNMPCResult
    status::Symbol
    accepted::Bool
    u::Union{Nothing,SVector{2,Float64}}
    termination_status::Symbol
    iterations::Int
    nlp_solves::Int
    objective::Float64
    Z::Matrix{Float64}
    frenet::Matrix{Float64}
    U::Matrix{Float64}
    path::Matrix{Float64}
    validation::NamedTuple
    violations::Vector{String}
    active_obstacles::Vector{Int}
    sides::Dict{Int,Symbol}
    attempts::Vector{Tuple{Symbol,Symbol,Int,Float64}}
    timing::NamedTuple{(:total, :setup, :guesses, :solve, :validation),NTuple{5,Float64}}
    message::String
end

"""
    DynamicNMPCController(vehicle::DynamicBicycle, track, config=DynamicNMPCConfig();
                          obstacles=CircleObstacle[], buffers=nothing, optimizer=Ipopt.Optimizer)

Progress-maximising nonlinear MPC for the six-state dynamic bicycle, on any piecewise
constant-curvature track, with any number of stationary circular obstacles. See
`docs/dynamic_bicycle_mpc.md`.

**NLP (direct multiple shooting).**
- *Knot variables.* The CoM's track coordinates `(s, ey, eψ)` with respect to an
  **assigned segment** (Parameters, as in the Cartesian-endpoint NMPC), and the body
  velocities `(vx, vy, r)`.
- *Knot poses.* Segment-local centreline formulas ([`frenet_pose_local`](@ref)), so there
  is no discontinuous segment look-up and no curvature averaging.
- *Dynamics.* RK4 sub-step equalities in Cartesian coordinates, with intermediate
  sub-step state variables.
- *Track and heading.* Bounds on `ey`, `eψ`.
- *Operating domain and handling.* Bounds on `vx`, the slip angles and the dynamic
  lateral acceleration.
- *Obstacles.* One constraint `‖p − o_i‖² ≥ D_i²` per obstacle slot, at every knot and
  every RK4 sub-step position.

**Per cycle.** The track coordinates of the measured CoM are computed by projection. Then
up to a few initialisation attempts run: the warm start, and the passing-side sequences of
[`side_sequence_guesses`](@ref). Each attempt:
1. rolls its inputs out (dynamically consistent);
2. assigns segments, then solves with Ipopt;
3. reassigns any knot that left its segment, and re-solves (bounded);
4. validates the plan independently ([`validate_dynamic_plan`](@ref)).

A plan is accepted only if every check passes.

**Obstacle slots.** The model has one constraint slot per obstacle (fixed capacity). A slot
is active (`D_i²`) when the obstacle is within reach of the horizon, and inactive (`0`,
trivially satisfied) otherwise. Distant obstacles therefore never leave stale constraints,
and all reachable obstacles are constrained simultaneously.
"""
mutable struct DynamicNMPCController
    vehicle::DynamicBicycle{Float64}
    track::Track{Float64}
    config::DynamicNMPCConfig
    obstacles::Vector{CircleObstacle}
    buffers::Vector{Float64}
    assoc::Vector{Tuple{Float64,Float64}}          # (s, ey) of each obstacle centre, s ∈ [0, L)
    model::JuMP.Model
    F::Matrix{VariableRef}                         # 6×N: knots 2…N+1, (s, ey, eψ, vx, vy, r)
    W::Matrix{VariableRef}                         # 6×(N(S−1)): intermediate RK4 sub-step states (Cartesian)
    P::Matrix{VariableRef}                         # 3×N: Cartesian poses of knots 2…N+1
    G::Array{VariableRef,3}                        # 6×3×(N S): RK4 stage states z2, z3, z4 of every sub-step
    U::Matrix{VariableRef}
    z0::Vector{VariableRef}                        # Parameters: measured CoM state
    s0::Any                                        # Parameter (VariableRef): measured progress
    uprev::Vector{VariableRef}                     # Parameters: previous input
    cf::Matrix{VariableRef}                        # Parameters 5×N: (s0, x0, y0, psi0, kappa) of the knots' segments
    op::Matrix{VariableRef}                        # Parameters 3×K: (ox, oy, D²) per obstacle slot
    ey_max::Float64
    last_U::Union{Nothing,Matrix{Float64}}
    s_prev::Float64
    committed::Dict{Int,Tuple{Symbol,Float64}}     # obstacle index => (side, unwrapped progress of the encounter)
    build_time::Float64
end

horizon(ctrl::DynamicNMPCController) = ctrl.config.horizon
reset_warm_start!(ctrl::DynamicNMPCController) = (ctrl.last_U = nothing; ctrl)
required_separation(ctrl::DynamicNMPCController, i::Integer) = required_separation(ctrl.obstacles[i], ctrl.config.ego_radius, ctrl.buffers[i])

function DynamicNMPCController(vehicle::DynamicBicycle, track::Track, config::DynamicNMPCConfig=DynamicNMPCConfig();
    obstacles=CircleObstacle[], buffers=nothing, optimizer=Ipopt.Optimizer)
    validate_config(config)
    E = track.width / 2 - config.track_margin
    E > 0 || throw(ArgumentError("track.width/2 - track_margin must be positive"))
    obs = collect(CircleObstacle, obstacles)
    buf = buffers === nothing ? fill(config.obstacle_buffer, length(obs)) : collect(Float64, buffers)
    length(buf) == length(obs) || throw(ArgumentError("one buffer per obstacle required"))
    for (o, b) in zip(obs, buf)
        required_separation(o, config.ego_radius, b)
    end
    assoc = Tuple{Float64,Float64}[]
    for o in obs
        s, ey, _ = try
            cartesian_to_frenet(track, o.x, o.y)
        catch err
            err isa ArgumentError || rethrow()
            throw(ArgumentError("obstacle at ($(o.x), $(o.y)) has no unique projection onto the track: $(sprint(showerror, err))"))
        end
        push!(assoc, (s, ey))
    end
    ctrl = DynamicNMPCController(vehicle, track, config, obs, buf, assoc, Model(), Matrix{VariableRef}(undef, 0, 0),
        Matrix{VariableRef}(undef, 0, 0), Matrix{VariableRef}(undef, 0, 0), Array{VariableRef,3}(undef, 0, 0, 0), Matrix{VariableRef}(undef, 0, 0), VariableRef[], nothing, VariableRef[],
        Matrix{VariableRef}(undef, 0, 0), Matrix{VariableRef}(undef, 0, 0), E, nothing, NaN, Dict{Int,Tuple{Symbol,Float64}}(), 0.0)
    ctrl.build_time = @elapsed build_dynamic_nlp!(ctrl, optimizer)
    return ctrl
end

"Knot `j + 1`'s Cartesian state as JuMP expressions (pose from the assigned segment frame)."
knot_state(ctrl::DynamicNMPCController, j) = (ctrl.P[1, j], ctrl.P[2, j], ctrl.P[3, j], ctrl.F[4, j], ctrl.F[5, j], ctrl.F[6, j])

"""
    rk4_stages(params, z, u, h) -> (z2, z3, z4, znext)

The three intermediate RK4 stage states and the end state of one step of length `h`:
`z2 = z + h/2 f(z)`, `z3 = z + h/2 f(z2)`, `z4 = z + h f(z3)`,
`znext = z + h/6 (f(z) + 2 f(z2) + 2 f(z3) + f(z4))`. Identical to
[`dynamic_rk4_step`](@ref); the NLP uses the stages as variables.
"""
function rk4_stages(p::DynamicBicycleParameters, z, u, h)
    f1 = dynamic_rhs(p, z, u)
    z2 = z .+ (h / 2) .* f1
    f2 = dynamic_rhs(p, z2, u)
    z3 = z .+ (h / 2) .* f2
    f3 = dynamic_rhs(p, z3, u)
    z4 = z .+ h .* f3
    f4 = dynamic_rhs(p, z4, u)
    return z2, z3, z4, z .+ (h / 6) .* (f1 .+ 2 .* f2 .+ 2 .* f3 .+ f4)
end

function build_dynamic_nlp!(ctrl::DynamicNMPCController, optimizer)
    c, p, N, S, E = ctrl.config, ctrl.vehicle.params, ctrl.config.horizon, ctrl.config.substeps, ctrl.ey_max
    K = length(ctrl.obstacles)
    m = Model(optimizer)
    set_silent(m)
    if optimizer === Ipopt.Optimizer
        set_attribute(m, "max_iter", c.max_iter)
        set_attribute(m, "tol", c.tol)
        set_attribute(m, "print_level", 0)
        for (k, v) in c.optimizer_attributes
            set_attribute(m, k, v)
        end
    end
    @operator(m, op_sinc, 1, sinc_un, sinc_un_d1, sinc_un_d2)
    @variable(m, z0[1:6] in Parameter(1.0))
    @variable(m, s0 in Parameter(0.0))
    @variable(m, uprev[1:2] in Parameter(0.0))
    @variable(m, cf[1:5, 1:N] in Parameter(0.0))
    @variable(m, op[1:3, 1:K] in Parameter(0.0))
    @variable(m, F[1:6, 1:N])
    for j in 1:N
        set_lower_bound(F[2, j], -E); set_upper_bound(F[2, j], E)
        set_lower_bound(F[3, j], -c.epsi_max); set_upper_bound(F[3, j], c.epsi_max)
        set_lower_bound(F[4, j], p.vx_min); set_upper_bound(F[4, j], p.vx_max)
    end
    nW = N * (S - 1)
    @variable(m, W[1:6, 1:nW])
    for j in 1:nW
        set_lower_bound(W[4, j], p.vx_min / 2)                   # model domain during the sub-steps
    end
    # Auxiliary variables keep every constraint expression small (no nested RK4 stages, no
    # repeated pose formulas): the knots' Cartesian poses and the RK4 stage states.
    @variable(m, P[1:3, 1:N])
    @variable(m, G[1:6, 1:3, 1:N*S])
    for j in 1:N*S, q in 1:3
        set_lower_bound(G[4, q, j], p.vx_min / 4)
    end
    @variable(m, U[1:2, 1:N])
    for k in 1:N
        set_lower_bound(U[1, k], -p.delta_max); set_upper_bound(U[1, k], p.delta_max)
        set_lower_bound(U[2, k], p.a_min); set_upper_bound(U[2, k], p.a_max)
    end
    ctrl.model, ctrl.F, ctrl.W, ctrl.U, ctrl.z0, ctrl.s0, ctrl.uprev, ctrl.cf, ctrl.op = m, F, W, U, z0, s0, uprev, cf, op
    ctrl.P, ctrl.G = P, G
    for j in 1:N
        pose = frenet_pose_local((s0=cf[1, j], x0=cf[2, j], y0=cf[3, j], psi0=cf[4, j], kappa=cf[5, j]), F[1, j], F[2, j], F[3, j], op_sinc)
        for i in 1:3
            @constraint(m, P[i, j] - pose[i] == 0)
        end
    end
    h = c.dt / S
    points = Any[]                                                # constrained positions (knots 2…N+1 and sub-steps)
    zk(j) = j == 0 ? Tuple(z0) : knot_state(ctrl, j)              # knot j+1 (j = 0: the measured state)
    for k in 1:N
        u = (U[1, k], U[2, k])
        zc = zk(k - 1)
        for j in 1:S
            idx = (k - 1) * S + j
            g2, g3, g4 = Tuple(G[:, 1, idx]), Tuple(G[:, 2, idx]), Tuple(G[:, 3, idx])
            f1 = dynamic_rhs(p, zc, u)
            f2 = dynamic_rhs(p, g2, u)
            f3 = dynamic_rhs(p, g3, u)
            f4 = dynamic_rhs(p, g4, u)
            target = j < S ? Tuple(W[:, (k-1)*(S-1)+j]) : zk(k)
            for i in 1:6
                @constraint(m, g2[i] - zc[i] - (h / 2) * f1[i] == 0)
                @constraint(m, g3[i] - zc[i] - (h / 2) * f2[i] == 0)
                @constraint(m, g4[i] - zc[i] - h * f3[i] == 0)
                @constraint(m, target[i] - zc[i] - (h / 6) * (f1[i] + 2 * f2[i] + 2 * f3[i] + f4[i]) == 0)
            end
            push!(points, (target[1], target[2]))
            zc = target
        end
        # Handling: front slip with u_k at the interval start; rear slip at the interval end
        # (the measured state's rear slip is not controllable); dynamic lateral
        # acceleration at both interval ends.
        zs, ze = zk(k - 1), zk(k)
        af = U[1, k] - atan((zs[5] + p.lf * zs[6]) / zs[4])
        @constraint(m, -p.alpha_max <= af <= p.alpha_max)
        ar = -atan((ze[5] - p.lr * ze[6]) / ze[4])
        @constraint(m, -p.alpha_max <= ar <= p.alpha_max)
        for zz in (zs, ze)
            ay = dynamic_lateral_acceleration(p, zz, u)
            @constraint(m, -c.ay_max <= ay <= c.ay_max)
        end
    end
    for i in 1:K, pt in points
        @constraint(m, (pt[1] - op[1, i])^2 + (pt[2] - op[2, i])^2 - op[3, i] >= 0)
    end
    s_scale = p.vx_max * N * c.dt
    a_ref = max(-p.a_min, p.a_max)
    J = -c.ws * (F[1, N] - s0) / s_scale
    for j in 1:N
        J += c.qy * (F[2, j] / E)^2 + c.qpsi * (F[3, j] / c.epsi_max)^2
    end
    for k in 1:N
        dprev, aprev = k == 1 ? (uprev[1], uprev[2]) : (U[1, k-1], U[2, k-1])
        J += c.rdelta * (U[1, k] / p.delta_max)^2 + c.ra * (U[2, k] / a_ref)^2 +
             c.rd_delta * ((U[1, k] - dprev) / p.delta_max)^2 + c.rd_a * ((U[2, k] - aprev) / a_ref)^2
    end
    @objective(m, Min, J)
    return ctrl
end

"""
    dynamic_rollout(model, z0, U, dt; substeps) -> (Z, path)

Rollout of the inputs `U` (2×N) with the MPC's RK4 discretisation: knot states `Z`
(6×(N+1)) and every sub-step position `path` (2×(N·substeps+1)). Throws a `DomainError` if
`vx` leaves the model domain.
"""
function dynamic_rollout(model::DynamicBicycle, z0::AbstractVector, U::AbstractMatrix, dt::Real; substeps::Integer=5)
    N = size(U, 2)
    Z = zeros(6, N + 1)
    path = zeros(2, N * substeps + 1)
    Z[:, 1] = z0
    path[:, 1] = z0[1:2]
    zz = Tuple(Float64.(z0))
    h = dt / substeps
    for k in 1:N
        u = (U[1, k], U[2, k])
        for j in 1:substeps
            zz = dynamic_rk4_step(model.params, zz, u, h)
            zz[4] > 0 || throw(DomainError(zz[4], "vx left the model domain in the rollout"))
            path[:, (k-1)*substeps+j+1] .= (zz[1], zz[2])
        end
        Z[:, k+1] .= zz
    end
    return Z, path
end

"Track coordinates `(s, ey, eψ)` of the CoM positions in `Z` (columns), chained from the hint `s0`."
function chain_frenet(track::Track, Z::AbstractMatrix, s0::Real; max_jump::Real=1.0)
    out = fill(NaN, 3, size(Z, 2))
    lasts, lastp = Float64(s0), (Z[1, 1], Z[2, 1])
    for k in axes(Z, 2)
        r = track_reference(track, Z[1, k], Z[2, k], Z[3, k]; s_hint=lasts + hypot(Z[1, k] - lastp[1], Z[2, k] - lastp[2]), max_jump,
            min_frenet_factor=1e-3)
        out[:, k] .= (r.s, r.ey, r.epsi)
        lasts, lastp = r.s, (Z[1, k], Z[2, k])
    end
    return out
end

"""
    validate_dynamic_plan(ctrl, z0, s0, U, Zplan) -> NamedTuple

Independent validation of a plan, using the fine RK4 integration (`plant_step`, 100 steps
per period by default) of its inputs from the measured state. Every check uses the true
track geometry (lap-unwrapped projections):

- **Consistency.** The largest difference between the plan's knots and the rollout:
  position, heading and velocity, each ≤ `consistency_tol`.
- **Track.** The corridor and `|eψ| ≤ epsi_max` at the knots; the physical edge
  (`width/2`) at `validation_samples` points per interval.
- **Operating domain.** `vx` and the inputs (`validation_tol`); the front slip angle at
  the start and the rear slip angle at the end of every interval, as imposed in the NLP,
  and the dynamic lateral acceleration at both ends (`handling_tol`).
- **Obstacles** (every obstacle, active or not), at every fine step:
  - contact, meaning a distance below `r_ego + r_i`, is a violation;
  - a distance below `D_i` is counted as a buffer violation.

  At the knots and RK4 sub-step times of the rollout, the distance must be at least
  `D_i − consistency_tol`.

Returns `(; violations, mismatch, rollout, min_track_clearance, max_abs_ey, max_slip,
max_ay, min_obstacle_distance, min_contact_clearance, min_buffer_clearance, collisions,
buffer_violations, frenet)`.
"""
function validate_dynamic_plan(ctrl::DynamicNMPCController, z0, s0, U, Zplan)
    c, veh, tr, N = ctrl.config, ctrl.vehicle, ctrl.track, ctrl.config.horizon
    p = veh.params
    viol = String[]
    fine = SVector{6,Float64}[]
    knots = zeros(6, N + 1)
    knots[:, 1] = z0
    z = SVector{6,Float64}(z0...)
    try
        for k in 1:N
            seg = integrate_dynamic(veh, z, U[:, k], c.dt; h=c.plant_step)
            append!(fine, k == 1 ? seg : seg[2:end])
            z = seg[end]
            knots[:, k+1] = z
        end
    catch err
        err isa DomainError || rethrow()
        push!(viol, "fine rollout left the model domain: " * sprint(showerror, err))
        return (; violations=viol, mismatch=(Inf, Inf, Inf), rollout=knots, min_track_clearance=-Inf, max_abs_ey=Inf, max_slip=Inf,
            max_ay=Inf, min_obstacle_distance=NaN, min_contact_clearance=NaN, min_buffer_clearance=NaN, collisions=0,
            buffer_violations=0, frenet=fill(NaN, 3, N + 1))
    end
    d = knots .- Zplan
    mis = (maximum(hypot(d[1, k], d[2, k]) for k in 1:N+1), maximum(abs, d[3, :]), maximum(abs, d[4:6, :]))
    mis[1] <= c.consistency_tol && mis[2] <= c.consistency_tol && mis[3] <= c.consistency_tol ||
        push!(viol, "plan and fine rollout differ by $(fmt.(mis)) (position, heading, velocity) > $(c.consistency_tol)")
    nper = (length(fine) - 1) ÷ N                                  # intervals exclude the initial sample
    # Track geometry.
    W2 = tr.width / 2
    fr = fill(NaN, 3, N + 1)
    mintrack, maxey = Inf, 0.0
    ok = true
    try
        idx = unique(vcat([round(Int, (k - 1) * nper + j * nper / c.validation_samples) + 1 for k in 1:N for j in 0:c.validation_samples-1], length(fine)))
        F = chain_frenet(tr, reduce(hcat, [collect(fine[i]) for i in idx]), s0)
        for (col, i) in enumerate(idx)
            mintrack = min(mintrack, W2 - abs(F[2, col]))
            if (i - 1) % nper == 0
                fr[:, (i-1)÷nper+1] .= F[:, col]
            end
        end
    catch err
        err isa TrackReferenceError || rethrow()
        push!(viol, "no valid track projection of the rollout: " * err.msg)
        ok, mintrack = false, -Inf
    end
    if ok
        for k in 2:N+1
            ey, ep = fr[2, k], fr[3, k]
            maxey = max(maxey, abs(ey))
            abs(ey) <= ctrl.ey_max + c.validation_tol || push!(viol, "knot $k: |ey| = $(fmt(abs(ey))) m > corridor $(fmt(ctrl.ey_max)) m")
            abs(ep) <= c.epsi_max + c.validation_tol || push!(viol, "knot $k: |eψ| = $(fmt(abs(ep))) rad > $(fmt(c.epsi_max))")
        end
        mintrack < -c.validation_tol && push!(viol, "physical track edge crossed (clearance $(fmt(mintrack)) m)")
    end
    # Operating domain and handling at both ends of every interval.
    maxslip, maxay = 0.0, 0.0
    for k in 1:N, zz in (knots[:, k], knots[:, k+1])
        u = U[:, k]
        af, ar = slip_angles(p, zz, u)
        maxslip = max(maxslip, abs(af), abs(ar))
        ay = abs(dynamic_lateral_acceleration(p, zz, u))
        maxay = max(maxay, ay)
    end
    for k in 1:N
        u = U[:, k]
        zk1 = knots[:, k+1]
        p.vx_min - c.validation_tol <= zk1[4] <= p.vx_max + c.validation_tol ||
            push!(viol, "knot $(k+1): vx = $(fmt(zk1[4])) m/s outside [$(p.vx_min), $(p.vx_max)]")
        (abs(u[1]) <= p.delta_max + c.validation_tol && p.a_min - c.validation_tol <= u[2] <= p.a_max + c.validation_tol) ||
            push!(viol, "stage $k: input $(u) outside the actuator limits")
        af, _ = slip_angles(p, knots[:, k], u)
        _, ar = slip_angles(p, zk1, u)
        max(abs(af), abs(ar)) <= p.alpha_max + c.handling_tol ||
            push!(viol, "stage $k: slip angle $(fmt(max(abs(af), abs(ar)))) rad > $(fmt(p.alpha_max))")
        for zz in (knots[:, k], zk1)
            ay = abs(dynamic_lateral_acceleration(p, zz, u))
            ay <= c.ay_max + c.handling_tol || push!(viol, "stage $k: |a_y| = $(fmt(ay)) m/s² > $(c.ay_max)")
        end
    end
    # Obstacles: dense distances, contact and buffer, and the constrained sample times.
    mind, mincon, minbuf, ncol, nbuf = Inf, Inf, Inf, 0, 0
    S = c.substeps
    for (i, o) in enumerate(ctrl.obstacles)
        D = required_separation(ctrl, i)
        Dc = c.ego_radius + o.radius
        step = max(1, nper ÷ c.obstacle_check_samples)
        for j in 1:step:length(fine)
            dd = hypot(fine[j][1] - o.x, fine[j][2] - o.y)
            mind = min(mind, dd)
            mincon = min(mincon, dd - Dc)
            minbuf = min(minbuf, dd - D)
            dd < Dc - c.acceptance_tol && (ncol += 1)
            dd < D - c.acceptance_tol && (nbuf += 1)
        end
        for k in 1:N, j in 1:S
            q = fine[(k-1)*nper+round(Int, j * nper / S)+1]
            dd = hypot(q[1] - o.x, q[2] - o.y)
            dd >= D - c.consistency_tol || (push!(viol, "obstacle $i: rollout at a constrained time is $(fmt(dd)) m < D = $(fmt(D)) m"); break)
        end
    end
    ncol > 0 && push!(viol, "obstacle contact in the dense evaluation ($ncol samples; minimum contact clearance $(fmt(mincon)) m)")
    return (; violations=viol, mismatch=mis, rollout=knots, min_track_clearance=mintrack, max_abs_ey=maxey, max_slip=maxslip, max_ay=maxay,
        min_obstacle_distance=isempty(ctrl.obstacles) ? NaN : mind, min_contact_clearance=isempty(ctrl.obstacles) ? NaN : mincon,
        min_buffer_clearance=isempty(ctrl.obstacles) ? NaN : minbuf, collisions=ncol, buffer_violations=nbuf, frenet=fr)
end

"""
    pursuit_guess(ctrl, z0, s0; lateral=s -> 0.0, accel=nothing) -> U (2×N)

Dynamically consistent initial guess, rolled out with the MPC's discretisation, that
follows the lateral target profile `lateral(s)` along the actual track.

**Steering.** Pure pursuit from the rear axle towards the point at progress `s + ld` and
offset `lateral(s + ld)`. The look-ahead is `clamp(0.5 vx, 0.5, 2) m`, limited by the
curvature within 2 m. On a constant-curvature path this reproduces the path's curvature
(curvature feedforward).

**Limits.** The steering is clipped to `delta_max` and to the kinematic steady-state angle
of `0.9 ay_max` at the current speed.

**Acceleration.** Towards the speed cap `sqrt(0.8 ay_max/|κ_path|)` of the offset path
within the next `vx + 0.5 m`, with braking room at 80 % of `|a_min|`, and above `vx_min`.
`accel`, if given, overrides it. The cap stays below the lateral-acceleration limit, which
keeps the slip angles inside the linear range.
"""
function pursuit_guess(ctrl::DynamicNMPCController, z0::AbstractVector, s0::Real; lateral=s -> 0.0, accel=nothing)
    c, veh, tr, N = ctrl.config, ctrl.vehicle, ctrl.track, ctrl.config.horizon
    p = veh.params
    L = p.lf + p.lr
    smax = tr.closed ? Inf : track_length(tr)
    kap(s) = curvature(tr, min(s, smax))
    U = zeros(2, N)
    z = Tuple(Float64.(z0))
    s = Float64(s0)
    lastp = (z[1], z[2])
    for k in 1:N
        ref = track_reference(tr, z[1], z[2], z[3]; s_hint=s + hypot(z[1] - lastp[1], z[2] - lastp[2]), max_jump=1.0, min_frenet_factor=1e-3)
        s, lastp = ref.s, (z[1], z[2])
        vx = z[4]
        kmax = maximum(abs(kap(s + ds)) for ds in 0.0:0.1:2.0)
        ld = clamp(0.5vx, 0.5, 2.0)
        kmax > 0 && (ld = min(ld, max(0.5, 1 / kmax)))
        st = min(s + ld, smax)
        pt = frenet_to_cartesian(tr, st, lateral(st))
        xr, yr, psi = rear_axle_pose(p, z)
        alpha = atan(pt[2] - yr, pt[1] - xr) - psi
        dmax = min(p.delta_max, atan(0.9 * c.ay_max * L / max(vx, 0.1)^2))
        delta = clamp(atan(2L * sin(alpha) / ld), -dmax, dmax)
        vt = p.vx_max
        for ds in 0.0:0.1:(vx + 0.5)
            kk = kap(s + ds)
            kk == 0 && continue
            kp = abs(kk / (1 - kk * lateral(s + ds)))
            vt = min(vt, sqrt(0.8 * c.ay_max / kp + 2 * 0.8 * abs(p.a_min) * ds))
        end
        vt = max(vt, p.vx_min + 0.05)
        a = accel === nothing ? clamp((vt - vx) / c.dt, p.a_min, p.a_max) : accel[k]
        U[:, k] .= (delta, a)
        h = c.dt / c.substeps
        for _ in 1:c.substeps
            z = dynamic_rk4_step(p, z, (delta, a), h)
        end
        z[4] > 0 || break
    end
    return U
end

"Unwrapped progress of obstacle `i`'s association nearest to (and not far behind) progress `s`."
function obstacle_progress(ctrl::DynamicNMPCController, i, s)
    so = ctrl.assoc[i][1]
    ctrl.track.closed || return so
    L = track_length(ctrl.track)
    n = floor((s - so) / L)
    cand = so + n * L
    D = required_separation(ctrl, i)
    return cand < s - D - ctrl.ey_max ? cand + L : cand
end

"""
    relevant_obstacles(ctrl, s) -> Vector{(index, s_o, ey_o, D)}

Obstacles whose association lies in the horizon window `[s − D_i − E, s + vx_max N dt +
D_i + E]` (unwrapped, across laps), sorted by progress. Used for the passing decisions; the
collision constraints use the Euclidean reach test instead.
"""
function relevant_obstacles(ctrl::DynamicNMPCController, s)
    c, p, E = ctrl.config, ctrl.vehicle.params, ctrl.ey_max
    out = Tuple{Int,Float64,Float64,Float64}[]
    for i in eachindex(ctrl.obstacles)
        D = required_separation(ctrl, i)
        so = obstacle_progress(ctrl, i, s)
        s - D - E <= so <= s + p.vx_max * c.horizon * c.dt + D + E && push!(out, (i, so, ctrl.assoc[i][2], D))
    end
    sort!(out; by=x -> x[2])
    return out
end

"Lateral targets of a passing-side sequence: smooth windows of the passage centre around each obstacle."
function side_profile(ctrl::DynamicNMPCController, plan, s_now)
    c, E = ctrl.config, ctrl.ey_max
    items = map(plan) do (i, so, eyo, D, side)
        target = side === :left ? (eyo + D + E) / 2 : (eyo - D - E) / 2
        ramp_in = clamp(so - D - s_now, c.obstacle_ramp_min, c.obstacle_ramp)
        (; so, D, target, ramp_in)
    end
    function w(it, s)
        dd = abs(s - it.so)
        dd <= it.D && return 1.0
        r = s < it.so ? it.ramp_in : c.obstacle_ramp
        dd >= it.D + r && return 0.0
        return 0.5 * (1 + cos(π * (dd - it.D) / r))
    end
    return s -> begin
        ws = [w(it, s) for it in items]
        tot = sum(ws; init=0.0)
        tot == 0 ? 0.0 : sum(ws[j] * items[j].target for j in eachindex(items)) / max(1.0, tot)
    end
end

"""
    side_sequence_guesses(ctrl, z0, s0, relevant) -> Vector{(label, U, sides, score)}

Bounded multi-obstacle initialisation.
1. **Fixed sides.** Obstacles that already have a committed side keep it.
2. **Enumerated sides.** The first `max_side_obstacles` uncommitted obstacles ahead each
   take every side with a geometric passage, where the passage-centre target
   `(ey_o ± D + ±E)/2` lies inside the corridor (`ey_o + D ≤ E` for left,
   `ey_o − D ≥ −E` for right). If neither side qualifies, both are tried.
3. **Greedy sides.** Further obstacles take the side with the wider gap.

This gives at most 2^`max_side_obstacles` (default 4) sequences. Each one is turned into a
lateral target profile ([`side_profile`](@ref)) and a dynamically consistent rollout
([`pursuit_guess`](@ref)).

**Ranking.** The rollouts are ranked by:
- whether the rollout clears every relevant obstacle by `D_i` at its knots and stays on
  the track;
- then the smallest knot clearance above `D_i` over the relevant obstacles;
- then the track clearance.
"""
function side_sequence_guesses(ctrl::DynamicNMPCController, z0, s0, relevant)
    c, E = ctrl.config, ctrl.ey_max
    fixed = Dict{Int,Symbol}(i => ctrl.committed[i][1] for (i, _, _, _) in relevant if haskey(ctrl.committed, i))
    free = [r for r in relevant if !haskey(fixed, r[1]) && r[2] >= s0 - r[4]]
    enum, rest = free[1:min(end, c.max_side_obstacles)], free[min(end, c.max_side_obstacles)+1:end]
    function options(r)
        _, _, eyo, D = r
        o = Symbol[]
        eyo + D <= E && push!(o, :left)
        eyo - D >= -E && push!(o, :right)
        return isempty(o) ? [:left, :right] : o
    end
    combos = Vector{Dict{Int,Symbol}}([Dict{Int,Symbol}()])
    for r in enum
        combos = [merge(cmb, Dict(r[1] => sd)) for cmb in combos for sd in options(r)]
    end
    greedy = Dict(r[1] => ((E - (r[3] + r[4])) >= (r[3] - r[4] + E) ? :left : :right) for r in rest)
    out = []
    for cmb in combos
        sides = merge(fixed, cmb, greedy)
        plan = [(i, so, eyo, D, sides[i]) for (i, so, eyo, D) in relevant if haskey(sides, i)]
        U = try
            pursuit_guess(ctrl, z0, s0; lateral=side_profile(ctrl, plan, s0))
        catch err
            err isa Union{DomainError,TrackReferenceError} || rethrow()
            continue
        end
        Z, _ = try
            dynamic_rollout(ctrl.vehicle, z0, U, c.dt; substeps=c.substeps)
        catch err
            err isa DomainError || rethrow()
            continue
        end
        clr = minimum((minimum(hypot(Z[1, k] - ctrl.obstacles[i].x, Z[2, k] - ctrl.obstacles[i].y) for k in 2:size(Z, 2)) - D
                       for (i, _, _, D) in relevant); init=Inf)
        trk = try
            ctrl.track.width / 2 - maximum(abs, chain_frenet(ctrl.track, Z, s0)[2, :])
        catch err
            err isa TrackReferenceError || rethrow()
            -Inf
        end
        label = Symbol(join(["$(i)$(sides[i] === :left ? 'L' : 'R')" for (i, _, _, _) in relevant if haskey(sides, i)], "_"))
        push!(out, (label, U, sides, (clr >= 0 && trk >= 0, clr, trk)))
    end
    sort!(out; by=x -> x[4], rev=true)
    return out
end

"Install the measurement, the obstacle slots and the segment frames, and set the start values from a rollout."
function install_dynamic!(ctrl::DynamicNMPCController, z0, s0, uprev, Zg, Wg, Ug, fr, frames, active; pinned=())
    c, N, S = ctrl.config, ctrl.config.horizon, ctrl.config.substeps
    # Knots in `pinned` keep their progress inside their assigned segment (assignment cycles);
    # all other knots are free.
    for j in 1:N
        sj = ctrl.F[1, j]
        if j in pinned
            set_lower_bound(sj, frames[j+1].s0)
            set_upper_bound(sj, frames[j+1].s1)
        else
            has_lower_bound(sj) && delete_lower_bound(sj)
            has_upper_bound(sj) && delete_upper_bound(sj)
        end
    end
    for i in 1:6
        set_parameter_value(ctrl.z0[i], z0[i])
    end
    set_parameter_value(ctrl.s0, s0)
    set_parameter_value(ctrl.uprev[1], uprev[1])
    set_parameter_value(ctrl.uprev[2], uprev[2])
    for (i, o) in enumerate(ctrl.obstacles)
        set_parameter_value(ctrl.op[1, i], o.x)
        set_parameter_value(ctrl.op[2, i], o.y)
        set_parameter_value(ctrl.op[3, i], i in active ? required_separation(ctrl, i)^2 : 0.0)   # inactive slot: always satisfied
    end
    for j in 1:N
        f = frames[j+1]
        for (r, v) in enumerate((f.s0, f.x0, f.y0, f.psi0, f.kappa))
            set_parameter_value(ctrl.cf[r, j], v)
        end
        for (r, v) in enumerate((fr[1, j+1], fr[2, j+1], fr[3, j+1], Zg[4, j+1], Zg[5, j+1], Zg[6, j+1]))
            set_start_value(ctrl.F[r, j], v)
        end
        set_start_value(ctrl.U[1, j], Ug[1, j])
        set_start_value(ctrl.U[2, j], Ug[2, j])
    end
    for j in 1:N, r in 1:3
        set_start_value(ctrl.P[r, j], Zg[r, j+1])
    end
    Wv, Gv = Wg
    for j in axes(ctrl.W, 2), r in 1:6
        set_start_value(ctrl.W[r, j], Wv[r, j])
    end
    for j in axes(ctrl.G, 3), q in 1:3, r in 1:6
        set_start_value(ctrl.G[r, q, j], Gv[r, q, j])
    end
    return nothing
end

"Intermediate sub-step states (6×N(S−1)) and RK4 stage states (6×3×NS) of a rollout with the MPC discretisation."
function substep_states(ctrl::DynamicNMPCController, z0, U)
    c, p, N, S = ctrl.config, ctrl.vehicle.params, ctrl.config.horizon, ctrl.config.substeps
    Wg = zeros(6, N * (S - 1))
    Gg = zeros(6, 3, N * S)
    z = Tuple(Float64.(z0))
    h = c.dt / S
    for k in 1:N, j in 1:S
        z2, z3, z4, z = rk4_stages(p, z, (U[1, k], U[2, k]), h)
        Gg[:, 1, (k-1)*S+j] .= z2
        Gg[:, 2, (k-1)*S+j] .= z3
        Gg[:, 3, (k-1)*S+j] .= z4
        j < S && (Wg[:, (k-1)*(S-1)+j] .= z)
    end
    return Wg, Gg
end

"Values of the current NLP solution: Cartesian knots, track coordinates, inputs, sub-step positions."
function extract_dynamic(ctrl::DynamicNMPCController, z0, s0)
    c, N, S = ctrl.config, ctrl.config.horizon, ctrl.config.substeps
    Fv, Uv, Wv = value.(ctrl.F), value.(ctrl.U), value.(ctrl.W)
    cf = parameter_value.(ctrl.cf)
    Z = zeros(6, N + 1)
    Z[:, 1] = z0
    fr = zeros(3, N + 1)
    fr[:, 1] .= (s0, NaN, NaN)
    for j in 1:N
        X, Y, psi = frenet_pose_local((s0=cf[1, j], x0=cf[2, j], y0=cf[3, j], psi0=cf[4, j], kappa=cf[5, j]), Fv[1, j], Fv[2, j], Fv[3, j])
        Z[:, j+1] .= (X, Y, psi, Fv[4, j], Fv[5, j], Fv[6, j])
        fr[:, j+1] .= Fv[1:3, j]
    end
    path = zeros(2, N * S + 1)
    path[:, 1] = z0[1:2]
    for k in 1:N, j in 1:S
        path[:, (k-1)*S+j+1] .= j < S ? Wv[1:2, (k-1)*(S-1)+j] : Z[1:2, k+1]
    end
    return Z, fr, Uv, path
end

"""
    solve!(ctrl::DynamicNMPCController, z; previous_input=(0, 0), s_hint=NaN) -> DynamicNMPCResult

One control cycle for the measured Cartesian CoM state `z = [X, Y, ψ, vx, vy, r]`.

**Track coordinates.** The measured progress is the track projection nearest the
previous cycle's progress (or `s_hint`; for the first call a unique projection is
required).

**Attempts.** Initialisation attempts run in order, while the time budget allows a further
one:

| situation | order |
|---|---|
| no relevant obstacle | warm start (shifted previous inputs), then the cold pursuit guess |
| a newly relevant obstacle | the ranked passing-side sequences ([`side_sequence_guesses`](@ref)), then the warm start |
| otherwise | the warm start, then the committed sides' sequence |

**Each attempt:**
1. roll the inputs out with the MPC discretisation;
2. assign segments to the knots ([`cartesian_assignment`](@ref)) and solve with Ipopt;
3. if a knot left its segment, reassign and re-solve (at most `max_reassignments`). If a
   reassignment would repeat an assignment already tried in this attempt (a knot near a
   boundary falls into the other segment in either segment's frame), the current
   assignment is kept instead. The alternating knots' progress is then bounded to their
   assigned segments for the remaining re-solves. The segment formulas are exact on the
   closed interval, so this is a restriction of the same problem, not an approximation;
4. validate the solution ([`validate_dynamic_plan`](@ref)).

The first attempt that passes is accepted.
"""
function solve!(ctrl::DynamicNMPCController, zmeas::AbstractVector; previous_input=(0.0, 0.0), s_hint::Real=NaN)
    t0 = time_ns()
    c, veh, tr, N = ctrl.config, ctrl.vehicle, ctrl.track, ctrl.config.horizon
    p = veh.params
    uprev = SVector{2,Float64}(previous_input...)
    z0 = collect(Float64, zmeas)
    t_setup, t_guess, t_solve, t_val = 0.0, 0.0, 0.0, 0.0
    attempts = Tuple{Symbol,Symbol,Int,Float64}[]
    nanres(st, msg; active=Int[]) = DynamicNMPCResult(st, false, nothing, :NOT_CALLED, 0, 0, NaN, fill(NaN, 6, N + 1), fill(NaN, 3, N + 1),
        fill(NaN, 2, N), fill(NaN, 2, N * c.substeps + 1), (;), String[], active, Dict{Int,Symbol}(), attempts,
        (; total=(time_ns() - t0) / 1e9, setup=t_setup, guesses=t_guess, solve=t_solve, validation=t_val), msg)
    # Measurement and its track coordinates.
    ref = try
        check_dynamic_domain(veh, z0, uprev)
        p.vx_min / 2 <= z0[4] || throw(DomainError(z0[4], "measured vx below half the operating minimum"))
        hint = isfinite(s_hint) ? s_hint : ctrl.s_prev
        if isfinite(hint)
            track_reference(tr, z0[1], z0[2], z0[3]; s_hint=hint, max_jump=1.0, min_frenet_factor=1e-3)
        else
            s, ey, ep = cartesian_to_frenet(tr, z0[1], z0[2], z0[3])
            (; s, ey, epsi=ep)
        end
    catch err
        err isa Union{DomainError,TrackReferenceError,ArgumentError} || rethrow()
        reset_warm_start!(ctrl)
        return nanres(:invalid_measurement, "measured state invalid: " * sprint(showerror, err))
    end
    s0 = ref.s
    ctrl.s_prev = s0
    # Obstacles: initial collision, active slots (Euclidean reach), relevance window.
    reach = c.obstacle_reach_factor * p.vx_max * N * c.dt
    active = Int[]
    for (i, o) in enumerate(ctrl.obstacles)
        D = required_separation(ctrl, i)
        d = hypot(z0[1] - o.x, z0[2] - o.y)
        if d < D - c.acceptance_tol
            reset_warm_start!(ctrl)
            return nanres(:initial_collision, "measured CoM is $(fmt(d)) m from obstacle $i (D = $(fmt(D)) m)")
        end
        d <= reach + D && push!(active, i)
    end
    relevant = relevant_obstacles(ctrl, s0)
    for i in collect(keys(ctrl.committed))                        # forget passed obstacles
        ctrl.committed[i][2] < s0 - required_separation(ctrl, i) - ctrl.ey_max && delete!(ctrl.committed, i)
    end
    newly = any(r -> !haskey(ctrl.committed, r[1]) && r[2] >= s0 - r[4], relevant)
    t_setup += (time_ns() - t0) / 1e9
    # Initialisation candidates (generated lazily, in order).
    tg = time_ns()
    warm = ctrl.last_U === nothing ? nothing : begin
        Uw = hcat(ctrl.last_U[:, 2:N], ctrl.last_U[:, N])
        try                                                       # tail: pursuit input at the end of the shifted rollout
            Zs, _ = dynamic_rollout(veh, z0, Uw[:, 1:N-1], c.dt; substeps=c.substeps)
            fs = chain_frenet(tr, Zs, s0)
            Uw[:, N] .= pursuit_guess(ctrl, Zs[:, N], fs[1, N])[:, 1]
        catch err
            err isa Union{DomainError,TrackReferenceError} || rethrow()
        end
        Uw
    end
    cands = Any[]
    if isempty(relevant)
        warm === nothing || push!(cands, (:warm, warm, Dict{Int,Symbol}()))
        push!(cands, (:cold, nothing, Dict{Int,Symbol}()))
    elseif newly
        append!(cands, [(g[1], g[2], g[3]) for g in side_sequence_guesses(ctrl, z0, s0, relevant)])
        warm === nothing || push!(cands, (:warm, warm, Dict{Int,Symbol}()))
    else
        warm === nothing || push!(cands, (:warm, warm, Dict{Int,Symbol}()))
        append!(cands, [(g[1], g[2], g[3]) for g in side_sequence_guesses(ctrl, z0, s0, relevant)][1:min(end, 1)])
    end
    t_guess += (time_ns() - tg) / 1e9
    longest = 0.0
    best = nothing
    last = nothing
    total_iter, nsolves = 0, 0
    tstat = :NOT_CALLED
    for (ai, (label, Ug0, sides)) in enumerate(cands)
        if ai > 1 && (time_ns() - t0) / 1e9 + longest > c.time_budget
            push!(attempts, (label, :not_attempted_budget, 0, 0.0))
            continue
        end
        ta = time_ns()
        tg = time_ns()
        Ug = Ug0 === nothing ? (try
            pursuit_guess(ctrl, z0, s0)
        catch err
            err isa Union{DomainError,TrackReferenceError} || rethrow()
            nothing
        end) : Ug0
        local Zg, fr
        ok = Ug !== nothing
        if ok
            try
                Zg, _ = dynamic_rollout(veh, z0, Ug, c.dt; substeps=c.substeps)
                fr = chain_frenet(tr, Zg, s0)
            catch err
                err isa Union{DomainError,TrackReferenceError} || rethrow()
                ok = false
            end
        end
        t_guess += (time_ns() - tg) / 1e9
        if !ok
            push!(attempts, (label, :initialization_failed, 0, (time_ns() - ta) / 1e9))
            continue
        end
        asg = cartesian_assignment(tr, fr[1, :]; knot_tol=c.knot_tol)
        tsu = time_ns()
        install_dynamic!(ctrl, z0, s0, uprev, Zg, substep_states(ctrl, z0, Ug), Ug, fr, asg.frames, active)
        t_setup += (time_ns() - tsu) / 1e9
        st, solves = :solver_failure, 0
        Z = fr2 = Uv = path = nothing
        objv = NaN
        seen = [copy(asg.seg)]                                    # assignments tried in this attempt
        pinned = Set{Int}()
        for re in 0:c.max_reassignments
            if re > 0 && (time_ns() - t0) / 1e9 + longest > c.time_budget
                st = :segment_assignment_invalid
                break
            end
            ts = time_ns()
            optimize!(ctrl.model)
            dt_s = (time_ns() - ts) / 1e9
            t_solve += dt_s
            longest = max(longest, dt_s)
            solves += 1
            tstat = Symbol(termination_status(ctrl.model))
            total_iter += try
                Int(barrier_iterations(ctrl.model))
            catch
                0
            end
            if !(termination_status(ctrl.model) == MOI.LOCALLY_SOLVED && primal_status(ctrl.model) == MOI.FEASIBLE_POINT)
                st = :solver_failure
                break
            end
            Z, fr2, Uv, path = extract_dynamic(ctrl, z0, s0)
            objv = objective_value(ctrl.model)                    # read now: a reassignment modifies the model
            Uv[1, :] .= clamp.(Uv[1, :], -p.delta_max, p.delta_max)
            Uv[2, :] .= clamp.(Uv[2, :], p.a_min, p.a_max)
            frames_now = [segment_frame(tr, id) for id in asg.seg]
            if all(frame_valid(tr, frames_now[j+1], fr2[1, j+1], c.knot_tol) for j in 1:N)
                st = :solved
                break
            end
            # Reassign the knots to the segments that contain their progress, re-solve from the solution.
            new = cartesian_assignment(tr, fr2[1, :]; knot_tol=c.knot_tol)
            if any(==(new.seg), seen)
                # Assignment cycle: a knot near a boundary lands in the other segment in each
                # segment's frame. Keep the current assignment and bound the alternating knots'
                # progress to their segments. The frame formulas are exact on the closed
                # interval, so the re-solve is a valid restriction of the same problem.
                union!(pinned, (j for j in 1:N if new.seg[j+1] != asg.seg[j+1]))
            else
                push!(seen, copy(new.seg))
                asg = new
            end
            frT = try
                chain_frenet(tr, Z, s0)
            catch err
                err isa TrackReferenceError || rethrow()
                st = :segment_assignment_invalid
                break
            end
            install_dynamic!(ctrl, z0, s0, uprev, Z, substep_states(ctrl, z0, Uv), Uv, frT, asg.frames, active; pinned)
            st = :segment_assignment_invalid
        end
        nsolves += solves
        res = (; label, sides, st, Z, fr=fr2, U=Uv, path, obj=objv, val=nothing)
        if st === :solved
            tv = time_ns()
            val = validate_dynamic_plan(ctrl, z0, s0, Uv, Z)
            t_val += (time_ns() - tv) / 1e9
            res = merge(res, (; val, st=isempty(val.violations) ? :success : :validation_failed))
        end
        push!(attempts, (label, res.st, solves, (time_ns() - ta) / 1e9))
        last = res
        if res.st === :success
            best = res
            break
        end
    end
    chosen = best !== nothing ? best : last
    if chosen === nothing
        reset_warm_start!(ctrl)
        timing = (; total=(time_ns() - t0) / 1e9, setup=t_setup, guesses=t_guess, solve=t_solve, validation=t_val)
        return DynamicNMPCResult(:solver_failure, false, nothing, tstat, total_iter, nsolves, NaN, fill(NaN, 6, N + 1), fill(NaN, 3, N + 1),
            fill(NaN, 2, N), fill(NaN, 2, N * c.substeps + 1), (;), String[], active, Dict{Int,Symbol}(), attempts, timing,
            "no initialisation attempt could be solved")
    end
    accepted = chosen.st === :success
    sides = Dict{Int,Symbol}()
    if accepted
        ctrl.last_U = chosen.U
        # Passing side of each obstacle as the accepted plan actually passes it: the side of the
        # track tangent on which the planned path's closest approach lies. The initial guess's
        # label is only a fallback while the plan ends before the obstacle (Ipopt may converge to
        # the other side, so the label alone could contradict the plan that is applied).
        for (i, so, eyo, D) in relevant
            so >= s0 - D || continue
            o = ctrl.obstacles[i]
            j = argmin([hypot(chosen.path[1, q] - o.x, chosen.path[2, q] - o.y) for q in axes(chosen.path, 2)])
            if j < size(chosen.path, 2)
                psi = frenet_to_cartesian(ctrl.track, so, 0.0, 0.0)[3]
                sd = cos(psi) * (chosen.path[2, j] - o.y) - sin(psi) * (chosen.path[1, j] - o.x) >= 0 ? :left : :right
            else
                sd = get(chosen.sides, i, haskey(ctrl.committed, i) ? ctrl.committed[i][1] : nothing)
                sd === nothing && continue
            end
            ctrl.committed[i] = (sd, so)
            sides[i] = sd
        end
    else
        reset_warm_start!(ctrl)
    end
    msg = accepted ? "" : chosen.st === :validation_failed ? join(chosen.val.violations, "; ") :
          chosen.st === :segment_assignment_invalid ? "a knot stayed outside its assigned segment after $(c.max_reassignments) reassignments" :
          "Ipopt returned $(tstat)"
    length(attempts) > 1 && (msg *= (isempty(msg) ? "" : " ") * "[attempts: " * join(["$(a[1]) → $(a[2])" for a in attempts], ", ") * "]")
    status = accepted ? :success : chosen.st
    val = chosen.val === nothing ? (;) : chosen.val
    Z = chosen.Z === nothing ? fill(NaN, 6, N + 1) : chosen.Z
    fr = chosen.fr === nothing ? fill(NaN, 3, N + 1) : chosen.fr
    U = chosen.U === nothing ? fill(NaN, 2, N) : chosen.U
    path = chosen.path === nothing ? fill(NaN, 2, N * c.substeps + 1) : chosen.path
    timing = (; total=(time_ns() - t0) / 1e9, setup=t_setup, guesses=t_guess, solve=t_solve, validation=t_val)
    return DynamicNMPCResult(status, accepted, accepted ? SVector(U[1, 1], U[2, 1]) : nothing, tstat, total_iter, nsolves,
        chosen.obj, Z, fr, U, path, val, chosen.val === nothing ? String[] : chosen.val.violations, active, sides, attempts, timing, msg)
end
