# Linearized MPC on multi-curvature tracks with a stationary circular obstacle
# (Milestone 4BC): local affine track coordinates, supporting-halfspace collision
# constraints, passing-side initialisation and nonlinear validation on the actual track.
# See docs/linearized_multicurvature_collision_avoidance.md.

"Raised when a Cartesian point has no valid, unambiguous local track reference."
struct TrackReferenceError <: Exception
    msg::String
end
Base.showerror(io::IO, e::TrackReferenceError) = print(io, "TrackReferenceError: ", e.msg)

"""
    track_reference(track, X, Y, psi; s_hint, max_jump=1.0, max_lateral=track.width,
                    min_frenet_factor=0.2) -> NamedTuple

Local track reference of the Cartesian pose `(X, Y, psi)`, found near the unwrapped
progress hint `s_hint`.

**Candidates.** Every segment's orthogonal projection with `|ey| ≤ max_lateral`, unwrapped
to the lap nearest the hint, with `|s - s_hint| ≤ max_jump`. A `TrackReferenceError` is
thrown (never a placeholder) if:
- there is no candidate (invalid);
- two distinct candidates exist (ambiguous). Projections on both sides of a segment
  boundary that coincide are one candidate;
- `1 - kappa·ey < min_frenet_factor` (too close to the centre of curvature).

**Curvature convention (one-sided).** `kappa` is the curvature of the segment that
contains `s` under the half-open convention `[s_i, s_{i+1})`: exactly at a boundary, the
downstream segment.

Returns `(; s, ey, epsi, kappa, factor, t, n, psic, xc, yc)`:
- `s`: unwrapped progress;
- `factor = 1 - kappa·ey`;
- `t`, `n`: the unit tangent and left normal of the centreline at `s`;
- `psic`: the unwrapped centreline heading;
- `epsi = psi - psic`, wrapped to `(-π, π]`.
"""
function track_reference(tr::Track, X::Real, Y::Real, psi::Real; s_hint::Real, max_jump::Real=1.0,
    max_lateral::Real=tr.width, min_frenet_factor::Real=0.2, tol::Real=1e-9)
    all(isfinite, (X, Y, psi, s_hint)) || throw(TrackReferenceError("non-finite pose or progress hint"))
    L = track_length(tr)
    cands = Tuple{Float64,Float64}[]
    for (i, seg) in enumerate(tr.segments)
        proj = project_onto_segment(seg, tr.poses[i], X, Y, tol)
        proj === nothing && continue
        d, ey = proj
        abs(ey) <= max_lateral + tol || continue
        s = tr.s_start[i] + d
        tr.closed && (s = s_hint + rem(s - s_hint, L, RoundNearest))
        abs(s - s_hint) <= max_jump || continue
        any(c -> abs(c[1] - s) <= 1e-6 && abs(c[2] - ey) <= 1e-6, cands) || push!(cands, (s, ey))
    end
    isempty(cands) && throw(TrackReferenceError("no centreline projection with |ey| ≤ $(fmt(max_lateral)) m within " *
                                                "$(fmt(max_jump)) m of progress $(fmt(s_hint)) m for ($(fmt(X)), $(fmt(Y)))"))
    length(cands) == 1 || throw(TrackReferenceError("ambiguous projection near progress $(fmt(s_hint)) m: candidates " *
                                                    join(["(s = $(fmt(c[1])), ey = $(fmt(c[2])))" for c in cands], ", ")))
    s, ey = cands[1]
    tr.closed || (s = clamp(s, 0.0, L))
    kappa = curvature(tr, s)
    f = 1 - kappa * ey
    f >= min_frenet_factor || throw(TrackReferenceError("1 - kappa·ey = $(fmt(f)) < $(min_frenet_factor) at progress $(fmt(s)) m"))
    xc, yc, psic = centerline_pose(tr, s)
    return (; s, ey, epsi=rem2pi(psi - psic, RoundNearest), kappa, factor=f, t=(cos(psic), sin(psic)), n=(-sin(psic), cos(psic)),
        psic, xc, yc)
end

"""
    trajectory_references(controller, Z, s0) -> Vector

[`track_reference`](@ref) of every column of the Cartesian trajectory `Z`, chained by
progress hints:
- the first knot uses the measured progress `s0`;
- each later knot uses the previous reference's progress plus
  `|Δp| cos(epsi)/(1 - kappa ey)`.

Uses the controller's `max_projection_jump` and `min_frenet_factor`.
"""
function trajectory_references(ctrl::LinearMPCController, Z::AbstractMatrix, s0::Real)
    c, tr = ctrl.config, ctrl.track
    refs = Vector{Any}(undef, size(Z, 2))
    hint = Float64(s0)
    for k in axes(Z, 2)
        if k > 1
            r = refs[k-1]
            hint = r.s + hypot(Z[1, k] - Z[1, k-1], Z[2, k] - Z[2, k-1]) * cos(r.epsi) / r.factor
        end
        refs[k] = track_reference(tr, Z[1, k], Z[2, k], Z[3, k]; s_hint=hint, max_jump=c.max_projection_jump,
            min_frenet_factor=c.min_frenet_factor)
    end
    return refs
end

"""
    validate_plan_on_track(vehicle, track, limits, z0, U; s0, substeps=10, tol=1e-4,
                           max_jump=1.0, obstacle=nothing, D=NaN, Dc=NaN,
                           check_samples=50, alphas=Float64[], acceptance=:collision) -> NamedTuple

Independent nonlinear validation of an input sequence from the Cartesian state `z0`
(measured progress `s0`) on any track. It is used for both the linear MPC and the
nonlinear MPC plans.

**Rollout.** Propagate with the exact Cartesian map, and project every knot and every
`substeps` sample per interval onto the actual track ([`track_reference`](@ref), chained
progress hints, lap-unwrapped).

**Checks**, with tolerance `tol`:
- the planning corridor `|ey| ≤ width/2 - margin` and `|epsi| ≤ epsi_max` at the knots
  `k ≥ 2`;
- the physical track `|ey| ≤ width/2` at every sample;
- a valid, unambiguous projection everywhere. A sample without one (e.g. far off the
  track) is a violation;
- the speed at the knots, with tolerance `speed_tol` (the linear controller passes the
  plant tolerance `acceptance_tol`: the speed limit is a physical operating limit);
- the input limits;
- the exact `|v² tan(delta)/L| ≤ ay_max` at both interval end points.

**Obstacle** (if `obstacle` is given): `D` is the required separation and `Dc` the contact
distance (ego radius plus obstacle radius).
- *Constrained points* (knots `k ≥ 2` and the interior fractions `alphas`): a distance
  below `D - tol` is a violation.
- *Dense evaluation* (`check_samples` per interval plus the final knot): a distance below
  `Dc - tol` (contact) is a violation. A distance below `D - tol` is counted as a
  separation violation, and is a violation only with `acceptance = :separation`.

`limits = (; dt, track_margin, epsi_max, ay_max)`. Returns `(; Zroll, frenet, violations,
max_abs_ey, max_abs_ay, min_track_clearance, min_corridor_clearance, progress, obstacle,
excess)`:
- `excess` is the largest relative constraint excess: corridor `/E`, heading
  `/epsi_max`, physical track `/(width/2)`, speed `/v_max`, `ay /ay_max`, constrained-point
  separation `/D` (0 if all hold; `Inf` without a valid projection). The local model's
  `:trust_region` step policy uses it to compare linearisation points;
- `frenet` is the 3×(N+1) true `(s, ey, epsi)` of the rollout knots (NaN after a failed
  projection);
- `obstacle` is `(; min_distance, constrained_min_distance, collisions,
  separation_violations, first_violation_time)`, or `nothing`.
"""
function validate_plan_on_track(veh::KinematicBicycle, track::Track, limits, z0::AbstractVector, U::AbstractMatrix; s0::Real,
    substeps::Integer=10, tol::Real=1e-4, max_jump::Real=1.0, obstacle=nothing, D::Real=NaN, Dc::Real=NaN, check_samples::Integer=50,
    alphas=Float64[], acceptance::Symbol=:collision, speed_tol::Real=tol)
    p, L, N, dt = veh.params, veh.params.wheelbase, size(U, 2), limits.dt
    W, E = track.width / 2, track.width / 2 - limits.track_margin
    Zr = exact_rollout(veh, z0, U, dt)
    viol = String[]
    excess = 0.0                                          # largest relative constraint excess (see docstring)
    fr = fill(NaN, 3, N + 1)
    maxey, maxay, mintrack, mincorr = 0.0, 0.0, Inf, Inf
    # Chained hints: last projected progress plus the distance travelled since that sample.
    proj(q, psi, hint) = track_reference(track, q[1], q[2], psi; s_hint=hint, max_jump, min_frenet_factor=1e-3)
    lasts, lastq, ok = Float64(s0), (Zr[1, 1], Zr[2, 1]), true
    for k in 1:N
        for j in 0:substeps-1
            q = j == 0 ? (Zr[1, k], Zr[2, k]) : interior_point(L, Zr[:, k], Zr[4, k], U[1, k], U[2, k], j * dt / substeps)
            r = try
                proj(q, Zr[3, k], lasts + hypot(q[1] - lastq[1], q[2] - lastq[2]))
            catch err
                err isa TrackReferenceError || rethrow()
                push!(viol, "no valid track projection at t = $(fmt((k - 1) * dt + j * dt / substeps)) s: $(err.msg)")
                ok = false
                nothing
            end
            r === nothing && break
            j == 0 && (fr[:, k] .= (r.s, r.ey, r.epsi))      # epsi is meaningful at the knots only
            mintrack = min(mintrack, W - abs(r.ey))
            lasts, lastq = r.s, q
        end
        ok || break
    end
    if ok
        r = try
            proj((Zr[1, N+1], Zr[2, N+1]), Zr[3, N+1], lasts + hypot(Zr[1, N+1] - lastq[1], Zr[2, N+1] - lastq[2]))
        catch err
            err isa TrackReferenceError || rethrow()
            push!(viol, "no valid track projection at the final knot: $(err.msg)")
            nothing
        end
        if r === nothing
            ok = false
        else
            fr[:, N+1] .= (r.s, r.ey, r.epsi)
            mintrack = min(mintrack, W - abs(r.ey))
        end
    end
    ok ? (excess = max(excess, -mintrack / W)) : (mintrack = -Inf; excess = Inf)
    for k in 2:N+1
        ey, ep = fr[2, k], fr[3, k]
        if !isnan(ey)
            maxey = max(maxey, abs(ey))
            mincorr = min(mincorr, E - abs(ey))
            excess = max(excess, (abs(ey) - E) / E, (abs(ep) - limits.epsi_max) / limits.epsi_max)
            abs(ey) > E + tol && push!(viol, "knot $k: |ey| = $(fmt(abs(ey))) m > planning corridor $(fmt(E)) m")
            abs(ep) > limits.epsi_max + tol && push!(viol, "knot $k: |epsi| = $(fmt(abs(ep))) rad > $(fmt(limits.epsi_max)) rad")
        end
        excess = max(excess, (Zr[4, k] - p.v_max) / p.v_max, (p.v_min - Zr[4, k]) / p.v_max)
        (p.v_min - speed_tol <= Zr[4, k] <= p.v_max + speed_tol) || push!(viol, "knot $k: speed $(fmt(Zr[4, k])) m/s outside [$(p.v_min), $(p.v_max)]")
    end
    mintrack < -tol && isfinite(mintrack) && push!(viol, "physical track edge crossed along the rollout (clearance $(fmt(mintrack)) m)")
    for k in 1:N
        (abs(U[1, k]) <= p.delta_max + tol && p.a_min - tol <= U[2, k] <= p.a_max + tol) ||
            push!(viol, "stage $k: input $(U[:, k]) outside the actuator limits")
        if limits.ay_max !== nothing
            for v in (Zr[4, k], Zr[4, k+1])
                ay = abs(v^2 * tan(U[1, k]) / L)
                maxay = max(maxay, ay)
                excess = max(excess, (ay - limits.ay_max) / limits.ay_max)
                ay > limits.ay_max + tol && push!(viol, "stage $k: |ay| = $(fmt(ay)) m/s² > $(limits.ay_max) m/s²")
            end
        end
    end
    obrep = nothing
    if obstacle !== nothing
        dist(q) = hypot(q[1] - obstacle.x, q[2] - obstacle.y)
        cmin = Inf
        for k in 1:N
            cmin = min(cmin, dist(Zr[:, k+1]))
            for a in alphas
                cmin = min(cmin, dist(interior_point(L, Zr[:, k], Zr[4, k], U[1, k], U[2, k], a * dt)))
            end
        end
        md, nsep, ncol, tfirst = Inf, 0, 0, NaN
        for k in 1:N, j in (k == N ? (0:check_samples) : (0:check_samples-1))
            tau = j * dt / check_samples
            q = j == 0 ? Zr[:, k] : interior_point(L, Zr[:, k], Zr[4, k], U[1, k], U[2, k], tau)
            d = dist(q)
            md = min(md, d)
            if d < D - tol
                nsep += 1
                isnan(tfirst) && (tfirst = (k - 1) * dt + tau)
            end
            d < Dc - tol && (ncol += 1)
        end
        excess = max(excess, (D - cmin) / D)
        cmin < D - tol && push!(viol, "obstacle: constrained-point distance $(fmt(cmin)) m < D = $(fmt(D)) m")
        ncol > 0 && push!(viol, "obstacle: contact in the dense evaluation (minimum distance $(fmt(md)) m < $(fmt(Dc)) m)")
        acceptance === :separation && nsep > 0 && push!(viol, "obstacle: required separation violated in the dense evaluation (minimum distance $(fmt(md)) m)")
        obrep = (; min_distance=md, constrained_min_distance=cmin, collisions=ncol, separation_violations=nsep, first_violation_time=tfirst)
    end
    return (; Zroll=Zr, frenet=fr, violations=viol, max_abs_ey=maxey, max_abs_ay=maxay, min_track_clearance=mintrack,
        min_corridor_clearance=mincorr, progress=fr[1, N+1] - fr[1, 1], obstacle=obrep, excess=max(excess, 0.0))
end

"Obstacle validation parameters of a linear controller (`nothing` without an obstacle)."
function obstacle_validation_kw(ctrl::LinearMPCController)
    ob, c = ctrl.obstacle, ctrl.config
    ob === nothing && return (; obstacle=nothing)
    return (; obstacle=ob, D=required_separation(ctrl), Dc=c.ego_radius + ob.radius, check_samples=c.obstacle_check_samples,
        alphas=ctrl.obs_alphas, acceptance=c.obstacle_acceptance)
end

validate_local(ctrl::LinearMPCController, s0, z0, U) = validate_plan_on_track(ctrl.vehicle, ctrl.track,
    (; ctrl.config.dt, ctrl.config.track_margin, ctrl.config.epsi_max, ctrl.config.ay_max), z0, U; s0,
    substeps=ctrl.config.validation_substeps, tol=ctrl.config.validation_tol, max_jump=ctrl.config.max_projection_jump,
    speed_tol=ctrl.config.acceptance_tol, obstacle_validation_kw(ctrl)...)

"""
    plan_objective_on_track(weights, vehicle, E, frenet, U, uprev; horizon, dt) -> Float64

The original MPC objective (the NMPC objective, [`NMPCConfig`](@ref) weights) evaluated on
true track coordinates `frenet` (3×(N+1), `(s, ey, epsi)`; for example the projected
exact rollout from [`validate_plan_on_track`](@ref)). `weights` is any configuration with
the fields `ws, qy, qpsi, rdelta, ra, rd_delta, rd_a, epsi_max`. Returns NaN when a
projection is missing.
"""
function plan_objective_on_track(w, veh::KinematicBicycle, E::Real, frenet::AbstractMatrix, U::AbstractMatrix, uprev; horizon::Integer, dt::Real)
    p, N = veh.params, horizon
    s_scale, a_ref = p.v_max * N * dt, max(-p.a_min, p.a_max)
    J = -w.ws * (frenet[1, N+1] - frenet[1, 1]) / s_scale
    for k in 2:N+1
        J += w.qy * (frenet[2, k] / E)^2 + w.qpsi * (frenet[3, k] / w.epsi_max)^2
    end
    for k in 1:N
        dprev, aprev = k == 1 ? (uprev[1], uprev[2]) : (U[1, k-1], U[2, k-1])
        J += w.rdelta * (U[1, k] / p.delta_max)^2 + w.ra * (U[2, k] / a_ref)^2 +
             w.rd_delta * ((U[1, k] - dprev) / p.delta_max)^2 + w.rd_a * ((U[2, k] - aprev) / a_ref)^2
    end
    return J
end

"""
    build_local_qp!(controller)

The QP of the local track model.

**Variables.** In addition to `Z`, `U` and `uprev`:
- the affine track coordinates `S[i]`, `EY[i]`, `EP[i]` of the knots `k = i + 1`;
- a fixed variable `s0` (the measured progress).

**Constraints.**
- *Dynamics and lateral acceleration:* as in the straight model.
- *Track-coordinate rows* (coefficients and right-hand sides updated per QP):

  ```
  S[i]  − (t/(1 − κ ēy))·p_k = s̄ − (t/(1 − κ ēy))·p̄_k
  EY[i] − n·p_k             = ēy − n·p̄_k
  EP[i] − psi_k + κ S[i]     = ēψ − ψ̄_k + κ s̄
  ```

- *Corridor and heading:* `|EY| ≤ E − mismatch_tol[1] − local_backoff.corridor` (backed
  off by the accepted position mismatch and a tightening) and
  `|EP| ≤ epsi_max − local_backoff.heading`.
- *Lateral acceleration:* the linearised `|ay| ≤ ay_max − local_backoff.ay`.
- *With an obstacle:* one halfspace row per knot `k ≥ 2` (coefficients on `p_k`) and per
  interior sample (coefficients on `z_k` and `u_k`).

**Slacks (softened inequalities).** Only the corridor, heading, lateral-acceleration and
obstacle inequalities have a nonnegative slack. The penalty is
`slack_weight · Σ slack²` with the default `slack_penalty = :quadratic`
(`slack_weight · Σ slack` with `:l1`; no slack with `:none`).

**Hard constraints.** These remain hard:
- the linearised dynamics, the fixed measured state, previous input and progress;
- the track-coordinate definitions;
- the variable bounds: trust region ∩ actuator box ∩ speed bounds.

**Feasibility.** A QP is feasible whenever the nominal trajectory satisfies those hard
bounds, because the nominal is then a feasible point. Its inputs lie in the actuator box by
construction; its speeds normally lie in `[v_min, v_max − local_backoff.speed]`. Otherwise
(or numerically) OSQP may report infeasibility or stop at its iteration limit, which is
handled as described in [`sqp_attempt!`](@ref).

**Effect of the slacks.** They keep the QP solvable when the linearisation point violates
a softened constraint and the trust region is small. A softened solution is not
necessarily physically feasible:
- the slack may be nonzero;
- the affine constraints are approximations anyway.

Every candidate is therefore validated on its exact nonlinear rollout. A candidate that
violates a true constraint is rejected, but its rollout can still move the next
linearisation point towards feasibility. The quadratic penalty is inexact (it permits a
violation of about `λ/(2·slack_weight)` for a multiplier `λ`); the `local_backoff`
tightenings absorb this on short steps.

**Objective.** Fixed and convex: the NMPC objective with `S`, `EY` and `EP` in place of the
true coordinates, plus the slack penalty.

Every coefficient that may change later is created with a nonzero placeholder, and OSQP is
set up once (see [`build_qp!`](@ref)).
"""
function build_local_qp!(ctrl::LinearMPCController)
    c, p, N, E = ctrl.config, ctrl.vehicle.params, ctrl.config.horizon, ctrl.ey_max
    m = Model(ctrl.optimizer)
    set_silent(m)
    if ctrl.optimizer === OSQP.Optimizer
        set_attribute(m, "eps_abs", c.local_eps)
        set_attribute(m, "eps_rel", c.local_eps)
        set_attribute(m, "max_iter", c.local_osqp_max_iter)
        set_attribute(m, "polish", c.polish)
        for (k, v) in c.osqp_settings
            set_attribute(m, k, v)
        end
    end
    @variable(m, -1e3 <= Z[1:4, 1:N+1] <= 1e3)
    @variable(m, -1e3 <= U[1:2, 1:N] <= 1e3)
    @variable(m, uprev[1:2])
    @variable(m, S[1:N])
    @variable(m, EY[1:N])
    @variable(m, EP[1:N])
    @variable(m, s0)
    # Slack variables of the approximated inequalities (exact L1 penalty, see the docstring).
    M = length(ctrl.obs_alphas)
    nobs = ctrl.obstacle === nothing ? 0 : (M + 1) * N
    @variable(m, slack[1:2N+2N+nobs] >= 0)
    sey, sep, say, sob = slack[1:N], slack[N+1:2N], slack[2N+1:4N], slack[4N+1:end]
    Eb = E - c.mismatch_tol[1] - c.local_backoff.corridor
    Pb = c.epsi_max - c.local_backoff.heading
    for i in 1:N
        @constraint(m, EY[i] <= Eb + sey[i])
        @constraint(m, EY[i] >= -Eb - sey[i])
        @constraint(m, EP[i] <= Pb + sep[i])
        @constraint(m, EP[i] >= -Pb - sep[i])
    end
    for i in 1:4
        fix(Z[i, 1], 0.0; force=true)
    end
    fix.(uprev, 0.0)
    fix(s0, 0.0)
    dyn = Matrix{ConstraintRef}(undef, 4, N)
    for k in 1:N, i in 1:4
        dyn[i, k] = @constraint(m, Z[i, k+1] - sum(Z[j, k] for j in 1:4) - sum(U[j, k] for j in 1:2) == 0)
    end
    con_s = [@constraint(m, S[i] - Z[1, i+1] - Z[2, i+1] == 0) for i in 1:N]
    con_ey = [@constraint(m, EY[i] - Z[1, i+1] - Z[2, i+1] == 0) for i in 1:N]
    con_ep = [@constraint(m, EP[i] - Z[3, i+1] + S[i] == 0) for i in 1:N]
    ay_hi = Matrix{ConstraintRef}(undef, N, 2)
    ay_lo = Matrix{ConstraintRef}(undef, N, 2)
    if c.ay_max !== nothing
        for k in 1:N, (j, kv) in enumerate((k, k + 1))
            ay_hi[k, j] = @constraint(m, Z[4, kv] + U[1, k] - say[2k-2+j] <= c.ay_max)
            ay_lo[k, j] = @constraint(m, Z[4, kv] + U[1, k] + say[2k-2+j] >= -c.ay_max)
        end
    end
    obs = Matrix{ConstraintRef}(undef, ctrl.obstacle === nothing ? 0 : M + 1, N)
    if ctrl.obstacle !== nothing
        for k in 1:N
            obs[1, k] = @constraint(m, Z[1, k+1] + Z[2, k+1] + sob[(k-1)*(M+1)+1] >= 0)
            for j in 1:M
                obs[j+1, k] = @constraint(m, sum(Z[i, k] for i in 1:4) + U[1, k] + U[2, k] + sob[(k-1)*(M+1)+1+j] >= 0)
            end
        end
    end
    s_scale = p.v_max * N * c.dt
    a_ref = max(-p.a_min, p.a_max)
    J = -c.ws * (S[N] - s0) / s_scale
    if c.slack_penalty === :l1
        J += c.slack_weight * sum(slack)
    elseif c.slack_penalty === :quadratic
        J += c.slack_weight * sum(slack .^ 2)
    else
        fix.(slack, 0.0; force=true)                     # :none: hard constraints
    end
    for i in 1:N
        J += c.qy * (EY[i] / E)^2 + c.qpsi * (EP[i] / c.epsi_max)^2
    end
    for k in 1:N
        dprev, aprev = k == 1 ? (uprev[1], uprev[2]) : (U[1, k-1], U[2, k-1])
        J += c.rdelta * (U[1, k] / p.delta_max)^2 + c.ra * (U[2, k] / a_ref)^2 +
             c.rd_delta * ((U[1, k] - dprev) / p.delta_max)^2 + c.rd_a * ((U[2, k] - aprev) / a_ref)^2
    end
    @objective(m, Min, J)
    if ctrl.optimizer === OSQP.Optimizer
        set_attribute(m, "max_iter", 1)
        optimize!(m)
        set_attribute(m, "max_iter", c.local_osqp_max_iter)
    end
    ctrl.model, ctrl.Z, ctrl.U, ctrl.uprev, ctrl.dyn, ctrl.ay_hi, ctrl.ay_lo = m, Z, U, uprev, dyn, ay_hi, ay_lo
    ctrl.S, ctrl.EY, ctrl.EP, ctrl.s0var, ctrl.slack = S, EY, EP, s0, slack
    ctrl.con_s, ctrl.con_ey, ctrl.con_ep, ctrl.obs = con_s, con_ey, con_ep, obs
    return ctrl
end

"""
    obstacle_direction(pbar, obstacle, ref_normal, side; min_distance) -> (n, degenerate)

Unit outward direction of the supporting halfspace `n·(p - o) ≥ D` for a nominal point
`pbar`:
- the radial direction `(pbar - o)/|pbar - o|` if `|pbar - o| ≥ min_distance`;
- otherwise (degenerate: at or too close to the centre) the track's left normal at the
  nominal reference, signed by the passing side (`:right` gives `-n`; `:left` and `:none`
  give `+n`). A zero vector is never normalised.

Any unit `n` gives a valid inner approximation of the collision-free set: every point with
`n·(p - o) ≥ D` lies outside the disk of radius `D`.
"""
function obstacle_direction(pbar, ob::CircleObstacle, ref_normal, side::Symbol; min_distance::Real)
    dx, dy = pbar[1] - ob.x, pbar[2] - ob.y
    d = hypot(dx, dy)
    d >= min_distance && return (dx / d, dy / d), false
    sg = side === :right ? -1.0 : 1.0
    return (sg * ref_normal[1], sg * ref_normal[2]), true
end

"""
    interior_jacobian(L, z, u, tau) -> (pbar, J)

Position `pbar` of the exact partial step of duration `tau` from the Cartesian state `z`
with input `u` ([`interior_point`](@ref)), and its Jacobian `J = ∂p/∂(z, u)` (2×6) by
ForwardDiff. Linearisation: `p(tau) ≈ pbar + J·((z, u) - (z̄, ū))`.
"""
function interior_jacobian(L, z::AbstractVector, u::AbstractVector, tau::Real)
    f = w -> collect(interior_point(L, (w[1], w[2], w[3]), w[4], w[5], w[6], tau))
    w = vcat(collect(Float64, z), collect(Float64, u))
    return f(w), ForwardDiff.jacobian(f, w)
end

"""
    install_track_rows!(controller, refs, Zb, Ub, side) -> (degenerate halfspaces, halfspace directions)

Update the track-coordinate rows from the nominal references `refs` (knots 1…N+1). With an
obstacle, also update the halfspace rows at the nominal trajectory `(Zb, Ub)`, with the
right-hand side `D + mismatch_tol[1] + local_backoff.obstacle + n·o` (backed off by the
accepted position mismatch and the obstacle back-off), and cache their linearisations in
`controller.lin_cache`.
"""
function install_track_rows!(ctrl::LinearMPCController, refs, Zb, Ub, side::Symbol)
    c, N, L = ctrl.config, ctrl.config.horizon, ctrl.vehicle.params.wheelbase
    for i in 1:N
        k, r = i + 1, refs[i+1]
        a = (r.t[1] / r.factor, r.t[2] / r.factor)
        set_normalized_coefficient(ctrl.con_s[i], ctrl.Z[1, k], -a[1])
        set_normalized_coefficient(ctrl.con_s[i], ctrl.Z[2, k], -a[2])
        set_normalized_rhs(ctrl.con_s[i], r.s - a[1] * Zb[1, k] - a[2] * Zb[2, k])
        set_normalized_coefficient(ctrl.con_ey[i], ctrl.Z[1, k], -r.n[1])
        set_normalized_coefficient(ctrl.con_ey[i], ctrl.Z[2, k], -r.n[2])
        set_normalized_rhs(ctrl.con_ey[i], r.ey - r.n[1] * Zb[1, k] - r.n[2] * Zb[2, k])
        set_normalized_coefficient(ctrl.con_ep[i], ctrl.S[i], r.kappa)
        set_normalized_rhs(ctrl.con_ep[i], r.epsi - Zb[3, k] + r.kappa * r.s)
    end
    ob = ctrl.obstacle
    ob === nothing && return 0, Tuple{Float64,Float64}[]
    D = required_separation(ctrl) + c.mismatch_tol[1] + c.local_backoff.obstacle
    ndeg, dirs = 0, Tuple{Float64,Float64}[]
    cache = Vector{Any}(undef, N)
    for k in 1:N
        n, deg = obstacle_direction((Zb[1, k+1], Zb[2, k+1]), ob, refs[k+1].n, side; min_distance=c.obstacle_min_direction)
        ndeg += deg
        push!(dirs, n)
        con = ctrl.obs[1, k]
        set_normalized_coefficient(con, ctrl.Z[1, k+1], n[1])
        set_normalized_coefficient(con, ctrl.Z[2, k+1], n[2])
        set_normalized_rhs(con, D + n[1] * ob.x + n[2] * ob.y)
        samples = Any[]
        for (j, a) in enumerate(ctrl.obs_alphas)
            pbar, J = interior_jacobian(L, Zb[:, k], Ub[:, k], a * c.dt)
            n, deg = obstacle_direction(pbar, ob, refs[k].n, side; min_distance=c.obstacle_min_direction)
            ndeg += deg
            push!(dirs, n)
            g = J' * collect(n)                                   # 6 coefficients on (z_k, u_k)
            w = vcat(Zb[:, k], Ub[:, k])
            con = ctrl.obs[j+1, k]
            for i in 1:4
                set_normalized_coefficient(con, ctrl.Z[i, k], g[i])
            end
            set_normalized_coefficient(con, ctrl.U[1, k], g[5])
            set_normalized_coefficient(con, ctrl.U[2, k], g[6])
            set_normalized_rhs(con, D + n[1] * ob.x + n[2] * ob.y - n[1] * pbar[1] - n[2] * pbar[2] + g' * w)
            push!(samples, (pbar, J, w))
        end
        cache[k] = samples
    end
    ctrl.lin_cache = cache
    return ndeg, dirs
end

"""
    interior_mismatch(controller, Zq, Uq, Zroll) -> Float64

Largest distance between the affine interior-sample positions of the QP solution
(`pbar + J·(w - w̄)`, from the cached halfspace linearisation) and the exact interior
positions of the rollout. It is zero without an obstacle or interior samples.
"""
function interior_mismatch(ctrl::LinearMPCController, Zq, Uq, Zroll)
    (ctrl.obstacle === nothing || isempty(ctrl.obs_alphas) || ctrl.lin_cache === nothing) && return 0.0
    L, dt, worst = ctrl.vehicle.params.wheelbase, ctrl.config.dt, 0.0
    for k in 1:ctrl.config.horizon, (j, a) in enumerate(ctrl.obs_alphas)
        pbar, J, wbar = ctrl.lin_cache[k][j]
        pq = pbar + J * (vcat(Zq[:, k], Uq[:, k]) - wbar)
        pe = interior_point(L, Zroll[:, k], Zroll[4, k], Uq[1, k], Uq[2, k], a * dt)
        worst = max(worst, hypot(pq[1] - pe[1], pq[2] - pe[2]))
    end
    return worst
end

"""
    pursuit_input_local(controller, z, ref) -> (delta, a)

Nominal input at the Cartesian state `z` with track reference `ref`, on any track.

**Steering.** Pure pursuit towards the actual centreline point at progress `s + ld`.
- The look-ahead is `ld = clamp(0.5 v, 0.5, 2) m`, limited to the smallest radius of
  curvature within 2 m.
- The target lies on the curved centreline. On a constant-curvature arc, from a pose on
  the centreline, this gives `delta = atan(L kappa)`: the curvature feedforward. Off the
  centreline it adds the feedback correction.
- The steering is clipped to the steering limit and to `|ay| ≤ ay_max` at the current
  speed.

**Acceleration.** `(v_target - v)/dt`, clipped to the actuator limits. `v_target` is
`v_max`, capped over the next `v + 0.5` m by `sqrt(0.9 ay_max/|kappa|)`, with braking room
at 80 % of `|a_min|`. This is the speed cap of the obstacle side guesses.
"""
function pursuit_input_local(ctrl::LinearMPCController, z::AbstractVector, ref)
    c, p, tr = ctrl.config, ctrl.vehicle.params, ctrl.track
    L, v, s = p.wheelbase, z[4], ref.s
    smax = tr.closed ? Inf : track_length(tr)
    kap(ss) = curvature(tr, min(ss, smax))
    kmax = maximum(abs(kap(s + ds)) for ds in 0.0:0.1:2.0)
    ld = clamp(0.5v, 0.5, 2.0)
    kmax > 0 && (ld = min(ld, max(0.5, 1 / kmax)))
    pt = centerline_position(tr, min(s + ld, smax))
    alpha = rem2pi(atan(pt[2] - z[2], pt[1] - z[1]) - z[3], RoundNearest)
    dmax = c.ay_max === nothing ? p.delta_max : min(p.delta_max, atan(c.ay_max * L / max(v, 0.1)^2))
    delta = clamp(atan(2L * sin(alpha) / ld), -dmax, dmax)
    vt = p.v_max
    if c.ay_max !== nothing
        for ds in 0.0:0.1:(v + 0.5)
            k = kap(s + ds)
            k == 0 && continue
            vt = min(vt, sqrt(0.9 * c.ay_max / abs(k) + 2 * 0.8 * abs(p.a_min) * ds))
        end
    end
    return delta, clamp((vt - v) / c.dt, p.a_min, p.a_max)
end

"""
    nominal_inputs_local(controller, z0, s0) -> U (2×N)

Cold-start inputs on any track: [`pursuit_input_local`](@ref) along an exact rollout from
`z0`, with chained projections (first hint: the measured progress `s0`). The nominal
trajectory is dynamically consistent.
"""
function nominal_inputs_local(ctrl::LinearMPCController, z0::AbstractVector, s0::Real)
    c, N, tr = ctrl.config, ctrl.config.horizon, ctrl.track
    U = zeros(2, N)
    z = collect(Float64, z0)
    ref = track_reference(tr, z[1], z[2], z[3]; s_hint=s0, max_jump=c.max_projection_jump, min_frenet_factor=c.min_frenet_factor)
    for k in 1:N
        U[:, k] .= pursuit_input_local(ctrl, z, ref)
        zn = collect(exact_cartesian_step(ctrl.vehicle, z, U[:, k], c.dt))
        hint = ref.s + hypot(zn[1] - z[1], zn[2] - z[2]) * cos(ref.epsi) / ref.factor
        z = zn
        k < N && (ref = track_reference(tr, z[1], z[2], z[3]; s_hint=hint, max_jump=c.max_projection_jump, min_frenet_factor=c.min_frenet_factor))
    end
    return U
end

"""
    shifted_inputs_local(controller, z0, s0) -> U (2×N)

Warm start on any track: the previous accepted inputs shifted by one stage, with the last
stage from [`pursuit_input_local`](@ref) at the end of the shifted rollout
(`warm_tail = :pursuit`), or the previous last input (`:repeat`).
"""
function shifted_inputs_local(ctrl::LinearMPCController, z0::AbstractVector, s0::Real)
    c, N, Up = ctrl.config, ctrl.config.horizon, ctrl.last_U
    U = hcat(Up[:, 2:N], Up[:, N])
    if c.warm_tail === :pursuit
        Zs = exact_rollout(ctrl.vehicle, z0, U[:, 1:N-1], c.dt)
        refs = trajectory_references(ctrl, Zs, s0)
        U[:, N] .= pursuit_input_local(ctrl, Zs[:, N], refs[N])
    end
    return U
end

"""
    default_nominal_local(controller, z0, s0) -> (U, excess)

Nominal inputs of an ordinary cycle of the local model, with the relative constraint excess
of their exact rollout (see [`validate_plan_on_track`](@ref)).
- **Cold** (no warm start): [`nominal_inputs_local`](@ref).
- **Warm:** the shifted previous inputs ([`shifted_inputs_local`](@ref)). If their rollout
  violates a constraint by more than `trust_memory_excess`, the cold pursuit nominal is
  also evaluated, and the one with the smaller excess is used.

A plan that is valid over its own horizon can end in a state with no feasible
continuation, for example too fast for the next bend: the MPC has no terminal constraint.
The shifted plan then leaves the corridor in its last stages. The cold pursuit nominal
respects the curvature speed cap, so it is often the better linearisation point in that
case.
"""
function default_nominal_local(ctrl::LinearMPCController, z0::AbstractVector, s0::Real)
    c = ctrl.config
    (c.warm_start && ctrl.last_U !== nothing) || return nominal_inputs_local(ctrl, z0, s0), nothing
    Uw = shifted_inputs_local(ctrl, z0, s0)
    ew = validate_local(ctrl, s0, z0, Uw).excess
    ew <= c.trust_memory_excess && return Uw, ew
    Uc = try
        nominal_inputs_local(ctrl, z0, s0)
    catch err
        err isa TrackReferenceError || rethrow()
        return Uw, ew
    end
    ec = validate_local(ctrl, s0, z0, Uc).excess
    return ec < ew ? (Uc, ec) : (Uw, ew)
end

"""
    set_obstacle!(controller::LinearMPCController, obstacle)

Replace the stationary obstacle of a linear controller built with one (its rows already
exist), and forget the passing side.
"""
function set_obstacle!(ctrl::LinearMPCController, ob::CircleObstacle)
    ctrl.obstacle === nothing && throw(ArgumentError("the controller was built without an obstacle (pass obstacle = ... to LinearMPCController)"))
    required_separation(ob, ctrl.config.ego_radius, ctrl.config.obstacle_buffer)
    ctrl.obstacle = ob
    ctrl.obstacle_side, ctrl.obstacle_encounter = :none, NaN
    return ctrl
end

"Side of the obstacle on which a plan with true track coordinates `frenet` passes."
function plan_side_local(ctrl::LinearMPCController, x, frenet)
    so, eyo = obstacle_frenet(ctrl, x)
    (isnan(so) || any(isnan, frenet[1, :])) && return :none
    k = argmin(abs.(frenet[1, :] .- so))
    return frenet[2, k] >= eyo ? :left : :right
end

const LINEAR_OBSTACLE_NONE = (; present=false, relevant=false, side=:none, attempts=Tuple{Symbol,Symbol,Int,Float64}[],
    guesses=Tuple{Symbol,Float64,Float64}[], required=NaN, collision_distance=NaN, min_distance=NaN, physical_clearance=NaN,
    buffer_clearance=NaN, constrained_clearance=NaN, collisions=0, separation_violations=0, degenerate_halfspaces=0,
    failure_cause=:none, association=(NaN, NaN))

"""
Classify a cycle in which no attempt was accepted (obstacle controller):
- `:infeasible_geometry` if the lateral-gap heuristic ([`passage_sides`](@ref)) finds no
  passage on either side (geometrically blocked, by that heuristic);
- `:collision` if a candidate made contact in the dense evaluation;
- `:obstacle_constraint` if a constrained point violated the required separation;
- `:qp_infeasible` if every QP was reported infeasible (the affine constraints within the
  trust region, which is not a proof of infeasibility of the original problem);
- `:invalid_reference`, `:qp_failure`, `:not_converged`, `:budget` (no attempt could be
  started) or `:other`.

Cycles rejected before any attempt report `:initial_collision` or `:invalid_measurement`.
"""
function linear_failure_cause(atts, pl=true, pr=true)
    isempty(atts) && return :budget
    !(pl || pr) && return :infeasible_geometry
    viol = reduce(vcat, [a.val === nothing ? String[] : a.val.violations for a in atts]; init=String[])
    any(v -> occursin("obstacle: contact", v), viol) && return :collision
    any(v -> occursin("obstacle: constrained-point", v), viol) && return :obstacle_constraint
    sts = [a.status for a in atts]
    all(a -> a.status === :qp_failure && !isempty(a.qst) && a.qst[end] in (:INFEASIBLE, :LOCALLY_INFEASIBLE), atts) && return :qp_infeasible
    all(==(:invalid_reference), sts) && return :invalid_reference
    any(==(:not_converged), sts) && return :not_converged
    any(==(:qp_failure), sts) && return :qp_failure
    any(==(:invalid_reference), sts) && return :invalid_reference
    return :other
end

"""
    solve_local!(controller, x, z0, uprev, t0) -> LinearMPCResult

One cycle of the local track model (curved tracks and/or a stationary obstacle). Each
*attempt* is a bounded successive-linearisation run ([`sqp_attempt!`](@ref)) from one
nominal input sequence. The time budget `sqp_time_budget` covers all attempts of the cycle.

**Without an obstacle, or when it is not relevant** to this horizon (or behind the vehicle;
[`obstacle_associations`](@ref)): one attempt from [`default_nominal_local`](@ref) (the
warm start, or the cold pursuit nominal if that violates fewer constraints).

**With a relevant obstacle ahead:**
1. If the measured disk already violates the required separation: `:initial_collision`.
2. **New encounter** (no side chosen yet, no warm start, or a different association):
   [`side_guess`](@ref) builds the passing guesses. Its inputs are re-rolled with the exact
   Cartesian map, so each nominal trajectory is dynamically consistent and its halfspaces
   separate it from the obstacle.
   - `encounter_strategy = :sequential` (default): the higher-ranked side is solved first;
     the other only if it is rejected and the budget allows. The ranking is the NMPC's:
     guess rollout succeeded, then larger clearance, then larger track clearance, then left.
   - `:both`: both sides are solved (budget permitting), and the accepted candidate with
     the lower nonlinear objective is kept.
3. **Otherwise:** one warm-started attempt, which keeps the side. If it is rejected, one
   attempt from the remembered side's guess (budget permitting).

A further attempt starts only if the elapsed time plus the longest QP so far fits within
`sqp_time_budget`.
"""
function solve_local!(ctrl::LinearMPCController, x, z0, uprev, t0)
    c, N, ob = ctrl.config, ctrl.config.horizon, ctrl.obstacle
    atts = Any[]
    attempts = Tuple{Symbol,Symbol,Int,Float64}[]
    guesses = Tuple{Symbol,Float64,Float64}[]
    t_guess, t_nomin = 0.0, 0.0
    longest = 0.0
    budget_ok() = (time_ns() - t0) / 1e9 + longest <= c.sqp_time_budget
    run!(label, Ufun, side) = begin
        ta = time_ns()
        tn = time_ns()
        nex = nothing
        Ub = try
            u = Ufun()
            u isa Tuple ? ((u, nex) = u; u) : u
        catch err
            err isa TrackReferenceError || rethrow()
            push!(attempts, (label, :invalid_reference, 0, (time_ns() - ta) / 1e9))
            push!(atts, (; status=:invalid_reference, msg="nominal trajectory: " * err.msg, val=nothing, qst=Symbol[], label))
            return nothing
        end
        t_nomin += (time_ns() - tn) / 1e9
        a = sqp_attempt!(ctrl, x, z0, uprev, Ub, t0; side, nominal_excess=nex)
        a = merge(a, (; label))
        push!(atts, a)
        push!(attempts, (label, a.status, length(a.qst), (time_ns() - ta) / 1e9))
        isempty(a.qst) || (longest = max(longest, a.t_sol / length(a.qst)))
        return a
    end
    default_U() = default_nominal_local(ctrl, z0, x[1])
    relevant, ahead, assoc, pl, pr = false, false, (NaN, NaN), true, true
    D = ob === nothing ? NaN : required_separation(ctrl)
    if ob !== nothing
        if hypot(z0[1] - ob.x, z0[2] - ob.y) < D - c.acceptance_tol
            msg = "measured state violates the required separation: distance $(fmt(hypot(z0[1] - ob.x, z0[2] - ob.y))) m < D = $(fmt(D)) m"
            reset_warm_start!(ctrl)
            return linear_result(ctrl, t0, :initial_collision, msg, nothing, uprev, x;
                obstacle=merge(LINEAR_OBSTACLE_NONE, (; present=true, required=D, failure_cause=:initial_collision)))
        end
        as = obstacle_associations(ctrl, obstacle_window(ctrl, x)...)
        relevant = !isempty(as)
        ahead = any(a -> a.s >= x[1] - D, as)
        assoc = obstacle_frenet(ctrl, x)
        pl, pr = passage_sides(ctrl, x)
    end
    E = ctrl.ey_max
    guess!(s, a) = begin
        tg = time_ns()
        g = side_guess(ctrl, x, s; a)
        push!(guesses, (s, g.clearance, g.rollout_ok ? g.track_clearance : -Inf))
        t_guess += (time_ns() - tg) / 1e9
        g
    end
    accel() = ctrl.last_U !== nothing ? vcat(ctrl.last_U[2, 2:N], ctrl.last_U[2, N]) :
              (try
        nominal_inputs_local(ctrl, z0, x[1])[2, :]
    catch err
        err isa TrackReferenceError || rethrow()
        zeros(N)
    end)
    if ob === nothing || !relevant || !ahead
        run!(:default, default_U, ctrl.obstacle_side)
    else
        side, enc = ctrl.obstacle_side, ctrl.obstacle_encounter
        new_encounter = side === :none || ctrl.last_U === nothing || isnan(enc) || abs(assoc[1] - enc) > 2(D + E)
        if new_encounter
            sides = [s for (s, open) in ((:left, pl), (:right, pr)) if open]
            isempty(sides) && (sides = [:left, :right])
            av = accel()
            gs = Dict(s => guess!(s, av) for s in sides)
            rank(s) = (gs[s].rollout_ok, gs[s].clearance, gs[s].track_clearance, s === :left)
            order = sort(sides; by=rank, rev=true)
            for (i, s) in enumerate(order)
                if i == 2 && c.encounter_strategy === :sequential && any(a -> a.status === :success, atts)
                    push!(attempts, (s, :not_attempted_sequential, 0, 0.0))
                    continue
                end
                if i == 2 && !budget_ok()
                    push!(attempts, (s, :not_attempted_budget, 0, 0.0))
                    continue
                end
                run!(s, () -> copy(gs[s].Ug), s)
            end
            length(sides) == 1 && push!(attempts, (sides[1] === :left ? :right : :left, :not_attempted_blocked, 0, 0.0))
        else
            a = run!(:warm, default_U, side)
            if a === nothing || a.status !== :success
                if budget_ok()
                    g = guess!(side, accel())
                    run!(side, () -> copy(g.Ug), side)
                else
                    push!(attempts, (side, :not_attempted_budget, 0, 0.0))
                end
            end
        end
    end
    real = [a for a in atts if haskey(a, :Zq)]
    acc = [a for a in real if a.status === :success]
    chosen = !isempty(acc) ? acc[argmin([a.nlobj for a in acc])] : (isempty(real) ? nothing : real[end])
    obrep = LINEAR_OBSTACLE_NONE
    if chosen !== nothing && chosen.status === :success
        ctrl.last_U, ctrl.last_Z = chosen.Uq, chosen.Zq
        ctrl.trust_memory = min(1.0, chosen.scales[end] * c.trust_region_growth)
        if ob !== nothing && relevant && ahead
            # The side of the guess that produced the plan; otherwise the plan's own side at
            # the obstacle (the previous side if the plan does not reach it yet).
            ps = plan_side_local(ctrl, x, chosen.val.frenet)
            ctrl.obstacle_side = chosen.label in (:left, :right) ? chosen.label : ctrl.obstacle_side !== :none ? ctrl.obstacle_side : ps
            ctrl.obstacle_encounter = assoc[1]
        end
    else
        reset_warm_start!(ctrl)
    end
    if ob !== nothing
        o = chosen === nothing || chosen.val === nothing ? nothing : chosen.val.obstacle
        Dc = c.ego_radius + ob.radius
        obrep = (; present=true, relevant=relevant && ahead, side=relevant && ahead ? ctrl.obstacle_side : :none, attempts, guesses,
            required=D, collision_distance=Dc, min_distance=o === nothing ? NaN : o.min_distance,
            physical_clearance=o === nothing ? NaN : o.min_distance - Dc, buffer_clearance=o === nothing ? NaN : o.min_distance - D,
            constrained_clearance=o === nothing ? NaN : o.constrained_min_distance - D, collisions=o === nothing ? 0 : o.collisions,
            separation_violations=o === nothing ? 0 : o.separation_violations,
            degenerate_halfspaces=chosen === nothing || !haskey(chosen, :ndeg) ? 0 : chosen.ndeg,
            failure_cause=chosen !== nothing && chosen.status === :success ? :none : linear_failure_cause(real, pl, pr), association=assoc)
    end
    if chosen === nothing
        st, msg = isempty(atts) ? (:not_converged, "no attempt could be started") : (atts[end].status, atts[end].msg)
        return linear_result(ctrl, t0, st, msg, nothing, uprev, x; obstacle=obrep, t_guess, t_nomin)
    end
    msg = length(attempts) > 1 ? chosen.msg * " [attempts: " * join(["$(a[1]) → $(a[2])" for a in attempts], ", ") * "]" : chosen.msg
    return linear_result(ctrl, t0, chosen.status, msg, chosen, uprev, x; obstacle=obrep, t_guess, t_nomin, all_attempts=real)
end

"""
    track_linearization_error(controller, Zq, s0) -> (progress, lateral, heading)

Largest difference, over the knots `k ≥ 2`, between the QP's affine track coordinates
(`S`, `EY`, `EP` at the solution) and the true track coordinates of the QP's predicted
states `Zq`. These are projected onto the actual track with chained hints. The result is
`(NaN, NaN, NaN)` if a projection fails. This is a diagnostic of the local track model;
acceptance is decided by the nonlinear validation of the rollout.
"""
function track_linearization_error(ctrl::LinearMPCController, Zq::AbstractMatrix, s0::Real)
    N = ctrl.config.horizon
    refs = try
        trajectory_references(ctrl, Zq, s0)
    catch err
        err isa TrackReferenceError || rethrow()
        return (NaN, NaN, NaN)
    end
    S, EY, EP = qp_values(ctrl, ctrl.S), qp_values(ctrl, ctrl.EY), qp_values(ctrl, ctrl.EP)
    return (maximum(abs(S[i] - refs[i+1].s) for i in 1:N), maximum(abs(EY[i] - refs[i+1].ey) for i in 1:N),
        maximum(abs(EP[i] - refs[i+1].epsi) for i in 1:N))
end
