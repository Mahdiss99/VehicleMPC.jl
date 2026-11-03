# Cartesian-endpoint NMPC (Milestone 2B.4): segment assignment, initial guess, validity
# checks and the independent exact-map reconstruction. The NLP itself is built in
# nonlinear_mpc.jl. See docs/cartesian_endpoint_nmpc.md.

"""
    cartesian_assignment(track, s; knot_tol=1e-6) -> (; seg, frames, boundary_knots)

Assign every nominal knot `s[k]` the segment containing it under the half-open convention
(`segment_id(track, s, :right)`). A knot exactly on a boundary gets the segment it
**enters**, which is the one a forward-driving vehicle (`sdot ≥ 0`) moves into.
`frames[k]` holds that segment's constants ([`segment_frame`](@ref)). `boundary_knots`
counts nominal knots within `knot_tol` of a boundary.

The centreline formulas of a frame equal the track on the closed interval `[s0, s1]`,
since the pose is continuous at the boundaries. A knot on either end point is therefore
mapped exactly.
"""
function cartesian_assignment(track::Track, s::AbstractVector; knot_tol::Real=1e-6)
    seg = [segment_id(track, x, :right) for x in s]
    near(x) = try
        !isempty(segment_boundaries(track, x - knot_tol, x + knot_tol))
    catch err
        err isa DomainError || rethrow()
        false                                            # open-track end: no neighbouring segment
    end
    return (; seg, frames=[segment_frame(track, id) for id in seg], boundary_knots=count(near, s))
end

"""
Whether progress `s` lies in the region where `frame`'s formulas equal the track: the
closed interval `[s0 - tol, s1 + tol]`. On a closed single-segment track (a circle), the
analytic continuation is the track itself, so every `s` qualifies.
"""
frame_valid(track::Track, frame, s, tol) =
    (track.closed && num_segments(track) == 1) || frame.s0 - tol <= s <= frame.s1 + tol

"Rollout with the event-aware integrator: `(states or nothing, simulation result)`."
function event_aware_rollout(ctrl::NMPCController, x, U)
    N, dt = horizon(ctrl), ctrl.config.dt
    sim = simulate_on_track(ctrl.vehicle, ctrl.track, x, [SVector{2,Float64}(U[1, k], U[2, k]) for k in 1:N], dt; max_step=dt / 5)
    return succeeded(sim) ? reduce(hcat, sim.states) : nothing, sim
end

"""
    cartesian_initial_guess(controller, x, warm) -> NamedTuple

Initial guess and nominal progress for the Cartesian-endpoint mode. The inputs are:
- **warm:** the shifted previous inputs;
- **cold:** the quasi-steady profile (`quasi_steady_guess`), or, on a single-segment
  track, curvature feed-forward steering with zero acceleration.

They are rolled out from `x` with the event-aware integrator on the actual track, so the
guess follows the true geometry. If the rollout reaches an end of an open track, the cycle
is a `:scheduling_failure` (as in the Frenet modes). If it stops early for another reason
(model domain), the guess is:
- **warm:** the shifted previous solution;
- **cold:** a centreline guess along the quasi-steady progress.

There is no fixed point: the nominal progress only selects the segment of each knot.
`kappas` holds the nominal stage curvatures, for reporting only.
"""
function cartesian_initial_guess(ctrl::NMPCController, x, warm::Bool)
    N, dt, track, p = horizon(ctrl), ctrl.config.dt, ctrl.track, ctrl.vehicle.params
    failure(msg) = (; converged=false, iterations=0, change=NaN, kappas=Float64[], Xg=fill(NaN, 4, N + 1), Ug=zeros(2, N),
        s_nominal=fill(NaN, N + 1), method=warm ? :warm : :cold, message=msg)
    if warm
        Ug = hcat(ctrl.last_U[:, 2:N], ctrl.last_U[:, N])
        fallback = hcat(collect(Float64, x), ctrl.last_X[:, 3:N+1], ctrl.last_X[:, N+1])
        method = :warm
    elseif num_segments(track) == 1
        Ug = zeros(2, N)
        Ug[1, :] .= clamp(atan(track.segments[1].curvature * p.wheelbase), -p.delta_max, p.delta_max)
        fallback = repeat(collect(Float64, x), 1, N + 1)
        method = :constant
    else
        s_nom, Ug = try
            quasi_steady_guess(ctrl, x)
        catch err
            err isa DomainError || rethrow()
            return failure("cold-start nominal leaves the track ($(sprint(showerror, err)))")
        end
        vg = [k <= N ? (s_nom[k+1] - s_nom[k]) / dt : (s_nom[N+1] - s_nom[N]) / dt for k in 1:N+1]
        fallback = vcat(s_nom', zeros(1, N + 1), zeros(1, N + 1), vg')
        fallback[:, 1] = x
        method = :cold
    end
    Xg, sim = event_aware_rollout(ctrl, x, Ug)
    sim.status in (:end_of_track, :start_of_track) && return failure("nominal rollout leaves the open track ($(sim.message))")
    Xg === nothing && (Xg = fallback)
    s_nominal = Xg[1, :]
    kappas = try
        stage_curvatures(track, s_nominal)
    catch err
        err isa DomainError || rethrow()
        return failure("nominal progress leaves the track ($(sprint(showerror, err)))")
    end
    return (; converged=true, iterations=0, change=0.0, kappas, Xg, Ug, s_nominal, method, message="")
end

"""
Segment reassignment for a correction solve: the nominal progress becomes the optimised
progress `core.X[1, :]`, and the previous solution becomes the initial guess.
"""
function cartesian_reassignment(ctrl::NMPCController, sched, core)
    s = core.X[1, :]
    kappas = try
        stage_curvatures(ctrl.track, s)
    catch err
        err isa DomainError || rethrow()
        return (; sched..., converged=false, message="optimised progress leaves the track ($(sprint(showerror, err)))")
    end
    return (; sched..., converged=true, kappas, Xg=core.X, Ug=core.U, s_nominal=s, iterations=sched.iterations + 1,
        method=endswith(string(sched.method), "_corrected") ? sched.method : Symbol(sched.method, :_corrected), message="")
end

"Write a Cartesian-endpoint assignment into the NLP Parameters (and, with `segment_membership`, the knot bounds)."
function install_cartesian!(ctrl::NMPCController, sched)
    c, N = ctrl.config, horizon(ctrl)
    asg = cartesian_assignment(ctrl.track, sched.s_nominal; knot_tol=c.boundary_knot_tol)
    for k in 1:N+1
        f = asg.frames[k]
        for (r, v) in enumerate((f.s0, f.x0, f.y0, f.psi0, f.kappa))
            set_parameter_value(ctrl.cframe[r, k], v)
        end
    end
    if c.segment_membership && !(ctrl.track.closed && num_segments(ctrl.track) == 1)
        for k in 2:N+1
            set_lower_bound(ctrl.X[1, k], asg.frames[k].s0)
            set_upper_bound(ctrl.X[1, k], asg.frames[k].s1)
        end
    end
    if !isempty(ctrl.alphas)                       # interior corridor samples (Milestone 2B.5)
        samples = interior_assignment(ctrl, sched.Xg, sched.Ug, asg.frames)
        install_interior!(ctrl, samples)
        asg = (; asg..., samples)
    end
    return (; sched..., lin=nothing, asg)
end

"""
Residual of the NLP's stage-`k` equalities, recomputed in `Float64` with the assigned
segment frames: the largest of the position, heading and speed residuals.
"""
function cartesian_stage_residual(ctrl::NMPCController, X, U, frames, k)
    L, dt = ctrl.vehicle.params.wheelbase, ctrl.config.dt
    pk = frenet_pose_local(frames[k], X[1, k], X[2, k], X[3, k])
    pn = frenet_pose_local(frames[k+1], X[1, k+1], X[2, k+1], X[3, k+1])
    inc = exact_bicycle_increment(L, pk[3], X[4, k], U[1, k], U[2, k], dt)
    return max(abs(pn[1] - pk[1] - inc[1]), abs(pn[2] - pk[2] - inc[2]), abs(pn[3] - pk[3] - inc[3]),
        abs(X[4, k+1] - X[4, k] - inc[4]))
end

"""
    cartesian_consistency(controller, sched, core, evaluate) -> ScheduleReport

Post-solve segment-assignment check of the Cartesian-endpoint mode:
- every optimised knot `k ≥ 2` must lie in its assigned segment (see `frame_valid`, with
  tolerance `boundary_knot_tol`); each knot outside counts in `configuration_changes`;
- `stage_residual` and `cumulative_residual` are computed on the true geometry:
  `r_k = psi_true(X*_{k+1}) - psi_true(X*_k) - phi_k`, with
  `psi_true = centerline_heading + epsi` and `phi_k` the exact-map heading change.

`consistent` requires no knot outside its segment (and `max_progress_mismatch`).
"""
function cartesian_consistency(ctrl::NMPCController, sched, core, evaluate::Bool)
    c, track, N, L = ctrl.config, ctrl.track, horizon(ctrl), ctrl.vehicle.params.wheelbase
    frames = sched.asg.frames
    dsmin = evaluate ? minimum(core.X[1, k+1] - core.X[1, k] for k in 1:N) : NaN
    report(sr, cr, pm, nc, ok, msg; nconf=0) = ScheduleReport(sched.method, sched.iterations, sched.converged, sched.change,
        collect(Float64, sched.kappas), collect(Float64, sched.s_nominal), sr, cr, pm, nc, ok, msg, :cartesian_endpoint_exact,
        NaN, nconf, sched.asg.boundary_knots, 0, 0, 0, 0, dsmin)
    evaluate || return report(NaN, NaN, NaN, 0, false, "not evaluated (solution rejected before the segment-assignment test)")
    X, U = core.X, core.U
    sstar = X[1, :]
    mismatch = maximum(abs, sstar - sched.s_nominal)
    tol = c.boundary_knot_tol
    outside = [k for k in 2:N+1 if !frame_valid(track, frames[k], sstar[k], tol)]
    overshoot = isempty(outside) ? 0.0 : maximum(max(frames[k].s0 - sstar[k], sstar[k] - frames[k].s1) for k in outside)
    r = zeros(N)
    ncross = 0
    try
        psi = [centerline_heading(track, sstar[k]) + X[3, k] for k in 1:N+1]
        for k in 1:N
            d = X[4, k] * c.dt + U[2, k] * c.dt^2 / 2
            r[k] = psi[k+1] - psi[k] - tan(U[1, k]) * d / L
            isempty(segment_boundaries(track, sstar[k], sstar[k+1])) || (ncross += 1)
        end
    catch err
        err isa DomainError || rethrow()
        return report(NaN, NaN, mismatch, ncross, false, "optimised progress leaves the track: " * sprint(showerror, err);
            nconf=length(outside))
    end
    msgs = String[]
    isempty(outside) || push!(msgs, "$(length(outside)) knot(s) outside their assigned segments (largest overshoot $(fmt(overshoot)) m)")
    mismatch > c.max_progress_mismatch && push!(msgs, "progress mismatch $(fmt(mismatch)) m > $(c.max_progress_mismatch) m")
    return report(maximum(abs, r), maximum(abs, cumsum(r)), mismatch, ncross, isempty(msgs), join(msgs, "; "); nconf=length(outside))
end

"""
    exact_map_check(controller, x, X, U; frames=nothing) -> ExactMapReport

Independent reconstruction of a prediction `(X, U)` from the measured state `x` with the
validated exact Cartesian map. It is applicable to any prediction model; the controller
runs it automatically in the Cartesian-endpoint mode. See [`ExactMapReport`](@ref).
`frames` (the assigned segment frames) enable `geometry_residual`.
"""
function exact_map_check(ctrl::NMPCController, x, X::AbstractMatrix, U::AbstractMatrix; frames=nothing)
    track, N, dt, veh = ctrl.track, horizon(ctrl), ctrl.config.dt, ctrl.vehicle
    ztrue = try
        [frenet_state_to_cartesian(track, X[:, k]) for k in 1:N+1]
    catch err
        err isa DomainError || rethrow()
        return ExactMapReport(true, :off_track, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, "knot outside the Frenet-valid region: " * sprint(showerror, err))
    end
    z0 = frenet_state_to_cartesian(track, x)
    zs = simulate_exact_cartesian(veh, z0, [SVector{2,Float64}(U[1, k], U[2, k]) for k in 1:N], dt)
    pos = [hypot(zs[k][1] - ztrue[k][1], zs[k][2] - ztrue[k][2]) for k in 1:N+1]
    head = maximum(abs(zs[k][3] - ztrue[k][3]) for k in 1:N+1)
    speed = maximum(abs(zs[k][4] - ztrue[k][4]) for k in 1:N+1)
    knot = 0.0
    for k in 1:N
        zn = exact_cartesian_step(veh, ztrue[k], U[:, k], dt)
        knot = max(knot, hypot(zn[1] - ztrue[k+1][1], zn[2] - ztrue[k+1][2]), abs(zn[3] - ztrue[k+1][3]), abs(zn[4] - ztrue[k+1][4]))
    end
    geo = NaN
    if frames !== nothing
        geo = 0.0
        for k in 1:N+1
            pl = frenet_pose_local(frames[k], X[1, k], X[2, k], X[3, k])
            geo = max(geo, hypot(pl[1] - ztrue[k][1], pl[2] - ztrue[k][2]), abs(pl[3] - ztrue[k][3]))
        end
    end
    status, msg = :success, ""
    ds, dey = 0.0, 0.0
    try
        maxlat = track.width > 0 ? track.width : Inf
        for k in 2:N+1
            s, ey, _ = cartesian_to_frenet(track, zs[k][1], zs[k][2], zs[k][3]; s_hint=X[1, k], max_lateral=maxlat)
            ds, dey = max(ds, abs(s - X[1, k])), max(dey, abs(ey - X[2, k]))
        end
    catch err
        err isa ArgumentError || rethrow()
        status, msg, ds, dey = :projection_failed, sprint(showerror, err), NaN, NaN
    end
    return ExactMapReport(true, status, maximum(pos), head, speed, ds, dey, pos[2], knot, geo, msg)
end
