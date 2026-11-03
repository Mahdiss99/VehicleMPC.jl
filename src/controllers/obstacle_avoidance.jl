# Stationary-obstacle collision avoidance for the Cartesian-endpoint NMPC (Milestone 3A):
# obstacle updates, left/right candidate initialisations, the independent collision
# evaluation and the candidate logic around solve_once!. The collision constraints are
# built in the NMPCController constructor. See docs/stationary_collision_avoidance.md.

"Copy of an `NMPCResult` with some fields replaced."
function with_fields(r::NMPCResult; kw...)
    return NMPCResult((haskey(kw, f) ? kw[f] : getfield(r, f) for f in fieldnames(NMPCResult))...)
end

"""
    set_obstacle!(controller, obstacle) -> controller

Move or resize the obstacle of a controller built with one, by updating the three
Parameters `(x, y, D)` of the persistent collision constraints. There is no rebuild. The
chosen passing side is forgotten.
"""
function set_obstacle!(ctrl::NMPCController, ob::CircleObstacle)
    ctrl.obstacle === nothing && throw(ArgumentError("the controller was built without an obstacle (pass obstacle = ... to NMPCController)"))
    D = required_separation(ob, ctrl.config.ego_radius, ctrl.config.obstacle_buffer)
    for (p, v) in zip(ctrl.obs_par, (ob.x, ob.y, D))
        set_parameter_value(p, v)
    end
    ctrl.obstacle = ob
    ctrl.obstacle_side = :none
    ctrl.obstacle_encounter = NaN
    return ctrl
end

required_separation(ctrl::Union{NMPCController,LinearMPCController}) = required_separation(ctrl.obstacle, ctrl.config.ego_radius, ctrl.config.obstacle_buffer)

"""
    obstacle_associations(controller, s_lo, s_hi; reach=ey_max + D) -> Vector{NamedTuple}

Every plausible local association of the obstacle with the centreline in the unwrapped
progress window `[s_lo, s_hi]`, computed from the obstacle's Cartesian position.

1. For every (lap-aware) segment overlapping the window, find the analytically closest
   centreline point within the overlap:
   - straight segment: the orthogonal projection, clamped;
   - arc: the angle of the obstacle around the arc centre, with the overlap end points as
     candidates.
2. Keep the points closer than `reach`. By default `reach` is `ey_max + D`: an obstacle
   whose centre lies outside the track can still reach the planning corridor.
3. Group the kept points into separate passes (gaps in progress larger than `2·reach`),
   and report the closest point of each.

The reach test uses the closest point within the window, which is conservative. The
reported `(s, ey)` is the segment's own closest point (unclamped by the window, provided
it lies within `2·reach` of the window). An obstacle at the window's edge therefore gets
its true local association.

Each association is `(; s, ey, dist)`:
- `s`: unwrapped progress;
- `ey`: signed lateral offset of the obstacle centre along the centreline's left normal
  at `s`;
- `dist`: centre distance from the centreline.

No single projection is relied on. Several associations mean that several parts of the
track (or laps) are within reach.
"""
function obstacle_associations(ctrl::Union{NMPCController,LinearMPCController}, s_lo::Real, s_hi::Real; reach::Real=ctrl.ey_max + required_separation(ctrl))
    tr, ob = ctrl.track, ctrl.obstacle
    L = track_length(tr)
    lo, hi = tr.closed ? (Float64(s_lo), Float64(s_hi)) : (max(Float64(s_lo), 0.0), min(Float64(s_hi), L))
    pts = NamedTuple{(:s, :ey, :dist),Tuple{Float64,Float64,Float64}}[]
    lo < hi || return pts
    for id in segment_id(tr, lo, :right):segment_id(tr, hi, :left)
        f = segment_frame(tr, id)
        a, b = max(f.s0, lo) - f.s0, min(f.s1, hi) - f.s0
        a <= b || continue
        len = f.s1 - f.s0
        inner = Float64[]
        if f.kappa == 0
            push!(inner, (ob.x - f.x0) * cos(f.psi0) + (ob.y - f.y0) * sin(f.psi0))
        else
            cx, cy = f.x0 - sin(f.psi0) / f.kappa, f.y0 + cos(f.psi0) / f.kappa
            if hypot(ob.x - cx, ob.y - cy) > 0
                phi = mod(sign(f.kappa) * (atan(ob.y - cy, ob.x - cx) - atan(f.y0 - cy, f.x0 - cx)), 2π)
                push!(inner, phi / abs(f.kappa), (phi - 2π) / abs(f.kappa))
            end
        end
        function closest(lo_, hi_)
            best = (Inf, NaN, NaN)
            for sg in vcat(lo_, hi_, inner)
                lo_ <= sg <= hi_ || continue
                xc, yc, psc = segment_local_pose(f.x0, f.y0, f.psi0, f.kappa, sg)
                d = hypot(ob.x - xc, ob.y - yc)
                d < best[1] && (best = (d, f.s0 + sg, -sin(psc) * (ob.x - xc) + cos(psc) * (ob.y - yc)))
            end
            return best
        end
        best = closest(a, b)                            # within the window: the reach test
        best[1] <= reach || continue
        full = closest(0.0, len)                        # the segment's own closest point: the reported association
        full[1] <= reach && lo - 2reach <= full[2] <= hi + 2reach && (best = full)
        push!(pts, (; s=best[2], ey=best[3], dist=best[1]))
    end
    sort!(pts; by=p -> p.s)
    out = similar(pts, 0)
    for p in pts                                        # one association per separate pass
        if !isempty(out) && p.s - out[end].s <= 2reach
            p.dist < out[end].dist && (out[end] = p)
        else
            push!(out, p)
        end
    end
    return out
end

"Progress window in which the obstacle can matter for this horizon: `[s - D - E, s + reach + D + E]`."
function obstacle_window(ctrl::Union{NMPCController,LinearMPCController}, x)
    D, E = required_separation(ctrl), ctrl.ey_max
    reach = max(x[4], ctrl.vehicle.params.v_max) * horizon(ctrl) * ctrl.config.dt
    return (x[1] - D - E, x[1] + reach + D + E)
end

"""
    obstacle_frenet(controller, x) -> (s, ey)

The first association ahead of the vehicle (see [`obstacle_associations`](@ref)) within the
horizon window of the state `x` (or of progress `x` with the maximum speed), as unwrapped
progress and signed lateral offset. Returns `(NaN, NaN)` if there is none.
"""
function obstacle_frenet(ctrl::Union{NMPCController,LinearMPCController}, x)
    xx = x isa Real ? (Float64(x), 0.0, 0.0, ctrl.vehicle.params.v_max) : x
    as = obstacle_associations(ctrl, obstacle_window(ctrl, xx)...)
    isempty(as) && return (NaN, NaN)
    D = required_separation(ctrl)
    ahead = filter(a -> a.s >= xx[1] - D, as)
    a = isempty(ahead) ? as[end] : ahead[1]
    return (a.s, a.ey)
end

"""
    passage_sides(controller, x) -> (left, right)

Lateral-gap heuristic for the association used by the initialisation: whether
`ey_o + D ≤ ey_max` (left) and `ey_o - D ≥ -ey_max` (right).
- Within one segment this is the exact closest approach of a path at a constant offset
  (along the normal through the obstacle, for lines and arcs).
- Across segment boundaries it is an approximation.
- It is **not** a proof of geometric or dynamic feasibility: it says nothing about
  reaching the offset in time, or about steering and lateral-acceleration limits.
"""
function passage_sides(ctrl::Union{NMPCController,LinearMPCController}, x)
    _, eyo = obstacle_frenet(ctrl, x)
    isnan(eyo) && return (true, true)                    # no association: both sides open
    D, E = required_separation(ctrl), ctrl.ey_max
    return (eyo + D <= E, eyo - D >= -E)
end

"""
    obstacle_relevant(controller, x) -> Bool

Whether the obstacle can matter within this horizon. The test is conservative and
geometric: some centreline point within the horizon window (unwrapped, across segment
boundaries and laps) lies within `ey_max + D` of the obstacle centre. It decides only the
initialisation; the collision constraints are always active.
"""
obstacle_relevant(ctrl::Union{NMPCController,LinearMPCController}, x) = !isempty(obstacle_associations(ctrl, obstacle_window(ctrl, x)...))

"""
    side_guess(controller, x, side; a=nothing) -> schedule named tuple

Dynamically consistent initial guess that passes the obstacle on `side` (`:left` or
`:right`), for straight and curved tracks.
- **Target offset:** the middle of the passage, `(ey_o + D + ey_max)/2` or
  `(ey_o - D - ey_max)/2` (±0.9·`ey_max` if that side is blocked). `(s_o, ey_o)` is the
  association from [`obstacle_frenet`](@ref).
- **Smooth window:**
  - the offset is held over `|s - s_o| ≤ D`;
  - the approach ramp is `clamp(s_o - D - s, obstacle_ramp_min, obstacle_ramp)`, so it is
    shortened when the obstacle is already close;
  - the departure ramp is `obstacle_ramp`.
- **Speed:** the acceleration (the shifted previous plan, else the quasi-steady profile,
  else zero) is reduced where needed. The speed over the next second must not exceed
  `sqrt(0.9·ay_max/|kappa_path|)`, with `kappa_path = kappa/(1 - kappa·ey_target)` the
  curvature of the offset parallel path, and must be reachable by braking at 80 % of
  `|a_min|`. On straight segments the acceleration is unchanged.
- **Steering:** pure pursuit towards the offset path.
  - Look-ahead `clamp(0.5·v, 0.5, 2) m`, but at most the smallest radius of curvature
    within 2 m, so that the guess does not cut into a tight bend early.
  - On a constant-curvature path, pure pursuit returns that path's curvature exactly.
  - The steering is clipped to the steering limit and to `|v² tan(delta)/L| ≤ ay_max`.
- **Rollout:** the event-aware integrator on the actual track, with inputs held per
  interval. If the model domain is left, the last state is held and the guess is marked.

The result can be passed to [`solve_once!`](@ref) as `initial`. Its fields `clearance` (the
smallest knot distance to the obstacle minus `D`) and `track_clearance` (the smallest
`width/2 - |ey|`) describe the guess itself.

The function also accepts a [`LinearMPCController`](@ref) with an obstacle (Milestone 4BC),
which passes `a` explicitly; its inputs `Ug` are then re-rolled with the exact Cartesian
map.
"""
function side_guess(ctrl::Union{NMPCController,LinearMPCController}, x, side::Symbol; a=nothing)
    side in (:left, :right) || throw(ArgumentError("side must be :left or :right"))
    N, dt, tr, veh, c = horizon(ctrl), ctrl.config.dt, ctrl.track, ctrl.vehicle, ctrl.config
    p, L = veh.params, veh.params.wheelbase
    ob = ctrl.obstacle
    so, eyo = obstacle_frenet(ctrl, x)
    D, E = required_separation(ctrl), ctrl.ey_max
    pl, pr = passage_sides(ctrl, x)
    target = side === :left ? (pl ? (eyo + D + E) / 2 : 0.9E) : (pr ? (eyo - D - E) / 2 : -0.9E)
    ramp_in = isnan(so) ? c.obstacle_ramp : clamp(so - D - x[1], c.obstacle_ramp_min, c.obstacle_ramp)
    ramp_out = c.obstacle_ramp
    function window(s)
        isnan(so) && return 0.0
        d = abs(s - so)
        d <= D && return 1.0
        r = s < so ? ramp_in : ramp_out
        d >= D + r && return 0.0
        return 0.5 * (1 + cos(π * (d - D) / r))
    end
    ey_target(s) = window(s) * target
    smax = tr.closed ? Inf : track_length(tr)
    kap(s) = curvature(tr, min(s, smax))
    if a === nothing
        a = if ctrl.last_U !== nothing
            vcat(ctrl.last_U[2, 2:N], ctrl.last_U[2, N])
        else
            try
                quasi_steady_guess(ctrl, x)[2][2, :]
            catch err
                err isa DomainError || rethrow()
                zeros(N)
            end
        end
    end
    ay = c.ay_max === nothing ? Inf : 0.9 * c.ay_max
    function target_speed(s, v)                       # curvature cap of the offset path, with braking room
        vt = Inf
        for ds in 0.0:0.1:(v * 1.0 + 0.5)
            k = kap(s + ds)
            k == 0 && continue
            kp = abs(k / (1 - k * ey_target(s + ds)))
            vt = min(vt, sqrt(sqrt(ay / kp)^2 + 2 * 0.8 * abs(p.a_min) * ds))
        end
        return vt
    end
    Xg, Ug = zeros(4, N + 1), zeros(2, N)
    Xg[:, 1] = x
    xk = SVector{4,Float64}(x...)
    alive = true
    for k in 1:N
        if alive
            v = xk[4]
            kmax = maximum(abs(kap(xk[1] + ds)) for ds in 0.0:0.1:2.0)
            ld = clamp(0.5v, 0.5, 2.0)
            kmax > 0 && (ld = min(ld, max(0.5, 1 / kmax)))
            st = min(xk[1] + ld, smax)
            pt = frenet_to_cartesian(tr, st, ey_target(st))
            pv = frenet_to_cartesian(tr, xk[1], xk[2], xk[3])
            alpha = atan(pt[2] - pv[2], pt[1] - pv[1]) - pv[3]
            dmax = c.ay_max === nothing ? p.delta_max : min(p.delta_max, atan(c.ay_max * L / max(v, 0.1)^2))
            delta = clamp(atan(2L * sin(alpha) / ld), -dmax, dmax)
            vt = target_speed(xk[1], v)
            acc = isfinite(vt) ? min(a[k], (vt - v) / dt) : a[k]
            Ug[:, k] = [delta, clamp(acc, p.a_min, p.a_max)]
            r = integrate_on_track(veh, tr, xk, Ug[:, k], dt; max_step=dt / 5)
            if succeeded(r)
                xk = r.x
            else
                alive = false
            end
        else
            Ug[:, k] = Ug[:, k-1]
        end
        Xg[:, k+1] = xk
    end
    s = Xg[1, :]
    clr = minimum(hypot(q[1] - ob.x, q[2] - ob.y) for q in (frenet_to_cartesian(tr, Xg[1, k], Xg[2, k]) for k in 1:N+1)) - D
    tclr = tr.width / 2 - maximum(abs, Xg[2, :])
    kappas = try
        stage_curvatures(tr, s)
    catch err
        err isa DomainError || rethrow()
        return (; converged=false, iterations=0, change=NaN, kappas=Float64[], Xg, Ug, s_nominal=s,
            method=Symbol(:obstacle_, side), message="obstacle-side guess leaves the track", clearance=clr, track_clearance=tclr,
            rollout_ok=alive)
    end
    return (; converged=true, iterations=0, change=0.0, kappas, Xg, Ug, s_nominal=s, method=Symbol(:obstacle_, side), message="",
        clearance=clr, track_clearance=tclr, rollout_ok=alive)
end

"""
    collision_check(controller, x, X, U; samples=config.obstacle_check_samples) -> NamedTuple

Independent collision evaluation of a plan `(X, U)` from the measured state `x`.
- **Constrained points:** the optimised knots `k ≥ 2` and the interior obstacle samples,
  with the true track geometry. Reported as the smallest distance and the smallest
  scaled margin [`obstacle_margin`](@ref).
- **Dense evaluation:** the plan's inputs propagated from the true pose of `x` with the
  exact Cartesian map, sampled at `j·dt/samples` on every interval plus the final knot.

This is a finite sampling, not a continuous-time certificate.
"""
function collision_check(ctrl::NMPCController, x, X::AbstractMatrix, U::AbstractMatrix; samples::Integer=ctrl.config.obstacle_check_samples)
    tr, ob, N, dt, veh, c = ctrl.track, ctrl.obstacle, horizon(ctrl), ctrl.config.dt, ctrl.vehicle, ctrl.config
    L, tol = veh.params.wheelbase, c.acceptance_tol
    D = required_separation(ctrl)
    Dc = c.ego_radius + ob.radius
    dist(px, py) = hypot(px - ob.x, py - ob.y)
    kd, km = Inf, Inf
    try
        for k in 1:N
            pk = frenet_state_to_cartesian(tr, X[:, k])
            pn = frenet_state_to_cartesian(tr, X[:, k+1])            # knots 2…N+1
            kd = min(kd, dist(pn[1], pn[2]))
            km = min(km, obstacle_margin(pn[1], pn[2], ob.x, ob.y, D))
            for a in ctrl.obs_alphas
                q = interior_point(L, pk, X[4, k], U[1, k], U[2, k], a * dt)
                kd = min(kd, dist(q...))
                km = min(km, obstacle_margin(q[1], q[2], ob.x, ob.y, D))
            end
        end
    catch err
        err isa DomainError || rethrow()
        kd, km = NaN, NaN
    end
    z = frenet_state_to_cartesian(tr, x)
    md, nsep, ncol, tfirst, between = Inf, 0, 0, NaN, false
    for k in 1:N
        u = SVector{2,Float64}(U[1, k], U[2, k])
        for j in (k == N ? (0:samples) : (0:samples-1))
            tau = j * dt / samples
            q = j == 0 ? (z[1], z[2]) : interior_point(L, z, z[4], u[1], u[2], tau)
            d = dist(q...)
            md = min(md, d)
            if d < D - tol
                nsep += 1
                isnan(tfirst) && (tfirst = (k - 1) * dt + tau)
                (0 < j < samples) && (between = true)
            end
            d < Dc - tol && (ncol += 1)
        end
        z = exact_cartesian_step(veh, z, u, dt)
    end
    return (; knot_min_distance=kd, knot_min_margin=km, min_distance=md, separation_violations=nsep, collisions=ncol,
        first_violation_time=tfirst, between, required=D, collision_distance=Dc)
end

function obstacle_report(ctrl, cc, side, attempts, pl, pr, cause, msg; assoc=(NaN, NaN), nassoc=0,
    guesses=Tuple{Symbol,Float64,Float64}[], phase=(NaN, NaN, NaN, NaN))
    ob, c = ctrl.obstacle, ctrl.config
    D, Dc = required_separation(ctrl), c.ego_radius + ob.radius
    cc === nothing && return ObstacleReport(true, ob.x, ob.y, ob.radius, c.ego_radius, c.obstacle_buffer, D, Dc, NaN, NaN, 0, NaN,
        NaN, NaN, 0, 0, NaN, false, side, attempts, pl, pr, cause, msg, assoc[1], assoc[2], nassoc, guesses, phase)
    return ObstacleReport(true, ob.x, ob.y, ob.radius, c.ego_radius, c.obstacle_buffer, D, Dc, cc.knot_min_distance,
        cc.knot_min_distance - D, c.obstacle_check_samples, cc.min_distance, cc.min_distance - D, cc.min_distance - Dc,
        cc.separation_violations, cc.collisions, cc.first_violation_time, cc.between, side, attempts, pl, pr, cause, msg,
        assoc[1], assoc[2], nassoc, guesses, phase)
end

"""
    obstacle_rejection(controller, cc) -> (status, message)

The obstacle acceptance rule applied to an otherwise accepted plan, given its
[`collision_check`](@ref) result `cc`:
- a violated imposed constraint gives `:obstacle_constraint_violation`;
- a collision in the dense evaluation (or, with `obstacle_acceptance = :separation`, a
  buffer violation) gives `:collision_detected`.

Returns `(nothing, "")` when the plan is acceptable. Shared by the obstacle wrapper and by
the branch-resolution candidate selection.
"""
function obstacle_rejection(ctrl::NMPCController, cc)
    c = ctrl.config
    tol = c.acceptance_tol
    if !(cc.knot_min_margin >= -tol)
        return :obstacle_constraint_violation, "collision constraint violated at a constrained point (margin $(fmt(cc.knot_min_margin)) m)"
    elseif cc.collisions > 0 || (c.obstacle_acceptance === :separation && cc.separation_violations > 0)
        return :collision_detected, cc.collisions > 0 ?
            "collision in the dense evaluation: minimum distance $(fmt(cc.min_distance)) m < $(fmt(cc.collision_distance)) m (first separation violation at t = $(fmt(cc.first_violation_time)) s)" :
            "required separation violated in the dense evaluation at t = $(fmt(cc.first_violation_time)) s (distance $(fmt(cc.min_distance)) m < D = $(fmt(cc.required)) m)"
    end
    return nothing, ""
end

"""
Run one attempt (warm or from an initial guess) and validate it against the obstacle.
Returns `(result, check)`. An accepted solve that fails the obstacle validation is turned
into a rejection.
"""
function obstacle_attempt!(ctrl, x, uprev, initial; cycle_start=nothing)
    c = ctrl.config
    r = solve_once!(ctrl, x; previous_input=uprev, initial, cycle_start)
    all(isfinite, r.X) && all(isfinite, r.U) || return r, nothing
    cc = collision_check(ctrl, x, r.X, r.U)
    r.accepted || return r, cc
    st, bad = obstacle_rejection(ctrl, cc)
    st === nothing && return r, cc
    reset_warm_start!(ctrl)
    msg = isempty(r.message) ? bad : r.message * "; " * bad
    return with_fields(r; status=st, accepted=false, u=nothing, message=msg), cc
end

"Side of the obstacle on which a plan passes, from the knot nearest to the progress of the obstacle's association."
function plan_side(ctrl, X)
    so, eyo = obstacle_frenet(ctrl, X[:, 1])
    isnan(so) && return :none
    k = argmin(abs.(X[1, :] .- so))
    return X[2, k] >= eyo ? :left : :right
end

"""
    solve_with_obstacle!(controller, x; previous_input) -> NMPCResult

`solve!` for a controller built with an obstacle.

1. If the measured disk already violates the required separation, return
   `:initial_collision` without solving.
2. If the obstacle is not relevant to this horizon ([`obstacle_relevant`](@ref), a
   geometric test across segments and laps): one ordinary solve.
3. At a **new encounter**, solve from both the left and the right [`side_guess`](@ref).
   Keep the accepted candidate with the lower objective, and remember its side and the
   association's unwrapped progress. This is the only case with two solves by design. An
   encounter is new when:
   - no side has been chosen yet;
   - there is no warm start; or
   - the association's unwrapped progress differs from the remembered one by more than
     `2(D + ey_max)`, e.g. the same obstacle one lap later.
4. Otherwise: one warm-started solve, which keeps the side. If it is rejected, retry once
   from the remembered side's guess.

**Encounter strategies** (Milestone 3B.1):
- `encounter_strategy = :both` (default): step 3 as above.
- `:sequential`: both side guesses are built, and the higher-ranked one is solved first.
  The ranking prefers a guess whose rollout succeeded, then the larger knot clearance
  above `D` from the obstacle, then the larger track clearance, then left. The other side
  is solved only if the first is rejected. This does not compare the two local optima.

**Budget.** With a finite `cycle_budget`, the second candidate, the warm-start retry,
reassignments and branch candidates start only if
[`cycle_budget_allows`](@ref) them. Skipped attempts are recorded as
`:not_attempted_budget`.

Every attempt is validated with [`collision_check`](@ref):
- a constraint violation at a constrained point gives `:obstacle_constraint_violation`;
- a collision in the dense evaluation (or, with `obstacle_acceptance = :separation`, a
  separation violation) gives `:collision_detected`.

All attempts, their statuses, NLP solves and times, and the clearance and track clearance
of each side guess, are recorded in `result.obstacle`. `timing.total` and `nlp_solves`
cover the whole cycle. No control is returned unless an attempt is accepted. The
collision constraints are active in every solve, whatever the relevance test says.
"""
function solve_with_obstacle!(ctrl::NMPCController, measured_state::AbstractVector; previous_input=(0.0, 0.0))
    t0 = time_ns()
    x = SVector{4,Float64}(measured_state...)
    uprev = SVector{2,Float64}(previous_input...)
    ob, c, tr = ctrl.obstacle, ctrl.config, ctrl.track
    D, E = required_separation(ctrl), ctrl.ey_max
    as = obstacle_associations(ctrl, obstacle_window(ctrl, x)...)
    assoc = obstacle_frenet(ctrl, x)
    pl, pr = passage_sides(ctrl, x)
    t_rel = (time_ns() - t0) / 1e9
    t_guess, t_att = 0.0, 0.0
    guesses = Tuple{Symbol,Float64,Float64}[]
    report(cc, side, attempts, cause, msg) = obstacle_report(ctrl, cc, side, attempts, pl, pr, cause, msg; assoc, nassoc=length(as), guesses,
        phase=(t_rel, t_guess, t_att, (time_ns() - t0) / 1e9 - t_rel - t_guess - t_att))
    pose = try
        frenet_state_to_cartesian(tr, x)
    catch err
        err isa DomainError || rethrow()
        nothing
    end
    if pose !== nothing && hypot(pose[1] - ob.x, pose[2] - ob.y) < D - c.acceptance_tol
        msg = "measured state violates the required separation: distance $(fmt(hypot(pose[1] - ob.x, pose[2] - ob.y))) m < D = $(fmt(D)) m"
        return obstacle_failure(ctrl, t0, :initial_collision, msg,
            report(nothing, ctrl.obstacle_side, Tuple{Symbol,Symbol,Int,Float64}[], :initial_collision, msg))
    end
    relevant = !isempty(as)
    ahead = any(a -> a.s >= x[1] - D, as)                             # candidates only for an obstacle ahead
    attempts = Tuple{Symbol,Symbol,Int,Float64}[]
    results = Tuple{NMPCResult,Any,Symbol}[]
    longest = 0.0                                                     # longest NLP solve so far in this cycle
    run!(label, initial) = begin
        ta = time_ns()
        r, cc = obstacle_attempt!(ctrl, x, uprev, initial; cycle_start=t0)
        dt = (time_ns() - ta) / 1e9
        t_att += dt
        longest = max(longest, r.nlp_solves > 0 ? r.timing.optimize / r.nlp_solves : 0.0)
        push!(attempts, (label, r.status, r.nlp_solves, dt))
        push!(results, (r, cc, label))
        r
    end
    guess!(s) = begin
        tg = time_ns()
        g = side_guess(ctrl, x, s)
        push!(guesses, (s, g.clearance, g.rollout_ok ? g.track_clearance : -Inf))
        t_guess += (time_ns() - tg) / 1e9
        g
    end
    optional_ok() = cycle_budget_allows(c, t0, longest)
    side = ctrl.obstacle_side
    enc = ctrl.obstacle_encounter
    new_encounter = side === :none || ctrl.last_U === nothing || isnan(enc) || abs(assoc[1] - enc) > 2(D + E)
    if !relevant || !ahead
        run!(:default, nothing)
    elseif new_encounter
        sides = [s for (s, open) in ((:left, pl), (:right, pr)) if open]
        isempty(sides) && (sides = [:left, :right])
        warmU = ctrl.last_U
        if c.encounter_strategy === :sequential && length(sides) == 2
            gs = Dict{Symbol,Any}()
            for s in sides
                ctrl.last_U = warmU                                   # each guess uses the same previous plan
                gs[s] = guess!(s)
            end
            rank(s) = (gs[s].converged && gs[s].rollout_ok, gs[s].clearance, gs[s].track_clearance, s === :left)
            order = sort(sides; by=rank, rev=true)
            for (i, s) in enumerate(order)
                if i == 2 && any(rr.accepted for (rr, _, _) in results)
                    push!(attempts, (s, :not_attempted_sequential, 0, 0.0))
                    continue
                end
                if i == 2 && !optional_ok()
                    push!(attempts, (s, :not_attempted_budget, 0, 0.0))
                    continue
                end
                ctrl.last_U = warmU
                gs[s].converged ? run!(s, gs[s]) : push!(attempts, (s, :scheduling_failure, 0, 0.0))
            end
        else
            for (i, s) in enumerate(sides)
                if i == 2 && !optional_ok()
                    push!(attempts, (s, :not_attempted_budget, 0, 0.0))
                    continue
                end
                ctrl.last_U = warmU                                   # each guess uses the same previous plan
                g = guess!(s)
                g.converged ? run!(s, g) : push!(attempts, (s, :scheduling_failure, 0, 0.0))
            end
        end
        length(sides) == 1 && push!(attempts, (sides[1] === :left ? :right : :left, :not_attempted_blocked, 0, 0.0))
    else
        r = run!(:warm, nothing)
        if !r.accepted
            if optional_ok()
                g = guess!(side)
                g.converged && run!(side, g)
            else
                push!(attempts, (side, :not_attempted_budget, 0, 0.0))
            end
        end
    end
    acc = [(r, cc, l) for (r, cc, l) in results if r.accepted]
    chosen = isempty(acc) ? (isempty(results) ? nothing : results[end]) : acc[argmin([r.objective for (r, _, _) in acc])]
    if chosen === nothing                                             # no attempt could be started
        msg = "no obstacle-side initial guess could be built"
        return obstacle_failure(ctrl, t0, :scheduling_failure, msg, report(nothing, ctrl.obstacle_side, attempts, :initialization, msg))
    end
    r, cc, label = chosen
    if r.accepted
        ctrl.last_X, ctrl.last_U = r.X, r.U
        if relevant && ahead
            ctrl.obstacle_side = plan_side(ctrl, r.X)
            ctrl.obstacle_encounter = assoc[1]
        end
    else
        reset_warm_start!(ctrl)
    end
    cause = r.accepted ? :none : failure_cause(results, pl, pr, guesses, c.obstacle_buffer)
    sideout = relevant && ahead ? ctrl.obstacle_side : :none
    tsum(f) = sum(getfield(rr.timing, f) for (rr, _, _) in results; init=0.0)
    timing = NMPCTiming(tsum(:update), tsum(:warm_start), tsum(:optimize), tsum(:solver), tsum(:validation),
        (time_ns() - t0) / 1e9, tsum(:schedule), tsum(:reference))
    nsolves = sum(rr.nlp_solves for (rr, _, _) in results; init=0)
    msg = length(results) > 1 ? r.message * " [attempts: " * join(["$(a[1]) → $(a[2])" for a in attempts], ", ") * "]" : r.message
    # Reassignment diagnostics of all attempts (trace entries tagged with their attempt).
    rep = ReassignmentReport(Any[(; attempt=l, e...) for (rr, _, l) in results for e in rr.reassignment.trace],
        sum(rr.reassignment.cycles for (rr, _, _) in results; init=0), maximum((rr.reassignment.cycle_length for (rr, _, _) in results); init=0),
        sort!(unique(reduce(vcat, [rr.reassignment.cycle_knots for (rr, _, _) in results]; init=Int[]))),
        Any[(; attempt=l, b...) for (rr, _, l) in results for b in rr.reassignment.branch], r.reassignment.branch_selected,
        join(filter(!isempty, ["$(l): $(rr.reassignment.message)" for (rr, _, l) in results if !isempty(rr.reassignment.message)]), "; "))
    return with_fields(r; timing, nlp_solves=nsolves, message=msg, reassignment=rep,
        obstacle=report(cc, sideout, attempts, cause, r.accepted ? "" : r.message))
end

"A rejected result without any solve (the solver was not called)."
function obstacle_failure(ctrl, t0, status, msg, report)
    reset_warm_start!(ctrl)
    N = horizon(ctrl)
    return NMPCResult(status, false, MOI.OPTIMIZE_NOT_CALLED, MOI.NO_SOLUTION, fill(NaN, 4, N + 1), fill(NaN, 2, N), nothing, NaN, -1,
        NMPCTiming(0.0, 0.0, 0.0, NaN, 0.0, (time_ns() - t0) / 1e9, 0.0, 0.0), NaN, NaN, false, msg,
        empty_schedule_report(; model=:cartesian_endpoint_exact), REFERENCE_NOT_RUN, 0, EXACT_NOT_RUN, CORRIDOR_NOT_RUN, report,
        REASSIGNMENT_NONE)
end

"""
Classify why no attempt was accepted (see `ObstacleReport.failure_cause`). In order:
1. a collision (or buffer violation) in the dense evaluation;
2. a violated imposed constraint;
3. only invalid segment assignments (`:segment_assignment`);
4. if every attempt was a solver failure:
   - `:infeasible_geometry` when the lateral-gap heuristic finds no passage on either side
     (geometrically blocked, by that heuristic);
   - `:initialization` when only one initialisation was tried;
   - `:solver_convergence` when at least one side guess was itself free of contact at its
     knots (clearance above `D` of at least `-buffer`) and on the track: a dynamically
     consistent path existed, but Ipopt did not converge;
   - otherwise `:dynamically_infeasible`: no candidate guess cleared the obstacle within
     the track. This is suspected, not proven.
"""
function failure_cause(results, pl, pr, guesses=Tuple{Symbol,Float64,Float64}[], buffer=0.0)
    sts = [r.status for (r, _, _) in results]
    any(==(:collision_detected), sts) && return any(cc -> cc !== nothing && cc.collisions > 0, (cc for (_, cc, _) in results)) ? :collision : :separation
    any(==(:obstacle_constraint_violation), sts) && return :constraint_violation
    !isempty(sts) && all(s -> s in (:segment_assignment_invalid, :interior_assignment_invalid), sts) && return :segment_assignment
    if !isempty(sts) && all(s -> s in (:solver_failure, :invalid_solution), sts)
        !(pl || pr) && return :infeasible_geometry
        length(results) < 2 && return :initialization
        any(g -> g[2] >= -buffer && g[3] >= 0, guesses) && return :solver_convergence
        return :dynamically_infeasible
    end
    return :other
end
