# Between-knot corridor constraints for the Cartesian-endpoint NMPC (Milestone 2B.5):
# interior sample points, their segment assignment and validation, the dense exact-map
# corridor check, and an analytic continuous-time clearance check (validation only).
# See docs/intersample_corridor_constraints.md.

"Interior sample fractions `alpha` of each interval for `interior_samples ∈ (0, 1, 3)`."
interior_alphas(n::Integer) = n == 0 ? Float64[] : n == 1 ? [0.5] : n == 3 ? [0.25, 0.5, 0.75] :
    throw(ArgumentError("interior_samples must be 0, 1 or 3, got $n"))

"""
    interior_point(L, pose, v, delta, a, tau, sincf=sinc_un) -> (px, py)

Exact position of the rear axle `tau` seconds into an interval that starts at Cartesian
`pose = (X, Y, psi)` with speed `v` and the constant input `(delta, a)`. It is the
position part of [`exact_bicycle_increment`](@ref) over `tau`. Generic, so the same
function builds the NLP expression.
"""
function interior_point(L, pose, v, delta, a, tau, sincf=sinc_un)
    inc = exact_bicycle_increment(L, pose[3], v, delta, a, tau, sincf)
    return (pose[1] + inc[1], pose[2] + inc[2])
end

"""
    corridor_parameters(frame, E) -> NTuple{8,Float64}

Constants `(sel, qx, qy, nx, ny, ir, lo2, hi2)` of the corridor inequalities for one
interior sample assigned to the segment `frame` (see [`segment_frame`](@ref)), with
corridor half-width `E`:
- **straight segment** (`sel = 0`): `q` is the segment's start point and `n` its left unit
  normal. `ir`, `lo2` and `hi2` are finite dummies (0);
- **curved segment** (`sel = 1`): `q` is the centre
  `O = (x0, y0) + (1/kappa)·(-sin psi0, cos psi0)`, `ir = 1/(2R)`, `lo2 = (R - E)²` and
  `hi2 = (R + E)²` with `R = 1/|kappa|`. `n = 0` is a dummy.

Both curvature signs are covered: the centre lies on the left normal for `kappa > 0` and on
the right normal for `kappa < 0`.
"""
function corridor_parameters(frame, E)
    s, c = sin(frame.psi0), cos(frame.psi0)
    frame.kappa == 0 && return (0.0, Float64(frame.x0), Float64(frame.y0), -s, c, 0.0, 0.0, 0.0)
    R = 1 / abs(frame.kappa)
    return (1.0, frame.x0 - s / frame.kappa, frame.y0 + c / frame.kappa, 0.0, 0.0, 1 / (2R), (R - E)^2, (R + E)^2)
end

"""
    corridor_margins(par, px, py, E) -> (lower, upper)

The two corridor inequalities `lower ≥ 0`, `upper ≥ 0` at the point `(px, py)`, with
`par = (sel, qx, qy, nx, ny, ir, lo2, hi2)` from [`corridor_parameters`](@ref):

    straight (sel = 0):  g = n·(p - q),   lower = g + E,   upper = E - g        (-E ≤ g ≤ E)
    curved   (sel = 1):  r² = |p - q|²,   lower = (r² - (R-E)²)/(2R),   upper = ((R+E)² - r²)/(2R)

The persistent form is `sel·(curved) + (1 - sel)·(straight)`, so the Parameter `sel`
switches between the two without rebuilding the model. Dividing by `2R` makes the curved
margins approximately metric near the arc: `(r² - (R-E)²)/(2R) ≈ r - (R - E)`. Generic in
the number type (Float64, duals, JuMP expressions).
"""
function corridor_margins(par, px, py, E)
    sel, qx, qy, nx, ny, ir, lo2, hi2 = par
    dx, dy = px - qx, py - qy
    g = nx * dx + ny * dy
    r2 = dx^2 + dy^2
    return (sel * (ir * (r2 - lo2)) + (1 - sel) * (g + E), sel * (ir * (hi2 - r2)) + (1 - sel) * (E - g))
end

"""
    sample_projection(track, X, Y, s_hint; max_lateral, window=1.0, tol=1e-9) -> NamedTuple

Independent projection of a point onto the actual track near the progress `s_hint`. Every
segment is tried with the orthogonal projection of `cartesian_to_frenet`. A candidate
counts if `|ey| ≤ max_lateral` and its (lap-unwrapped) progress is within `window` of the
hint. Duplicates at a shared boundary point are merged.

Returns `(; status, s, ey)` with `status`:
- `:ok`: exactly one candidate;
- `:missing`: no candidate;
- `:ambiguous`: several distinct candidates. `s` and `ey` then belong to the one nearest the
  hint.
"""
function sample_projection(track::Track, X::Real, Y::Real, s_hint::Real; max_lateral::Real, window::Real=1.0, tol::Real=1e-9)
    L = track_length(track)
    best, n, cands = (NaN, NaN), 0, Tuple{Float64,Float64}[]
    for (i, seg) in enumerate(track.segments)
        proj = project_onto_segment(seg, track.poses[i], X, Y, tol)
        proj === nothing && continue
        d, ey = proj
        abs(ey) <= max_lateral + tol || continue
        s = track.s_start[i] + d
        track.closed && (s = s_hint + rem(s - s_hint, L, RoundNearest))
        abs(s - s_hint) <= window || continue
        any(c -> abs(c[1] - s) <= 1e-7 && abs(c[2] - ey) <= 1e-7, cands) && continue
        push!(cands, (s, ey))
    end
    isempty(cands) && return (; status=:missing, s=NaN, ey=NaN)
    s, ey = cands[argmin([abs(c[1] - s_hint) for c in cands])]
    return (; status=length(cands) == 1 ? :ok : :ambiguous, s, ey)
end

"Lateral search radius for interior projections: the track width, but below the smallest radius of curvature."
function projection_radius(track::Track)
    kmax = maximum(abs(seg.curvature) for seg in track.segments)
    r = track.width > 0 ? track.width : 1.0
    return kmax > 0 ? min(r, 0.99 / kmax) : r
end

"""
    interior_assignment(ctrl, Xn, Un, frames) -> (; ids, frames, hints, fallbacks)

Segment of every interior sample of the nominal trajectory `(Xn, Un)`.
1. Map the nominal knot `k` with its assigned frame.
2. Compute the exact interior point at `alpha·dt`.
3. Project it with [`sample_projection`](@ref), using the linearly interpolated nominal
   progress as the hint.
4. Assign the segment containing the projection (half-open: a point on a boundary enters
   the next segment).

If the projection is missing or ambiguous, the segment containing the hint is used
instead, and the sample is counted in `fallbacks`. Columns are ordered `(k, alpha)` with
`alpha` fastest.
"""
function interior_assignment(ctrl::NMPCController, Xn, Un, frames)
    track, N, dt, L = ctrl.track, horizon(ctrl), ctrl.config.dt, ctrl.vehicle.params.wheelbase
    alphas = ctrl.alphas
    maxlat = projection_radius(track)
    ids, hints, nfb = Int[], Float64[], 0
    for k in 1:N, a in alphas
        hint = Xn[1, k] + a * (Xn[1, k+1] - Xn[1, k])
        pose = frenet_pose_local(frames[k], Xn[1, k], Xn[2, k], Xn[3, k])
        px, py = interior_point(L, pose, Xn[4, k], Un[1, k], Un[2, k], a * dt)
        pr = sample_projection(track, px, py, hint; max_lateral=maxlat)
        s = pr.status === :ok ? pr.s : (nfb += 1; hint)
        if !track.closed                                   # keep open-track look-ups in range
            s = clamp(s, 0.0, track_length(track))
        end
        push!(ids, segment_id(track, s, :right))
        push!(hints, hint)
    end
    return (; ids, frames=[segment_frame(track, id) for id in ids], hints, fallbacks=nfb)
end

"Write the interior-sample corridor Parameters (`ctrl.iframe`) for an interior assignment."
function install_interior!(ctrl::NMPCController, samp)
    for (col, f) in enumerate(samp.frames)
        for (r, v) in enumerate(corridor_parameters(f, ctrl.ey_max))
            set_parameter_value(ctrl.iframe[r, col], v)
        end
    end
    return nothing
end

"""
    interior_assignment_check(controller, sched, core) -> NamedTuple

Post-solve validation of the interior samples. For every sample:
1. Reconstruct its position from the optimised variables, as the NLP does (knot `k` in its
   assigned frame, exact step over `alpha·dt`).
2. Project the position independently onto the actual track ([`sample_projection`](@ref),
   with the optimised progress as the hint).
3. Require a unique projection inside the sample's assigned segment, within
   `boundary_knot_tol`.

A point that satisfies the full-circle annulus but projects outside its finite arc is
therefore detected.

Returns `(; failures, missing, ambiguous, inconsistent, min_margin, message)`.
`min_margin` is the smallest corridor-inequality value at the samples, recomputed in
`Float64`.
"""
function interior_assignment_check(ctrl::NMPCController, sched, core)
    track, N, dt, L = ctrl.track, horizon(ctrl), ctrl.config.dt, ctrl.vehicle.params.wheelbase
    X, U, frames, samp = core.X, core.U, sched.asg.frames, sched.asg.samples
    tol = ctrl.config.boundary_knot_tol
    maxlat = projection_radius(track)
    nmiss, namb, ninc, mmin = 0, 0, 0, Inf
    first_bad = ""
    col = 0
    for k in 1:N, a in ctrl.alphas
        col += 1
        pose = frenet_pose_local(frames[k], X[1, k], X[2, k], X[3, k])
        px, py = interior_point(L, pose, X[4, k], U[1, k], U[2, k], a * dt)
        mmin = min(mmin, corridor_margins(corridor_parameters(samp.frames[col], ctrl.ey_max), px, py, ctrl.ey_max)...)
        hint = X[1, k] + a * (X[1, k+1] - X[1, k])
        pr = sample_projection(track, px, py, hint; max_lateral=maxlat)
        f = samp.frames[col]
        bad = if pr.status === :missing
            nmiss += 1; "no projection"
        elseif pr.status === :ambiguous
            namb += 1; "ambiguous projection"
        elseif !frame_valid(track, f, pr.s, tol)
            ninc += 1; "projects to s = $(fmt(pr.s)) outside its assigned segment [$(fmt(f.s0)), $(fmt(f.s1))]"
        else
            ""
        end
        isempty(bad) || isempty(first_bad) || continue
        isempty(bad) || (first_bad = "stage $k, alpha = $a: " * bad)
    end
    n = nmiss + namb + ninc
    msg = n == 0 ? "" : "$n interior sample(s) with an invalid segment assignment (missing $nmiss, ambiguous $namb, outside $ninc); first: $first_bad"
    return (; failures=n, missing=nmiss, ambiguous=namb, inconsistent=ninc, min_margin=mmin, message=msg)
end

"""
    corridor_check(controller, x, X, U; samples=20, interior=nothing) -> CorridorReport

Dense independent check of a plan `(X, U)` from the measured state `x`, using the exact
Cartesian map and the true track projection.
1. The knots are reconstructed with [`exact_cartesian_step`](@ref) from the true Cartesian
   pose of `x`.
2. Every interval is sampled at `samples` equally spaced times (`j·dt/samples`,
   `j = 0 … samples-1`), plus the final knot.
3. Each sample is projected with [`sample_projection`](@ref) (hint: linearly interpolated
   predicted progress).

Reported: the minimum physical clearance `width/2 - |ey|` and corridor clearance
`ey_max - |ey|` with the time and stage of the worst one, the number of samples outside
the corridor (beyond `acceptance_tol`), the largest excess, and the projection failures. In
the Cartesian-endpoint mode with interior samples, `interior_prediction_error` compares the
NLP's interior points with the exact reconstruction at the same times.

This is a finite sampling: it is not a continuous-time guarantee (see
[`analytic_corridor_clearance`](@ref)). `interior` (from
[`interior_assignment_check`](@ref)) is copied into the report.
"""
function corridor_check(ctrl::NMPCController, x, X::AbstractMatrix, U::AbstractMatrix; samples::Integer=20, interior=nothing)
    track, N, dt, veh = ctrl.track, horizon(ctrl), ctrl.config.dt, ctrl.vehicle
    L, tol, W = veh.params.wheelbase, ctrl.config.acceptance_tol, track.width / 2
    maxlat = projection_radius(track)
    z = try
        frenet_state_to_cartesian(track, x)
    catch err
        err isa DomainError || rethrow()
        return CorridorReport(samples, NaN, NaN, NaN, 0, 0, NaN, 0, interior_fields(ctrl, interior)..., NaN,
            "measured state outside the Frenet-valid region")
    end
    worst, wt, wk, nviol, maxv, nfail = Inf, NaN, 0, 0, 0.0, 0
    minw = Inf
    perr = isempty(ctrl.cframe) || isempty(ctrl.alphas) ? NaN : 0.0
    for k in 1:N
        u = SVector{2,Float64}(U[1, k], U[2, k])
        js = k == N ? (0:samples) : (0:samples-1)
        for j in js
            tau = j * dt / samples
            px, py = j == 0 ? (z[1], z[2]) : interior_point(L, z, z[4], u[1], u[2], tau)
            hint = X[1, k] + (j / samples) * (X[1, k+1] - X[1, k])
            pr = sample_projection(track, px, py, hint; max_lateral=maxlat)
            if pr.status !== :ok
                nfail += 1
                continue
            end
            cl = ctrl.ey_max - abs(pr.ey)
            minw = min(minw, W - abs(pr.ey))
            if cl < worst
                worst, wt, wk = cl, (k - 1) * dt + tau, k
            end
            if cl < -tol
                nviol += 1
                maxv = max(maxv, -cl)
            end
        end
        if !isnan(perr)
            pk = frenet_pose_local(segment_frame(track, segment_id(track, X[1, k], :right)), X[1, k], X[2, k], X[3, k])
            for a in ctrl.alphas
                q = interior_point(L, pk, X[4, k], U[1, k], U[2, k], a * dt)
                r = interior_point(L, z, z[4], U[1, k], U[2, k], a * dt)
                perr = max(perr, hypot(q[1] - r[1], q[2] - r[2]))
            end
        end
        z = exact_cartesian_step(veh, z, u, dt)
    end
    msg = nfail > 0 ? "$nfail sample(s) without a unique track projection" : ""
    return CorridorReport(samples, minw, worst, wt, wk, nviol, maxv, nfail, interior_fields(ctrl, interior)..., perr, msg)
end

interior_fields(ctrl, interior) = interior === nothing ? (length(ctrl.alphas) * horizon(ctrl), 0, NaN) :
    (length(ctrl.alphas) * horizon(ctrl), interior.failures, interior.min_margin)

"""
    analytic_corridor_clearance(controller, x, X, U) -> NamedTuple

Continuous-time minimum clearance of a plan, computed analytically. **Validation only**;
it is not part of the optimisation.

**Why it works.** In each interval the rear axle follows an arc of the circle (or line)
with curvature `c = tan(delta)/L`, parametrised by arc length
`sigma ∈ [0, d]`, `d = v dt + a dt²/2`, with heading `psi(sigma) = psi0 + c·sigma`. Where
the true track projection is unique, `ey(sigma)` is piecewise smooth:
- each piece is the signed distance to a line, or `±(R - |p - O|)` for an arc;
- pieces change only where the point crosses the normal line at a segment boundary.

Extremes of `|ey|` therefore occur at:
1. the interval end points;
2. points where the vehicle heading is parallel to a straight segment
   (`psi = psi_i + mπ`);
3. points where the vehicle circle's centre, the point and an arc's centre are collinear
   (`(p - O)·t = 0`, i.e. `psi = angle(C_v - O) ± π/2 + mπ`);
4. boundary crossings: roots of `t_b·(p(sigma) - b) = 0`, i.e.
   `sin(psi - psi_b) = -(c·t_b·(p0 - b) + sin(psi_b - psi0))`.

All four are evaluated in closed form, for every segment of the track. The clearance is
then evaluated exactly at these candidates, with the true projection. Superfluous
candidates do no harm.

**Scope.**
- It assumes a unique, continuous projection along the arc: `|ey|` below the smallest
  radius of curvature, and no other track part within the search window.
- Candidates are computed in `Float64`.
- For `|c·d| < 1e-9` the path is treated as straight for the candidate search (critical
  points of a line: none; of a circle: the foot of the perpendicular).

Returns `(; min_track, min_corridor, worst_time, worst_stage, candidates, failures)`.
"""
function analytic_corridor_clearance(ctrl::NMPCController, x, X::AbstractMatrix, U::AbstractMatrix)
    track, N, dt, veh = ctrl.track, horizon(ctrl), ctrl.config.dt, ctrl.vehicle
    L, W = veh.params.wheelbase, track.width / 2
    maxlat = projection_radius(track)
    z = frenet_state_to_cartesian(track, x)
    mt, mc, wt, wk, ncand, nfail = Inf, Inf, NaN, 0, 0, 0
    for k in 1:N
        delta, a, v = U[1, k], U[2, k], z[4]
        d = v * dt + a * dt^2 / 2
        c = tan(delta) / L
        p0, psi0 = (z[1], z[2]), z[3]
        sig = Float64[0.0, d]
        straight = abs(c * d) < 1e-9
        # Heading targets psi = base + m·step inside [psi0, psi0 + c d].
        function add_heading!(base, step)
            straight && return
            lo, hi = minmax(psi0, psi0 + c * d)
            for m in ceil(Int, (lo - base) / step):floor(Int, (hi - base) / step)
                push!(sig, (base + m * step - psi0) / c)
            end
        end
        for (i, seg) in enumerate(track.segments)
            xb, yb, psib = track.poses[i]
            if seg.curvature == 0
                add_heading!(psib, π)                                      # heading parallel to the line
            else
                O = (xb - sin(psib) / seg.curvature, yb + cos(psib) / seg.curvature)
                if straight
                    push!(sig, (O[1] - p0[1]) * cos(psi0) + (O[2] - p0[2]) * sin(psi0))
                else
                    Cv = (p0[1] - sin(psi0) / c, p0[2] + cos(psi0) / c)
                    add_heading!(atan(Cv[2] - O[2], Cv[1] - O[1]) + π / 2, π)
                end
            end
            # Crossing of the normal line at the start of segment i: t_b·(p(sigma) - b) = 0.
            tb = (cos(psib), sin(psib))
            g0 = tb[1] * (p0[1] - xb) + tb[2] * (p0[2] - yb)
            if straight
                den = tb[1] * cos(psi0) + tb[2] * sin(psi0)
                abs(den) > 1e-12 && push!(sig, -g0 / den)
            else
                rhs = -(c * g0 + sin(psib - psi0))
                if abs(rhs) <= 1
                    as = asin(rhs)
                    add_heading!(psib + as, 2π)
                    add_heading!(psib + π - as, 2π)
                end
            end
        end
        dsk = X[1, k+1] - X[1, k]
        for sg in sig
            0 <= sg <= d || continue
            ncand += 1
            th = c * sg
            cc = sg * sinc_un(th / 2)
            px, py = p0[1] + cc * cos(psi0 + th / 2), p0[2] + cc * sin(psi0 + th / 2)
            pr = sample_projection(track, px, py, X[1, k] + (d > 0 ? sg / d : 0.0) * dsk; max_lateral=maxlat)
            if pr.status !== :ok
                nfail += 1
                continue
            end
            mt = min(mt, W - abs(pr.ey))
            cl = ctrl.ey_max - abs(pr.ey)
            if cl < mc
                tau = abs(a) < 1e-12 ? (v > 0 ? sg / v : 0.0) : (-v + sqrt(max(v^2 + 2a * sg, 0.0))) / a
                mc, wt, wk = cl, (k - 1) * dt + tau, k
            end
        end
        z = exact_cartesian_step(veh, z, SVector(delta, a), dt)
    end
    return (; min_track=mt, min_corridor=mc, worst_time=wt, worst_stage=wk, candidates=ncand, failures=nfail)
end
