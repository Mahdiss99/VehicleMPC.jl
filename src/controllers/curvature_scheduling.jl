# Stagewise curvature schedule for the NMPC on multi-segment tracks (Milestone 2B.2):
# fixed-point construction before the solve, consistency residuals after it, and the
# independent event-aware validation on the actual track. See docs/multisegment_nmpc.md.

"""
    stage_curvatures(track, s; ds_min=AVERAGE_CURVATURE_DS_MIN) -> Vector{Float64}

Stagewise average curvature over the nominal progress `s[1:N+1]` (as
`curvature_schedule(track, s)`). A stage whose nominal progress changes by less than
`ds_min` uses the curvature of the segment containing `s[k]` (the right limit, half-open
segments). Under the forward-driving model (`v ≥ 0`, `|epsi| ≤ epsi_max < π/2`,
hence `sdot ≥ 0`) that is the segment the vehicle can enter. This makes a stationary state
exactly on a curvature boundary well defined, instead of the ambiguous zero-length limit
that `average_curvature` rejects. Throws a `DomainError` outside an open track.
"""
function stage_curvatures(track::Track, s::AbstractVector; ds_min::Real=AVERAGE_CURVATURE_DS_MIN)
    return [abs(s[k+1] - s[k]) < ds_min ? curvature(track, s[k]) : average_curvature(track, s[k], s[k+1]; ds_min)
            for k in 1:length(s)-1]
end

"""
    set_curvature_schedule!(controller, kappas)

Write the stagewise curvature into the NLP's `kappa[k]` Parameters. Every value must lie in
the controller's legal range `[kappa_min, kappa_max]` (up to 1e-9). The constructor proved
the RK4 stage-point bounds Frenet-safe on that range; values outside it throw an
`ArgumentError`, because they would break that guarantee.
"""
function set_curvature_schedule!(ctrl::NMPCController, kappas::AbstractVector)
    length(kappas) == horizon(ctrl) || throw(DimensionMismatch("need $(horizon(ctrl)) curvature values, got $(length(kappas))"))
    kmin, kmax = ctrl.kappa_range
    for (k, v) in enumerate(kappas)
        (isfinite(v) && kmin - 1e-9 <= v <= kmax + 1e-9) ||
            throw(ArgumentError("curvature $v for stage $k is outside the legal range [$kmin, $kmax]"))
        set_parameter_value(ctrl.kappa[k], v)
    end
    return ctrl
end

"""
    segment_id(track, s, side) -> Int

Unique id of the segment containing unwrapped progress `s`: `lap·n + i` (`lap = 0` on an
open track).
- `side = :right` uses the half-open convention `[s_i, s_{i+1})`;
- `side = :left` uses `(s_i, s_{i+1}]`, i.e. the segment that ends at a boundary.

Used to compare the segment configuration of a stage before and after the solve.
"""
function segment_id(track::Track, s::Real, side::Symbol)
    n = num_segments(track)
    lap, sw = split_progress(track, s)
    if side === :right
        return lap * n + segment_index(track, sw)
    end
    j = searchsortedfirst(track.s_start, sw) - 1
    if j < 1
        track.closed || return 1                         # open track start: no left segment
        j, lap = n, lap - 1
    end
    return lap * n + min(j, n)
end

"Segment configuration of a stage `[a, b]` with `b ≥ a`: (segment entered at `a`, segment left at `b`)."
stage_configuration(track::Track, a::Real, b::Real) =
    b >= a ? (segment_id(track, a, :right), segment_id(track, b, :left)) :
             (segment_id(track, a, :left), segment_id(track, b, :right))

"""
    curvature_linearization(track, s; min_interval=1e-3, knot_tol=1e-6, ds_min=AVERAGE_CURVATURE_DS_MIN)

First-order model of the stagewise average curvature around nominal knots `s[1:N+1]`:

    K(a, b) = (psi_c(b) - psi_c(a)) / (b - a)
    ∂K/∂a = (K - kappa(a)) / (b - a)          ∂K/∂b = (kappa(b) - K) / (b - a)

These derivatives hold wherever `kappa` is continuous at the end point, i.e. where the
end point is not on a segment boundary.

Returns `(; kbar, A, B, boundary_knots, short_intervals)` with `kbar_k = K(s_k, s_{k+1})`
(as `stage_curvatures`), `A_k = ∂K/∂a` and `B_k = ∂K/∂b`. Special cases:

- **No boundary in the closed interval:** `K` is constant in a neighbourhood, so
  `A = B = 0` is set exactly. It is not computed from heading differences, whose rounding
  error is of order `eps·|psi|/Δs²`.
- **`|Δs| < min_interval`** (short or stationary interval): constant local model
  `A = B = 0`, counted in `short_intervals`. `kbar` keeps its value; below `ds_min` that
  value is the right-limit curvature (see `stage_curvatures`).
- **End point within `knot_tol` of a boundary:** the ordinary derivative does not exist.
  The mean of the left and right one-sided derivatives is used, i.e. `kappa` at that knot
  is replaced by `(kappa_left + kappa_right)/2`. The knot is counted in `boundary_knots`.

Closed-track lap boundaries are handled through unwrapped progress. No curvature is
smoothed.
"""
function curvature_linearization(track::Track, s::AbstractVector; min_interval::Real=1e-3, knot_tol::Real=1e-6,
    ds_min::Real=AVERAGE_CURVATURE_DS_MIN)
    N = length(s) - 1
    kbar = stage_curvatures(track, s; ds_min)
    A, B = zeros(N), zeros(N)
    nb, nshort = 0, 0
    # One-sided curvature at a knot; the averaged value if the knot is on a boundary.
    on_boundary(x) = !isempty(segment_boundaries(track, x - knot_tol, x + knot_tol))
    function knot_curvature(x)
        on_boundary(x) || return curvature(track, x), false
        kl = curvature(track, x - 2knot_tol)
        kr = curvature(track, x + 2knot_tol)
        return (kl + kr) / 2, true
    end
    for k in 1:N
        a, b = s[k], s[k+1]
        d = b - a
        if abs(d) < min_interval
            nshort += 1
            continue
        end
        ba, bb = on_boundary(a), on_boundary(b)
        if !ba && !bb && isempty(segment_boundaries(track, a, b))
            continue                                     # one segment: K constant, A = B = 0 exactly
        end
        ka, fa = knot_curvature(a)
        kb, fb = knot_curvature(b)
        nb += fa + fb
        A[k] = (kbar[k] - ka) / d
        B[k] = (kb - kbar[k]) / d
    end
    return (; kbar, A, B, boundary_knots=nb, short_intervals=nshort)
end

"""
    heading_increment(track, a, b) -> Float64

Centreline heading increment `psi_c(b) - psi_c(a)`, computed locally as the curvature
integral `Σ_j kappa_j · |[a, b] ∩ segment_j|` (signed by the direction from `a` to `b`).
No global heading offset enters, so the result does not lose precision on later laps.
Unwrapped progress is used on closed tracks. Outside an open track a `DomainError` is
thrown.
"""
function heading_increment(track::Track, a::Real, b::Real)
    a == b && return 0.0
    lo, hi = minmax(Float64(a), Float64(b))
    sgn = b >= a ? 1.0 : -1.0
    L, total = track_length(track), 0.0
    if track.closed
        laps = floor(Int, lo / L):floor(Int, hi / L)
    else
        split_progress(track, lo); split_progress(track, hi)   # range check
        laps = 0:0
    end
    for lap in laps, (j, seg) in enumerate(track.segments)
        seg.curvature == 0 && continue
        slo, shi = lap * L + track.s_start[j], lap * L + track.s_start[j+1]
        total += seg.curvature * max(0.0, min(hi, shi) - max(lo, slo))
    end
    return sgn * total
end

"""
    segment_pair_assignment(track, s; min_interval=1e-3, knot_tol=1e-6)

Segment assignment and coefficients of the exact segment-pair curvature model for nominal
knots `s[1:N+1]`.

**Knots.** Knot `k` is assigned the segment containing `s[k]` under the half-open
convention, i.e. the entering segment for a knot exactly on a boundary. The centreline
heading is affine on that segment's closed interval `[lo_k, hi_k]` (unwrapped).

**Stages.** Stage `k` gets one of three forms:
- **crossing** (`w = 1`): `kappa_eff·(s_{k+1} - s_k) = dpsi + kj (s_{k+1} - sbar_{k+1}) - ki (s_k - sbar_k)`,
  with `dpsi = heading_increment(sbar_k, sbar_{k+1})`. It is exact as long as
  `s_k ∈ [lo_k, hi_k]` and `s_{k+1} ∈ [lo_{k+1}, hi_{k+1}]`, including several interior
  segments.
- **same segment** (`w = 0`): `kappa_eff = kc` with the segment's curvature. This is exact
  while both knots stay in that segment.
- **short or stationary** (`|Δsbar| < min_interval`, `w = 0`): `kappa_eff = kc`, the
  *entering* segment's curvature (that of knot `k`). There is no division and no free
  curvature at zero progress. The heading error is at most `|Δkappa|·min_interval` if the
  stage straddles a boundary; such stages are counted in `short_intervals`.

Returns `(; w, dpsi, ki, kj, kc, lo, hi, seg, kbar, short_intervals, boundary_knots)`.
`kbar` is the nominal stage curvature, kept for reporting.
"""
function segment_pair_assignment(track::Track, s::AbstractVector; min_interval::Real=1e-3, knot_tol::Real=1e-6)
    N = length(s) - 1
    L, n = track_length(track), num_segments(track)
    seg = [segment_id(track, x, :right) for x in s]
    lap_of(id) = fld(id - 1, n)
    idx_of(id) = mod(id - 1, n) + 1
    lo = [lap_of(id) * L + track.s_start[idx_of(id)] for id in seg]
    hi = [lap_of(id) * L + track.s_start[idx_of(id)+1] for id in seg]
    kap = [track.segments[idx_of(id)].curvature for id in seg]
    w, dpsi, kc, kbar = zeros(N), zeros(N), zeros(N), zeros(N)
    nshort = 0
    for k in 1:N
        d = s[k+1] - s[k]
        if abs(d) < min_interval
            nshort += 1
            kc[k] = kap[k]                                 # entering segment
            kbar[k] = kc[k]
        elseif seg[k] == seg[k+1]
            kc[k] = kap[k]
            kbar[k] = kc[k]
        else
            w[k] = 1.0
            dpsi[k] = heading_increment(track, s[k], s[k+1])
            kbar[k] = dpsi[k] / d
        end
    end
    nb = count(x -> !isempty(segment_boundaries(track, x - knot_tol, x + knot_tol)), s)
    return (; w, dpsi, ki=kap[1:N], kj=kap[2:N+1], kc, lo, hi, seg, kbar, short_intervals=nshort, boundary_knots=nb)
end

"""
Write a schedule into the NLP: the stagewise curvature Parameters and, in the
progress-linearized or segment-pair modes, their additional Parameters (and, with
`segment_membership`, the knot bounds). Returns the schedule extended by `lin` (the
mode's coefficients, or `nothing`). In the Cartesian-endpoint mode it installs the segment
assignment instead (`install_cartesian!`) and adds `asg`.
"""
function install_schedule!(ctrl::NMPCController, sched)
    set_curvature_schedule!(ctrl, sched.kappas)
    isempty(ctrl.cframe) || return install_cartesian!(ctrl, sched)
    isempty(ctrl.kappa_eff) && return (; sched..., lin=nothing)
    isempty(ctrl.sp_w) || return install_segment_pair!(ctrl, sched)
    c = ctrl.config
    lin = curvature_linearization(ctrl.track, sched.s_nominal; min_interval=c.linearization_min_interval,
        knot_tol=c.boundary_knot_tol)
    for k in 1:horizon(ctrl)
        set_parameter_value(ctrl.lin_kbar[k], lin.kbar[k])
        set_parameter_value(ctrl.lin_A[k], lin.A[k])
        set_parameter_value(ctrl.lin_B[k], lin.B[k])
    end
    N = horizon(ctrl)
    for k in 1:N+1
        set_parameter_value(ctrl.lin_sbar[k], sched.s_nominal[k])
    end
    # Optional trust region: knots bounding a stage with non-zero sensitivities.
    tr = fill(Inf, N + 1)
    if !isempty(ctrl.lin_tr)
        rho = c.linearization_trust_region
        for k in 1:N
            (lin.A[k] != 0 || lin.B[k] != 0) || continue
            r = rho * abs(sched.s_nominal[k+1] - sched.s_nominal[k])
            tr[k] = min(tr[k], r)
            tr[k+1] = min(tr[k+1], r)
        end
        for k in 2:N+1
            set_parameter_value(ctrl.lin_tr[k], isfinite(tr[k]) ? tr[k] : 1e6)
        end
    end
    return (; sched..., lin=(; lin..., tr))
end

function install_segment_pair!(ctrl::NMPCController, sched)
    c, N = ctrl.config, horizon(ctrl)
    sp = segment_pair_assignment(ctrl.track, sched.s_nominal; min_interval=c.linearization_min_interval,
        knot_tol=c.boundary_knot_tol)
    for k in 1:N
        set_parameter_value(ctrl.sp_w[k], sp.w[k])
        set_parameter_value(ctrl.sp_dpsi[k], sp.dpsi[k])
        set_parameter_value(ctrl.sp_ki[k], sp.ki[k])
        set_parameter_value(ctrl.sp_kj[k], sp.kj[k])
        set_parameter_value(ctrl.sp_kc[k], sp.kc[k])
    end
    for k in 1:N+1
        set_parameter_value(ctrl.lin_sbar[k], sched.s_nominal[k])
    end
    if c.segment_membership                        # approach B: knots stay in their segments
        for k in 2:N+1
            set_lower_bound(ctrl.X[1, k], sp.lo[k])
            set_upper_bound(ctrl.X[1, k], sp.hi[k])
        end
    end
    lin = (; kbar=sp.kbar, A=zeros(N), B=zeros(N), boundary_knots=sp.boundary_knots, short_intervals=sp.short_intervals,
        tr=fill(Inf, N + 1), sp)
    return (; sched..., lin)
end

"Rollout with the checked prediction model; `nothing` if a step leaves the model domain."
function strict_rollout(ctrl::NMPCController, x, U, kappas)
    N, dt = horizon(ctrl), ctrl.config.dt
    X = Matrix{Float64}(undef, 4, N + 1)
    X[:, 1] = x
    for k in 1:N
        try
            X[:, k+1] = discrete_dynamics(ctrl.vehicle, X[:, k], U[:, k], kappas[k], dt, ctrl.config.integrator)
        catch err
            err isa DomainError || rethrow()
            return nothing
        end
    end
    return X
end

"""
    curvature_fixed_point(controller, x, U, s_init) -> NamedTuple

Self-consistent curvature schedule for the input sequence `U` from the measured state `x`,
starting from the nominal progress `s_init[1:N+1]`:

    repeat:  kappa = stage_curvatures(track, s);  X = rollout(x, U, kappa)
             if max_k |X[1,k] - s_k| ≤ schedule_tol: converged (return kappa, X and s)
             s = X[1, :]

This stops after `schedule_max_iter` iterations. At convergence, the returned `kappas` were
computed from `s_nominal`, and the rollout with them reproduces `s_nominal` within
`schedule_tol`. Convergence is **not** guaranteed: the result reports `converged = false`
together with the last progress change, and is then never used for a solve.
"""
function curvature_fixed_point(ctrl::NMPCController, x, U, s_init)
    c = ctrl.config
    s = collect(Float64, s_init)
    change = Inf
    for it in 1:c.schedule_max_iter
        kappas = try
            stage_curvatures(ctrl.track, s)
        catch err
            err isa DomainError || rethrow()
            return (; converged=false, iterations=it, change, kappas=Float64[], Xg=fill(NaN, 4, length(s)), s_nominal=s,
                message="nominal progress leaves the track ($(sprint(showerror, err)))")
        end
        X = strict_rollout(ctrl, x, U, kappas)
        X === nothing && return (; converged=false, iterations=it, change, kappas, Xg=fill(NaN, 4, length(s)), s_nominal=s,
            message="rollout with the schedule leaves the model domain")
        change = maximum(abs, X[1, :] - s)
        change <= c.schedule_tol && return (; converged=true, iterations=it, change, kappas, Xg=X, s_nominal=s, message="")
        s = X[1, :]
    end
    return (; converged=false, iterations=c.schedule_max_iter, change, kappas=Float64[], Xg=fill(NaN, 4, length(s)),
        s_nominal=s, message="fixed point not converged after $(c.schedule_max_iter) iterations (last change $(change) m)")
end

"""
Initial guess and curvature schedule for one control cycle.

- **Single-segment track:** the exact segment curvature on every stage (`:constant`, no
  averaging), with the Milestone 2A warm/cold guesses.
- **Warm start:**
  1. shift the previous inputs;
  2. take the nominal progress from the shifted previous solution, re-anchored at the
     measured progress;
  3. run the fixed point.

  If it does not converge, one cold initialisation is tried and reported as
  `:cold_fallback`.
- **Cold start:**
  1. build a quasi-steady centreline speed profile (`quasi_steady_guess`): accelerate and
     brake within the limits, `v ≤ sqrt(ay_max/|kappa|)`, curvature feed-forward steering;
  2. run the fixed point with those inputs.
"""
function initial_schedule(ctrl::NMPCController, x, warm::Bool)
    N, dt, track = horizon(ctrl), ctrl.config.dt, ctrl.track
    if num_segments(track) == 1
        kappas = fill(track.segments[1].curvature, N)
        Xg, Ug = warm ? warm_start_guess(ctrl, x, kappas) : cold_start_guess(ctrl, x, kappas)
        return (; converged=true, iterations=0, change=0.0, kappas, Xg, Ug, s_nominal=Xg[1, :], method=:constant, message="")
    end
    if warm
        Ug = hcat(ctrl.last_U[:, 2:N], ctrl.last_U[:, N])
        s_prev = ctrl.last_X[1, :]
        s_init = vcat(x[1], s_prev[3:N+1] .- s_prev[2] .+ x[1])
        push!(s_init, s_init[end] + (s_prev[N+1] - s_prev[N]))
        r = curvature_fixed_point(ctrl, x, Ug, s_init)
        r.converged && return (; r..., Ug, method=:warm)
        rc = cold_schedule(ctrl, x)
        return (; rc..., method=:cold_fallback, iterations=r.iterations + rc.iterations,
            message=rc.converged ? "warm fixed point failed ($(r.message)); used one cold initialisation" :
                    "warm fixed point failed ($(r.message)); cold initialisation failed ($(rc.message))")
    end
    rc = cold_schedule(ctrl, x)
    return (; rc..., method=:cold)
end

"""
    quasi_steady_guess(controller, x) -> (s_nominal, U)

Cold-start guess on a multi-segment track: a quasi-steady centreline speed profile.

- Speed limit along the track: `v_cap(s) = min(v_max, sqrt(ay_max/|kappa(s)|))` (the
  kinematic lateral acceleration `v²·kappa` on the centreline).
- A forward pass with `a_max` and a backward pass with `|a_min|` on a progress grid of 1 cm
  give the profile `v(s)`, starting from the measured speed.
- `s_nominal` follows `ds/dt = v(s)`. The inputs are `a_k = (v_{k+1} - v_k)/dt` and the
  feed-forward steering `atan(kappa_avg_k·L)`, both clipped to the actuator limits.

This is only an initial guess. The NLP may deviate from it, and the post-solve
consistency test and the reference check decide acceptance.
"""
function quasi_steady_guess(ctrl::NMPCController, x)
    N, dt, p, c, track = horizon(ctrl), ctrl.config.dt, ctrl.vehicle.params, ctrl.config, ctrl.track
    ds = 0.01
    reach = max(x[4], p.v_max) * N * dt + 1.0
    if !track.closed
        reach = min(reach, track_length(track) - x[1])
        reach > 2ds || throw(DomainError(x[1], "no room left on the open track"))
    end
    sg = x[1] .+ (0:ds:reach)
    ay = c.ay_max === nothing ? Inf : c.ay_max
    vcap = [min(p.v_max, abs(curvature(track, s)) > 0 ? sqrt(ay / abs(curvature(track, s))) : Inf) for s in sg]
    v = similar(vcap)
    v[1] = min(x[4], vcap[1])
    for i in 2:length(sg)                                # forward: acceleration limit
        v[i] = min(vcap[i], sqrt(v[i-1]^2 + 2 * p.a_max * ds))
    end
    for i in length(sg)-1:-1:1                           # backward: braking limit
        v[i] = min(v[i], sqrt(v[i+1]^2 + 2 * abs(p.a_min) * ds))
    end
    v[1] = x[4]                                          # the measured speed, even above the profile
    vat(s) = (i = clamp(1 + floor(Int, (s - x[1]) / ds), 1, length(sg)); v[i])
    s_nom = zeros(N + 1)
    s_nom[1] = x[1]
    for k in 1:N                                         # midpoint integration of ds/dt = v(s)
        vm = vat(s_nom[k] + vat(s_nom[k]) * dt / 2)
        s_nom[k+1] = min(s_nom[k] + vm * dt, sg[end])
    end
    U = zeros(2, N)
    kap = stage_curvatures(track, s_nom)
    for k in 1:N
        U[1, k] = clamp(atan(kap[k] * p.wheelbase), -p.delta_max, p.delta_max)
        U[2, k] = clamp((vat(s_nom[k+1]) - (k == 1 ? x[4] : vat(s_nom[k]))) / dt, p.a_min, p.a_max)
    end
    return s_nom, U
end

function cold_schedule(ctrl::NMPCController, x)
    N = horizon(ctrl)
    s_init, Ug = try
        quasi_steady_guess(ctrl, x)
    catch err
        err isa DomainError || rethrow()
        return (; converged=false, iterations=0, change=Inf, kappas=Float64[], Xg=fill(NaN, 4, N + 1), Ug=zeros(2, N),
            s_nominal=fill(NaN, N + 1), message="cold-start nominal leaves the track ($(sprint(showerror, err)))")
    end
    r = curvature_fixed_point(ctrl, x, Ug, s_init)
    return (; r..., Ug)
end

"""
    schedule_consistency(controller, sched, core, evaluate) -> ScheduleReport

Post-solve test of the curvature schedule along the optimised progress `s* = X*[1, :]`:

    r_k = psi_c(s*_{k+1}) - psi_c(s*_k) - kappa_k·(s*_{k+1} - s*_k)          [rad]

`r_k` is exactly the absolute-heading error that the averaged curvature adds on stage `k`
along the optimised trajectory (docs/multisegment_prediction.md, §2). The partial sums are
the predicted heading errors at the knots. The schedule is `consistent` when
`max|r_k| ≤ max_stage_curvature_residual`,
`max|Σ_{j≤k} r_j| ≤ max_cumulative_curvature_residual` and
`max|s* - s_nominal| ≤ max_progress_mismatch`. On a single-segment track `r ≡ 0` by
construction.
"""
function schedule_consistency(ctrl::NMPCController, sched, core, evaluate::Bool)
    c, track, N = ctrl.config, ctrl.track, horizon(ctrl)
    model = curvature_model(ctrl)
    lin = get(sched, :lin, nothing)
    nbk, nshort = lin === nothing ? (0, 0) : (lin.boundary_knots, lin.short_intervals)
    nsat, ntr, ntra = 0, 0, 0
    if lin !== nothing && evaluate
        kmin, kmax = ctrl.kappa_range
        tolb = 1e-7
        for k in 1:N
            ke = hasproperty(core, :kappas_used) ? core.kappas_used[k] : sched.kappas[k]
            atbound = ke <= kmin + tolb || ke >= kmax - tolb
            nominal_at = lin.kbar[k] <= kmin + tolb || lin.kbar[k] >= kmax - tolb
            atbound && !nominal_at && (nsat += 1)
        end
        for k in 2:N+1
            isfinite(lin.tr[k]) || continue
            ntr += 1
            abs(core.X[1, k] - sched.s_nominal[k]) >= lin.tr[k] - 1e-7 && (ntra += 1)
        end
    end
    dsmin = evaluate ? minimum(core.X[1, k+1] - core.X[1, k] for k in 1:N) : NaN
    report(sr, cr, pm, nc, ok, msg; merr=NaN, nconf=0) = ScheduleReport(sched.method, sched.iterations, sched.converged,
        sched.change, collect(Float64, sched.kappas), collect(Float64, sched.s_nominal), sr, cr, pm, nc, ok, msg,
        model, merr, nconf, nbk, nshort, nsat, ntr, ntra, dsmin)
    evaluate || return report(NaN, NaN, NaN, 0, false, "not evaluated (solution rejected before the schedule test)")
    sstar = core.X[1, :]
    mismatch = maximum(abs, sstar - sched.s_nominal)
    if num_segments(track) == 1
        ok = mismatch <= c.max_progress_mismatch
        return report(0.0, 0.0, mismatch, 0, ok, ok ? "" : "progress mismatch $(fmt(mismatch)) m > $(c.max_progress_mismatch) m";
            merr=0.0)
    end
    # The curvature the NLP actually used on each stage: the optimised kappa_eff in the
    # linearized mode; the schedule's Parameters otherwise.
    kused = hasproperty(core, :kappas_used) ? core.kappas_used : sched.kappas
    r = zeros(N)
    ncross, nconf, merr = 0, 0, 0.0
    try
        psi = [centerline_heading(track, s) for s in sstar]
        kexact = stage_curvatures(track, sstar)
        for k in 1:N
            r[k] = psi[k+1] - psi[k] - kused[k] * (sstar[k+1] - sstar[k])
            isempty(segment_boundaries(track, sstar[k], sstar[k+1])) || (ncross += 1)
            merr = max(merr, abs(kused[k] - kexact[k]))
            if lin !== nothing && hasproperty(lin, :sp)
                # Segment-pair validity: both optimised knots inside their assigned segments.
                sp, tolk = lin.sp, 1e-7
                inside(i) = sp.lo[i] - tolk <= sstar[i] <= sp.hi[i] + tolk
                (inside(k) && inside(k + 1)) || (nconf += 1)
            else
                stage_configuration(track, sched.s_nominal[k], sched.s_nominal[k+1]) ==
                    stage_configuration(track, sstar[k], sstar[k+1]) || (nconf += 1)
            end
        end
    catch err
        err isa DomainError || rethrow()
        return report(NaN, NaN, mismatch, ncross, false, "optimised progress leaves the track: " * sprint(showerror, err))
    end
    sr, cr = maximum(abs, r), maximum(abs, cumsum(r))
    msgs = String[]
    sr > c.max_stage_curvature_residual && push!(msgs, "stage residual $(fmt(sr)) rad > $(c.max_stage_curvature_residual) rad")
    cr > c.max_cumulative_curvature_residual && push!(msgs, "cumulative residual $(fmt(cr)) rad > $(c.max_cumulative_curvature_residual) rad")
    mismatch > c.max_progress_mismatch && push!(msgs, "progress mismatch $(fmt(mismatch)) m > $(c.max_progress_mismatch) m")
    if lin !== nothing && hasproperty(lin, :sp) && nconf > 0 && c.segment_change_policy === :reject
        push!(msgs, "$nconf stage(s) with knots outside their assigned segments (segment-pair identity not valid)")
    end
    return report(sr, cr, mismatch, ncross, isempty(msgs), join(msgs, "; "); merr, nconf)
end

"""
    reference_check(controller, x, X, U) -> ReferenceReport

Independent validation on the **actual** track:
1. re-simulate the inputs `U` from `x` with `simulate_on_track` (event-aware, actual
   piecewise curvature) at `reference_substeps` samples per interval;
2. at every sample, check the actual track edge (no margin), speed and input limits,
   `|epsi| ≤ epsi_max` and `|ay| ≤ ay_max` (`ay` at both ends of every sub-interval),
   with tolerance `acceptance_tol`;
3. compare the predicted knots `X` with the reference.

Leaving the model domain or the open track also fails the check.
"""
function reference_check(ctrl::NMPCController, x, X, U)
    c, N, veh, track = ctrl.config, horizon(ctrl), ctrl.vehicle, ctrl.track
    m = c.reference_substeps
    h = c.dt / m
    inputs = [SVector{2,Float64}(U[1, k], U[2, k]) for k in 1:N for _ in 1:m]
    sim = simulate_on_track(veh, track, x, inputs, h; max_step=h)
    tol = c.acceptance_tol
    violations = Pair{Float64,ConstraintViolation}[]
    max_ay = 0.0
    for (j, xs) in enumerate(sim.states)
        u = j <= length(inputs) ? inputs[j] : nothing
        for v in check_operating_constraints(veh, track, xs, u; tol)
            push!(violations, sim.t[j] => v)
        end
        abs(xs[3]) > c.epsi_max + tol && push!(violations, sim.t[j] => ConstraintViolation(:heading_error, xs[3],
            copysign(c.epsi_max, xs[3]), "heading error $(fmt(xs[3])) rad exceeds epsi_max = $(fmt(c.epsi_max)) rad"))
        if j < length(sim.states) && j <= length(inputs)
            for xe in (xs, sim.states[j+1])            # both ends of the sub-interval, input j
                ay = lateral_acceleration(veh, xe[4], inputs[j][1])
                max_ay = max(max_ay, abs(ay))
                if c.ay_max !== nothing && abs(ay) > c.ay_max + tol
                    push!(violations, sim.t[j] => ConstraintViolation(:lateral_acceleration, ay, copysign(c.ay_max, ay),
                        "lateral acceleration $(fmt(ay)) m/s² exceeds ay_max = $(fmt(c.ay_max)) m/s²"))
                end
            end
        end
    end
    knots = [k for k in 1:N+1 if 1 + (k - 1) * m <= length(sim.states)]
    pe = prediction_errors(track, [SVector{4,Float64}(X[:, k]) for k in knots], [sim.states[1+(k-1)*m] for k in knots])
    nanmax(v) = (w = filter(!isnan, v); isempty(w) ? NaN : maximum(w))
    passed = succeeded(sim) && isempty(violations)
    msg = !succeeded(sim) ? "reference simulation stopped with $(sim.status): $(sim.message)" :
          isempty(violations) ? "" :
          "$(length(violations)) violation(s), first at t = $(fmt(violations[1].first)) s: $(violations[1].second.message)"
    max_ey = maximum(x -> abs(x[2]), sim.states)
    first_err = length(pe.position) >= 2 ? pe.position[2] : NaN
    return ReferenceReport(true, sim.status, violations, nanmax(pe.position), nanmax(pe.heading), nanmax(pe.s),
        nanmax(pe.ey), max_ey, max_ay, length(sim.crossings), passed, msg, first_err, track.width / 2 - max_ey,
        ctrl.ey_max - max_ey)
end
