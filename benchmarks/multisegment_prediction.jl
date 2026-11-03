# Milestone 2B.1: accuracy of curvature approximations in a discrete-time prediction on
# multi-segment tracks, against the event-aware reference and the independent Cartesian model.
#
# Run from the repository root:
#
#     julia --project=. benchmarks/multisegment_prediction.jl          # tables only
#     julia --project=examples benchmarks/multisegment_prediction.jl   # tables + figures (needs Plots)
#
# Writes benchmarks/results/multisegment_prediction.md and, when Plots is available,
# docs/figures/multiseg_*.png.

using VehicleMPC
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const LW = VEH.params.wheelbase
const HORIZON_T = 2.0
const WIDTH_HALF = 0.35
const LOOP = Track([TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0),
        TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0)]; width=0.7, closed=true)

"Transition cases: (name, track, boundary progress used to place the first crossing)."
function transitions()
    q = quarter_track()
    lab = legacy_track(:lab)
    return [
        ("straight→curve (κ 0→+1)", q, q.s_start[2]),
        ("curve→curve (κ +1→−1)", q, q.s_start[3]),
        ("curve→straight (κ +0.5→0)", q, q.s_start[5]),
        ("lap boundary (κ 0→+1, lap 1→2)", LOOP, track_length(LOOP)),
        ("lab: short segments, κ ±1.43", lab, lab.s_start[6]),
    ]
end

quant(xs, q) = (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))

"Matched inputs: centreline feed-forward along a constant-speed plan (+ optional sinusoidal steering)."
function plan_inputs(track, s0, v, dt, N, wobble)
    snom = [s0 + v * dt * k for k in 0:N]
    U = zeros(2, N)
    for k in 1:N
        U[1, k] = clamp(atan(LW * average_curvature(track, snom[k], snom[k+1])) + wobble * sin(2π * (k - 1) * dt / 1.2),
            -VEH.params.delta_max, VEH.params.delta_max)
    end
    return U, snom
end

function horizon_steps(track, s0, v, dt)
    N = round(Int, HORIZON_T / dt)
    track.closed && return N
    return max(1, min(N, floor(Int, (track_length(track) - 0.3 - s0) / (v * dt))))   # stay on the open track
end

"""
Self-consistent schedule: repeat `predict(s_nominal)` with the nominal replaced by the
prediction's own progress until the progress stops changing (`tol`) or `maxit` is reached.
Returns `(prediction, iterations, converged)`.
"""
function fixed_point(predict, s_init; maxit=30, tol=1e-9)
    s = s_init
    P = predict(s)
    for it in 1:maxit
        succeeded(P) || return P, it, false
        snew = [x[1] for x in P.states]
        maximum(abs, snew - s) <= tol && return P, it, true
        s = snew
        P = predict(s)
    end
    return P, maxit, false
end

"Run all methods for one case; return per-method error summaries."
function run_case(track, b, v, dt, frac, wobble)
    s0 = b - frac * v * dt                     # first crossing at ≈ frac·dt (sdot = v at ey = epsi = 0)
    x0 = [s0, 0.0, 0.0, v]
    N = horizon_steps(track, s0, v, dt)
    U, snom = plan_inputs(track, s0, v, dt, N, wobble)
    useq = [U[:, k] for k in 1:N]

    t_ref = @elapsed ref = simulate_on_track(VEH, track, x0, useq, dt; max_step=dt / 100)
    succeeded(ref) || return (; failed=true, status=ref.status, message=ref.message)
    sref = [x[1] for x in ref.states]

    # Independent Cartesian reference, back-projected with progress hints.
    zs = simulate_cartesian(VEH, frenet_state_to_cartesian(track, x0), useq, dt; max_step=1e-4)
    cart_pos, cart_frenet, proj_fail = 0.0, 0.0, 0
    for k in eachindex(zs)
        p = frenet_state_to_cartesian(track, ref.states[k])
        cart_pos = max(cart_pos, hypot(p[1] - zs[k][1], p[2] - zs[k][2]), abs(p[3] - zs[k][3]))
        try
            s, ey, epsi = cartesian_to_frenet(track, zs[k][1], zs[k][2], zs[k][3]; s_hint=ref.states[k][1], max_lateral=Inf)
            cart_frenet = max(cart_frenet, abs(s - ref.states[k][1]), abs(ey - ref.states[k][2]),
                abs(rem2pi(epsi - ref.states[k][3], RoundNearest)))
        catch err
            err isa ArgumentError || rethrow()
            proj_fail += 1
        end
    end

    A = predict_start_curvature(VEH, track, x0, U, dt)
    Bn = predict_average_curvature(VEH, track, x0, U, dt, snom)
    Bi = succeeded(Bn) ? predict_average_curvature(VEH, track, x0, U, dt, [x[1] for x in Bn.states]) : Bn
    Bo = predict_average_curvature(VEH, track, x0, U, dt, sref)
    Dn = predict_split_curvature(VEH, track, x0, U, dt, snom)
    Bf, itB, convB = fixed_point(s -> predict_average_curvature(VEH, track, x0, U, dt, s), snom)
    Df, itD, convD = fixed_point(s -> predict_split_curvature(VEH, track, x0, U, dt, s), snom)
    times = [Float64[c.t - (k - 1) * dt for c in ref.crossings if (k - 1) * dt <= c.t < k * dt] for k in 1:N]
    nominal_bounds = [length(segment_boundaries(track, sref[k], sref[k+1])) for k in 1:N]
    Do = all(length.(times) .== nominal_bounds) ?
         predict_split_curvature(VEH, track, x0, U, dt, sref; crossing_times=times) : nothing

    methods = (A=A, B_nominal=Bn, B_iterated=Bi, B_fixedpoint=Bf, B_oracle=Bo, D_nominal=Dn, D_fixedpoint=Df, D_oracle=Do)
    out = Dict{Symbol,Any}()
    ref_viol = count(x -> abs(x[2]) > WIDTH_HALF, ref.states)
    eps_g = VEH.eps_geometry
    for (name, P) in pairs(methods)
        P === nothing && continue
        e = prediction_errors(track, P.states, ref.states)
        pviol = count(x -> abs(x[2]) > WIDTH_HALF, P.states)
        # Model-domain failure: the rollout threw, or a predicted knot lies outside the model
        # domain of the actual track geometry (1 - kappa(s)·ey ≤ eps_geometry).
        geom_bad = count(P.states) do x
            try
                1 - curvature(track, x[1]) * x[2] <= eps_g
            catch err
                err isa DomainError || rethrow()
                true                           # beyond the end of an open track
            end
        end
        nanmax(v) = (w = filter(!isnan, v); isempty(w) ? NaN : maximum(w))
        out[name] = (; ok=succeeded(P) && geom_bad == 0, rollout_ok=succeeded(P), geom_bad,
            one=(pos=e.position[2], head=e.heading[2], ey=e.ey[2], s=e.s[2]),
            hor=(pos=nanmax(e.position), head=nanmax(e.heading), ey=maximum(e.ey), s=maximum(e.s)),
            viol_mismatch=abs(pviol - ref_viol), series=e)
    end
    return (; failed=false, out, N, ref, itB, convB, itD, convD, crossings=length(ref.crossings), first_cross=isempty(ref.crossings) ? NaN : ref.crossings[1].t / dt,
        cart_pos, cart_frenet, proj_fail, t_ref, ref_viol)
end

const METHODS = (:A, :B_nominal, :B_iterated, :B_fixedpoint, :B_oracle, :D_nominal, :D_fixedpoint, :D_oracle)
const LABEL = Dict(:A => "A start-of-step", :B_nominal => "B average (nominal plan)", :B_iterated => "B average (1 refinement)",
    :B_fixedpoint => "B average (self-consistent)", :B_oracle => "B average (reference endpoints)",
    :D_nominal => "D split (nominal times)", :D_fixedpoint => "D split (self-consistent)", :D_oracle => "D split (exact times)")

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    speeds = (0.5, 1.0, 2.0, 3.0, 4.0)
    fracs = (0.1, 0.5, 0.9)
    dts = (0.1, 0.05)
    wobbles = (0.0, 0.08)
    cases, excluded = [], String[]
    for (tname, track, b) in transitions(), v in speeds, dt in dts, frac in fracs, w in wobbles
        r = run_case(track, b, v, dt, frac, w)
        if r.failed
            push!(excluded, "$tname, v = $v, dt = $dt, crossing at $(frac)·dt, wobble = $w: reference status `$(r.status)` ($(r.message))")
            continue
        end
        push!(cases, (; tname, v, dt, frac, w, r))
    end
    emit("""
    # Multi-segment prediction accuracy (Milestone 2B.1)

    - $(length(cases) + length(excluded)) cases = 5 transitions × 5 speeds × 2 time steps × 3 crossing positions × 2 input sequences (feed-forward, and feed-forward + 0.08 rad sinusoidal steering); $(length(excluded)) excluded because the *reference* trajectory itself is invalid (listed at the end)
    - Reference C: event-aware RK4 with max_step = dt/100. Horizon $(HORIZON_T) s (N = 20 at dt = 0.1, N = 40 at dt = 0.05), shortened on the open lab track so it ends 0.3 m before the track end.
    - All predictions use RK4 with one step per stage (as in the NMPC), the same initial state `[s0, 0, 0, v]` and the same inputs.
    - Errors at knots vs C: position = Cartesian distance [m], heading = |ψ_pred − ψ_ref| [rad], ey [m], s [m]. "One-step" = knot 2; "horizon" = maximum over all knots.

    """)

    # Cross-check of the reference against the Cartesian model.
    cp = maximum(c.r.cart_pos for c in cases)
    cf = maximum(c.r.cart_frenet for c in cases)
    pf = sum(c.r.proj_fail for c in cases)
    emit(@sprintf("""
    ## Reference cross-check (event-aware Frenet vs independent Cartesian RK4, max_step = 1e-4)

    | quantity | value |
    |:--|--:|
    | max position or heading difference over all knots and cases | %.1e |
    | max |Δs|, |Δey|, |Δepsi| after back-projection (with progress hints) | %.1e |
    | back-projection failures | %d |
    | max crossings resolved in one 2 s horizon | %d |

    """, cp, cf, pf, maximum(c.r.crossings for c in cases)))

    # Overall summary per method.
    nB = count(c -> c.r.convB, cases); nD = count(c -> c.r.convD, cases)
    emit(@sprintf("""
    ## Self-consistent (fixed-point) schedules

    The schedule is recomputed from the prediction's own progress until the progress changes by less than 1e-9 m (at most 30 rollouts).

    | method | converged | rollouts (median / max) |
    |:--|--:|:-:|
    | B average | %d / %d | %d / %d |
    | D split | %d / %d | %d / %d |

    """, nB, length(cases), quant([c.r.itB for c in cases], 0.5), maximum(c.r.itB for c in cases),
        nD, length(cases), quant([c.r.itD for c in cases], 0.5), maximum(c.r.itD for c in cases)))

    # Self-consistent schedules: converged vs not converged.
    t = "| method | cases | horizon pos. (median / max) | horizon heading (max) |
|:--|--:|:-:|--:|
"
    for (m, convf) in ((:B_fixedpoint, c -> c.r.convB), (:D_fixedpoint, c -> c.r.convD)), conv in (true, false)
        sub = filter(c -> convf(c) == conv && c.r.out[m].ok, cases)
        isempty(sub) && continue
        hp = filter(!isnan, [c.r.out[m].hor.pos for c in sub])
        hh = filter(!isnan, [c.r.out[m].hor.head for c in sub])
        t *= @sprintf("| %s, %s | %d | %.1e / %.1e | %.1e |
", LABEL[m], conv ? "converged" : "**not converged**", length(sub),
            quant(hp, 0.5), maximum(hp), maximum(hh))
    end
    emit("## Self-consistent schedules: converged vs not converged (domain failures excluded)

" * t * "
")

    # Horizon position error by speed, feed-forward inputs.
    t = "## Horizon position error [m] by speed, feed-forward inputs (maximum over transitions and crossing positions)

| method | dt | " *
        join(["v = $v" for v in speeds], " | ") * " |
|:--|--:|" * repeat("--:|", length(speeds)) * "
"
    for m in (:A, :B_fixedpoint, :B_oracle, :D_nominal, :D_fixedpoint, :D_oracle), dt in dts
        vals = map(speeds) do v
            xs = filter(!isnan, [c.r.out[m].hor.pos for c in cases if c.v == v && c.dt == dt && c.w == 0.0 && haskey(c.r.out, m)])
            isempty(xs) ? NaN : maximum(xs)
        end
        t *= "| $(LABEL[m]) | $dt | " * join([@sprintf("%.1e", x) for x in vals], " | ") * " |
"
    end
    emit(t * "
")

    for (fam, wsel) in (("feed-forward inputs", 0.0), ("feed-forward + 0.08 rad sinusoidal steering", 0.08))
        sub = filter(c -> c.w == wsel, cases)
        t = """
        ## Summary, $fam ($(length(sub)) cases)

        | method | model-domain failures (cases) | one-step pos. (median / max) | one-step heading (median / max) | horizon pos. (median / max) | horizon heading (max) | horizon ey (max) | horizon s (max) |
        |:--|--:|:-:|:-:|:-:|--:|--:|--:|
        """
        for m in METHODS
            rs = [c.r.out[m] for c in sub if haskey(c.r.out, m)]
            isempty(rs) && continue
            g(sel) = filter(!isnan, [sel(r) for r in rs])
            t *= @sprintf("| %s | %d / %d | %.1e / %.1e | %.1e / %.1e | %.1e / %.1e | %.1e | %.1e | %.1e |
", LABEL[m],
                count(r -> !r.ok, rs), length(rs),
                quant(g(r -> r.one.pos), 0.5), maximum(g(r -> r.one.pos)), quant(g(r -> r.one.head), 0.5), maximum(g(r -> r.one.head)),
                quant(g(r -> r.hor.pos), 0.5), maximum(g(r -> r.hor.pos)), maximum(g(r -> r.hor.head)), maximum(g(r -> r.hor.ey)),
                maximum(g(r -> r.hor.s)))
        end
        emit(t * "
")
    end

    t = """
    ## Summary over all cases

    | method | model-domain failures (cases) | one-step pos. (median / max) | one-step heading (median / max) | horizon pos. (median / max) | horizon heading (max) | horizon ey (max) | horizon s (max) | boundary-count mismatches |
    |:--|--:|:-:|:-:|:-:|--:|--:|--:|--:|
    """
    for m in METHODS
        rs = [c.r.out[m] for c in cases if haskey(c.r.out, m)]
        isempty(rs) && continue
        f(sel) = filter(!isnan, [sel(r) for r in rs])
        t *= @sprintf("| %s | %d / %d | %.1e / %.1e | %.1e / %.1e | %.1e / %.1e | %.1e | %.1e | %.1e | %d |\n", LABEL[m],
            count(r -> !r.ok, rs), length(rs),
            quant(f(r -> r.one.pos), 0.5), maximum(f(r -> r.one.pos)), quant(f(r -> r.one.head), 0.5), maximum(f(r -> r.one.head)),
            quant(f(r -> r.hor.pos), 0.5), maximum(f(r -> r.hor.pos)), maximum(f(r -> r.hor.head)), maximum(f(r -> r.hor.ey)),
            maximum(f(r -> r.hor.s)), sum(f(r -> r.viol_mismatch)))
    end
    emit(t * "\n")

    # Dependence on speed and dt (max over transitions, crossing positions, inputs).
    for (title, sel) in (("One-step heading error [rad]", r -> r.one.head), ("One-step position error [m]", r -> r.one.pos),
        ("Horizon position error [m]", r -> r.hor.pos), ("Horizon lateral error |Δey| [m]", r -> r.hor.ey))
        t = "## $title: maximum over cases, by speed and dt\n\n| method | dt | " * join(["v = $v" for v in speeds], " | ") *
            " |\n|:--|--:|" * repeat("--:|", length(speeds)) * "\n"
        for m in METHODS, dt in dts
            vals = [begin
                rs = [sel(c.r.out[m]) for c in cases if c.v == v && c.dt == dt && haskey(c.r.out, m)]
                isempty(rs) ? NaN : maximum(rs)
            end for v in speeds]
            t *= "| $(LABEL[m]) | $dt | " * join([@sprintf("%.1e", x) for x in vals], " | ") * " |\n"
        end
        emit(t * "\n")
    end

    # Per transition at dt = 0.1, v = 3, by crossing position (one-step heading error).
    t = "## One-step heading error [rad] by transition and crossing position (dt = 0.1, v = 3 m/s, feed-forward inputs)\n\n| transition | crossing at | " *
        join([LABEL[m] for m in METHODS], " | ") * " |\n|:--|--:|" * repeat("--:|", length(METHODS)) * "\n"
    for c in cases
        (c.dt == 0.1 && c.v == 3.0 && c.w == 0.0) || continue
        t *= @sprintf("| %s | %.2f·dt | ", c.tname, c.r.first_cross) *
             join([haskey(c.r.out, m) ? @sprintf("%.1e", c.r.out[m].one.head) : "–" for m in METHODS], " | ") * " |\n"
    end
    emit(t * "\n")

    # Sensitivity of B and D to a wrong nominal progress (speed scaled), v = 3, dt = 0.1.
    t = "## Sensitivity to the nominal progress: horizon position error [m], v = 3 m/s, dt = 0.1, crossing at 0.5·dt\n\n| transition | nominal speed / actual | B average | D split |\n|:--|--:|--:|--:|\n"
    for (tname, track, b) in transitions()[1:2], scale in (0.8, 0.9, 1.0, 1.1, 1.2)
        v, dt = 3.0, 0.1
        s0 = b - 0.5 * v * dt
        x0 = [s0, 0.0, 0.0, v]
        N = horizon_steps(track, s0, v, dt)
        U, _ = plan_inputs(track, s0, v, dt, N, 0.0)
        ref = simulate_on_track(VEH, track, x0, [U[:, k] for k in 1:N], dt; max_step=dt / 100)
        snom = [s0 + scale * v * dt * k for k in 0:N]
        eB = maximum(prediction_errors(track, predict_average_curvature(VEH, track, x0, U, dt, snom).states, ref.states).position)
        eD = maximum(prediction_errors(track, predict_split_curvature(VEH, track, x0, U, dt, snom).states, ref.states).position)
        t *= @sprintf("| %s | %.1f | %.1e | %.1e |\n", tname, scale, eB, eD)
    end
    emit(t * "\n")

    # Computational cost.
    tr = quarter_track()
    x0, u = [1.95, 0.0, 0.0, 3.0], [atan(LW), 0.0]
    cost(f, n) = (f(); GC.gc(); 1e6 * (@elapsed for _ in 1:n; f(); end) / n)
    c_rk4 = cost(() -> discrete_dynamics(VEH, x0, u, 1.0, 0.1, RK4()), 20_000)
    c_ev1 = cost(() -> integrate_on_track(VEH, tr, x0, u, 0.1; max_step=0.1), 20_000)
    c_ev10 = cost(() -> integrate_on_track(VEH, tr, x0, u, 0.1; max_step=0.01), 5_000)
    c_ev100 = cost(() -> integrate_on_track(VEH, tr, x0, u, 0.1; max_step=0.001), 1_000)
    U20, sn = plan_inputs(tr, 1.95, 3.0, 0.1, 20, 0.0)
    c_A = cost(() -> predict_start_curvature(VEH, tr, x0, U20, 0.1), 2_000)
    c_B = cost(() -> predict_average_curvature(VEH, tr, x0, U20, 0.1, sn), 2_000)
    c_D = cost(() -> predict_split_curvature(VEH, tr, x0, U20, 0.1, sn), 2_000)
    c_sched = cost(() -> curvature_schedule(tr, sn), 20_000)
    c_ref = cost(() -> simulate_on_track(VEH, tr, x0, [U20[:, k] for k in 1:20], 0.1; max_step=0.001), 200)
    emit(@sprintf("""
    ## Computational cost (Julia %s, single thread, wall-clock mean)

    | operation | time [µs] |
    |:--|--:|
    | one RK4 step, constant curvature (`discrete_dynamics`) | %.2f |
    | `integrate_on_track`, 0.1 s with one crossing, max_step = 0.1 | %.2f |
    | `integrate_on_track`, 0.1 s with one crossing, max_step = 0.01 | %.2f |
    | `integrate_on_track`, 0.1 s with one crossing, max_step = 0.001 | %.2f |
    | `curvature_schedule` for N = 20 | %.2f |
    | 20-stage prediction, method A | %.2f |
    | 20-stage prediction, method B | %.2f |
    | 20-stage prediction, method D (nominal splits) | %.2f |
    | 2 s reference (`simulate_on_track`, max_step = 0.001) | %.0f |

    """, VERSION, c_rk4, c_ev1, c_ev10, c_ev100, c_sched, c_A, c_B, c_D, c_ref))

    emit("## Excluded cases (no valid reference trajectory)

" * (isempty(excluded) ? "none
" : join(["- " * e for e in excluded], "
") * "
"))

    out = joinpath(@__DIR__, "results", "multisegment_prediction.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return cases
end

cases = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    figdir = mkpath(joinpath(@__DIR__, "..", "docs", "figures"))
    # Figure 1: one representative case, errors along the horizon.
    let c = only(filter(c -> startswith(c.tname, "curve→curve") && c.v == 3.0 && c.dt == 0.1 && c.frac == 0.5 && c.w == 0.08, cases))
        tt = 0.1 .* (0:c.r.N)
        p1 = plot(; ylabel="position error [m]", yscale=:log10, legend=:bottomright, title="κ +1→−1, v = 3 m/s, dt = 0.1 s, perturbed steering")
        p2 = plot(; ylabel="heading error [rad]", xlabel="t [s]", yscale=:log10, legend=false)
        for m in (:A, :B_nominal, :B_fixedpoint, :D_nominal, :D_oracle)
            haskey(c.r.out, m) || continue
            e = c.r.out[m].series
            plot!(p1, tt[2:end], max.(e.position[2:end], 1e-16); label=LABEL[m], marker=:circle, ms=2)
            plot!(p2, tt[2:end], max.(e.heading[2:end], 1e-16); marker=:circle, ms=2)
        end
        vline!(p1, [cr.t for cr in c.r.ref.crossings]; ls=:dot, lc=:gray, label="crossings (ref.)")
        vline!(p2, [cr.t for cr in c.r.ref.crossings]; ls=:dot, lc=:gray)
        savefig(plot(p1, p2; layout=(2, 1), size=(900, 700), left_margin=5Plots.mm), joinpath(figdir, "multiseg_horizon_example.png"))
    end
    # Figure 2: one-step heading error vs speed (max over cases), both dt.
    speeds = (0.5, 1.0, 2.0, 3.0, 4.0)
    ps = map((0.1, 0.05)) do dt
        p = plot(; title="dt = $dt s", xlabel="v [m/s]", ylabel="max one-step heading error [rad]", yscale=:log10, legend=(dt == 0.1 ? :bottomright : false))
        for m in (:A, :B_nominal, :B_oracle, :D_nominal, :D_oracle)
            vals = [maximum([c.r.out[m].one.head for c in cases if c.v == v && c.dt == dt && haskey(c.r.out, m)]; init=0.0) for v in speeds]
            plot!(p, collect(speeds), max.(vals, 1e-16); label=LABEL[m], marker=:circle)
        end
        p
    end
    savefig(plot(ps...; layout=(1, 2), size=(1100, 450), left_margin=5Plots.mm, bottom_margin=5Plots.mm), joinpath(figdir, "multiseg_speed_dt.png"))
    println("saved figures to ", figdir)
end
