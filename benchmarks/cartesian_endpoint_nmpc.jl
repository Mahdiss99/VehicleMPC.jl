# Milestone 2B.4: exact Cartesian-endpoint NMPC vs the Frenet formulations.
#
#     julia --project=. benchmarks/cartesian_endpoint_nmpc.jl          # tables
#     julia --project=examples benchmarks/cartesian_endpoint_nmpc.jl   # tables + figure
#
# Writes benchmarks/results/cartesian_endpoint_nmpc.md (and
# docs/figures/cartesian_endpoint_quarter.png). Every configuration is run REPS times,
# interleaved, in one process after a common warm-up. All cycles count, failed ones
# included. The closed loop below is simulate_closed_loop (event-aware plant, 10 substeps)
# with every solve! wrapped in @timed, to record allocations and GC time per cycle; it is
# checked against simulate_closed_loop at the start. Every controller is primed with one
# solve at the start state (then reset_warm_start!) before its run: the first optimize! of
# a new JuMP model builds the NLP evaluator, which is initial model construction and is
# reported separately.

using VehicleMPC
using VehicleMPC.StaticArrays: SVector
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const N_CYCLES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 250
const REPS = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 3
const Q = quarter_track()
const X0 = [0.0, 0.0, 0.0, 0.5]
const TRANS = Track([TrackSegment(3.0, 0.0), TrackSegment(1.5π, 0.5), TrackSegment(6.0, 0.0)]; width=0.7)
const TOL = 1e-6

quant(xs, q) = isempty(xs) ? NaN : (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))
nanmax(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : maximum(w))
nanmin(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : minimum(w))
ms(x) = 1e3 * x

const CART = (prediction_model=:cartesian_endpoint_exact,)
configs(N, m) = [
    ("A stagewise, ≤ 8 corr., reject", NMPCConfig(horizon=N, track_margin=m, schedule_corrections=8)),
    ("B segment-pair D2, ≤ 8 corr.", NMPCConfig(horizon=N, track_margin=m, schedule_corrections=8, curvature_model=:segment_pair_exact,
        segment_change_policy=:reject)),
    ("C Cartesian, 1 solve", NMPCConfig(; CART..., horizon=N, track_margin=m)),
    ("D Cartesian, ≤ 8 reassignments", NMPCConfig(; CART..., horizon=N, track_margin=m, schedule_corrections=8)),
    ("E Cartesian + membership bounds, 1 solve", NMPCConfig(; CART..., horizon=N, track_margin=m, segment_membership=true)),
    ("F = D + strict corridor", NMPCConfig(; CART..., horizon=N, track_margin=m, schedule_corrections=8, strict_corridor=true)),
]
const GRID = [(N, m) for N in (10, 20) for m in (0.05, 0.10)]
"The same configuration without the correction time budget (algorithmic behaviour, independent of wall time)."
nobudget(cfg) = NMPCConfig(; (f => getfield(cfg, f) for f in fieldnames(NMPCConfig))..., correction_time_budget=1e3)
"A controller primed with one solve at the start state (builds the NLP evaluator), then reset."
function primed(track, cfg, x=X0)
    c = NMPCController(VEH, track, cfg)
    solve!(c, x)
    reset_warm_start!(c)
    return c
end

"simulate_closed_loop (event-aware plant) with per-cycle @timed statistics."
function closed_loop(ctrl, n)
    h = ctrl.config.dt / 10
    x, uprev = SVector{4,Float64}(X0...), SVector(0.0, 0.0)
    states, results, alloc, gct, fine = [x], NMPCResult[], Float64[], Float64[], [x]
    status, msg = :success, ""
    for k in 1:n
        tm = @timed solve!(ctrl, x; previous_input=uprev)
        r = tm.value
        push!(results, r); push!(alloc, tm.bytes / 1e6); push!(gct, ms(tm.gctime))
        if !r.accepted
            status, msg = :controller_failure, "control period $k (t = $(round((k - 1) * ctrl.config.dt; digits=1)) s): $(r.status): $(r.message)"
            break
        end
        for _ in 1:10
            st = integrate_on_track(VEH, Q, x, r.u, h; max_step=h)
            succeeded(st) || error("plant: $(st.message)")
            x = st.x
            push!(fine, x)
        end
        push!(states, x)
        uprev = r.u
    end
    return (; states, results, alloc, gct, fine, status, msg)
end

"Minimum track clearance width/2 - max|ey| of the re-simulated plan at m samples per interval."
function clearance(x, U, m)
    h = 0.1 / m
    sim = simulate_on_track(VEH, Q, x, [SVector(U[1, k], U[2, k]) for k in 1:size(U, 2) for _ in 1:m], h; max_step=h)
    succeeded(sim) || return NaN
    return Q.width / 2 - maximum(abs(s[2]) for s in sim.states)
end

function runstats(c, cl; resolution=false, prime=NaN)
    rs = cl.results
    acc = filter(r -> r.accepted, rs)
    tot = [ms(r.timing.total) for r in rs]
    # Independent exact-map reconstruction for every accepted plan (also for the Frenet modes).
    ex = [exact_map_check(c, r.X[:, 1], r.X, r.U) for r in acc]
    res = resolution ? [(clearance(r.X[:, 1], r.U, 5), clearance(r.X[:, 1], r.U, 20), clearance(r.X[:, 1], r.U, 100)) for r in acc] : nothing
    slow = findall(>(80), tot)
    return (; status=cl.status, cycles=length(rs), laps=(cl.states[end][1] - cl.states[1][1]) / track_length(Q),
        accepted=length(acc), rejected=length(rs) - length(acc),
        st=Dict(s => count(r -> r.status === s, rs) for s in unique(r.status for r in rs)),
        refviol=count(r -> r.reference.checked && !r.reference.passed, rs),
        corrviol=count(r -> r.reference.min_corridor_distance < -TOL, acc),
        solves=[r.nlp_solves for r in rs], iters=[r.iterations for r in rs], tot, build=ms(c.build_time), prime=ms(prime),
        solver=filter(!isnan, [ms(r.timing.solver) for r in rs]), alloc=cl.alloc, gct=cl.gct,
        slow=[(k, tot[k], ms(rs[k].timing.solver), ms(rs[k].timing.optimize), cl.gct[k], cl.alloc[k], rs[k].nlp_solves, rs[k].iterations) for k in slow],
        e1=nanmax(e.first_step_position_error for e in ex), eh=nanmax(e.max_position_error for e in ex),
        ehead=nanmax(e.max_heading_error for e in ex), ev=nanmax(e.max_speed_error for e in ex),
        eknot=nanmax(e.knot_residual for e in ex), es=nanmax(e.max_progress_error for e in ex), eey=nanmax(e.max_ey_error for e in ex),
        geo=nanmax(r.exact.geometry_residual for r in acc), dyn=nanmax(r.max_dynamics_residual for r in acc),
        r1=nanmax(r.reference.first_step_position_error for r in acc), rh=nanmax(r.reference.max_position_error for r in acc),
        segacc=sum((r.schedule.configuration_changes > 0 for r in acc); init=0),
        edge=nanmin(r.reference.min_boundary_distance for r in acc), corridor=nanmin(r.reference.min_corridor_distance for r in acc),
        res, plant_viol=count(x -> !isempty(check_operating_constraints(VEH, Q, x; tol=TOL)), cl.fine), plant_edge=Q.width / 2 - maximum(abs(x[2]) for x in cl.fine), plant_corridor=c.ey_max - maximum(abs(x[2]) for x in cl.fine),
        stop=cl.msg)
end

pooled(runs, f) = reduce(vcat, [f(r) for r in runs])

function tables(names, results)
    t1 = """
    | configuration | laps per run | accepted / rejected cycles | rejections by status | NLP solves mean / max | reassignments per cycle | Ipopt iter. mean / max | model build / priming solve [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms |
    |:--|:--|:-:|:--|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    t2 = """
    | configuration | plant violations | plant clearance: track / corridor [m] | predicted clearance: track / corridor [m] | predicted reference violations / corridor violations (accepted) | exact-map error: first step / horizon [m] | heading [rad] / speed [m/s] | event-aware reference: first step / horizon [m] | NLP dyn. residual / geometry residual | accepted cycles with segment change |
    |:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|--:|
    """
    for name in names
        runs = results[name]
        laps = join([r.status === :success ? @sprintf("%.2f", r.laps) : @sprintf("stop %.2f (c%d)", r.laps, r.cycles) for r in runs], "; ")
        sol, it, tot = pooled(runs, r -> r.solves), pooled(runs, r -> r.iters), pooled(runs, r -> r.tot)
        rej = mergewith(+, [Dict(k => v for (k, v) in r.st if !(k in (:success, :success_stale_schedule))) for r in runs]...)
        t1 *= @sprintf("| %s | %s | %d / %d | %s | %.2f / %d | %.2f | %.1f / %d | %.0f / %.0f | %.1f / %.1f / %.1f / %.1f | %d / %d |\n",
            name, laps, sum(r.accepted for r in runs), sum(r.rejected for r in runs),
            isempty(rej) ? "–" : join(["$k $v" for (k, v) in rej], ", "),
            sum(sol) / length(sol), maximum(sol), sum(sol .- 1) / length(sol), sum(it) / length(it), maximum(it),
            sum(r.build for r in runs) / length(runs), sum(r.prime for r in runs) / length(runs), quant(tot, 0.5), quant(tot, 0.95), quant(tot, 0.99), maximum(tot),
            count(>(80), tot), count(>(100), tot))
        t2 *= @sprintf("| %s | %d | %.3f / %.3f | %.3f / %.3f | %d / %d | %.1e / %.1e | %.1e / %.1e | %.1e / %.1e | %.1e / %.1e | %d |\n", name,
            sum(r.plant_viol for r in runs), minimum(r.plant_edge for r in runs), minimum(r.plant_corridor for r in runs),
            nanmin(r.edge for r in runs), nanmin(r.corridor for r in runs), sum(r.refviol for r in runs), sum(r.corrviol for r in runs),
            nanmax(r.e1 for r in runs), nanmax(r.eh for r in runs), nanmax(r.ehead for r in runs), nanmax(r.ev for r in runs),
            nanmax(r.r1 for r in runs), nanmax(r.rh for r in runs), nanmax(r.dyn for r in runs), nanmax(r.geo for r in runs),
            sum(r.segacc for r in runs))
    end
    return t1, t2
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Cartesian-endpoint NMPC benchmark (Milestone 2B.4)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `$(X0)`
    - $(N_CYCLES) control cycles (control period 0.1 s, prediction step 0.1 s) per run, $(REPS) interleaved runs per configuration; event-aware plant (10 substeps); all cycles counted
    - Corrections/reassignments use the existing mechanism (≤ 8 per cycle, 80 ms budget). Objective, weights, limits, vehicle and Ipopt options are identical in all configurations
    - "Corridor" = tightened planning corridor `|ey| ≤ width/2 − margin`; "track" = physical edge `|ey| ≤ width/2`. Predicted clearances are from the event-aware re-simulation of each accepted plan at 5 samples per interval; corridor violations are counted with tolerance $(TOL) m
    - "Exact-map error" = optimised knots (true geometry) vs the exact Cartesian reconstruction of the plan's inputs from the measured state, computed after the run for every configuration

    """)

    # --- exact map validation (independent references)
    emit("## Exact Cartesian map: independent validation\n\n")
    function rk4big(z0, u, dt; m=4000)
        setprecision(BigFloat, 256) do
            L, d, a, h = big(VEH.params.wheelbase), big(u[1]), big(u[2]), big(dt) / m
            f(z) = [z[4] * cos(z[3]), z[4] * sin(z[3]), z[4] * tan(d) / L, a]
            z = big.(collect(Float64, z0))
            for _ in 1:m
                k1 = f(z); k2 = f(z .+ h / 2 .* k1); k3 = f(z .+ h / 2 .* k2); k4 = f(z .+ h .* k3)
                z = z .+ h / 6 .* (k1 .+ 2k2 .+ 2k3 .+ k4)
            end
            Float64.(z)
        end
    end
    t = "| case (dt = 0.1 s) | exact vs BigFloat RK4 (4000 steps): position / heading / speed | `integrate_cartesian` (RK4, 1 ms) vs BigFloat: position |\n|:--|:-:|:-:|\n"
    for (name, z0, u) in [("straight", [0.0, 0, 0, 2.0], [0.0, 0.0]), ("left", [1.0, -2, 0.3, 2.5], [0.4, 0.0]),
        ("right", [1.0, -2, 0.3, 2.5], [-0.4, 0.0]), ("accelerate", [0.0, 0, 1.0, 1.0], [0.2, 5.0]),
        ("brake", [0.0, 0, -2.0, 3.0], [-0.3, -5.0]), ("zero steering, accelerate", [0.0, 0, 0.7, 1.0], [0.0, 3.0]),
        ("steering 1e-9 rad", [0.0, 0, 0.7, 2.0], [1e-9, 1.0]), ("from rest", [0.0, 0, 0.0, 0.0], [0.43, 5.0]),
        ("heading 40 rad (unwrapped)", [5.0, 3, 40.0, 4.0], [0.3, -2.0]), ("δ = 25°, v = 5 m/s, a = 5", [0.0, 0, 0, 5.0], [deg2rad(25), 5.0]),
        ("brake to rest at dt", [0.0, 0, 0.5, 0.5], [0.3, -5.0])]
        ze, zr, zi = exact_cartesian_step(VEH, z0, u, 0.1), rk4big(z0, u, 0.1), integrate_cartesian(VEH, z0, u, 0.1)
        t *= @sprintf("| %s | %.1e / %.1e / %.1e | %.1e |\n", name, hypot(ze[1] - zr[1], ze[2] - zr[2]), abs(ze[3] - zr[3]), abs(ze[4] - zr[4]),
            hypot(zi[1] - zr[1], zi[2] - zr[2]))
    end
    emit(t * "\n")
    function feedback(track, x0, n; a=k -> 0.0)
        x, us = SVector{4}(x0...), SVector{2,Float64}[]
        for k in 1:n
            u = SVector(clamp(atan(curvature(track, x[1] + 0.3) * VEH.params.wheelbase) - 1.5x[2] - 1.2x[3], -0.43, 0.43), a(k))
            push!(us, u)
            x = integrate_on_track(VEH, track, x, u, 0.1; max_step=1e-3).x
        end
        return us
    end
    t = "| event-aware Frenet plant (RK4, 1 ms, located crossings) mapped to Cartesian vs exact map | crossings | position [m] | heading [rad] | speed [m/s] |\n|:--|--:|:-:|:-:|:-:|\n"
    for (name, tr, x0, us) in [("quarter: straight → R 1 m, accelerating, 20 steps", Q, [1.0, 0.1, 0.05, 2.0], feedback(Q, [1.0, 0.1, 0.05, 2.0], 20; a=k -> 0.5)),
        ("quarter: R 1 m → R −1 m S-bend", Q, [4.5, -0.1, 0.0, 2.0], feedback(Q, [4.5, -0.1, 0.0, 2.0], 15)),
        ("quarter: lap boundary", Q, [track_length(Q) - 1.0, 0.05, 0.0, 2.5], feedback(Q, [track_length(Q) - 1.0, 0.05, 0.0, 2.5], 10)),
        ("quarter: second lap, braking, 60 steps", Q, [track_length(Q) + 7.0, 0.0, 0.0, 2.5], feedback(Q, [track_length(Q) + 7.0, 0.0, 0.0, 2.5], 60; a=k -> k < 30 ? 0.0 : -0.5)),
        ("quarter: from rest at s = 1.9", Q, [1.9, 0.0, 0.0, 0.0], feedback(Q, [1.9, 0.0, 0.0, 0.0], 10; a=k -> 3.0))]
        sim = simulate_on_track(VEH, tr, x0, us, 0.1; max_step=1e-3)
        zs = simulate_exact_cartesian(VEH, frenet_state_to_cartesian(tr, x0), us, 0.1)
        zt = [frenet_state_to_cartesian(tr, x) for x in sim.states]
        t *= @sprintf("| %s | %d | %.1e | %.1e | %.1e |\n", name, length(sim.crossings), maximum(hypot(a[1] - b[1], a[2] - b[2]) for (a, b) in zip(zt, zs)),
            maximum(abs(a[3] - b[3]) for (a, b) in zip(zt, zs)), maximum(abs(a[4] - b[4]) for (a, b) in zip(zt, zs)))
    end
    emit(t * "\n")

    # --- compilation and first construction, then warm-up
    tfirst = Dict{String,Tuple{Float64,Float64}}()
    for (name, cfg) in (("stagewise", NMPCConfig(schedule_corrections=8)), ("Cartesian", NMPCConfig(; CART..., schedule_corrections=8)))
        tb = @elapsed (c = NMPCController(VEH, Q, cfg))
        ts = @elapsed solve!(c, X0)
        tfirst[name] = (tb, ts)
    end
    emit(@sprintf("First construction / first solve in this process, compilation included: stagewise %.2f s / %.2f s, Cartesian %.2f s / %.2f s (the Cartesian figures exclude what the stagewise run had already compiled). All other timings below are warmed up.\n\n",
        tfirst["stagewise"]..., tfirst["Cartesian"]...))
    for (_, cfg) in configs(20, 0.05)
        closed_loop(NMPCController(VEH, Q, cfg), 20)
    end
    # The benchmark loop reproduces simulate_closed_loop exactly.
    cfg = NMPCConfig(; CART..., schedule_corrections=8, correction_time_budget=100.0)
    a = closed_loop(primed(Q, cfg), 40)
    b = simulate_closed_loop(NMPCController(VEH, Q, cfg), VEH, X0, 40)
    emit("Benchmark loop with a primed controller vs `simulate_closed_loop` without priming (40 cycles, Cartesian, no time budget): states identical = $(a.states == b.states).\n\n")

    keep = Dict{String,Any}()
    for (N, m) in GRID
        cfgs = configs(N, m)
        results = Dict{String,Vector{Any}}()
        for rep in 1:REPS, (name, cfg) in (isodd(rep) ? cfgs : reverse(cfgs))
            c = NMPCController(VEH, Q, cfg)
            tprime = @elapsed solve!(c, X0)
            reset_warm_start!(c)                   # as primed(); the priming time is reported
            cl = closed_loop(c, N_CYCLES)
            push!(get!(results, name, Any[]), runstats(c, cl; resolution=rep == 1, prime=tprime))
            rep == 1 && (keep["$name | N=$N m=$m"] = (c, cl))
        end
        t1, t2 = tables(first.(cfgs), results)
        emit("## Quarter track, N = $N ($(N * 0.1) s horizon), margin $(m) m\n\n" * t1 * "\n" * t2 * "\n")
        emit("Stop reasons (run 1):\n\n" * join(["- **$(n)**: " * (isempty(results[n][1].stop) ? "completed" : first(results[n][1].stop, 320)) for n in first.(cfgs)], "\n") * "\n\n")
        # Sampling sensitivity of the predicted clearance (run 1, accepted plans).
        t = "Predicted track clearance vs re-simulation sampling (run 1, accepted plans; min over cycles) [m]:\n\n| configuration | 5 / interval | 20 / interval | 100 / interval |\n|:--|:-:|:-:|:-:|\n"
        for n in first.(cfgs)
            rr = results[n][1].res
            isempty(rr) && continue
            t *= @sprintf("| %s | %.4f | %.4f | %.4f |\n", n, nanmin(r[1] for r in rr), nanmin(r[2] for r in rr), nanmin(r[3] for r in rr))
        end
        emit(t * "\n")
        # Timing details: run-to-run variation and slow cycles.
        t = "Timing by run (median / p99 / max [ms]; Ipopt time median / max [ms]; allocations median / max [MB]; GC time total / max [ms]):\n\n| configuration | run 1 | run 2 | run 3 |\n|:--|:--|:--|:--|\n"
        for n in first.(cfgs)
            t *= "| $n | " * join([@sprintf("%.1f / %.1f / %.1f; Ipopt %.1f / %.1f; %.1f / %.1f MB; GC %.1f / %.1f", quant(r.tot, 0.5), quant(r.tot, 0.99),
                maximum(r.tot), quant(r.solver, 0.5), maximum(r.solver), quant(r.alloc, 0.5), maximum(r.alloc), sum(r.gct), maximum(r.gct))
                                  for r in results[n]], " | ") * " |\n"
        end
        slow = [(n, i, s) for n in first.(cfgs) for (i, r) in enumerate(results[n]) for s in r.slow]
        if !isempty(slow)
            t *= "\nCycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):\n\n"
            for (n, i, s) in slow
                t *= @sprintf("- %s, run %d, cycle %d: %.1f / %.1f / %.1f ms, GC %.1f ms, %.1f MB, %d solve(s), %d iterations\n", n, i, s...)
            end
        end
        emit(t * "\n")
    end

    # --- the same grid without the correction time budget (one run each)
    t = """
    ## Without the correction time budget (one run per configuration)

    The 80 ms budget makes the correction/reassignment decisions depend on wall time. Here every configuration is rerun once with the budget lifted (`correction_time_budget = 1000 s`; corrections still capped at 8), so the outcome depends only on the computation. Timings are not reported.

    | N | margin | configuration | laps / stop | accepted cycles | NLP solves mean / max | stop reason |
    |--:|--:|:--|:--|--:|:-:|:--|
    """
    for (N, m) in GRID, (name, cfg) in configs(N, m)
        cl = closed_loop(primed(Q, nobudget(cfg)), N_CYCLES)
        sol = [r.nlp_solves for r in cl.results]
        laps = (cl.states[end][1] - cl.states[1][1]) / track_length(Q)
        t *= @sprintf("| %d | %.2f | %s | %s | %d | %.2f / %d | %s |\n", N, m, name,
            cl.status === :success ? @sprintf("%.2f", laps) : @sprintf("stop %.2f (c%d)", laps, length(cl.results)),
            count(r -> r.accepted, cl.results), sum(sol) / length(sol), maximum(sol), isempty(cl.msg) ? "completed" : first(cl.msg, 200))
    end
    emit(t * "\n")

    # --- single transition, cold start, one cycle
    t = """
    ## Straight → arc (κ 0 → 0.5), one cycle, cold start, N = 20, margin 0.05

    Each controller is primed (one solve at the same state, then reset), so the timed cycle excludes the evaluator construction.

    | v [m/s] | φ | formulation | status | NLP solves | exact-map horizon error [m] | event-aware reference horizon error [m] |
    |:-:|:-:|:--|:--|--:|:-:|:-:|
    """
    for v in (1.0, 2.0, 2.8), phi in (0.1, 0.5, 0.9), (fname, cfg) in (("stagewise", NMPCConfig()),
        ("segment-pair", NMPCConfig(curvature_model=:segment_pair_exact)), ("Cartesian, 1 solve", NMPCConfig(; CART...)),
        ("Cartesian, ≤ 8 reassignments", NMPCConfig(; CART..., schedule_corrections=8)))
        x = [3.0 - phi * v * 0.1, 0.0, 0.0, v]
        c = primed(TRANS, cfg, x)
        r = solve!(c, x)
        e = all(isfinite, r.X) ? exact_map_check(c, x, r.X, r.U).max_position_error : NaN
        t *= @sprintf("| %.1f | %.1f | %s | %s | %d | %.1e | %.1e |\n", v, phi, fname, r.status, r.nlp_solves, e, r.reference.max_position_error)
    end
    emit(t * "\n")

    # --- single-segment tracks: Cartesian vs Frenet RK4 (exact on these tracks up to RK4 error)
    t = "## Single-segment tracks (Cartesian mode vs default Frenet RK4)\n\n| track | Cartesian status | max state difference | s_end Frenet / Cartesian [m] | Cartesian max exact-map error [m] |\n|:--|:--|:-:|:-:|:-:|\n"
    for (tname, tr, x0, n) in (("straight 150 m", straight_track(150.0; width=0.7), [0.0, 0.2, 0.1, 0.5], 100),
        ("circle R = 1 m", circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5], 70))
        a = simulate_closed_loop(NMPCController(VEH, tr, NMPCConfig()), VEH, x0, n)
        b = simulate_closed_loop(NMPCController(VEH, tr, NMPCConfig(; CART..., schedule_corrections=8)), VEH, x0, n)
        d = maximum(maximum(abs.(p - q)) for (p, q) in zip(a.states, b.states))
        t *= @sprintf("| %s | %s | %.1e | %.4f / %.4f | %.1e |\n", tname, b.status, d, a.states[end][1], b.states[end][1],
            nanmax(r.exact.max_position_error for r in b.controller))
    end
    emit(t * "\n")

    out = joinpath(@__DIR__, "results", "cartesian_endpoint_nmpc.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return keep
end

keep = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    sel = [("B segment-pair D2, ≤ 8 corr. | N=20 m=0.1", "B segment-pair D2, N = 20, margin 0.10"),
        ("D Cartesian, ≤ 8 reassignments | N=20 m=0.1", "D Cartesian, N = 20, margin 0.10"),
        ("D Cartesian, ≤ 8 reassignments | N=20 m=0.05", "D Cartesian, N = 20, margin 0.05")]
    ps = map(sel) do (key, ttl)
        c, cl = keep[key]
        rs = filter(r -> r.accepted, cl.results)
        tc = 0.1 .* (0:length(rs)-1)
        p = scatter(tc, [max(r.reference.max_position_error, 1e-9) for r in rs]; ms=2, yscale=:log10, ylims=(1e-9, 1e-1),
            ylabel="horizon prediction error [m]", title=ttl, label="vs event-aware reference")
        q = plot(tc, [r.reference.min_corridor_distance for r in rs]; label="predicted corridor clearance", ylabel="[m]", xlabel="t [s]")
        plot!(q, tc, [r.reference.min_boundary_distance for r in rs]; label="predicted track clearance")
        hline!(q, [0.0]; lc=:black, ls=:dash, label="")
        plot(p, q; layout=(2, 1))
    end
    savefig(plot(ps...; layout=(1, 3), size=(1800, 700), left_margin=6Plots.mm, bottom_margin=6Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "cartesian_endpoint_quarter.png"))
    println("saved figure")
end
