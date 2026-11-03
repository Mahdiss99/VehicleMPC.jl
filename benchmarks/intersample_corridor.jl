# Milestone 2B.5: between-knot corridor constraints for the Cartesian-endpoint NMPC.
#
#     julia --project=. benchmarks/intersample_corridor.jl          # tables
#     julia --project=examples benchmarks/intersample_corridor.jl   # tables + figure
#
# Writes benchmarks/results/intersample_corridor.md (and
# docs/figures/intersample_corridor_quarter.png). Every configuration is run REPS times,
# interleaved, in one process after a common warm-up; all cycles count, failed ones
# included. Every controller is primed with one solve at the start state (then
# reset_warm_start!), which builds the NLP evaluator; the priming time is reported
# separately. The closed loop is simulate_closed_loop (event-aware plant, 10 substeps)
# with each solve! wrapped in @timed.

using VehicleMPC
using VehicleMPC.StaticArrays: SVector
using VehicleMPC.JuMP: all_variables, is_parameter, num_constraints
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

const BASE = (prediction_model=:cartesian_endpoint_exact, schedule_corrections=8, dense_check_samples=20)
configs(N, m) = [
    ("A Cartesian, no interior samples, strict", NMPCConfig(; BASE..., horizon=N, track_margin=m, strict_corridor=true)),
    ("B Cartesian + midpoint, strict", NMPCConfig(; BASE..., horizon=N, track_margin=m, interior_samples=1, strict_corridor=true)),
    ("C Cartesian + 3 samples, strict", NMPCConfig(; BASE..., horizon=N, track_margin=m, interior_samples=3, strict_corridor=true)),
    ("A0 = A without strict corridor (2B.4)", NMPCConfig(; BASE..., horizon=N, track_margin=m)),
    ("B0 = B without strict corridor", NMPCConfig(; BASE..., horizon=N, track_margin=m, interior_samples=1)),
    ("C0 = C without strict corridor", NMPCConfig(; BASE..., horizon=N, track_margin=m, interior_samples=3)),
]
const GRID = [(N, m) for N in (10, 20) for m in (0.05, 0.10)]

function primed(track, cfg, x=X0)
    c = NMPCController(VEH, track, cfg)
    t = @elapsed solve!(c, x)
    reset_warm_start!(c)
    return c, t
end

"simulate_closed_loop (event-aware plant) with per-cycle @timed statistics."
function closed_loop(ctrl, track, x0, n)
    h = ctrl.config.dt / 10
    x, uprev = SVector{4,Float64}(x0...), SVector(0.0, 0.0)
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
            st = integrate_on_track(VEH, track, x, r.u, h; max_step=h)
            if !succeeded(st)
                status, msg = :plant_stopped, "control period $k: plant $(st.status): $(st.message)"
                push!(states, st.x)
                return (; states, results, alloc, gct, fine, status, msg)
            end
            x = st.x
            push!(fine, x)
        end
        push!(states, x)
        uprev = r.u
    end
    return (; states, results, alloc, gct, fine, status, msg)
end

function runstats(c, cl, prime; resolution=false)
    rs = cl.results
    acc = filter(r -> r.accepted, rs)
    tot = [ms(r.timing.total) for r in rs]
    slow = findall(>(80), tot)
    res = resolution ? [(r.X[:, 1], r.X, r.U) for r in acc] : nothing
    rejected = Dict(s => count(r -> r.status === s, rs) for s in unique(r.status for r in rs) if !(s in (:success, :success_stale_schedule)))
    return (; status=cl.status, cycles=length(rs), laps=(cl.states[end][1] - cl.states[1][1]) / track_length(Q),
        accepted=length(acc), rejected, solves=[r.nlp_solves for r in rs], iters=[r.iterations for r in rs], tot,
        val=[ms(r.timing.reference) for r in rs], build=ms(c.build_time), prime=ms(prime),
        nvar=count(!is_parameter, all_variables(c.model)), ncon=num_constraints(c.model; count_variable_in_set_constraints=false),
        dense_corr=nanmin(r.corridor.min_corridor_clearance for r in acc), dense_track=nanmin(r.corridor.min_track_clearance for r in acc),
        ref_corr=nanmin(r.reference.min_corridor_distance for r in acc), viol_cycles=count(r -> r.corridor.violations > 0, acc),
        maxexc=nanmax(r.corridor.max_violation for r in acc), int_fail=sum((r.corridor.interior_failures for r in rs); init=0),
        seg_fail=count(r -> r.status in (:segment_assignment_invalid, :interior_assignment_invalid), rs),
        proj_fail=sum((r.corridor.projection_failures for r in acc); init=0), int_margin=nanmin(r.corridor.interior_min_margin for r in acc),
        int_err=nanmax(r.corridor.interior_prediction_error for r in acc),
        worst=isempty(acc) ? (NaN, 0) : (r = acc[argmin([r.corridor.min_corridor_clearance for r in acc])]; (r.corridor.worst_time, r.corridor.worst_stage)),
        plant_viol=count(x -> !isempty(check_operating_constraints(VEH, Q, x; tol=TOL)), cl.fine),
        plant_edge=Q.width / 2 - maximum(abs(x[2]) for x in cl.fine), plant_corr=c.ey_max - maximum(abs(x[2]) for x in cl.fine),
        slow=[(k, tot[k], ms(rs[k].timing.solver), ms(rs[k].timing.optimize), ms(rs[k].timing.reference), cl.gct[k], cl.alloc[k],
            rs[k].nlp_solves, rs[k].iterations) for k in slow],
        res, gc_total=sum(cl.gct), gc_max=maximum(cl.gct), stop=cl.msg)
end

pooled(runs, f) = reduce(vcat, [f(r) for r in runs])

function tables(names, results)
    t1 = """
    | configuration | laps per run | accepted cycles | rejections by status | NLP solves mean / max | corrections per cycle | Ipopt iter. mean / max | decision variables / constraints | build / priming [ms] | median / p95 / p99 / max [ms] | > 80 / > 100 ms | validation median / max [ms] |
    |:--|:--|--:|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    t2 = """
    | configuration | plant violations | plant clearance: track / corridor [m] | predicted (dense, 20/interval): track / corridor [m] | worst at t [s] (stage) | accepted cycles with corridor violation / largest excursion [m] | interior: min margin / assignment failures / prediction error [m] | event-aware re-simulation corridor [m] |
    |:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    for name in names
        runs = results[name]
        laps = join([r.status === :success ? @sprintf("%.2f", r.laps) : @sprintf("stop %.2f (c%d)", r.laps, r.cycles) for r in runs], "; ")
        sol, it, tot, val = pooled(runs, r -> r.solves), pooled(runs, r -> r.iters), pooled(runs, r -> r.tot), pooled(runs, r -> r.val)
        rej = mergewith(+, [r.rejected for r in runs]...)
        r1 = runs[1]
        t1 *= @sprintf("| %s | %s | %d / %d | %s | %.2f / %d | %.2f | %.1f / %d | %d / %d | %.0f / %.0f | %.1f / %.1f / %.1f / %.1f | %d / %d | %.2f / %.1f |\n",
            name, laps, sum(r.accepted for r in runs), sum(r.cycles for r in runs), isempty(rej) ? "–" : join(["$k $v" for (k, v) in rej], ", "),
            sum(sol) / length(sol), maximum(sol), sum(sol .- 1) / length(sol), sum(it) / length(it), maximum(it), r1.nvar, r1.ncon,
            sum(r.build for r in runs) / length(runs), sum(r.prime for r in runs) / length(runs),
            quant(tot, 0.5), quant(tot, 0.95), quant(tot, 0.99), maximum(tot), count(>(80), tot), count(>(100), tot), quant(val, 0.5), maximum(val))
        w = runs[argmin([isnan(r.dense_corr) ? Inf : r.dense_corr for r in runs])].worst
        t2 *= @sprintf("| %s | %d | %.4f / %.4f | %.4f / %.4f | %.3f (%d) | %d / %.4f | %.1e / %d / %.1e | %.4f |\n", name,
            sum(r.plant_viol for r in runs), minimum(r.plant_edge for r in runs), minimum(r.plant_corr for r in runs),
            nanmin(r.dense_track for r in runs), nanmin(r.dense_corr for r in runs), w[1], w[2],
            sum(r.viol_cycles for r in runs), nanmax(r.maxexc for r in runs),
            nanmin(r.int_margin for r in runs), sum(r.int_fail for r in runs), nanmax(r.int_err for r in runs), nanmin(r.ref_corr for r in runs))
    end
    return t1, t2
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Between-knot corridor constraints benchmark (Milestone 2B.5)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `$(X0)`
    - $(N_CYCLES) control cycles (control period 0.1 s, prediction step 0.1 s) per run, $(REPS) interleaved runs per configuration; event-aware plant (10 substeps); all cycles counted
    - All configurations: Cartesian-endpoint prediction, ≤ 8 segment reassignments within the 80 ms budget, dense exact-map corridor check at 20 samples per interval in every cycle. Objective, weights, limits, vehicle, Ipopt options and `acceptance_tol` = $(TOL) are identical
    - "strict" = `strict_corridor = true`: a candidate whose event-aware re-simulation (5/interval) or dense check (20/interval) leaves the planning corridor by more than $(TOL) m is rejected. A0/B0/C0 are the same controllers without that rule (diagnostic acceptance; A0 is the Milestone 2B.4 controller)
    - "Corridor" = `|ey| ≤ width/2 − margin`; "track" = `|ey| ≤ width/2`. Predicted clearances are minima over accepted plans
    - Timing covers everything in `solve!` (assignments, all solves, reference check, exact-map check, interior check, dense check); "validation" is the part spent in the independent checks

    """)

    # warm-up (compilation) on every configuration type
    for (_, cfg) in configs(20, 0.05)
        c, _ = primed(Q, cfg)
        closed_loop(c, Q, X0, 15)
    end
    keep = Dict{String,Any}()
    for (N, m) in GRID
        cfgs = configs(N, m)
        results = Dict{String,Vector{Any}}()
        for rep in 1:REPS, (name, cfg) in (isodd(rep) ? cfgs : reverse(cfgs))
            c, tp = primed(Q, cfg)
            cl = closed_loop(c, Q, X0, N_CYCLES)
            push!(get!(results, name, Any[]), runstats(c, cl, tp; resolution=rep == 1))
            rep == 1 && (keep["$name | N=$N m=$m"] = (c, cl))
        end
        t1, t2 = tables(first.(cfgs), results)
        emit("## Quarter track, N = $N ($(N * 0.1) s horizon), margin $(m) m\n\n" * t1 * "\n" * t2 * "\n")
        emit("Stop reasons (run 1):\n\n" * join(["- **$(n)**: " * (isempty(results[n][1].stop) ? "completed" : first(results[n][1].stop, 420)) for n in first.(cfgs)], "\n") * "\n\n")
        # Resolution study (run 1, accepted plans): dense 20/50/100 vs the analytic minimum.
        t = "Predicted corridor clearance vs validation resolution (run 1, accepted plans; minimum over cycles) [m], and analytic − dense(100) gap:\n\n| configuration | plans | 20 / interval | 50 / interval | 100 / interval | analytic (continuous) | max (dense 100 − analytic) | analytic: candidates / projection failures |\n|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|\n"
        for n in first.(cfgs)
            rr = results[n][1].res
            (rr === nothing || isempty(rr)) && continue
            c = keep["$n | N=$N m=$m"][1]
            d20 = [corridor_check(c, x, X, U; samples=20).min_corridor_clearance for (x, X, U) in rr]
            d50 = [corridor_check(c, x, X, U; samples=50).min_corridor_clearance for (x, X, U) in rr]
            d100 = [corridor_check(c, x, X, U; samples=100).min_corridor_clearance for (x, X, U) in rr]
            an = [analytic_corridor_clearance(c, x, X, U) for (x, X, U) in rr]
            t *= @sprintf("| %s | %d | %.5f | %.5f | %.5f | %.5f | %.1e | %d / %d |\n", n, length(rr), minimum(d20), minimum(d50), minimum(d100),
                minimum(a.min_corridor for a in an), maximum(d100 .- [a.min_corridor for a in an]), sum(a.candidates for a in an),
                sum(a.failures for a in an))
        end
        emit(t * "\n")
        t = "Timing by run (median / p99 / max [ms]; GC total / max [ms]):\n\n| configuration | run 1 | run 2 | run 3 |\n|:--|:--|:--|:--|\n"
        for n in first.(cfgs)
            t *= "| $n | " * join([@sprintf("%.1f / %.1f / %.1f; GC %.1f / %.1f", quant(r.tot, 0.5), quant(r.tot, 0.99), maximum(r.tot),
                r.gc_total, r.gc_max) for r in results[n]], " | ") * " |\n"
        end
        slow = [(n, i, s) for n in first.(cfgs) for (i, r) in enumerate(results[n]) for s in r.slow]
        if !isempty(slow)
            t *= "\nCycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! / validation [ms], GC [ms], allocated [MB], NLP solves, Ipopt iterations):\n\n"
            for (n, i, s) in slow
                t *= @sprintf("- %s, run %d, cycle %d: %.1f / %.1f / %.1f / %.1f ms, GC %.1f ms, %.1f MB, %d solve(s), %d iterations\n", n, i, s...)
            end
        end
        emit(t * "\n")
    end

    # Without the correction time budget: separates runtime limits from algorithmic failures.
    t = """
    ## Without the correction time budget (diagnostic acceptance, one run each)

    `correction_time_budget = 1000 s` (still ≤ 8 reassignments), primed controllers. The outcome then depends only on the computation; the timings show what the 80 ms budget would have cut.

    | N | margin | configuration | laps / stop | NLP solves mean / max | median / p99 / max [ms] | > 80 / > 100 ms | predicted dense corridor clearance [m] |
    |--:|--:|:--|:--|:-:|:-:|:-:|:-:|
    """
    for (N, m) in GRID, (name, cfg) in configs(N, m)[4:6]
        cfg = NMPCConfig(; (f => getfield(cfg, f) for f in fieldnames(NMPCConfig))..., correction_time_budget=1e3)
        c, _ = primed(Q, cfg)
        cl = closed_loop(c, Q, X0, N_CYCLES)
        sol, tot = [r.nlp_solves for r in cl.results], [ms(r.timing.total) for r in cl.results]
        laps = (cl.states[end][1] - cl.states[1][1]) / track_length(Q)
        t *= @sprintf("| %d | %.2f | %s | %s | %.2f / %d | %.1f / %.1f / %.1f | %d / %d | %.4f |\n", N, m, name,
            cl.status === :success ? @sprintf("%.2f", laps) : @sprintf("stop %.2f (c%d): %s", laps, length(cl.results), first(cl.msg, 120)),
            sum(sol) / length(sol), maximum(sol), quant(tot, 0.5), quant(tot, 0.99), maximum(tot), count(>(80), tot), count(>(100), tot),
            nanmin(r.corridor.min_corridor_clearance for r in cl.results if r.accepted))
    end
    emit(t * "\n")

    # Other tracks: straight, circle, single transition (one run each, 100 cycles; N = 20, margin 0.05).
    t = """
    ## Other tracks (N = 20, margin 0.05, one run of 100 cycles)

    | track | configuration | status | accepted cycles | predicted dense corridor clearance [m] | analytic corridor clearance [m] | plant clearance: track / corridor [m] | median / max [ms] |
    |:--|:--|:--|--:|:-:|:-:|:-:|:-:|
    """
    for (tname, tr, x0) in (("straight 150 m", straight_track(150.0; width=0.7), [0.0, 0.2, 0.1, 0.5]),
        ("circle R = 1 m", circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5]), ("straight → arc → straight", TRANS, [0.0, 0.0, 0.0, 0.5]))
        for (name, cfg) in configs(20, 0.05)[1:3]
            c, _ = primed(tr, cfg, x0)
            cl = closed_loop(c, tr, x0, 100)
            acc = filter(r -> r.accepted, cl.results)
            an = [analytic_corridor_clearance(c, r.X[:, 1], r.X, r.U).min_corridor for r in acc]
            tot = [ms(r.timing.total) for r in cl.results]
            t *= @sprintf("| %s | %s | %s | %d / %d | %.4f | %.4f | %.4f / %.4f | %.1f / %.1f |\n", tname, name,
                cl.status === :success ? "completed" : first(cl.msg, 160), length(acc), length(cl.results),
                nanmin(r.corridor.min_corridor_clearance for r in acc), nanmin(an), tr.width / 2 - maximum(abs(x[2]) for x in cl.fine),
                c.ey_max - maximum(abs(x[2]) for x in cl.fine), quant(tot, 0.5), maximum(tot))
        end
    end
    emit(t * "\n")

    out = joinpath(@__DIR__, "results", "intersample_corridor.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return keep
end

keep = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    sel = [("A0 = A without strict corridor (2B.4) | N=20 m=0.05", "no interior samples (A0), N = 20, margin 0.05"),
        ("B0 = B without strict corridor | N=20 m=0.05", "midpoint (B0), N = 20, margin 0.05"),
        ("C0 = C without strict corridor | N=20 m=0.05", "3 samples (C0), N = 20, margin 0.05")]
    ps = map(sel) do (key, ttl)
        c, cl = keep[key]
        rs = filter(r -> r.accepted, cl.results)
        tc = 0.1 .* (0:length(rs)-1)
        p = plot(tc, [r.corridor.min_corridor_clearance for r in rs]; label="dense (20/interval)", ylabel="predicted corridor clearance [m]",
            title=ttl, ylims=(-0.045, 0.25))
        hline!(p, [0.0]; lc=:black, ls=:dash, label="")
        q = scatter(tc, [1e3r.timing.total for r in rs]; ms=2, label="", ylabel="solve! [ms]", xlabel="t [s]", ylims=(0, 120))
        hline!(q, [80, 100]; lc=[:orange :red], ls=:dash, label="")
        plot(p, q; layout=(2, 1))
    end
    savefig(plot(ps...; layout=(1, 3), size=(1800, 700), left_margin=6Plots.mm, bottom_margin=6Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "intersample_corridor_quarter.png"))
    println("saved figure")
end
