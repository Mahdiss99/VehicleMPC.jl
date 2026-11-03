# Milestone 2B.3B: exact segment-pair curvature vs the existing formulations.
#
#     julia --project=. benchmarks/segment_pair_curvature.jl          # tables
#     julia --project=examples benchmarks/segment_pair_curvature.jl   # tables + figure
#
# Writes benchmarks/results/segment_pair_curvature.md (and
# docs/figures/segment_pair_quarter.png). Every configuration is run REPS times,
# interleaved, in one process after a common warm-up. All cycles count, failed ones
# included.

using VehicleMPC
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const N_CYCLES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 250
const REPS = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 3
const Q = quarter_track()
const TRANS = Track([TrackSegment(3.0, 0.0), TrackSegment(1.5π, 0.5), TrackSegment(6.0, 0.0)]; width=0.7)

quant(xs, q) = isempty(xs) ? NaN : (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))
nanmax(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : maximum(w))
nanmin(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : minimum(w))
ms(x) = 1e3 * x

const SP = (curvature_model=:segment_pair_exact,)
configs(N, m) = [
    ("A stagewise, ≤ 8 corr., reject", NMPCConfig(horizon=N, track_margin=m, schedule_corrections=8)),
    ("B progress-linearized, no TR, ≤ 8 corr., reject", NMPCConfig(horizon=N, track_margin=m, schedule_corrections=8, curvature_model=:progress_linearized)),
    ("C segment-pair, 1 solve", NMPCConfig(; SP..., horizon=N, track_margin=m)),
    ("D segment-pair, ≤ 8 corr. (residual policy)", NMPCConfig(; SP..., horizon=N, track_margin=m, schedule_corrections=8)),
    ("D2 segment-pair, ≤ 8 corr., reject segment changes", NMPCConfig(; SP..., horizon=N, track_margin=m, schedule_corrections=8, segment_change_policy=:reject)),
    ("E segment-pair + membership bounds, 1 solve", NMPCConfig(; SP..., horizon=N, track_margin=m, segment_membership=true)),
    ("E8 segment-pair + membership bounds, ≤ 8 corr.", NMPCConfig(; SP..., horizon=N, track_margin=m, segment_membership=true, schedule_corrections=8)),
]
const GRID = [(N, m) for N in (10, 20) for m in (0.05, 0.10)]

function runstats(c, cl)
    rs = cl.controller
    acc = filter(r -> r.accepted, rs)
    hard = filter(k -> 7.5 <= mod(cl.states[k][1], track_length(Q)) <= 9.5, eachindex(rs))
    return (; status=cl.status, cycles=length(rs), laps=(cl.states[end][1] - cl.states[1][1]) / track_length(Q),
        consistent=count(r -> r.status === :success, rs), qualified=count(r -> r.status === :success_stale_schedule, rs),
        solver_fail=count(r -> r.status in (:solver_failure, :invalid_solution), rs),
        sched_fail=count(r -> r.status === :scheduling_failure, rs),
        segcfg_fail=count(r -> r.status === :schedule_inconsistent && r.schedule.configuration_changes > 0, rs),
        stale_fail=count(r -> r.status === :schedule_inconsistent, rs),
        ref_fail=count(r -> r.status === :reference_violation, rs),
        segcfg_acc=sum((r.schedule.configuration_changes > 0 for r in acc); init=0),
        solves=[r.nlp_solves for r in rs], iters=[r.iterations for r in rs], tot=[ms(r.timing.total) for r in rs],
        hard_solves=isempty(hard) ? NaN : sum(rs[k].nlp_solves for k in hard) / length(hard),
        stage_res=nanmax(r.schedule.stage_residual for r in acc), cum_res=nanmax(r.schedule.cumulative_residual for r in acc),
        first=nanmax(r.reference.first_step_position_error for r in acc), pos=nanmax(r.reference.max_position_error for r in acc),
        head=nanmax(r.reference.max_heading_error for r in acc), sprog=nanmax(r.reference.max_progress_error for r in acc),
        edge=nanmin(r.reference.min_boundary_distance for r in acc), corridor=nanmin(r.reference.min_corridor_distance for r in acc),
        dsmin=nanmin(r.schedule.min_progress_increment for r in acc),
        plant_viol=length(cl.violations), plant_edge=Q.width / 2 - maximum(abs(x[2]) for x in cl.fine_states),
        plant_corridor=c.ey_max - maximum(abs(x[2]) for x in cl.fine_states), stop=cl.message)
end

pooled(runs, f) = reduce(vcat, [f(r) for r in runs])

function tables(names, results)
    t1 = """
    | configuration | laps per run | fully consistent cycles | failures: solver / scheduling / segment-config. / other stale / reference | NLP solves mean / max | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m mean solves |
    |:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    t2 = """
    | configuration | plant violations | min plant clearance: track / corridor [m] | predicted (reference) clearance: track / corridor [m] | max stage / cum. residual [rad] | accepted cycles with segment change | reference error: first-step / horizon [m] | heading [rad] / progress [m] | min Δs* [m] |
    |:--|--:|:-:|:-:|:-:|--:|:-:|:-:|--:|
    """
    for name in names
        runs = results[name]
        laps = join([r.status === :success ? @sprintf("%.2f", r.laps) : @sprintf("stop %.2f (c%d)", r.laps, r.cycles) for r in runs], "; ")
        sol, it, tot = pooled(runs, r -> r.solves), pooled(runs, r -> r.iters), pooled(runs, r -> r.tot)
        ncyc = sum(r.cycles for r in runs)
        nc = sum(r.consistent for r in runs)
        hs = filter(!isnan, [r.hard_solves for r in runs])
        t1 *= @sprintf("| %s | %s | %d / %d (%.1f %%) | %d / %d / %d / %d / %d | %.2f / %d | %.1f / %d | %.1f / %.1f / %.1f / %.1f | %d / %d | %s |\n",
            name, laps, nc, ncyc, 100nc / ncyc, sum(r.solver_fail for r in runs), sum(r.sched_fail for r in runs),
            sum(r.segcfg_fail for r in runs), sum(r.stale_fail - r.segcfg_fail for r in runs), sum(r.ref_fail for r in runs),
            sum(sol) / length(sol), maximum(sol), sum(it) / length(it), maximum(it),
            quant(tot, 0.5), quant(tot, 0.95), quant(tot, 0.99), maximum(tot), count(>(80), tot), count(>(100), tot),
            isempty(hs) ? "–" : @sprintf("%.2f", sum(hs) / length(hs)))
        t2 *= @sprintf("| %s | %d | %.3f / %.3f | %.3f / %.3f | %.1e / %.1e | %d | %.1e / %.1e | %.1e / %.1e | %.3f |\n", name,
            sum(r.plant_viol for r in runs), minimum(r.plant_edge for r in runs), minimum(r.plant_corridor for r in runs),
            nanmin(r.edge for r in runs), nanmin(r.corridor for r in runs), nanmax(r.stage_res for r in runs), nanmax(r.cum_res for r in runs),
            sum(r.segcfg_acc for r in runs), nanmax(r.first for r in runs), nanmax(r.pos for r in runs), nanmax(r.head for r in runs),
            nanmax(r.sprog for r in runs), nanmin(r.dsmin for r in runs))
    end
    return t1, t2
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Segment-pair curvature benchmark (Milestone 2B.3B)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `[0, 0, 0, 0.5]`
    - $(N_CYCLES) control cycles (control period 0.1 s) per run, $(REPS) interleaved runs per configuration; event-aware plant; all cycles counted
    - Corrections use the existing mechanism (≤ 8 per cycle, 80 ms budget). Objective, weights and limits unchanged; horizon N and corridor margin as stated per table
    - "Corridor" = tightened planning corridor `|ey| ≤ width/2 − margin`; "track" = physical edge `|ey| ≤ width/2`

    """)
    # Geometric identity check (independent endpoints).
    worst = 0.0
    for tr in (Q, legacy_track(:lab), Track([TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0),
        TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0)]; closed=true))
        L = track_length(tr)
        hi = tr.closed ? 3L : L
        for a in range(0.0, hi * 0.97; length=60), d in (1e-4, 0.07, 0.4, 1.3)
            b = a + d
            (tr.closed || b <= L) || continue
            worst = max(worst, abs(heading_increment(tr, a, b) - (centerline_heading(tr, b) - centerline_heading(tr, a))))
        end
    end
    emit(@sprintf("Geometric identity: max |heading_increment − Δcenterline_heading| over quarter, lab and a closed loop (up to 3 laps, Δs from 1e-4 to 1.3 m): **%.1e rad**.\n\n", worst))

    for cfg in (NMPCConfig(schedule_corrections=8, correction_time_budget=100.0),
        NMPCConfig(; SP..., schedule_corrections=8, correction_time_budget=100.0),
        NMPCConfig(curvature_model=:progress_linearized, schedule_corrections=8, correction_time_budget=100.0))
        simulate_closed_loop(NMPCController(VEH, Q, cfg), VEH, [0.0, 0.0, 0.0, 0.5], 20)
    end

    keep = Dict{String,Any}()
    for (N, m) in GRID
        cfgs = configs(N, m)
        results = Dict{String,Vector{Any}}()
        for rep in 1:REPS, (name, cfg) in (isodd(rep) ? cfgs : reverse(cfgs))
            c = NMPCController(VEH, Q, cfg)
            cl = simulate_closed_loop(c, VEH, [0.0, 0.0, 0.0, 0.5], N_CYCLES)
            push!(get!(results, name, Any[]), runstats(c, cl))
            rep == 1 && (keep["$name | N=$N m=$m"] = (c, cl))
        end
        t1, t2 = tables(first.(cfgs), results)
        emit("## Quarter track, N = $N ($(N * 0.1) s horizon), margin $(m) m\n\n" * t1 * "\n" * t2 * "\n")
        emit("Stop reasons (run 1):\n\n" * join(["- **$(n)**: " * (isempty(results[n][1].stop) ? "completed" : first(results[n][1].stop, 300)) for n in first.(cfgs)], "\n") * "\n\n")
    end

    # Single straight → arc transition, one control cycle, cold start.
    t = """
    ## Straight → arc (κ 0 → 0.5), one cycle, cold start, N = 20, margin 0.05

    | v [m/s] | φ | formulation | status | NLP solves | stage residual [rad] | segment changes | reference: first-step / horizon [m] |
    |:-:|:-:|:--|:--|--:|--:|--:|:-:|
    """
    for v in (1.0, 2.0, 2.8), phi in (0.1, 0.5, 0.9), (fname, cfg) in (("stagewise", NMPCConfig()),
        ("progress-linearized", NMPCConfig(curvature_model=:progress_linearized)), ("segment-pair", NMPCConfig(; SP...)))
        r = solve!(NMPCController(VEH, TRANS, cfg), [3.0 - phi * v * 0.1, 0.0, 0.0, v])
        t *= @sprintf("| %.1f | %.1f | %s | %s | %d | %.1e | %d | %.1e / %.1e |\n", v, phi, fname, r.status, r.nlp_solves,
            r.schedule.stage_residual, r.schedule.configuration_changes, r.reference.first_step_position_error, r.reference.max_position_error)
    end
    emit(t * "\n")

    t = "## Single-segment regression\n\n| track | segment-pair identical to stagewise | s_end [m] |\n|:--|:-:|--:|\n"
    for (tname, tr, x0, n) in (("straight 150 m", straight_track(150.0; width=0.7), [0.0, 0.2, 0.1, 0.5], 100),
        ("circle R = 1 m", circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5], 70))
        a = simulate_closed_loop(NMPCController(VEH, tr, NMPCConfig()), VEH, x0, n)
        b = simulate_closed_loop(NMPCController(VEH, tr, NMPCConfig(; SP...)), VEH, x0, n)
        t *= @sprintf("| %s | %s | %.4f |\n", tname, a.states == b.states && a.inputs == b.inputs, b.states[end][1])
    end
    emit(t * "\n")

    out = joinpath(@__DIR__, "results", "segment_pair_curvature.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return keep
end

keep = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    sel = [("A stagewise, ≤ 8 corr., reject | N=20 m=0.1", "A stagewise, N = 20, margin 0.10"),
        ("D segment-pair, ≤ 8 corr. (residual policy) | N=20 m=0.1", "D segment-pair, N = 20, margin 0.10")]
    ps = map(sel) do (key, ttl)
        c, cl = keep[key]
        rs = cl.controller
        tc = cl.t[1:length(rs)]
        p = scatter(tc, [ms(r.timing.total) for r in rs]; ms=2, group=[string(r.status) for r in rs], ylabel="solve! [ms]",
            title=ttl, legend=:topright, ylims=(0, 140))
        hline!(p, [80, 100]; lc=[:orange :red], ls=:dash, label="")
        q = bar(tc, [r.nlp_solves for r in rs]; ylabel="NLP solves", xlabel="t [s]", label="", lw=0, ylims=(0, 9))
        plot(p, q; layout=(2, 1))
    end
    savefig(plot(ps...; layout=(1, 2), size=(1400, 650), left_margin=5Plots.mm, bottom_margin=5Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "segment_pair_quarter.png"))
    println("saved figure")
end
