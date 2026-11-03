# Milestone 2B.3A: progress-linearized curvature vs stagewise-average curvature.
#
#     julia --project=. benchmarks/progress_aware_curvature.jl          # tables
#     julia --project=examples benchmarks/progress_aware_curvature.jl   # tables + figure
#
# Writes benchmarks/results/progress_aware_curvature.md (and
# docs/figures/progress_aware_quarter.png). All configurations run in the same process,
# interleaved by repetition, after a common warm-up. Every cycle counts in the statistics,
# including failed ones.

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

const LIN = (curvature_model=:progress_linearized,)
const CONFIGS = [
    ("A  stagewise, 1 solve, reject", NMPCConfig()),
    ("B  stagewise, ≤ 8 corrections / 80 ms, reject", NMPCConfig(schedule_corrections=8)),
    ("C3 stagewise, ≤ 8 corr., qualified, margin 0.05 (2B.2 D3)", NMPCConfig(schedule_corrections=8, stale_schedule_policy=:accept_if_reference_valid)),
    ("C4 stagewise, ≤ 8 corr., qualified, margin 0.10 (2B.2 D4)", NMPCConfig(track_margin=0.10, schedule_corrections=8, stale_schedule_policy=:accept_if_reference_valid)),
    ("D0 linearized, no trust region, 1 solve, reject", NMPCConfig(; LIN...)),
    ("D  linearized, TR 0.1, 1 solve, reject, margin 0.05", NMPCConfig(; LIN..., linearization_trust_region=0.1)),
    ("D' linearized, TR 0.1, 1 solve, reject, margin 0.10", NMPCConfig(; LIN..., linearization_trust_region=0.1, track_margin=0.10)),
    ("E  linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.05", NMPCConfig(; LIN..., linearization_trust_region=0.1, schedule_corrections=8)),
    ("E' linearized, TR 0.1, ≤ 8 corr. / 80 ms, reject, margin 0.10", NMPCConfig(; LIN..., linearization_trust_region=0.1, schedule_corrections=8, track_margin=0.10)),
]
const HORIZON_CONFIGS = vcat([
    [("C4 stagewise, ≤ 8 corr., qualified, margin 0.10, N = $N", NMPCConfig(horizon=N, track_margin=0.10, schedule_corrections=8, stale_schedule_policy=:accept_if_reference_valid)),
     ("B' stagewise, ≤ 8 corr., reject, margin 0.10, N = $N", NMPCConfig(horizon=N, track_margin=0.10, schedule_corrections=8)),
     ("L1 linearized, no TR, 1 solve, reject, margin 0.10, N = $N", NMPCConfig(; LIN..., horizon=N, track_margin=0.10)),
     ("L8 linearized, no TR, ≤ 8 corr., reject, margin 0.10, N = $N", NMPCConfig(; LIN..., horizon=N, track_margin=0.10, schedule_corrections=8)),
     ("L8s linearized, no TR, ≤ 8 corr., reject, margin 0.05, N = $N", NMPCConfig(; LIN..., horizon=N, schedule_corrections=8)),
     ("T1 linearized, TR 0.1, 1 solve, reject, margin 0.10, N = $N", NMPCConfig(; LIN..., horizon=N, linearization_trust_region=0.1, track_margin=0.10))]
    for N in (10, 20)]...)

function run_quarter(cfg)
    c = NMPCController(VEH, Q, cfg)
    cl = simulate_closed_loop(c, VEH, [0.0, 0.0, 0.0, 0.5], N_CYCLES)
    return c, cl
end

"Statistics of one closed-loop run (all cycles, including a final failed one)."
function runstats(c, cl)
    rs = cl.controller
    acc = filter(r -> r.accepted, rs)
    tot = [ms(r.timing.total) for r in rs]
    hard = filter(k -> 7.5 <= mod(cl.states[k][1], track_length(Q)) <= 9.5, eachindex(rs))   # the s ≈ 8.5 m transition
    return (; status=cl.status, cycles=length(rs), laps=(cl.states[end][1] - cl.states[1][1]) / track_length(Q),
        failures=count(r -> !r.accepted, rs), consistent=count(r -> r.status === :success, rs),
        qualified=count(r -> r.status === :success_stale_schedule, rs),
        solves=[r.nlp_solves for r in rs], iters=[r.iterations for r in rs], tot,
        hard_solves=isempty(hard) ? NaN : sum(rs[k].nlp_solves for k in hard) / length(hard),
        hard_maxsolves=isempty(hard) ? 0 : maximum(rs[k].nlp_solves for k in hard),
        stage_res=nanmax(r.schedule.stage_residual for r in acc), cum_res=nanmax(r.schedule.cumulative_residual for r in acc),
        model_err=nanmax(r.schedule.model_curvature_error for r in acc),
        conf=sum((r.schedule.configuration_changes for r in acc); init=0), sat=sum((r.schedule.saturated_stages for r in acc); init=0),
        tr_active=sum((r.schedule.trust_region_active for r in acc); init=0),
        first=nanmax(r.reference.first_step_position_error for r in acc), pos=nanmax(r.reference.max_position_error for r in acc),
        head=nanmax(r.reference.max_heading_error for r in acc), sprog=nanmax(r.reference.max_progress_error for r in acc),
        ey=nanmax(r.reference.max_ey_error for r in acc), edge=nanmin(r.reference.min_boundary_distance for r in acc),
        plant_viol=length(cl.violations), plant_edge=track_length(Q) > 0 ? Q.width / 2 - maximum(abs(x[2]) for x in cl.fine_states) : NaN,
        stop=cl.message)
end

pooled(runs, f) = reduce(vcat, [f(r) for r in runs])

function table_rows(names, results)
    t1 = """
    | configuration | outcomes (laps, per run) | failed cycles | fully consistent / qualified accepted | NLP solves: mean / max | Ipopt iterations: mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | s ≈ 8.5 m: mean / max solves |
    |:--|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    t2 = """
    | configuration | plant violations | min plant edge distance [m] | max stage / cum. residual [rad] | max κ model error [1/m] | config. changes / saturated stages / trust region active | reference: first-step / max position [m] | heading [rad] / progress / ey [m] | min predicted edge distance [m] |
    |:--|--:|--:|:-:|--:|:-:|:-:|:-:|--:|
    """
    for name in names
        runs = results[name]
        laps = join([r.status === :success ? @sprintf("%.2f", r.laps) : @sprintf("stop %.2f (c%d)", r.laps, r.cycles) for r in runs], "; ")
        sol, it, tot = pooled(runs, r -> r.solves), pooled(runs, r -> r.iters), pooled(runs, r -> r.tot)
        hs = filter(!isnan, [r.hard_solves for r in runs])
        t1 *= @sprintf("| %s | %s | %d | %d / %d | %.2f / %d | %.1f / %d | %.1f / %.1f / %.1f / %.1f | %d / %d | %s |\n", name, laps,
            sum(r.failures for r in runs), sum(r.consistent for r in runs), sum(r.qualified for r in runs),
            sum(sol) / length(sol), maximum(sol), sum(it) / length(it), maximum(it),
            quant(tot, 0.5), quant(tot, 0.95), quant(tot, 0.99), maximum(tot), count(>(80), tot), count(>(100), tot),
            isempty(hs) ? "–" : @sprintf("%.2f / %d", sum(hs) / length(hs), maximum(r.hard_maxsolves for r in runs)))
        t2 *= @sprintf("| %s | %d | %.3f | %.1e / %.1e | %.1e | %d / %d / %d | %.1e / %.1e | %.1e / %.1e / %.1e | %.3f |\n", name,
            sum(r.plant_viol for r in runs), minimum(r.plant_edge for r in runs), nanmax(r.stage_res for r in runs), nanmax(r.cum_res for r in runs),
            nanmax(r.model_err for r in runs), sum(r.conf for r in runs), sum(r.sat for r in runs), sum(r.tr_active for r in runs),
            nanmax(r.first for r in runs), nanmax(r.pos for r in runs), nanmax(r.head for r in runs), nanmax(r.sprog for r in runs),
            nanmax(r.ey for r in runs), nanmin(r.edge for r in runs))
    end
    return t1, t2
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Progress-linearized curvature benchmark (Milestone 2B.3A)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; `quarter_track()` from `[0, 0, 0, 0.5]`
    - $(N_CYCLES) control cycles per run (control period 0.1 s), $(REPS) runs per configuration, interleaved by run; event-aware plant
    - Time per cycle = the complete `solve!` (schedule, linearization, parameter updates, all NLP solves, validation, consistency test, reference check); failed cycles included
    - "TR ρ": trust region `|s_k - s̄_k| ≤ ρ·|Δs̄|` on knots of stages with non-zero sensitivities (an additional constraint, off by default)

    """)
    # Warm-up (compilation) for both formulations.
    for cfg in (NMPCConfig(schedule_corrections=8, correction_time_budget=100.0),
        NMPCConfig(; LIN..., linearization_trust_region=0.1, schedule_corrections=8, correction_time_budget=100.0))
        c = NMPCController(VEH, Q, cfg)
        simulate_closed_loop(c, VEH, [0.0, 0.0, 0.0, 0.5], 20)
    end

    results = Dict{String,Vector{Any}}()
    keep = Dict{String,Any}()
    for rep in 1:REPS, (name, cfg) in (isodd(rep) ? CONFIGS : reverse(CONFIGS))
        c, cl = run_quarter(cfg)
        push!(get!(results, name, Any[]), runstats(c, cl))
        rep == 1 && (keep[name] = (c, cl))
    end
    t1, t2 = table_rows(first.(CONFIGS), results)
    emit("## Quarter track: completion, solves and timing\n\nOutcome per run: laps completed (`stop x (cN)` = stopped at cycle N after x laps).\n\n" * t1 * "\n")
    emit("## Quarter track: constraints, curvature consistency and prediction accuracy (accepted cycles)\n\n" * t2 * "\n")
    emit("### Stop reasons (run 1)\n\n" * join(["- **$(n)**: " * (isempty(results[n][1].stop) ? "completed" : first(results[n][1].stop, 400)) for n in first.(CONFIGS)], "\n") * "\n\n")

    # Horizon study (prediction dt = control period = 0.1 s).
    hres = Dict{String,Vector{Any}}()
    for rep in 1:REPS, (name, cfg) in (isodd(rep) ? HORIZON_CONFIGS : reverse(HORIZON_CONFIGS))
        c, cl = run_quarter(cfg)
        push!(get!(hres, name, Any[]), runstats(c, cl))
        rep == 1 && (keep[name] = (c, cl))
    end
    h1, h2 = table_rows(first.(HORIZON_CONFIGS), hres)
    emit("## Prediction-horizon study (dt = 0.1 s = control period)\n\n" * h1 * "\n" * h2 * "\n")

    # Straight → arc, one cycle, cold start (crossing positions and speeds as in 2B.2).
    t = """
    ## Straight → arc (κ 0 → 0.5), one control cycle, cold start

    | v [m/s] | φ | formulation | status | NLP solves | stage / cum. residual [rad] | κ model error | reference: first-step / max position [m] |
    |:-:|:-:|:--|:--|--:|:-:|--:|:-:|
    """
    for v in (1.0, 2.0, 2.8), phi in (0.1, 0.5, 0.9), (fname, cfg) in (("stagewise", NMPCConfig()),
        ("linearized TR 0.1", NMPCConfig(; LIN..., linearization_trust_region=0.1)), ("linearized, no TR", NMPCConfig(; LIN...)))
        r = solve!(NMPCController(VEH, TRANS, cfg), [3.0 - phi * v * 0.1, 0.0, 0.0, v])
        t *= @sprintf("| %.1f | %.1f | %s | %s | %d | %.1e / %.1e | %.1e | %.1e / %.1e |\n", v, phi, fname, r.status, r.nlp_solves,
            r.schedule.stage_residual, r.schedule.cumulative_residual, r.schedule.model_curvature_error,
            r.reference.first_step_position_error, r.reference.max_position_error)
    end
    emit(t * "\n")

    # Single-segment regression: the linearized option must not change straight/circle results.
    t = "## Single-segment regression\n\n| track | formulation | status | cycles | s_end [m] | identical to stagewise |\n|:--|:--|:--|--:|--:|:-:|\n"
    for (tname, tr, x0, n) in (("straight 150 m", straight_track(150.0; width=0.7), [0.0, 0.2, 0.1, 0.5], 100),
        ("circle R = 1 m", circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5], 70))
        cls = map((NMPCConfig(), NMPCConfig(; LIN..., linearization_trust_region=0.1))) do cfg
            simulate_closed_loop(NMPCController(VEH, tr, cfg), VEH, x0, n)
        end
        same = cls[1].states == cls[2].states && cls[1].inputs == cls[2].inputs
        for (fname, cl) in zip(("stagewise", "linearized"), cls)
            t *= @sprintf("| %s | %s | %s | %d | %.4f | %s |\n", tname, fname, cl.status, length(cl.controller), cl.states[end][1], same)
        end
    end
    emit(t * "\n")

    out = joinpath(@__DIR__, "results", "progress_aware_curvature.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return keep
end

keep = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    names = ("C4 stagewise, ≤ 8 corr., qualified, margin 0.10 (2B.2 D4)",
        "B' stagewise, ≤ 8 corr., reject, margin 0.10, N = 10",
        "L8 linearized, no TR, ≤ 8 corr., reject, margin 0.10, N = 10")
    titles = ("C4: stagewise, N = 20, qualified", "B': stagewise, N = 10, strict", "L8: progress-linearized, N = 10, strict")
    ps = map(zip(names, titles)) do (name, ttl)
        c, cl = keep[name]
        rs = cl.controller
        tc = cl.t[1:length(rs)]
        p = scatter(tc, [ms(r.timing.total) for r in rs]; ms=2, group=[string(r.status) for r in rs], ylabel="solve! [ms]",
            title=ttl, legend=:topright, ylims=(0, 140))
        hline!(p, [80, 100]; lc=[:orange :red], ls=:dash, label="")
        q = bar(tc, [r.nlp_solves for r in rs]; ylabel="NLP solves", xlabel="t [s]", label="", lw=0, ylims=(0, 9))
        plot(p, q; layout=(2, 1))
    end
    savefig(plot(ps...; layout=(1, 3), size=(1800, 650), left_margin=5Plots.mm, bottom_margin=5Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "progress_aware_quarter.png"))
    println("saved figure")
end
