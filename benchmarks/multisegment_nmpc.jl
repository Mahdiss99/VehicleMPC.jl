# Milestone 2B.2 experiments: multi-segment NMPC with stagewise average curvature.
#
#     julia --project=. benchmarks/multisegment_nmpc.jl            # tables
#     julia --project=examples benchmarks/multisegment_nmpc.jl     # tables + figure (needs Plots)
#
# Writes benchmarks/results/multisegment_nmpc.md (and docs/figures/multiseg_nmpc_quarter.png).
# Wall-clock timings depend on the machine and its load; the correction budget makes some
# outcomes timing-dependent, which is reported, not hidden.

using VehicleMPC
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const N_CYCLES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 250
const TRANS = Track([TrackSegment(3.0, 0.0), TrackSegment(1.5π, 0.5), TrackSegment(6.0, 0.0)]; width=0.7)

quant(xs, q) = isempty(xs) ? NaN : (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))
ms(x) = 1e3 * x
nanmax(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : maximum(w))
counts(xs) = join(["$(k): $(count(==(k), xs))" for k in sort(unique(xs); by=string)], ", ")

"Closed-loop summary row."
function summarize(name, ctrl, cl)
    rs = cl.controller
    tot = [ms(r.timing.total) for r in rs]
    acc = filter(r -> r.accepted, rs)
    L = track_length(ctrl.track)
    onestep = [hypot((frenet_to_cartesian(ctrl.track, rs[k].X[1, 2], rs[k].X[2, 2], rs[k].X[3, 2])[1:2] .-
                      frenet_to_cartesian(ctrl.track, cl.states[k+1][1], cl.states[k+1][2], cl.states[k+1][3])[1:2])...)
               for k in eachindex(cl.inputs)]
    return (; name, status=cl.status, message=cl.message, cycles=length(rs), laps=is_closed(ctrl.track) ? (cl.states[end][1] - cl.states[1][1]) / L : NaN,
        s_end=cl.states[end][1], statuses=counts([r.status for r in rs]), solves=counts([r.nlp_solves for r in rs]),
        median=quant(tot, 0.5), p95=quant(tot, 0.95), p99=quant(tot, 0.99), max=isempty(tot) ? NaN : maximum(tot),
        over80=count(>(80), tot), over100=count(>(100), tot),
        t_sched=sum(ms(r.timing.schedule) for r in rs) / length(rs), t_update=sum(ms(r.timing.update + r.timing.warm_start) for r in rs) / length(rs),
        t_opt=sum(ms(r.timing.optimize) for r in rs) / length(rs), t_ipopt=sum(r -> isnan(r.timing.solver) ? 0.0 : ms(r.timing.solver), rs) / length(rs),
        t_ref=sum(ms(r.timing.reference) for r in rs) / length(rs), t_val=sum(ms(r.timing.validation) for r in rs) / length(rs),
        solver_fail=count(r -> r.status === :solver_failure, rs), sched_fail=count(r -> r.status === :scheduling_failure, rs),
        stale=count(r -> r.status in (:schedule_inconsistent, :success_stale_schedule), rs),
        ref_fail=count(r -> r.status === :reference_violation, rs),
        sched_nonconv=count(r -> !r.schedule.converged, rs), sched_it_max=maximum(r -> r.schedule.iterations, rs),
        acc_stage_res=nanmax(r.schedule.stage_residual for r in acc), acc_cum_res=nanmax(r.schedule.cumulative_residual for r in acc),
        acc_pos=nanmax(r.reference.max_position_error for r in acc), acc_head=nanmax(r.reference.max_heading_error for r in acc),
        acc_s=nanmax(r.reference.max_progress_error for r in acc), acc_ey=nanmax(r.reference.max_ey_error for r in acc),
        onestep=nanmax(onestep), plant_viol=length(cl.violations), plant_ey=maximum(abs(x[2]) for x in cl.fine_states),
        crossings=length(cl.crossings))
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Multi-segment NMPC experiments (Milestone 2B.2)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; N = 20, dt = 0.1 s, RK4
    - Every cycle's `solve!` time includes schedule construction, parameter update, all NLP solves, knot validation, schedule-consistency test and the event-aware reference check.
    - Plants: event-aware (`integrate_on_track`) on multi-segment tracks; the Milestone 2A plant on single-segment tracks.

    """)

    # Warm-up (compilation) on every track type.
    for tr in (straight_track(200.0; width=0.7), circular_track(1.0; width=0.7), quarter_track())
        c = NMPCController(VEH, tr, NMPCConfig(schedule_corrections=8, correction_time_budget=100.0))
        simulate_closed_loop(c, VEH, [0.0, 0.0, 0.0, 0.5], 5)
    end

    # ---------------- Experiments A and B: single-segment regression
    rows = []
    for (name, tr, x0, n) in (("A: straight 150 m", straight_track(150.0; width=0.7), [0.0, 0.2, 0.1, 0.5], 100),
        ("B: circle R = 1 m", circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5], 70))
        c = NMPCController(VEH, tr, NMPCConfig())
        push!(rows, summarize(name, c, simulate_closed_loop(c, VEH, x0, n)))
    end
    t = """
    ## Experiments A and B: single-segment regression (baseline configuration)

    | experiment | status | cycles | statuses | NLP solves/cycle | s_end [m] | median / p99 / max [ms] | mean reference-check time [ms] | plant violations |
    |:--|:--|--:|:--|:--|--:|:-:|--:|--:|
    """
    for r in rows
        t *= @sprintf("| %s | %s | %d | %s | %s | %.4f | %.1f / %.1f / %.1f | %.2f | %d |\n", r.name, r.status, r.cycles, r.statuses,
            r.solves, r.s_end, r.median, r.p99, r.max, r.t_ref, r.plant_viol)
    end
    emit(t * "\nThe trajectories are compared bitwise with Milestone 2A separately (see docs/multisegment_nmpc.md).\n\n")

    # ---------------- Experiment C: single transition, cold and warm, crossing position, speed
    t = """
    ## Experiment C: straight → arc (κ 0 → 0.5), one control cycle

    The state is placed so that the reference crosses the boundary at a fraction φ of the first interval (`s0 = 3 − φ·v·dt`, `ey = epsi = 0`). *Warm*: after an accepted solve one period earlier, the measured state is that solve's predicted next state (so φ is only approximate in warm rows). Configurations: **default** (1 solve, reject stale) and **corrections** (≤ 8 corrections, no time limit; this shows how many solves self-consistency needs).

    | v [m/s] | φ | init | config | status | NLP solves | schedule it. | stage / cum. residual [rad] | reference: pos / heading / ey error | reference passed |
    |:-:|:-:|:--|:--|:--|--:|--:|:-:|:-:|:-:|
    """
    for v in (1.0, 2.0, 2.8), phi in (0.1, 0.5, 0.9), (cname, cfg) in (("default", NMPCConfig()),
        ("corrections", NMPCConfig(schedule_corrections=8, correction_time_budget=100.0)))
        x = [3.0 - phi * v * 0.1, 0.0, 0.0, v]
        for init in (:cold, :warm)
            c = NMPCController(VEH, TRANS, cfg)
            xm, uprev = x, (0.0, 0.0)
            if init === :warm
                # A preparatory accepted solve one period earlier; the measured state is its
                # predicted next state (as in closed loop with an accurate plant).
                c2 = NMPCController(VEH, TRANS, NMPCConfig(schedule_corrections=8, correction_time_budget=100.0))
                r0 = solve!(c2, [x[1] - v * 0.1, 0.0, 0.0, v])
                r0.accepted || (t *= "| $v | $phi | warm | $cname | (preparatory solve failed: $(r0.status)) |||||\n"; continue)
                c.last_X, c.last_U = r0.X, r0.U
                xm, uprev = r0.X[:, 2], r0.u
            end
            r = solve!(c, xm; previous_input=uprev)
            t *= @sprintf("| %.1f | %.1f | %s | %s | %s | %d | %d | %.1e / %.1e | %.1e / %.1e / %.1e | %s |\n", v, phi, init, cname, r.status,
                r.nlp_solves, r.schedule.iterations, r.schedule.stage_residual, r.schedule.cumulative_residual,
                r.reference.max_position_error, r.reference.max_heading_error, r.reference.max_ey_error, r.reference.passed)
        end
    end
    emit(t * "\n")

    # ---------------- Experiment D: quarter track closed loop
    q = quarter_track()
    configs = [
        ("D1 baseline: 1 solve, reject stale, margin 0.05", NMPCConfig()),
        ("D2 reject stale, ≤ 8 corrections in 80 ms, margin 0.05", NMPCConfig(schedule_corrections=8)),
        ("D3 qualified, ≤ 8 corrections in 80 ms, margin 0.05", NMPCConfig(schedule_corrections=8, stale_schedule_policy=:accept_if_reference_valid)),
        ("D4 qualified, ≤ 8 corrections in 80 ms, margin 0.10 (warm)", NMPCConfig(track_margin=0.10, schedule_corrections=8, stale_schedule_policy=:accept_if_reference_valid)),
        ("D5 = D4 with cold start every cycle", NMPCConfig(track_margin=0.10, schedule_corrections=8, stale_schedule_policy=:accept_if_reference_valid, warm_start=false)),
    ]
    drows, cls = [], Dict{String,Any}()
    repeats = Dict{String,String}()
    for (name, cfg) in configs
        outcomes = String[]
        for rep in 1:3                                   # repeated: some outcomes depend on wall-clock time
            c = NMPCController(VEH, q, cfg)
            cl = simulate_closed_loop(c, VEH, [0.0, 0.0, 0.0, 0.5], N_CYCLES)
            row = summarize(name, c, cl)
            push!(outcomes, row.status === :success ? @sprintf("%.2f laps", row.laps) :
                            @sprintf("stopped at cycle %d (%.2f laps)", row.cycles, row.laps))
            if rep == 1
                push!(drows, row)
                cls[name] = (c, cl)
            end
        end
        repeats[name] = join(outcomes, "; ")
    end
    t = """
    ## Experiment D: `quarter_track()` closed loop, start `[0, 0, 0, 0.5]`, up to $(N_CYCLES) cycles

    Each configuration was run three times; the detailed tables below are from run 1.

    | configuration | run 1 outcome | cycles | laps | statuses | NLP solves/cycle | outcomes of runs 1–3 |
    |:--|:--|--:|--:|:--|:--|:--|
    """
    for r in drows
        t *= @sprintf("| %s | %s | %d | %.2f | %s | %s | %s |\n", r.name, r.status === :success ? "completed all cycles" :
            "stopped: $(r.status)", r.cycles, r.laps, r.statuses, r.solves, repeats[r.name])
    end
    t *= "\nStop reasons:\n\n" * join(["- **$(r.name)**: " * (isempty(r.message) ? "none" : r.message) for r in drows], "\n") * "\n\n"
    t *= """
    ### Constraints, schedule and prediction accuracy (accepted cycles)

    | configuration | plant violations | max plant \\|ey\\| [m] | crossings | stale cycles | schedule non-converged | max schedule it. | max stage / cum. residual [rad] | reference pos / heading / s / ey error | max one-step plant mismatch [m] |
    |:--|--:|--:|--:|--:|--:|--:|:-:|:-:|--:|
    """
    for r in drows
        t *= @sprintf("| %s | %d | %.3f | %d | %d | %d | %d | %.1e / %.1e | %.1e / %.1e / %.1e / %.1e | %.1e |\n", r.name, r.plant_viol, r.plant_ey,
            r.crossings, r.stale, r.sched_nonconv, r.sched_it_max, r.acc_stage_res, r.acc_cum_res, r.acc_pos, r.acc_head, r.acc_s, r.acc_ey, r.onestep)
    end
    t *= """

    ### Computation time per control cycle [ms] (all cycles, including the first)

    | configuration | median | p95 | p99 | max | > 80 ms | > 100 ms | mean schedule | mean update + guess | mean optimize (wall) | mean Ipopt | mean knot validation | mean reference check | solver / scheduling / reference failures |
    |:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:-:|
    """
    for r in drows
        t *= @sprintf("| %s | %.1f | %.1f | %.1f | %.1f | %d | %d | %.2f | %.2f | %.1f | %.1f | %.2f | %.2f | %d / %d / %d |\n", r.name, r.median, r.p95, r.p99,
            r.max, r.over80, r.over100, r.t_sched, r.t_update, r.t_opt, r.t_ipopt, r.t_val, r.t_ref, r.solver_fail, r.sched_fail, r.ref_fail)
    end
    emit(t * "\n")

    out = joinpath(@__DIR__, "results", "multisegment_nmpc.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return cls
end

cls = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    name = "D3 qualified, ≤ 8 corrections in 80 ms, margin 0.05"
    ctrl, cl = cls[name]
    tr = ctrl.track
    sg = range(0, track_length(tr); length=800)
    edge(ey) = [frenet_to_cartesian(tr, s, ey)[1:2] for s in sg]
    p1 = plot(first.(edge(0.0)), last.(edge(0.0)); ls=:dot, lc=:gray, label="centreline", aspect_ratio=:equal,
        xlabel="X [m]", ylabel="Y [m]", title="quarter_track(), configuration D3", legend=:bottomright)
    for (ey, lab, st) in ((tr.width / 2, "track edge", :solid), (-tr.width / 2, "", :solid), (ctrl.ey_max, "NMPC bound", :dash), (-ctrl.ey_max, "", :dash))
        plot!(p1, first.(edge(ey)), last.(edge(ey)); lc=(st === :solid ? :black : :red), ls=st, label=lab)
    end
    plot!(p1, [q[1] for q in cl.poses], [q[2] for q in cl.poses]; lc=:blue, lw=1.5, label="rear axle")
    F = reduce(hcat, cl.fine_states)
    p2 = plot(cl.fine_t, F[2, :]; ylabel="ey [m]", label="plant")
    hline!(p2, [-tr.width / 2, tr.width / 2]; lc=:black, label="edge")
    hline!(p2, [-ctrl.ey_max, ctrl.ey_max]; lc=:red, ls=:dash, label="bound")
    vline!(p2, [c.t for c in cl.crossings]; lc=:gray, ls=:dot, label="", alpha=0.4)
    p3 = plot(cl.fine_t, F[4, :]; ylabel="v [m/s]", label="")
    rs = cl.controller
    tc = cl.t[1:length(rs)]
    p4 = scatter(tc, [ms(r.timing.total) for r in rs]; ms=2, ylabel="solve! [ms]", xlabel="t [s]",
        group=[string(r.status) for r in rs], legend=:topright)
    hline!(p4, [80, 100]; lc=[:orange :red], ls=:dash, label="")
    savefig(plot(p1, plot(p2, p3, p4; layout=(3, 1)); layout=(1, 2), size=(1400, 750), left_margin=5Plots.mm, bottom_margin=5Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "multiseg_nmpc_quarter.png"))
    println("saved figure")
end
