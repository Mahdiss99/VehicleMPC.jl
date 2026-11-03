# Milestone 3B.1: reliable segment reassignment and encounter computation.
#
#     julia --project=. benchmarks/reassignment_reliability.jl
#
# Writes benchmarks/results/reassignment_reliability.md. Same scenarios, vehicle and
# obstacle parameters as Milestone 3B (benchmarks/multicurvature_obstacle.jl). Four
# configurations:
#   A original reassignment + two-candidate encounters (the 3B controller)
#   B cycle detection + branch resolution, two-candidate encounters
#   C original reassignment, sequential encounters
#   D both improvements
# Phase 1: no restrictive budget (correction_time_budget = 1000 s, cycle_budget = Inf),
# one run each; the outcome then depends only on the computation.
# Phase 2: the 3B correction budget (80 ms) alone, REPS runs each.
# Phase 3: the 80 ms correction budget plus cycle_budget = 0.09 s, REPS runs each.
# Controllers are primed (one solve at the start state, then reset). All cycles count.

using VehicleMPC
using VehicleMPC.StaticArrays: SVector
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const N_CYCLES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 250
const REPS = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 3
const Q = quarter_track()
const LQ = track_length(Q)
const X0 = [0.0, 0.0, 0.0, 0.5]
const EGO, BUF, ROB = 0.05, 0.03, 0.10
const TOL = 1e-6

quant(xs, q) = isempty(xs) ? NaN : (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))
nanmin(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : minimum(w))
ms(x) = 1e3 * x

base(; kw...) = NMPCConfig(; prediction_model=:cartesian_endpoint_exact, horizon=10, dt=0.1, track_margin=0.05, schedule_corrections=8,
    interior_samples=0, strict_corridor=false, ego_radius=EGO, obstacle_buffer=BUF, obstacle_samples=3, kw...)
const VARIANTS = [
    ("A original", (;)),
    ("B cycle detection + branch", (reassignment_cycle_detection=true, branch_resolution=true)),
    ("C sequential encounters", (encounter_strategy=:sequential,)),
    ("D both", (reassignment_cycle_detection=true, branch_resolution=true, encounter_strategy=:sequential)),
]
const NOBUDGET = (correction_time_budget=1e3, cycle_budget=Inf)
const CORRONLY = (correction_time_budget=0.08, cycle_budget=Inf)
const BUDGET = (correction_time_budget=0.08, cycle_budget=0.09)

const SCEN = [
    ("A no obstacle", nothing), ("B R 1 m left, s = 3.5", (3.5, 0.0)), ("C R 1 m right, s = 6.6", (6.6, 0.0)),
    ("D R 2 m, s = 9.7", (9.7, 0.0)), ("E1 s = 1.95", (1.95, 0.0)), ("E2 s = 2.10", (2.10, 0.0)), ("E3 s = 8.20", (8.20, 0.0)),
    ("E4 s = 8.40", (8.40, 0.0)), ("F lap boundary", (LQ - 0.3, 0.0)), ("G1 +0.08 on R 1 m left", (3.5, 0.08)),
    ("G2 −0.08 on R 1 m right", (6.6, -0.08)),
]
mkob(spec) = spec === nothing ? nothing : obstacle_on_track(Q, spec[1], spec[2]; radius=ROB)

function primed(cfg, ob)
    c = ob === nothing ? NMPCController(VEH, Q, cfg) : NMPCController(VEH, Q, cfg; obstacle=ob)
    solve!(c, X0)
    reset_warm_start!(c)
    ob === nothing || set_obstacle!(c, ob)
    return c
end

"simulate_closed_loop (event-aware plant, 10 substeps) with per-cycle @timed statistics."
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
            status, msg = :controller_failure, "c$k: $(r.status) [cause $(r.obstacle.failure_cause)]: $(first(r.message, 260))"
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

function stats(ctrl, cl, ob)
    rs = cl.results
    acc = filter(r -> r.accepted, rs)
    tot = [ms(r.timing.total) for r in rs]
    poses = [frenet_to_cartesian(Q, x[1], x[2], x[3]) for x in cl.fine]
    dmin = ob === nothing ? NaN : minimum(hypot(q[1] - ob.x, q[2] - ob.y) for q in poses)
    enc = [k for (k, r) in enumerate(rs) if any(a -> a[1] in (:left, :right), r.obstacle.attempts)]
    sides = [rs[k].obstacle.side for k in enc]
    sideseq = [r.obstacle.side for r in rs if r.obstacle.side !== :none]
    switches = count(i -> sideseq[i] != sideseq[i-1] && !(i in 1:1), 2:length(sideseq))
    attempt_solves = [a[3] for r in rs for a in r.obstacle.attempts if a[3] > 0]
    return (; status=cl.status, cycles=length(rs), laps=(cl.states[end][1] - cl.states[1][1]) / LQ, accepted=length(acc), dmin,
        plant_track=Q.width / 2 - maximum(abs(x[2]) for x in cl.fine),
        plant_viol=count(x -> !isempty(check_operating_constraints(VEH, Q, x; tol=TOL)), cl.fine),
        cyc=count(r -> r.reassignment.cycles > 0, rs), br_ok=count(r -> r.reassignment.branch_selected > 0, rs),
        br_fail=count(r -> r.reassignment.cycles > 0 && r.reassignment.branch_selected == 0, rs),
        br_att=sum((length(r.reassignment.branch) for r in rs); init=0), maxatt=isempty(attempt_solves) ? 0 : maximum(attempt_solves),
        solves=[r.nlp_solves for r in rs], enc=length(enc), enc_att=isempty(enc) ? NaN : sum(count(a -> a[3] > 0, rs[k].obstacle.attempts) for k in enc) / length(enc),
        switches, firstenc=isempty(enc) ? (:none, NaN) : (rs[enc[1]].obstacle.side, rs[enc[1]].objective),
        tot, enc_tot=[tot[k] for k in enc], ord_tot=[tot[k] for k in eachindex(tot) if !(k in enc)],
        solver=sum(ms(r.timing.solver) for r in rs if !isnan(r.timing.solver); init=0.0), gc=sum(cl.gct), totsum=sum(tot),
        alloc=sum(cl.alloc) / max(length(rs), 1), stop=cl.msg)
end

function run_phase(budget, reps, n)
    res = Dict{Tuple{String,String},Vector{Any}}()
    for rep in 1:reps, (sname, spec) in SCEN, (vname, vkw) in (isodd(rep) ? VARIANTS : reverse(VARIANTS))
        ob = mkob(spec)
        ctrl = primed(base(; vkw..., budget...), ob)
        cl = closed_loop(ctrl, n)
        push!(get!(res, (sname, vname), Any[]), stats(ctrl, cl, ob))
    end
    return res
end

fmtrun(r) = r.status === :success ? @sprintf("%.2f", r.laps) : @sprintf("stop %s", first(r.stop, findfirst(':', r.stop) === nothing ? 6 : findfirst(':', r.stop) - 1))

function tables(res, reps)
    t1 = """
    | scenario | variant | laps / stop per run | accepted / cycles | cycles with reassignment cycle: resolved / not | branch solves | max solves in one attempt | NLP solves mean | encounters, solves per encounter | side switches | first encounter: side, objective | plant: min distance [m] (contact 0.15, D 0.18), track clearance [m], violations |
    |:--|:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    t2 = """
    | scenario | variant | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: median / max [ms] | ordinary median [ms] | Ipopt share of total time | GC share | mean allocation per cycle [MB] |
    |:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    for (sname, _) in SCEN, (vname, _) in VARIANTS
        runs = res[(sname, vname)]
        tot, sol = reduce(vcat, [r.tot for r in runs]), reduce(vcat, [r.solves for r in runs])
        et, ot = reduce(vcat, [r.enc_tot for r in runs]), reduce(vcat, [r.ord_tot for r in runs])
        r1 = runs[1]
        t1 *= @sprintf("| %s | %s | %s | %d / %d | %d / %d | %d | %d | %.2f | %d, %.2f | %d | %s %.4f | %.4f, %.4f, %d |\n", sname, vname,
            join(fmtrun.(runs), "; "), sum(r.accepted for r in runs), sum(r.cycles for r in runs), sum(r.br_ok for r in runs),
            sum(r.br_fail for r in runs), sum(r.br_att for r in runs), maximum(r.maxatt for r in runs), sum(sol) / length(sol),
            sum(r.enc for r in runs), nanmin([r.enc_att for r in runs]) , sum(r.switches for r in runs), r1.firstenc[1], r1.firstenc[2],
            minimum(r.dmin for r in runs), minimum(r.plant_track for r in runs), sum(r.plant_viol for r in runs))
        t2 *= @sprintf("| %s | %s | %.1f / %.1f / %.1f / %.1f | %d / %d | %s | %.1f | %.0f %% | %.1f %% | %.2f |\n", sname, vname,
            quant(tot, 0.5), quant(tot, 0.95), quant(tot, 0.99), maximum(tot), count(>(80), tot), count(>(100), tot),
            isempty(et) ? "–" : @sprintf("%.1f / %.1f", quant(et, 0.5), maximum(et)), quant(ot, 0.5),
            100 * sum(r.solver for r in runs) / sum(r.totsum for r in runs), 100 * sum(r.gc for r in runs) / sum(r.totsum for r in runs),
            sum(r.alloc for r in runs) / length(runs))
    end
    return t1, t2
end

function e3_replay(emit)
    ob = mkob((8.20, 0.0))
    ctrl = primed(base(; NOBUDGET...), ob)
    cl = closed_loop(ctrl, N_CYCLES)
    cl.status === :success && return emit("E3 did not fail with the original controller in this run; replay skipped.\n\n")
    k = length(cl.results)
    x = cl.states[end]
    up = k >= 2 ? SVector(cl.results[k-1].U[1, 1], cl.results[k-1].U[2, 1]) : SVector(0.0, 0.0)
    t = "## E3 replay: failing cycle $k of the original controller (no budget)\n\nState `$(round.(collect(x); digits=4))`; R −1 → R 2 boundary at s = $(round(Q.s_start[4] + LQ; digits=3)) m (lap 2). Left-side initial guess, `solve_once!` with 8 reassignments:\n\n"
    for (vname, vkw) in VARIANTS[[1, 2]]
        c = primed(base(; vkw..., NOBUDGET...), ob)
        r = solve_once!(c, x; previous_input=up, initial=side_guess(c, x, :left))
        t *= "**$(vname)**: status `$(r.status)`, $(r.nlp_solves) NLP solves. $(r.reassignment.message)\n\n" *
             "| solve | status | Ipopt it. | objective | dyn. residual | knot clearance above D [m] | solver [ms] | knots changed | knots outside: progress [m] | knot segment ids 2…11 |\n|--:|:--|--:|:-:|:-:|:-:|:-:|:-:|:-:|:--|\n"
        for e in r.reassignment.trace
            t *= @sprintf("| %s | %s | %d | %.4f | %.1e | %+.4f | %.1f | %s | %s | %s |\n", e.solve < 0 ? "branch $(e.branch)" : string(e.solve), e.status,
                e.iterations, e.objective, e.dyn_res, e.obstacle_clearance, ms(e.solver_time), string(e.changed),
                join([@sprintf("%d: %.3f", a, b) for (a, b) in zip(e.outside, e.outside_s)], ", "), join(string.(e.ids[2:end]), " "))
        end
        for b in r.reassignment.branch
            t *= @sprintf("\nbranch candidate: knot %d bounded to segment %d [%.3f, %.3f] → %s, objective %.4f, %.1f ms", b.knot, b.segment, b.s0, b.s1,
                b.status, b.objective, ms(b.seconds))
        end
        t *= "\n\n"
    end
    emit(t)
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Segment-reassignment reliability benchmark (Milestone 3B.1)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; unchanged `quarter_track()`; start `$(X0)`
    - Milestone 3B scenarios and parameters: N = 10, dt = 0.1 s, margin 0.05 m, ≤ 8 reassignments, obstacle radius $(ROB) m, ego radius $(EGO) m, buffer $(BUF) m (D = 0.18 m, contact 0.15 m), 3 interior obstacle samples, `obstacle_acceptance = :collision`
    - Variants: A original (3B); B `reassignment_cycle_detection` + `branch_resolution`; C `encounter_strategy = :sequential`; D both
    - Phase 1: no restrictive budget (correction 1000 s, cycle Inf), one run each. Phase 2: correction budget 80 ms only (the 3B setting), $(REPS) runs each. Phase 3: correction budget 80 ms and `cycle_budget = 0.09 s`, $(REPS) runs each
    - $(N_CYCLES) control cycles per run; event-aware plant; all cycles counted; timing = the whole `solve!` cycle

    """)
    for (_, vkw) in VARIANTS
        closed_loop(primed(base(; vkw..., NOBUDGET...), mkob((3.5, 0.0))), 15)
    end
    e3_replay(emit)
    r1 = run_phase(NOBUDGET, 1, N_CYCLES)
    t1, t2 = tables(r1, 1)
    emit("## Phase 1: no restrictive budget (one run)\n\n" * t1 * "\n" * t2 * "\n")
    st = ["- $(s) / $(v): $(r1[(s, v)][1].stop)" for (s, _) in SCEN, (v, _) in VARIANTS if !isempty(r1[(s, v)][1].stop)]
    isempty(st) || emit("Stops:\n\n" * join(st, "\n") * "\n\n")
    for (title, budget) in (("Phase 2: correction budget 80 ms only", CORRONLY), ("Phase 3: correction budget 80 ms + cycle budget 90 ms", BUDGET))
        r2 = run_phase(budget, REPS, N_CYCLES)
        t1, t2 = tables(r2, REPS)
        emit("## $(title) ($(REPS) runs)\n\n" * t1 * "\n" * t2 * "\n")
        st = ["- $(s) / $(v), run $(i): $(r.stop)" for (s, _) in SCEN, (v, _) in VARIANTS for (i, r) in enumerate(r2[(s, v)]) if !isempty(r.stop)]
        isempty(st) || emit("Stops:\n\n" * join(st, "\n") * "\n\n")
    end
    out = joinpath(@__DIR__, "results", "reassignment_reliability.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
end

main()
