# Milestone 3B: stationary-obstacle avoidance on the multi-curvature quarter track.
#
#     julia --project=. benchmarks/multicurvature_obstacle.jl          # tables
#     julia --project=examples benchmarks/multicurvature_obstacle.jl   # tables + figures
#
# Writes benchmarks/results/multicurvature_obstacle.md (and
# docs/figures/multicurvature_obstacle_*.png). Every scenario is run REPS times,
# interleaved, after a common warm-up; all cycles count. Controllers are primed with one
# solve at the start state, then reset (set_obstacle! forgets the side), which builds the
# NLP evaluator.

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

"The primary configuration of the milestone, every option explicit."
cfg(N; kw...) = NMPCConfig(; prediction_model=:cartesian_endpoint_exact, horizon=N, dt=0.1, track_margin=0.05, schedule_corrections=8,
    interior_samples=0, strict_corridor=false, ego_radius=EGO, obstacle_buffer=BUF, obstacle_samples=3, kw...)

const SCEN = [
    ("A no obstacle", nothing),
    ("B R 1 m left bend, s = 3.5", (3.5, 0.0, ROB)),
    ("C R 1 m right bend (S-bend), s = 6.6", (6.6, 0.0, ROB)),
    ("D R 2 m curve, s = 9.7", (9.7, 0.0, ROB)),
    ("E1 before straight→R 1 m boundary, s = 1.95", (1.95, 0.0, ROB)),
    ("E2 after straight→R 1 m boundary, s = 2.10", (2.10, 0.0, ROB)),
    ("E3 before R −1→R 2 boundary, s = 8.20", (8.20, 0.0, ROB)),
    ("E4 after R −1→R 2 boundary, s = 8.40", (8.40, 0.0, ROB)),
    ("F lap boundary, s = L − 0.3", (LQ - 0.3, 0.0, ROB)),
    ("G1 offset +0.08 on R 1 m left bend", (3.5, 0.08, ROB)),
    ("G2 offset −0.08 on R 1 m right bend", (6.6, -0.08, ROB)),
    ("G3 blocked: radius 0.30 at s = 3.5", (3.5, 0.0, 0.30)),
]

mkob(spec) = spec === nothing ? nothing : obstacle_on_track(Q, spec[1], spec[2]; radius=spec[3])

function primed(c::NMPCConfig, ob)
    ctrl = ob === nothing ? NMPCController(VEH, Q, c) : NMPCController(VEH, Q, c; obstacle=ob)
    t = @elapsed solve!(ctrl, X0)
    reset_warm_start!(ctrl)
    ob === nothing || set_obstacle!(ctrl, ob)
    return ctrl, t
end

"simulate_closed_loop (event-aware plant, 10 substeps) with per-cycle @timed statistics."
function closed_loop(ctrl, n)
    h = ctrl.config.dt / 10
    x, uprev = SVector{4,Float64}(X0...), SVector(0.0, 0.0)
    states, results, inputs, alloc, gct, fine, tf = [x], NMPCResult[], SVector{2,Float64}[], Float64[], Float64[], [x], [0.0]
    status, msg = :success, ""
    for k in 1:n
        tm = @timed solve!(ctrl, x; previous_input=uprev)
        r = tm.value
        push!(results, r); push!(alloc, tm.bytes / 1e6); push!(gct, ms(tm.gctime))
        if !r.accepted
            status, msg = :controller_failure, "control period $k (t = $(round((k - 1) * 0.1; digits=1)) s): $(r.status) [cause $(r.obstacle.failure_cause)]: $(r.message)"
            break
        end
        for j in 1:10
            st = integrate_on_track(VEH, Q, x, r.u, h; max_step=h)
            succeeded(st) || error("plant: $(st.message)")
            x = st.x
            push!(fine, x); push!(tf, (k - 1) * 0.1 + j * h)
        end
        push!(states, x); push!(inputs, r.u)
        uprev = r.u
    end
    return (; states, results, inputs, alloc, gct, fine, tf, status, msg)
end

function runstats(ctrl, cl, prime, ob)
    rs = cl.results
    acc = filter(r -> r.accepted, rs)
    tot = [ms(r.timing.total) for r in rs]
    p = VEH.params
    poses = [frenet_to_cartesian(Q, x[1], x[2], x[3]) for x in cl.fine]
    dmin = ob === nothing ? NaN : minimum(hypot(q[1] - ob.x, q[2] - ob.y) for q in poses)
    D, Dc = ob === nothing ? (NaN, NaN) : (EGO + ob.radius + BUF, EGO + ob.radius)
    enc = [k for (k, r) in enumerate(rs) if length(r.obstacle.attempts) >= 2 || any(a -> a[1] in (:left, :right), r.obstacle.attempts)]
    ay = [abs(lateral_acceleration(VEH, x[4], u[1])) for (x, u) in zip(cl.states, cl.inputs)]
    return (; status=cl.status, cycles=length(rs), laps=(cl.states[end][1] - cl.states[1][1]) / LQ, accepted=length(acc), dmin, D, Dc,
        plant_track=Q.width / 2 - maximum(abs(x[2]) for x in cl.fine), plant_corr=ctrl.ey_max - maximum(abs(x[2]) for x in cl.fine),
        maxey=maximum(abs(x[2]) for x in cl.fine), plant_viol=count(x -> !isempty(check_operating_constraints(VEH, Q, x; tol=TOL)), cl.fine),
        input_viol=count(u -> !within_input_limits(p, u), cl.inputs), ay_viol=count(>(ctrl.config.ay_max + TOL), ay), maxay=isempty(ay) ? NaN : maximum(ay),
        pred_clear=nanmin(r.obstacle.min_clearance for r in acc), pred_phys=nanmin(r.obstacle.physical_clearance for r in acc),
        pred_track=nanmin(r.reference.min_boundary_distance for r in acc), buffer_cycles=count(r -> r.obstacle.separation_violations > 0, acc),
        dyn=isempty(acc) ? NaN : maximum(r.max_dynamics_residual for r in acc), knotm=nanmin(r.obstacle.knot_min_clearance for r in acc),
        segchg=sum((r.schedule.configuration_changes > 0 for r in acc); init=0),
        sides=[(k, rs[k].obstacle.side) for k in enc], enc, attempts=sum(length(r.obstacle.attempts) for r in rs),
        multi_assoc=count(r -> r.obstacle.associations > 1, rs), tot, solves=[r.nlp_solves for r in rs],
        reassign=[max(r.nlp_solves - max(count(a -> a[3] > 0, r.obstacle.attempts), r.nlp_solves > 0 ? 1 : 0), 0) for r in rs],
        iters=[r.iterations for r in rs],
        enc_tot=[tot[k] for k in enc], ord_tot=[tot[k] for k in eachindex(tot) if !(k in enc)], prime=ms(prime),
        causes=[r.obstacle.failure_cause for r in rs if !r.accepted],
        slow=[(k, tot[k], ms(rs[k].timing.solver), ms(rs[k].timing.optimize), cl.gct[k], cl.alloc[k], rs[k].nlp_solves, length(rs[k].obstacle.attempts))
              for k in eachindex(tot) if tot[k] > 80], stop=cl.msg, cl)
end

fmtlaps(r) = r.status === :success ? @sprintf("%.2f", r.laps) : @sprintf("stop c%d (%.2f)", r.cycles, r.laps)
sidestr(r) = isempty(r.sides) ? "–" : join(["c$(k):$(s)" for (k, s) in r.sides], " ")

function table(names, results; timing=true)
    t1 = """
    | scenario | laps per run | accepted cycles | plant min distance [m] | plant clearance above contact / above D [m] | plant track clearance / corridor clearance [m] | max |ey| [m] | predicted (dense) clearance above contact / above D [m] | accepted plans in the buffer | encounters: cycle:side (run 1) | failure causes |
    |:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:--|:--|
    """
    t2 = """
    | scenario | NLP solves mean / max | reassignment solves per cycle | initialisation attempts | Ipopt iter. mean / max | median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles: n, median / max [ms] | ordinary cycles: median / max [ms] | plant: input / lateral-acc. violations, max |ay| | max dyn. residual / min knot clearance above D |
    |:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
    """
    for name in names
        haskey(results, name) || continue
        runs = results[name]
        r1 = runs[1]
        causes = unique(reduce(vcat, [r.causes for r in runs]))
        t1 *= @sprintf("| %s | %s | %d / %d | %.4f | %.4f / %.4f | %.4f / %.4f | %.3f | %.4f / %.4f | %d | %s | %s |\n", name,
            join(fmtlaps.(runs), "; "), sum(r.accepted for r in runs), sum(r.cycles for r in runs), minimum(r.dmin for r in runs),
            minimum(r.dmin for r in runs) - r1.Dc, minimum(r.dmin for r in runs) - r1.D, minimum(r.plant_track for r in runs),
            minimum(r.plant_corr for r in runs), maximum(r.maxey for r in runs), nanmin(r.pred_phys for r in runs),
            nanmin(r.pred_clear for r in runs), sum(r.buffer_cycles for r in runs), sidestr(r1), isempty(causes) ? "–" : join(string.(causes), ", "))
        timing || continue
        tot, sol, it = reduce(vcat, [r.tot for r in runs]), reduce(vcat, [r.solves for r in runs]), reduce(vcat, [r.iters for r in runs])
        re, et, ot = reduce(vcat, [r.reassign for r in runs]), reduce(vcat, [r.enc_tot for r in runs]), reduce(vcat, [r.ord_tot for r in runs])
        t2 *= @sprintf("| %s | %.2f / %d | %.2f | %d | %.1f / %d | %.1f / %.1f / %.1f / %.1f | %d / %d | %d, %s | %.1f / %.1f | %d / %d, %.2f | %.1e / %.4f |\n", name,
            sum(sol) / length(sol), maximum(sol), sum(re) / length(re), sum(r.attempts for r in runs), sum(it) / length(it), maximum(it),
            quant(tot, 0.5), quant(tot, 0.95), quant(tot, 0.99), maximum(tot), count(>(80), tot), count(>(100), tot), length(et),
            isempty(et) ? "–" : @sprintf("%.1f / %.1f", quant(et, 0.5), maximum(et)), quant(ot, 0.5), isempty(ot) ? NaN : maximum(ot),
            sum(r.input_viol for r in runs), sum(r.ay_viol for r in runs), maximum(r.maxay for r in runs),
            maximum(filter(!isnan, [r.dyn for r in runs]); init=NaN), nanmin(r.knotm for r in runs))
    end
    return t1, t2
end

function run_set(names_specs, mkcfg, reps, n)
    results, keep = Dict{String,Vector{Any}}(), Dict{String,Any}()
    for rep in 1:reps, (name, spec) in (isodd(rep) ? names_specs : reverse(names_specs))
        ob = mkob(spec)
        ctrl, tp = primed(mkcfg(), ob)
        cl = closed_loop(ctrl, n)
        push!(get!(results, name, Any[]), runstats(ctrl, cl, tp, ob))
        rep == 1 && (keep[name] = (ctrl, cl, ob))
    end
    return results, keep
end

slowlist(names, results) = (s = [(n, i, x) for n in names if haskey(results, n) for (i, r) in enumerate(results[n]) for x in r.slow];
    isempty(s) ? "" : "Cycles above 80 ms (scenario, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, attempts):\n\n" *
    join([@sprintf("- %s, run %d, cycle %d: %.1f / %.1f / %.1f ms, GC %.1f ms, %.1f MB, %d solve(s), %d attempt(s)", n, i, x...) for (n, i, x) in s], "\n") * "\n\n")

stops(names, results) = (s = ["- **$(n)** (run 1): " * first(results[n][1].stop, 450) for n in names if haskey(results, n) && !isempty(results[n][1].stop)];
    isempty(s) ? "" : "Stops:\n\n" * join(s, "\n") * "\n\n")

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Multi-curvature stationary-obstacle avoidance benchmark (Milestone 3B)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; unchanged `quarter_track()` (L = $(round(LQ; digits=3)) m, width 0.7 m); start `$(X0)`
    - Primary configuration (all options explicit): Cartesian-endpoint, N = 10, dt = 0.1 s, track_margin = 0.05, schedule_corrections = 8 (80 ms budget), interior_samples = 0, strict_corridor = false, ego_radius = $(EGO), obstacle_buffer = $(BUF), obstacle_samples = 3, obstacle_acceptance = :collision; objective, weights, limits and Ipopt options unchanged
    - Obstacles are placed with `obstacle_on_track(quarter_track(), s, ey; radius)` and stay fixed in Cartesian space; radius $(ROB) m (D = 0.18 m, contact at 0.15 m) unless stated
    - $(N_CYCLES) control cycles per run, $(REPS) interleaved runs; event-aware plant (10 substeps); plant distances at every substep; dense collision evaluation of every candidate at 50 samples per interval
    - "Encounter cycles" are cycles with a passing-side initialisation (two candidate solves at a new encounter, or a warm-start retry); timing covers the whole `solve!` cycle

    """)
    tb = @elapsed (c = NMPCController(VEH, Q, cfg(10); obstacle=mkob(SCEN[2][2])))
    ts = @elapsed solve!(c, X0)
    emit(@sprintf("First construction / first solve with an obstacle in this process (compilation included): %.2f s / %.2f s.\n\n", tb, ts))
    for (_, spec) in SCEN[1:3]
        ctrl, _ = primed(cfg(10), mkob(spec))
        closed_loop(ctrl, 15)
    end

    names = first.(SCEN)
    res, keep = run_set(SCEN, () -> cfg(10), REPS, N_CYCLES)
    t1, t2 = table(names, res)
    emit("## Primary: N = 10, obstacle_acceptance = :collision\n\n" * t1 * "\n" * t2 * "\n" * stops(names, res) * slowlist(names, res))

    # Without the correction time budget (one run): separates the budget from the algorithm.
    nob = [s for s in SCEN if haskey(res, s[1]) && any(r -> r.status !== :success, res[s[1]])]
    if !isempty(nob)
        rb, _ = run_set(nob, () -> cfg(10; correction_time_budget=1e3), 1, N_CYCLES)
        t1, t2 = table(first.(nob), rb)
        emit("## Failed primary scenarios rerun without the correction time budget (one run, budget 1000 s)\n\n" * t1 * "\n" * t2 * "\n" *
             stops(first.(nob), rb))
    end

    # Strict buffer: reject any intrusion into D.
    strict = SCEN[2:4]
    rs, _ = run_set(strict, () -> cfg(10; obstacle_acceptance=:separation), REPS, N_CYCLES)
    t1, _ = table(first.(strict), rs; timing=false)
    emit("## Strict-buffer experiment: N = 10, obstacle_acceptance = :separation\n\n" * t1 * "\n" * stops(first.(strict), rs))

    # Secondary: N = 20 (one run each).
    sec = SCEN[[1, 2, 3, 4, 9]]
    r20, _ = run_set(sec, () -> cfg(20), 1, N_CYCLES)
    t1, t2 = table(first.(sec), r20)
    emit("## Secondary: N = 20 (one run each)\n\n" * t1 * "\n" * t2 * "\n" * stops(first.(sec), r20) * slowlist(first.(sec), r20))

    # Dynamically infeasible: an obstacle 0.9 m ahead at 5 m/s on the 6 m straight (one cycle).
    t = """
    ## Single-cycle feasibility cases

    | case | status | failure cause | attempts | guesses (side, knot clearance above D, track clearance) [m] | passage left / right |
    |:--|:--|:--|:--|:--|:-:|
    """
    for (label, spec, x) in (("dynamically infeasible: obstacle (s 25.9, ey 0) at 0.9 m ahead, v = 5 m/s", (25.9, 0.0, ROB), [25.0, 0.0, 0.0, 5.0]),
        ("geometrically blocked: radius 0.30 at s = 3.5, from s = 2.4, v = 2 m/s", (3.5, 0.0, 0.30), [2.4, 0.0, 0.0, 2.0]),
        ("feasible reference: obstacle (s 27.9, ey 0) 2.9 m ahead, v = 5 m/s", (27.9, 0.0, ROB), [25.0, 0.0, 0.0, 5.0]))
        ctrl, _ = primed(cfg(10), mkob(spec))
        r = solve!(ctrl, x)
        t *= @sprintf("| %s | %s | %s | %s | %s | %s / %s |\n", label, r.status, r.obstacle.failure_cause,
            join(["$(a[1]) → $(a[2])" for a in r.obstacle.attempts], ", "),
            join([@sprintf("%s %.3f %.3f", g...) for g in r.obstacle.guesses], "; "), r.obstacle.passage_left, r.obstacle.passage_right)
    end
    emit(t * "\n")

    # Diagnosis of the E3 failure without the budget: replay the failing cycle with k corrections.
    spec = (8.20, 0.0, ROB)
    ob = mkob(spec)
    ctrl, _ = primed(cfg(10; correction_time_budget=1e3), ob)
    cl = closed_loop(ctrl, N_CYCLES)
    if cl.status !== :success
        x, up = cl.states[end], cl.inputs[end]
        bnd = [lap * LQ + Q.s_start[i] for lap in 0:3 for i in 1:num_segments(Q)]
        t = "## Diagnosis: E3 (s = 8.20) without the budget, failing cycle $(length(cl.results)) replayed with k reassignments

" *
            "State `$(round.(collect(x); digits=4))`. Left-side guess, then k reassignment solves. The knot outside its assigned segment alternates:

" *
            "| k | status | NLP solves | optimised knot(s) outside the assigned segment [m] | nearest segment boundary [m] |
|--:|:--|--:|:--|:-:|
"
        for k in 0:8
            cc = NMPCController(VEH, Q, cfg(10; correction_time_budget=1e3, schedule_corrections=k); obstacle=ob)
            rr = solve_once!(cc, x; previous_input=up, initial=side_guess(cc, x, :left))
            outk = [rr.X[1, j] for j in 2:11 if !VehicleMPC.frame_valid(Q, segment_frame(Q, segment_id(Q, rr.schedule.s_nominal[j], :right)), rr.X[1, j], 1e-6)]
            b = isempty(outk) ? NaN : bnd[argmin(abs.(bnd .- outk[1]))]
            t *= @sprintf("| %d | %s | %d | %s | %.3f |
", k, rr.status, rr.nlp_solves, join([@sprintf("%.3f", v) for v in outk], ", "), b)
        end
        emit(t * "
")
    end

    out = joinpath(@__DIR__, "results", "multicurvature_obstacle.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return keep
end

keep = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    edge(ey) = (P = [frenet_to_cartesian(Q, s, ey) for s in range(0, LQ; length=2000)]; ([p[1] for p in P], [p[2] for p in P]))
    xy(states) = (P = [frenet_to_cartesian(Q, x[1], x[2], x[3]) for x in states]; ([p[1] for p in P], [p[2] for p in P]))
    circle(cx, cy, r) = (θ = range(0, 2π; length=200); (cx .+ r .* cos.(θ), cy .+ r .* sin.(θ)))
    _, cl0, _ = keep["A no obstacle"]
    function trackplot(; kw...)
        p = plot(; aspect_ratio=:equal, xlabel="X [m]", ylabel="Y [m]", legend=:outertopright, kw...)
        for (ey, lc, ls, lab) in ((0.35, :black, :solid, "physical track edge"), (-0.35, :black, :solid, ""), (0.30, :gray, :dash, "planning corridor"),
            (-0.30, :gray, :dash, ""), (0.0, :gray, :dot, "centreline"))
            plot!(p, edge(ey)...; lc, ls, lw=ls === :solid ? 1.5 : 1, label=lab)
        end
        return p
    end
    function scene!(p, name; lap1=true, pred=true)
        ctrl, cl, ob = keep[name]
        D = EGO + ob.radius + BUF
        plot!(p, circle(ob.x, ob.y, D)...; seriestype=:shape, fillcolor=:orange, fillalpha=0.3, lc=:orange, label="required separation D = $(round(D; digits=2)) m")
        plot!(p, circle(ob.x, ob.y, ob.radius)...; seriestype=:shape, fillcolor=:red, lc=:red, label="obstacle")
        n1 = lap1 ? findfirst(x -> x[1] > LQ, cl.fine) : nothing
        fs = n1 === nothing ? cl.fine : cl.fine[1:n1]
        plot!(p, xy(fs)...; lc=:blue, lw=2, label="actual ego trajectory (plant, lap 1)")
        if pred
            k = findfirst(r -> any(a -> a[1] in (:left, :right), r.obstacle.attempts), cl.results)
            if k !== nothing
                plot!(p, xy([cl.results[k].X[:, j] for j in axes(cl.results[k].X, 2)])...; lc=:purple, ls=:dash, lw=1.5, marker=:circle, ms=2,
                    label="predicted plan at the encounter (cycle $k)")
            end
        end
        return p
    end
    p1 = trackplot(; title="Quarter track: obstacle on the R = 1 m left bend (s = 3.5), N = 10")
    plot!(p1, xy(cl0.fine[1:something(findfirst(x -> x[1] > LQ, cl0.fine), length(cl0.fine))])...; lc=:gray, lw=1, ls=:dashdot, label="no-obstacle reference (lap 1)")
    scene!(p1, "B R 1 m left bend, s = 3.5")
    _, _, obB = keep["B R 1 m left bend, s = 3.5"]
    _, _, obC = keep["C R 1 m right bend (S-bend), s = 6.6"]
    p2 = trackplot(; title="Zoom: R = 1 m left bend (B)", xlims=(obB.x - 0.8, obB.x + 0.8), ylims=(obB.y - 0.8, obB.y + 0.8), legend=false)
    plot!(p2, xy(cl0.fine)...; lc=:gray, lw=1, ls=:dashdot, label="")
    scene!(p2, "B R 1 m left bend, s = 3.5")
    p3 = trackplot(; title="Zoom: R = 1 m right bend of the S-bend (C)", xlims=(obC.x - 0.8, obC.x + 0.8), ylims=(obC.y - 0.8, obC.y + 0.8), legend=false)
    plot!(p3, xy(cl0.fine)...; lc=:gray, lw=1, ls=:dashdot, label="")
    scene!(p3, "C R 1 m right bend (S-bend), s = 6.6")
    savefig(plot(p1, plot(p2, p3; layout=(1, 2)); layout=(2, 1), size=(1500, 1500), left_margin=6Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "multicurvature_obstacle_trajectory.png"))
    # Signals for B and C (lap 1).
    sig = map(["B R 1 m left bend, s = 3.5", "C R 1 m right bend (S-bend), s = 6.6"]) do name
        _, cl, ob = keep[name]
        D, Dc = EGO + ob.radius + BUF, EGO + ob.radius
        n = length(cl.inputs)                             # the whole run (all encounters)
        t = 0.1 .* (0:n-1)
        d = [hypot(q[1] - ob.x, q[2] - ob.y) for q in (frenet_to_cartesian(Q, x[1], x[2], x[3]) for x in cl.fine[1:10n+1])]
        q1 = plot(t, [rad2deg(u[1]) for u in cl.inputs[1:n]]; seriestype=:steppost, ylabel="steering [deg]", label="", title=first(name, 40))
        q2 = plot(0.1 .* (0:n), [x[4] for x in cl.states[1:n+1]]; ylabel="speed [m/s]", label="")
        q3 = plot(cl.tf[1:10n+1], d; ylabel="distance to obstacle [m]", xlabel="t [s]", label="centre distance (plant)", ylims=(0, 1.0))
        hline!(q3, [D]; lc=:orange, ls=:dash, label="D = $(round(D; digits=2)) m (with buffer)")
        hline!(q3, [Dc]; lc=:red, ls=:dash, label="contact ($(round(Dc; digits=2)) m)")
        plot(q1, q2, q3; layout=(3, 1))
    end
    savefig(plot(sig...; layout=(1, 2), size=(1500, 900), left_margin=6Plots.mm), joinpath(@__DIR__, "..", "docs", "figures", "multicurvature_obstacle_signals.png"))
    println("saved figures")
end
