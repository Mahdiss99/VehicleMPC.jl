# Milestone 3A: stationary-obstacle collision avoidance with the Cartesian-endpoint NMPC.
#
#     julia --project=. benchmarks/stationary_obstacle.jl          # tables
#     julia --project=examples benchmarks/stationary_obstacle.jl   # tables + figures
#
# Writes benchmarks/results/stationary_obstacle.md (and docs/figures/stationary_obstacle_*.png).
# Every configuration is run REPS times, interleaved, after a common warm-up; all cycles
# count. Controllers are primed with one solve at the start state (then reset; the
# obstacle side is forgotten with set_obstacle!), which builds the NLP evaluator.

using VehicleMPC
using VehicleMPC.StaticArrays: SVector
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const N_CYCLES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 100
const REPS = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 3
const TR = straight_track(80.0; width=0.7)
const X0 = [0.0, 0.0, 0.0, 0.5]
const EGO, BUF = 0.05, 0.03
const OBS = Dict(:centre => CircleObstacle(x=8.0, y=0.0, radius=0.10), :offset => CircleObstacle(x=8.0, y=0.08, radius=0.10),
    :blocked => CircleObstacle(x=8.0, y=0.0, radius=0.30))
const TOL = 1e-6

quant(xs, q) = isempty(xs) ? NaN : (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))
nanmin(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : minimum(w))
ms(x) = 1e3 * x

base(N) = (prediction_model=:cartesian_endpoint_exact, schedule_corrections=8, horizon=N, ego_radius=EGO, obstacle_buffer=BUF)
configs(N) = [("no obstacle (baseline)", NMPCConfig(; base(N)...), false),
    ("knots only", NMPCConfig(; base(N)..., obstacle_samples=0), true),
    ("knots + 1 interior sample", NMPCConfig(; base(N)..., obstacle_samples=1), true),
    ("knots + 3 interior samples", NMPCConfig(; base(N)..., obstacle_samples=3), true)]

function primed(cfg, ob)
    c = ob === nothing ? NMPCController(VEH, TR, cfg) : NMPCController(VEH, TR, cfg; obstacle=ob)
    t = @elapsed solve!(c, X0)
    reset_warm_start!(c)
    ob === nothing || set_obstacle!(c, ob)
    return c, t
end

"simulate_closed_loop (event-aware plant) with per-cycle @timed statistics."
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
            status, msg = :controller_failure, "control period $k (t = $(round((k - 1) * ctrl.config.dt; digits=1)) s): $(r.status): $(r.message)"
            break
        end
        for j in 1:10
            st = integrate_on_track(VEH, TR, x, r.u, h; max_step=h)
            succeeded(st) || error("plant: $(st.message)")
            x = st.x
            push!(fine, x); push!(tf, (k - 1) * 0.1 + j * h)
        end
        push!(states, x); push!(inputs, r.u)
        uprev = r.u
    end
    return (; states, results, inputs, alloc, gct, fine, tf, status, msg)
end

function runstats(c, cl, prime, ob)
    rs = cl.results
    acc = filter(r -> r.accepted, rs)
    tot = [ms(r.timing.total) for r in rs]
    poses = [frenet_to_cartesian(TR, x[1], x[2], x[3]) for x in cl.fine]
    D, Dc = ob === nothing ? (NaN, NaN) : (EGO + ob.radius + BUF, EGO + ob.radius)
    dmin = ob === nothing ? NaN : minimum(hypot(p[1] - ob.x, p[2] - ob.y) for p in poses)
    two = [k for (k, r) in enumerate(rs) if length(r.obstacle.attempts) >= 2]
    fails = Dict(r.obstacle.failure_cause => 1 for r in rs if !r.accepted)
    return (; status=cl.status, cycles=length(rs), s_end=cl.states[end][1], accepted=length(acc), dmin, D, Dc,
        plant_track=TR.width / 2 - maximum(abs(x[2]) for x in cl.fine),
        plant_viol=count(x -> !isempty(check_operating_constraints(VEH, TR, x; tol=TOL)), cl.fine),
        pred_clear=nanmin(r.obstacle.min_clearance for r in acc), pred_phys=nanmin(r.obstacle.physical_clearance for r in acc),
        knot_clear=nanmin(r.obstacle.knot_min_clearance for r in acc),
        buffer_cycles=count(r -> r.obstacle.separation_violations > 0, acc), between=count(r -> r.obstacle.violation_between_knots, acc),
        pred_track=nanmin(r.reference.min_boundary_distance for r in acc),
        sides=unique(r.obstacle.side for r in rs if r.obstacle.side !== :none), two, tot, solves=[r.nlp_solves for r in rs],
        iters=[r.iterations for r in rs], prime=ms(prime), build=ms(c.build_time), fails, stop=cl.msg,
        slow=[(k, tot[k], ms(rs[k].timing.solver), ms(rs[k].timing.optimize), cl.gct[k], cl.alloc[k], rs[k].nlp_solves,
            length(rs[k].obstacle.attempts)) for k in eachindex(tot) if tot[k] > 80],
        two_tot=[tot[k] for k in two], one_tot=[tot[k] for k in eachindex(tot) if !(k in two)], cl)
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    emit("""
    # Stationary-obstacle collision avoidance benchmark (Milestone 3A)

    - Julia $(VERSION), $(strip(Sys.cpu_info()[1].model)), $(Threads.nthreads()) Julia thread(s); reference test vehicle; straight track 80 m, width 0.7 m (planning corridor |ey| ≤ 0.30 m with the 5 cm margin); start `$(X0)`
    - Cartesian-endpoint prediction, ≤ 8 segment reassignments within the 80 ms budget, `interior_samples = 0`, `strict_corridor = false`; objective, weights, limits and Ipopt options unchanged
    - Ego collision disk radius $(EGO) m (= the planning margin, so the disk stays inside the physical track whenever its reference point is in the corridor), safety buffer $(BUF) m
    - Obstacles (radius / centre): centre = 0.10 m at (8, 0); offset = 0.10 m at (8, 0.08); blocked = 0.30 m at (8, 0). Required separation D = 0.18 m (0.38 m blocked); collision (disks touching) at 0.15 m (0.35 m)
    - $(N_CYCLES) control cycles per run, $(REPS) interleaved runs per configuration, event-aware plant (10 substeps); dense collision evaluation of every candidate at 50 samples per interval; plant distances evaluated at the 10 plant substeps per period
    - Timing covers the whole `solve!` cycle (candidate initialisations, all NLP solves, reassignments, validation, collision evaluation)

    """)
    # Compilation and first-solve latency (first in this process).
    tb = @elapsed (c = NMPCController(VEH, TR, NMPCConfig(; base(20)..., obstacle_samples=3); obstacle=OBS[:centre]))
    ts = @elapsed solve!(c, X0)
    emit(@sprintf("First construction / first solve with an obstacle in this process (compilation included): %.2f s / %.2f s. All other timings are warmed up.\n\n", tb, ts))
    for N in (10, 20), (_, cfg, has) in configs(N)
        c, _ = primed(cfg, has ? OBS[:centre] : nothing)
        closed_loop(c, 10)
    end

    keep = Dict{String,Any}()
    for scen in (:centre, :offset), N in (10, 20)
        cfgs = configs(N)
        results = Dict{String,Vector{Any}}()
        for rep in 1:REPS, (name, cfg, has) in (isodd(rep) ? cfgs : reverse(cfgs))
            scen === :offset && !has && rep > 1 && continue
            ob = has ? OBS[scen] : nothing
            c, tp = primed(cfg, ob)
            cl = closed_loop(c, N_CYCLES)
            push!(get!(results, name, Any[]), runstats(c, cl, tp, OBS[scen]))
            rep == 1 && (keep["$scen | $name | N=$N"] = (c, cl))
        end
        t1 = """
        ## Scenario `$(scen)`, N = $N ($(N * 0.1) s horizon)

        | configuration | outcome per run (s_end [m] or stop) | accepted cycles | plant min distance to obstacle [m] | plant clearance above D / above collision [m] | plant track clearance [m] / violations | predicted (dense) clearance above D / above collision [m] | accepted plans entering the buffer (between knots) | passing side |
        |:--|:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
        """
        t2 = """
        | configuration | NLP solves mean / max | Ipopt iter. mean / max | build / priming [ms] | mean / median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles (2 candidate solves): count, median / max [ms] | single-attempt cycles median [ms] |
        |:--|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
        """
        for (name, _, has) in cfgs
            haskey(results, name) || continue
            runs = results[name]
            out = join([r.status === :success ? @sprintf("%.2f", r.s_end) : @sprintf("stop c%d", r.cycles) for r in runs], "; ")
            sides = unique(reduce(vcat, [r.sides for r in runs]))
            t1 *= @sprintf("| %s | %s | %d / %d | %.4f | %.4f / %.4f | %.4f / %d | %.4f / %.4f | %d (%d) | %s |\n", name, out,
                sum(r.accepted for r in runs), sum(r.cycles for r in runs), minimum(r.dmin for r in runs),
                minimum(r.dmin for r in runs) - runs[1].D, minimum(r.dmin for r in runs) - runs[1].Dc,
                minimum(r.plant_track for r in runs), sum(r.plant_viol for r in runs),
                has ? nanmin(r.pred_clear for r in runs) : NaN, has ? nanmin(r.pred_phys for r in runs) : NaN,
                sum(r.buffer_cycles for r in runs), sum(r.between for r in runs), isempty(sides) ? "–" : join(string.(sides), ", "))
            tot, sol, it = reduce(vcat, [r.tot for r in runs]), reduce(vcat, [r.solves for r in runs]), reduce(vcat, [r.iters for r in runs])
            two, one = reduce(vcat, [r.two_tot for r in runs]), reduce(vcat, [r.one_tot for r in runs])
            t2 *= @sprintf("| %s | %.2f / %d | %.1f / %d | %.0f / %.0f | %.1f / %.1f / %.1f / %.1f / %.1f | %d / %d | %d, %s | %.1f |\n", name,
                sum(sol) / length(sol), maximum(sol), sum(it) / length(it), maximum(it), sum(r.build for r in runs) / length(runs),
                sum(r.prime for r in runs) / length(runs), sum(tot) / length(tot), quant(tot, 0.5), quant(tot, 0.95), quant(tot, 0.99),
                maximum(tot), count(>(80), tot), count(>(100), tot), length(two),
                isempty(two) ? "–" : @sprintf("%.1f / %.1f", quant(two, 0.5), maximum(two)), quant(one, 0.5))
        end
        emit(t1 * "\n" * t2 * "\n")
        stops = ["- **$(n)** (run 1): " * first(results[n][1].stop, 400) for (n, _, _) in cfgs if haskey(results, n) && !isempty(results[n][1].stop)]
        isempty(stops) || emit("Stops:\n\n" * join(stops, "\n") * "\n\n")
        slow = [(n, i, s) for (n, _, _) in cfgs if haskey(results, n) for (i, r) in enumerate(results[n]) for s in r.slow]
        if !isempty(slow)
            t = "Cycles above 80 ms (configuration, run, cycle: total / Ipopt / optimize! [ms], GC [ms], allocated [MB], NLP solves, attempts):\n\n"
            for (n, i, s) in slow
                t *= @sprintf("- %s, run %d, cycle %d: %.1f / %.1f / %.1f ms, GC %.1f ms, %.1f MB, %d solve(s), %d attempt(s)\n", n, i, s...)
            end
            emit(t * "\n")
        end
    end

    # Blocked corridor: no lateral passage.
    t = """
    ## Scenario `blocked` (no lateral passage), one control cycle

    | start state | N | configuration | status | failure cause | attempts | passage left / right | NLP solves | time [ms] |
    |:--|--:|:--|:--|:--|:--|:-:|--:|--:|
    """
    for x in ([0.0, 0.0, 0.0, 1.0], [6.6, 0.0, 0.0, 5.0]), N in (10, 20)
        cfg = NMPCConfig(; base(N)..., obstacle_samples=3)
        c = NMPCController(VEH, TR, cfg; obstacle=OBS[:blocked])
        solve!(c, X0); reset_warm_start!(c); set_obstacle!(c, OBS[:blocked])
        r = solve!(c, x)
        t *= @sprintf("| %s | %d | knots + 3 interior samples | %s | %s | %s | %s / %s | %d | %.1f |\n", string(x), N, r.status,
            r.obstacle.failure_cause, join(["$(a[1]) → $(a[2])" for a in r.obstacle.attempts], ", "), r.obstacle.passage_left,
            r.obstacle.passage_right, r.nlp_solves, ms(r.timing.total))
    end
    emit(t * "\n")

    out = joinpath(@__DIR__, "results", "stationary_obstacle.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("wrote ", out)
    return keep
end

keep = main()

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    ob = OBS[:centre]
    D, Dc = EGO + ob.radius + BUF, EGO + ob.radius
    c, cl = keep["centre | knots + 3 interior samples | N=20"]
    _, cl0 = keep["centre | no obstacle (baseline) | N=20"]
    xy(cl) = (P = [frenet_to_cartesian(TR, x[1], x[2], x[3]) for x in cl.fine]; ([p[1] for p in P], [p[2] for p in P]))
    circle(r; cx=ob.x, cy=ob.y) = (θ = range(0, 2π; length=200); (cx .+ r .* cos.(θ), cy .+ r .* sin.(θ)))
    function scene(xlims; equal=false)
        p = plot(; xlabel="X [m]", ylabel="Y [m]", xlims, ylims=(-0.42, 0.42), legend=:bottomright, aspect_ratio=equal ? :equal : :auto)
        hline!(p, [-0.35, 0.35]; lc=:black, lw=2, label="physical track edge")
        hline!(p, [-0.30, 0.30]; lc=:gray, ls=:dash, label="planning corridor (5 cm margin)")
        plot!(p, circle(D)...; seriestype=:shape, fillcolor=:orange, fillalpha=0.25, lc=:orange, label="required separation D = $(round(D; digits=3)) m")
        plot!(p, circle(ob.radius)...; seriestype=:shape, fillcolor=:red, lc=:red, label="obstacle (r = $(ob.radius) m)")
        X0p, Y0p = xy(cl0)
        plot!(p, X0p, Y0p; lc=:gray, ls=:dot, lw=1.5, label="same MPC without obstacle")
        Xp, Yp = xy(cl)
        plot!(p, Xp, Yp; lc=:blue, lw=2, label="ego reference point")
        for k in 1:5:length(cl.states)
            equal || break                                         # disks only where the axes are equal
            x = cl.states[k]
            q = frenet_to_cartesian(TR, x[1], x[2], x[3])
            xlims[1] - 0.2 <= q[1] <= xlims[2] + 0.2 && plot!(p, circle(EGO; cx=q[1], cy=q[2])...; lc=:blue, lw=0.7, la=0.5, label="")
        end
        scatter!(p, [Xp[1]], [Yp[1]]; mc=:green, ms=6, label="start")
        scatter!(p, [Xp[end]], [Yp[end]]; mc=:black, ms=6, label="end")
        return p
    end
    p1 = plot(scene((-0.5, 30.0)); title="Straight track, stationary obstacle (N = 20, knots + 3 interior samples)")
    p2 = plot(scene((6.8, 9.6); equal=true); title="Zoom (equal axes); blue circles: ego collision disk (r = $(EGO) m) every 0.5 s", legend=false)
    savefig(plot(p1, p2; layout=(2, 1), size=(1400, 1000), left_margin=6Plots.mm), joinpath(@__DIR__, "..", "docs", "figures", "stationary_obstacle_trajectory.png"))
    t = 0.1 .* (0:length(cl.inputs)-1)
    dist = [hypot(frenet_to_cartesian(TR, x[1], x[2], x[3])[1] - ob.x, frenet_to_cartesian(TR, x[1], x[2], x[3])[2] - ob.y) for x in cl.fine]
    q1 = plot(t, [rad2deg(u[1]) for u in cl.inputs]; seriestype=:steppost, ylabel="steering [deg]", label="", title="Applied inputs and states")
    q2 = plot(0.1 .* (0:length(cl.states)-1), [x[4] for x in cl.states]; ylabel="speed [m/s]", label="")
    q3 = plot(cl.tf, dist .- D; ylabel="clearance [m]", xlabel="t [s]", label="distance − D (required separation)", ylims=(-0.03, 0.2),
        xlims=(1.0, 3.5), title="Closest approach (plant, 10 samples per period)")
    plot!(q3, cl.tf, dist .- Dc; label="distance − (ego radius + obstacle radius): collision below 0")
    hline!(q3, [0.0]; lc=:black, ls=:dash, label="")
    savefig(plot(q1, q2, q3; layout=(3, 1), size=(1100, 900), left_margin=6Plots.mm), joinpath(@__DIR__, "..", "docs", "figures", "stationary_obstacle_signals.png"))
    println("saved figures")
end
