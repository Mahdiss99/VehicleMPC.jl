# Shared helpers for the standalone examples A–D (linear and nonlinear MPC): command-line
# options, a warm-up run that excludes compilation from the statistics, the closed-loop run
# (`simulate_closed_loop`), a report, and a CSV trajectory writer. No plotting.

using VehicleMPC

"""
    example_options(default_cycles) -> (; cycles, time_budget, csv)

Command-line options of an example:
- `--cycles=N`: control periods (default `default_cycles`);
- `--no-time-budget`: remove the controllers' per-cycle time budgets, so the outcome does
  not depend on the machine's speed (used by CI smoke tests). The validated configurations
  use the budgets;
- `--csv=PATH`: where to write the trajectory (default `examples/output/<example>.csv`);
  `--no-csv` writes nothing.
"""
function example_options(default_cycles::Integer)
    cycles, time_budget, csv = default_cycles, true, :default
    for a in ARGS
        if startswith(a, "--cycles=")
            cycles = parse(Int, split(a, "=")[2])
        elseif a == "--no-time-budget"
            time_budget = false
        elseif startswith(a, "--csv=")
            csv = String(split(a, "="; limit=2)[2])
        elseif a == "--no-csv"
            csv = nothing
        else
            error("unknown option $a (use --cycles=N, --no-time-budget, --csv=PATH or --no-csv)")
        end
    end
    return (; cycles, time_budget, csv)
end

"Linear-interpolation quantile (no Statistics dependency)."
function quantile_of(xs, q)
    ys = sort(collect(xs))
    isempty(ys) && return NaN
    h = (length(ys) - 1) * q + 1
    lo, hi = floor(Int, h), ceil(Int, h)
    return ys[lo] + (h - lo) * (ys[hi] - ys[lo])
end

"""
    run_example(make_controller, vehicle, x0, cycles; warmup_cycles=min(cycles, 150)) -> (controller, ClosedLoopResult)

Build a controller with `make_controller()` and run `simulate_closed_loop` from `x0` for
`cycles` control periods.

**Warm-up first.** Julia compiles each code path on its first call, which takes up to
seconds. Under the controllers' per-cycle time budgets, such a first call would make a
cycle fail. Before the measured run, separate controllers from the same
`make_controller()` therefore run discarded closed loops of `warmup_cycles` periods. A
warm-up loop also stops at a failed cycle, so it is repeated (at most 5 times) until one
completes. The reported statistics therefore exclude compilation. A rarely used code path
can still be compiled for the first time during the measured run.
"""
function run_example(make_controller, vehicle, x0, cycles; warmup_cycles=min(cycles, 150))
    for _ in 1:5
        simulate_closed_loop(make_controller(), vehicle, x0, warmup_cycles).status === :success && break
    end
    ctrl = make_controller()
    return ctrl, simulate_closed_loop(ctrl, vehicle, x0, cycles)
end

solves(r::LinearMPCResult) = r.qp_solves
solves(r::NMPCResult) = r.nlp_solves
solver_statuses(r::LinearMPCResult) = r.qp_status                        # one per QP solve
solver_statuses(r::NMPCResult) = [Symbol(r.termination_status)]          # Ipopt, last NLP solve
iterations(r::LinearMPCResult) = r.qp_iterations                         # OSQP iterations per QP
iterations(r::NMPCResult) = [r.iterations]                               # Ipopt iterations (last NLP solve)

"""
    report(name, ctrl, cl; obstacle=nothing) -> Bool

Print the outcome of a closed loop and return whether it succeeded:
- status and failure message;
- accepted and rejected cycles, and controller and solver status counts. A solver status
  such as `OPTIMAL` is not an accepted control: acceptance additionally requires the
  controller's nonlinear validation;
- final progress;
- minimum physical track clearance on the plant;
- obstacle clearances (above contact and above the required separation D);
- computation statistics;
- plant operating-constraint violations.
"""
function report(name, ctrl, cl; obstacle=nothing)
    tr, c = ctrl.track, ctrl.config
    res = cl.controller
    acc = count(r -> r.accepted, res)
    println("== ", name)
    println("status: ", cl.status, isempty(cl.message) ? "" : "  ($(cl.message))")
    println("cycles: $(length(res)) solved, $acc accepted, $(length(res) - acc) rejected")
    st = [r.status for r in res]
    println("controller statuses: ", join(["$s × $(count(==(s), st))" for s in unique(st)], ", "))
    ss = reduce(vcat, [solver_statuses(r) for r in res]; init=Symbol[])
    println("solver statuses (per solve): ", join(["$s × $(count(==(s), ss))" for s in unique(ss)], ", "))
    x0, xf = cl.states[1], cl.states[end]
    print("final progress: $(round(xf[1] - x0[1]; digits=2)) m in $(round(cl.t[end]; digits=1)) s")
    is_closed(tr) ? println(" ($(round((xf[1] - x0[1]) / track_length(tr); digits=2)) laps)") : println()
    println("final state [s, ey, epsi, v]: ", round.(collect(xf); digits=3))
    maxey = maximum(abs(x[2]) for x in cl.fine_states)
    println("minimum physical track clearance (plant): $(round(tr.width / 2 - maxey; digits=4)) m (max |ey| $(round(maxey; digits=4)) m, half-width $(tr.width / 2) m)")
    if obstacle !== nothing
        D = round(obstacle.radius + c.ego_radius + c.obstacle_buffer; digits=10)
        Dc = round(obstacle.radius + c.ego_radius; digits=10)
        d = minimum(hypot(p[1] - obstacle.x, p[2] - obstacle.y) for p in (frenet_to_cartesian(tr, x[1], x[2], x[3]) for x in cl.fine_states))
        println("obstacle: minimum centre distance $(round(d; digits=4)) m; clearance above contact ($(Dc) m) ",
            "$(round(d - Dc; digits=4)) m; above the required separation D = $(D) m: $(round(d - D; digits=4)) m")
    end
    tt = [1e3 * r.timing.total for r in res]
    if !isempty(tt)
        println("controller computation per cycle [ms] (after warm-up): median $(round(quantile_of(tt, 0.5); digits=2)), ",
            "p95 $(round(quantile_of(tt, 0.95); digits=2)), p99 $(round(quantile_of(tt, 0.99); digits=2)), max $(round(maximum(tt); digits=2)); ",
            "cycles > 80 ms: $(count(>(80), tt)), > 100 ms: $(count(>(100), tt))")
        sv = [solves(r) for r in res]
        it = reduce(vcat, [iterations(r) for r in res]; init=Int[])
        kind = ctrl isa LinearMPCController ? "QP (OSQP iterations per QP)" : "NLP (Ipopt iterations per cycle)"
        println("solves per cycle: mean $(round(sum(sv) / length(sv); digits=2)), max $(maximum(sv)) $kind; iterations median $(quantile_of(it, 0.5)), max $(maximum(it))")
    end
    println("plant operating-constraint violations (tol $(c.acceptance_tol)): $(length(cl.violations))")
    return cl.status === :success
end

"""
    write_trajectory_csv(path, cl) -> path

One row per control period: time, Frenet state, Cartesian pose, applied input, controller
status, acceptance, solves and computation time.
"""
function write_trajectory_csv(path, cl)
    mkpath(dirname(path))
    open(path, "w") do io
        println(io, "t,s,ey,epsi,v,X,Y,psi,delta,a,status,accepted,solves,solve_ms")
        for k in eachindex(cl.states)
            x, p = cl.states[k], cl.poses[k]
            u = k <= length(cl.inputs) ? cl.inputs[k] : (NaN, NaN)
            r = k <= length(cl.controller) ? cl.controller[k] : nothing
            meta = r === nothing ? ("", "", "", "") : (r.status, r.accepted, solves(r), 1e3 * r.timing.total)
            println(io, join((cl.t[k], x..., p..., u..., meta...), ","))
        end
    end
    return path
end

"Write the CSV (unless disabled), print where, and exit with status 1 if the closed loop failed."
function finish_example(name, ok, cl, opts)
    if opts.csv !== nothing
        path = opts.csv === :default ? joinpath(@__DIR__, "output", "$name.csv") : opts.csv
        println("wrote ", write_trajectory_csv(path, cl))
    end
    ok || (println("the closed loop did not complete (see the status above)"); exit(1))
    return nothing
end
