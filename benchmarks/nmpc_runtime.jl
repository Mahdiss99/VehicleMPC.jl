# NMPC runtime benchmark (Milestone 2A).
#
# Run from the repository root:
#
#     julia --project=. benchmarks/nmpc_runtime.jl [n_solves] [n_rounds]
#
# Every scenario runs `n_solves` (default 250) consecutive closed-loop control periods with
# a fresh controller, after a process-wide warm-up that absorbs compilation. The set of
# scenarios is repeated `n_rounds` times (default 3), alternating the order, so that slow
# drifts in machine load affect all scenarios alike. Statistics pool all rounds. The first
# period of each warm-start run is necessarily a cold solve and is included. Results are
# printed and written to benchmarks/results/nmpc_runtime.md. Timings are wall-clock
# measurements on one machine, with no real-time guarantee.

using VehicleMPC
using JuMP
import Ipopt
using Printf

const N_SOLVES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 250
const N_ROUNDS = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 3
const VEHICLE = KinematicBicycle(reference_test_vehicle())

quantile_sorted(xs, q) = (h = (length(xs) - 1) * q + 1; lo = floor(Int, h); hi = ceil(Int, h); xs[lo] + (h - lo) * (xs[hi] - xs[lo]))
ms(x) = 1e3 * x
median_ms(res) = quantile_sorted(sort([ms(r.timing.total) for r in res]), 0.5)

circle() = (circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5])
straight() = (straight_track(1500.0; width=0.7), [0.0, 0.2, 0.1, 0.5])

const SCENARIOS = [
    ("circle", "RK4", 20, true, circle),
    ("circle", "RK4", 20, false, circle),
    ("circle", "Euler", 20, true, circle),
    ("circle", "Euler", 20, false, circle),
    ("circle", "RK4", 10, true, circle),
    ("circle", "RK4", 30, true, circle),
    ("straight", "RK4", 20, true, straight),
    ("straight", "RK4", 20, false, straight),
]

function run_scenario(trackfn, integ, N, warm, n)
    track, x0 = trackfn()
    cfg = NMPCConfig(horizon=N, integrator=integ == "RK4" ? RK4() : ForwardEuler(), warm_start=warm)
    GC.gc()
    t_build = @elapsed ctrl = NMPCController(VEHICLE, track, cfg)
    cl = simulate_closed_loop(ctrl, VEHICLE, x0, n)
    return (; cl, t_build)
end

"Pool the runs (one per round) of one scenario."
function stats_row(name, integ, N, warm, runs)
    res = reduce(vcat, [run.cl.controller for run in runs])
    tot = sort([ms(r.timing.total) for r in res])
    n = length(tot)
    comp(f) = sum(ms(getfield(r.timing, f)) for r in res) / n
    its = [r.iterations for r in res]
    finite_max(f) = maximum(r -> isnan(getfield(r, f)) ? Inf : getfield(r, f), res)
    round_medians = [median_ms(run.cl.controller) for run in runs]
    ipopt = comp(:solver)
    return (; name, integ, N, warm=warm ? "warm" : "cold", n,
        status=join(unique(string(run.cl.status) for run in runs), ","),
        build_ms=sum(ms(run.t_build) for run in runs) / length(runs),
        mean=sum(tot) / n, median=quantile_sorted(tot, 0.5), p95=quantile_sorted(tot, 0.95),
        p99=quantile_sorted(tot, 0.99), max=tot[end], rmin=minimum(round_medians), rmax=maximum(round_medians),
        it_mean=sum(its) / n, it_median=sort(its)[cld(n, 2)], it_max=maximum(its),
        fails=count(r -> !r.accepted, res), viol=finite_max(:max_constraint_violation), dyn=finite_max(:max_dynamics_residual),
        plant_viol=sum(length(run.cl.violations) for run in runs), over100=count(>(100), tot), over80=count(>(80), tot),
        update=comp(:update), warmprep=comp(:warm_start), optimize=comp(:optimize), ipopt,
        per_iter=ipopt / (sum(its) / n), validation=comp(:validation))
end

function main()
    io = IOBuffer()
    emit(s) = (print(s); print(io, s))
    ipopt_version = MOI.get(Model(Ipopt.Optimizer), MOI.SolverVersion())
    emit("""
    # NMPC runtime benchmark

    - Julia $(VERSION), $(Sys.KERNEL) $(Sys.ARCH), CPU: $(strip(Sys.cpu_info()[1].model)) ($(Sys.CPU_THREADS) logical cores), Julia threads: $(Threads.nthreads())
    - JuMP $(pkgversion(JuMP)), Ipopt.jl $(pkgversion(Ipopt)), Ipopt $(ipopt_version) (default linear solver, MUMPS)
    - $(N_SOLVES) consecutive closed-loop control periods per scenario and round, $(N_ROUNDS) rounds with alternating order ($(N_SOLVES * N_ROUNDS) solves per scenario); dt = 0.1 s; fresh controller per run
    - Times are wall-clock milliseconds of one `solve!` call (parameter update + initial guess + Ipopt + validation). The plant simulation is excluded.

    """)

    # Process-wide warm-up: the first construction and the first solves include compilation.
    track, x0 = circle()
    t_build0 = @elapsed ctrl0 = NMPCController(VEHICLE, track, NMPCConfig())
    t_first = @elapsed r_first = solve!(ctrl0, x0)
    t_second = @elapsed solve!(ctrl0, x0; previous_input=r_first.u)
    for (_, integ, N, warm, trackfn) in SCENARIOS
        run_scenario(trackfn, integ, N, warm, 3)
    end
    emit("""
    ## Start-up (includes compilation)

    | quantity | value |
    |:--|--:|
    | first model construction (RK4, N = 20) | $(@sprintf("%.0f", ms(t_build0))) ms |
    | first solve, cold (includes compilation) | $(@sprintf("%.0f", ms(t_first))) ms |
    | second solve on the same controller | $(@sprintf("%.1f", ms(t_second))) ms |

    """)

    runs = [Any[] for _ in SCENARIOS]
    for round in 1:N_ROUNDS
        for i in (isodd(round) ? eachindex(SCENARIOS) : reverse(eachindex(SCENARIOS)))
            (_, integ, N, warm, trackfn) = SCENARIOS[i]
            push!(runs[i], run_scenario(trackfn, integ, N, warm, N_SOLVES))
        end
    end
    rows = [stats_row(s[1], s[2], s[3], s[4], runs[i]) for (i, s) in enumerate(SCENARIOS)]

    t1 = """
    ## End-to-end `solve!` latency

    All times in milliseconds. "Round medians" is the range of the per-round medians, which shows run-to-run variability.

    | track | integrator | N | init | solves | mean | median | round medians | p95 | p99 | max | > 80 ms | > 100 ms | Ipopt iterations: mean (median / max) | rejected |
    |:--|:--|--:|:--|--:|--:|--:|:-:|--:|--:|--:|--:|--:|--:|--:|
    """
    for r in rows
        t1 *= @sprintf("| %s | %s | %d | %s | %d | %.1f | %.1f | %.1f–%.1f | %.1f | %.1f | %.1f | %d (%.1f %%) | %d (%.1f %%) | %.1f (%d / %d) | %d |\n",
            r.name, r.integ, r.N, r.warm, r.n, r.mean, r.median, r.rmin, r.rmax, r.p95, r.p99, r.max,
            r.over80, 100r.over80 / r.n, r.over100, 100r.over100 / r.n, r.it_mean, r.it_median, r.it_max, r.fails)
    end
    t2 = """

    ## Mean time per component [ms]

    | track | integrator | N | init | model build | parameter update | initial guess | `optimize!` (wall) | Ipopt-reported solve | Ipopt per iteration | extraction + validation |
    |:--|:--|--:|:--|--:|--:|--:|--:|--:|--:|--:|
    """
    for r in rows
        t2 *= @sprintf("| %s | %s | %d | %s | %.1f | %.3f | %.3f | %.2f | %.2f | %.2f | %.3f |\n",
            r.name, r.integ, r.N, r.warm, r.build_ms, r.update, r.warmprep, r.optimize, r.ipopt, r.per_iter, r.validation)
    end
    t3 = """

    ## Validation

    | track | integrator | N | init | closed-loop status | max dynamics residual | max knot constraint violation | plant operating violations |
    |:--|:--|--:|:--|:--|--:|--:|--:|
    """
    for r in rows
        t3 *= @sprintf("| %s | %s | %d | %s | %s | %.1e | %.1e | %d |\n",
            r.name, r.integ, r.N, r.warm, r.status, r.dyn, r.viol, r.plant_viol)
    end
    emit(t1 * t2 * t3)
    out = joinpath(@__DIR__, "results", "nmpc_runtime.md")
    mkpath(dirname(out))
    write(out, String(take!(io)))
    println("\nwrote ", out)
end

main()
