# Example C: linearized MPC (JuMP + OSQP) with a stationary obstacle on the quarter track.
#
# Run from the repository root:
#
#     julia --project=. examples/linear_obstacle.jl [--cycles=N] [--no-time-budget] [--csv=PATH | --no-csv]
#
# Writes examples/output/linear_obstacle.csv. Optional plot: examples/plot_examples.jl.

include("example_common.jl")

# ---- configuration (the validated Milestone 4BC setup, scenario B) ----
"""
    example_c(; time_budget=true)

Reference test vehicle on `quarter_track()`, with a stationary circular obstacle of radius
0.10 m on the centreline of the R = 1 m left bend (`s = 3.5 m`).

`LinearMPCConfig` settings:
- N = 10;
- ego radius 0.05 m, buffer 0.03 m (required separation D = 0.18 m, contact at 0.15 m);
- three interior halfspace samples per interval (`obstacle_samples = 3`);
- every other setting at its default (sequential passing-side strategy, 80 ms budget).

Start at `[0, 0, 0, 0.5]`.
"""
function example_c(; time_budget=true)
    vehicle = KinematicBicycle(reference_test_vehicle())
    track = quarter_track()
    obstacle = obstacle_on_track(track, 3.5, 0.0; radius=0.10)
    kw = (horizon=10, ego_radius=0.05, obstacle_buffer=0.03, obstacle_samples=3)
    config = time_budget ? LinearMPCConfig(; kw...) : LinearMPCConfig(; kw..., sqp_time_budget=Inf)
    return (; vehicle, track, config, obstacle, x0=[0.0, 0.0, 0.0, 0.5])
end

if abspath(PROGRAM_FILE) == @__FILE__
    opts = example_options(250)
    ex = example_c(; opts.time_budget)
    ctrl, cl = run_example(() -> LinearMPCController(ex.vehicle, ex.track, ex.config; obstacle=ex.obstacle), ex.vehicle, ex.x0, opts.cycles)
    println("track model: ", ctrl.track_model)
    ok = report("Example C: linear MPC, stationary obstacle on the quarter track", ctrl, cl; obstacle=ex.obstacle)
    enc = [(k, r.obstacle.side, [(a[1], a[2]) for a in r.obstacle.attempts]) for (k, r) in enumerate(cl.controller)
           if any(a -> a[1] in (:left, :right), r.obstacle.attempts)]
    println("passing-side decisions (cycle, side, attempts): ", isempty(enc) ? "none" : join(string.(enc), "; "))
    cl.status === :success || println("failure cause: ", cl.controller[end].obstacle.failure_cause)
    finish_example("linear_obstacle", ok, cl, opts)
end
