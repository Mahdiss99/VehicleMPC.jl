# Example D: nonlinear MPC (JuMP + Ipopt) with a stationary obstacle on the quarter track.
#
# Run from the repository root:
#
#     julia --project=. examples/nmpc_obstacle.jl [--cycles=N] [--no-time-budget] [--csv=PATH | --no-csv]
#
# Writes examples/output/nmpc_obstacle.csv. Optional plot: examples/plot_examples.jl.

include("example_common.jl")

# ---- configuration (the NMPC comparison setup of Milestone 4BC, scenario B) ----
"""
    example_d(; time_budget=true)

The same vehicle, track, obstacle, horizon and start as Example C, with the
Cartesian-endpoint Ipopt NMPC.

`NMPCConfig` settings:
- `prediction_model = :cartesian_endpoint_exact`, N = 10;
- up to 8 segment reassignments within the 80 ms correction budget;
- three interior obstacle samples;
- the Milestone 3B.1 cycle detection, branch resolution and sequential encounters.

All other settings are the `NMPCConfig` defaults (same objective weights and limits as the
linear MPC). Without the time budget, the correction budget is set to 1000 s.
"""
function example_d(; time_budget=true)
    vehicle = KinematicBicycle(reference_test_vehicle())
    track = quarter_track()
    obstacle = obstacle_on_track(track, 3.5, 0.0; radius=0.10)
    kw = (prediction_model=:cartesian_endpoint_exact, horizon=10, schedule_corrections=8, ego_radius=0.05, obstacle_buffer=0.03,
        obstacle_samples=3, reassignment_cycle_detection=true, branch_resolution=true, encounter_strategy=:sequential)
    config = time_budget ? NMPCConfig(; kw...) : NMPCConfig(; kw..., correction_time_budget=1e3)
    return (; vehicle, track, config, obstacle, x0=[0.0, 0.0, 0.0, 0.5])
end

if abspath(PROGRAM_FILE) == @__FILE__
    opts = example_options(250)
    ex = example_d(; opts.time_budget)
    ctrl, cl = run_example(() -> NMPCController(ex.vehicle, ex.track, ex.config; obstacle=ex.obstacle), ex.vehicle, ex.x0, opts.cycles)
    ok = report("Example D: nonlinear MPC, stationary obstacle on the quarter track", ctrl, cl; obstacle=ex.obstacle)
    enc = [(k, r.obstacle.side, [(a[1], a[2]) for a in r.obstacle.attempts]) for (k, r) in enumerate(cl.controller)
           if any(a -> a[1] in (:left, :right), r.obstacle.attempts)]
    println("passing-side decisions (cycle, side, attempts): ", isempty(enc) ? "none" : join(string.(enc), "; "))
    cl.status === :success || println("failure cause: ", cl.controller[end].obstacle.failure_cause)
    finish_example("nmpc_obstacle", ok, cl, opts)
end
