# Example A: linearized MPC (JuMP + OSQP) on a straight track, no obstacle.
#
# Run from the repository root:
#
#     julia --project=. examples/linear_straight.jl [--cycles=N] [--no-time-budget] [--csv=PATH | --no-csv]
#
# Writes examples/output/linear_straight.csv. Optional plot: examples/plot_examples.jl.

include("example_common.jl")

# ---- configuration (the Milestone 4A recovery scenario) ----
"""
    example_a(; time_budget=true)

Reference test vehicle on a 400 m straight track, 1.0 m wide. Default `LinearMPCConfig()`:
- N = 10, dt = 0.1 s, NMPC objective weights;
- the straight track model, selected automatically.

The start `[s, ey, epsi, v] = [0, 0.35, -0.25, 2.0]` is off-centre and misaligned.
"""
function example_a(; time_budget=true)
    vehicle = KinematicBicycle(reference_test_vehicle())
    track = straight_track(400.0; width=1.0)
    config = time_budget ? LinearMPCConfig() : LinearMPCConfig(sqp_time_budget=Inf)
    return (; vehicle, track, config, obstacle=nothing, x0=[0.0, 0.35, -0.25, 2.0])
end

if abspath(PROGRAM_FILE) == @__FILE__
    opts = example_options(250)
    ex = example_a(; opts.time_budget)
    ctrl, cl = run_example(() -> LinearMPCController(ex.vehicle, ex.track, ex.config), ex.vehicle, ex.x0, opts.cycles)
    println("track model: ", ctrl.track_model)
    ok = report("Example A: linear MPC, straight track", ctrl, cl)
    finish_example("linear_straight", ok, cl, opts)
end
