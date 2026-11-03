# Example B: linearized MPC (JuMP + OSQP) on the multi-curvature quarter track.
#
# Run from the repository root:
#
#     julia --project=. examples/linear_quarter.jl [--cycles=N] [--no-time-budget] [--csv=PATH | --no-csv]
#
# Writes examples/output/linear_quarter.csv. Optional plot: examples/plot_examples.jl.

include("example_common.jl")

# ---- configuration (the Milestone 4BC stage A setup) ----
"""
    example_b(; time_budget=true)

Reference test vehicle on the legacy `quarter_track()`: closed, 0.7 m wide, straight
segments and R = 1, −1 and 2 m arcs. `LinearMPCConfig(horizon=10)` with
`track_model = :auto`, which selects the local track model on this track. Start at
`[0, 0, 0, 0.5]`.
"""
function example_b(; time_budget=true)
    vehicle = KinematicBicycle(reference_test_vehicle())
    track = quarter_track()
    config = time_budget ? LinearMPCConfig(horizon=10) : LinearMPCConfig(horizon=10, sqp_time_budget=Inf)
    return (; vehicle, track, config, obstacle=nothing, x0=[0.0, 0.0, 0.0, 0.5])
end

if abspath(PROGRAM_FILE) == @__FILE__
    opts = example_options(250)
    ex = example_b(; opts.time_budget)
    ctrl, cl = run_example(() -> LinearMPCController(ex.vehicle, ex.track, ex.config), ex.vehicle, ex.x0, opts.cycles)
    println("track model: ", ctrl.track_model)
    ok = report("Example B: linear MPC, quarter track", ctrl, cl)
    te = [maximum(abs, r.track_error[2:3]) for r in cl.controller if r.accepted && all(isfinite, r.track_error)]
    isempty(te) || println("track-linearisation error of accepted plans (max lateral/heading): $(round(maximum(te); sigdigits=3))")
    finish_example("linear_quarter", ok, cl, opts)
end
