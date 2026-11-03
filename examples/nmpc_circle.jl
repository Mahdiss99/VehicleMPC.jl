# Example B: progress-maximising NMPC on a circular track (R = 1 m, width 0.7 m).
#
# Run from the repository root:
#
#     julia --project=. examples/nmpc_circle.jl
#
# Writes examples/output/nmpc_circle_*.csv. Plots: examples/plot_nmpc.jl.

include("nmpc_common.jl")

"""
    run_nmpc_circle(; config=NMPCConfig(), n_steps=70)

Reference test vehicle on `circular_track(1.0; width=0.7)`, starting on the centreline at
0.5 m/s. 70 control periods (7 s) cover more than two laps. The curvature is constant, so
there is no segment transition inside the horizon.
"""
function run_nmpc_circle(; config=NMPCConfig(), n_steps=70)
    vehicle = KinematicBicycle(reference_test_vehicle())
    track = circular_track(1.0; width=0.7)
    ctrl = NMPCController(vehicle, track, config)
    cl = simulate_closed_loop(ctrl, vehicle, [0.0, 0.0, 0.0, 0.5], n_steps)
    return (; ctrl, cl)
end

"""
Scenarios shown by the example:

- `baseline`: default weights. The lateral penalty dominates, so the vehicle stays near
  the centreline and the speed is set by `ay_max` (`v ≈ √(ay_max·R) = 2 m/s`).
- `progress_dominant`: the same constraints with `qy = qpsi = 0`. Only progress and input
  smoothness are weighted, so the controller moves to the inside edge and the track
  boundary becomes active. The constraints are not loosened in either scenario.
"""
const CIRCLE_SCENARIOS = (
    baseline=NMPCConfig(),
    progress_dominant=NMPCConfig(qy=0.0, qpsi=0.0),
)

if abspath(PROGRAM_FILE) == @__FILE__
    for (name, cfg) in pairs(CIRCLE_SCENARIOS)
        (; ctrl, cl) = run_nmpc_circle(; config=cfg)
        summarize("Example B: circle, $name", ctrl, cl)
        println("wrote ", write_closed_loop_csv(joinpath(@__DIR__, "output", "nmpc_circle_$name.csv"), cl))
        println()
    end
end
