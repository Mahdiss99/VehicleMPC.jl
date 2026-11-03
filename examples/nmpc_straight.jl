# Example A: progress-maximising NMPC on a straight track.
#
# Run from the repository root:
#
#     julia --project=. examples/nmpc_straight.jl
#
# Writes examples/output/nmpc_straight.csv. Plots: examples/plot_nmpc.jl.

include("nmpc_common.jl")

"""
    run_nmpc_straight(; n_steps=100)

Reference test vehicle on a 150 m straight track, 0.7 m wide, with the baseline
`NMPCConfig()` (RK4, N = 20, dt = 0.1 s). It starts off-centre and slow at
`[s, ey, epsi, v] = [0, 0.2, 0.1, 0.5]`. The progress reward drives the speed to the
`v_max = 5 m/s` bound, while the lateral and heading penalties bring the vehicle back to
the centreline. 100 control periods (10 s, about 48 m) fit inside the track.
"""
function run_nmpc_straight(; n_steps=100)
    vehicle = KinematicBicycle(reference_test_vehicle())
    track = straight_track(150.0; width=0.7)
    ctrl = NMPCController(vehicle, track, NMPCConfig())
    cl = simulate_closed_loop(ctrl, vehicle, [0.0, 0.2, 0.1, 0.5], n_steps)
    return (; ctrl, cl)
end

if abspath(PROGRAM_FILE) == @__FILE__
    (; ctrl, cl) = run_nmpc_straight()
    summarize("Example A: straight track", ctrl, cl)
    println("wrote ", write_closed_loop_csv(joinpath(@__DIR__, "output", "nmpc_straight.csv"), cl))
end
