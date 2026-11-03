# Open-loop demonstration of the kinematic bicycle model on a circular track.
#
# Run from the repository root:
#
#     julia --project=. examples/kinematic_open_loop.jl
#
# The numerical results are written to examples/output/kinematic_open_loop.csv.
# Plotting is separate and optional; see examples/plot_open_loop.jl.

using VehicleMPC

"Reference track of the example: counter-clockwise circle, radius 2 m, width 0.7 m."
example_track() = circular_track(2.0; width=0.7)

"""
    run_open_loop_example(; dt=0.05) -> SimulationResult

Reference test vehicle on a counter-clockwise circle of radius 2 m, driven by a fixed
steering/acceleration schedule:

| time [s] | steering                     | acceleration [m/s²] |
|:---------|:-----------------------------|:--------------------|
| 0–2      | steady-state `atan(L/R)`     | +0.5                |
| 2–4      | steady-state + 0.05 rad      | 0.0                 |
| 4–6      | steady-state − 0.05 rad      | −0.25               |
"""
function run_open_loop_example(; dt=0.05)
    model = KinematicBicycle(reference_test_vehicle())
    track = example_track()
    R = 1 / curvature(track, 0.0)
    x0 = [0.0, 0.1, 0.0, 0.5]                     # s [m], ey [m], epsi [rad], v [m/s]
    delta_ss = atan(model.params.wheelbase / R)
    steps = round(Int, 2.0 / dt)
    inputs = vcat(fill([delta_ss, 0.5], steps),
        fill([delta_ss + 0.05, 0.0], steps),
        fill([delta_ss - 0.05, -0.25], steps))
    return simulate_open_loop(model, track, x0, inputs, dt)
end

"Write time, Frenet state, Cartesian pose, input and curvature to a CSV file."
function write_csv(path, res::SimulationResult)
    mkpath(dirname(path))
    open(path, "w") do io
        println(io, "t,s,ey,epsi,v,X,Y,psi,delta,a,kappa")
        for k in eachindex(res.states)
            x, p = res.states[k], res.poses[k]
            u, kappa = k <= length(res.inputs) ? (res.inputs[k], res.curvature[k]) : ((NaN, NaN), NaN)
            println(io, join((res.t[k], x..., p..., u..., kappa), ","))
        end
    end
    return path
end

if abspath(PROGRAM_FILE) == @__FILE__
    res = run_open_loop_example()
    println("status: ", res.status, isempty(res.message) ? "" : " ($(res.message))")
    xf = res.states[end]
    println("final state  s=$(round(xf[1]; digits=4)) m, ey=$(round(xf[2]; digits=4)) m, ",
        "epsi=$(round(xf[3]; digits=4)) rad, v=$(round(xf[4]; digits=4)) m/s")
    println("final pose   X=$(round(res.poses[end][1]; digits=4)) m, Y=$(round(res.poses[end][2]; digits=4)) m")
    viol = trajectory_violations(KinematicBicycle(reference_test_vehicle()), example_track(), res)
    println("operating-constraint violations: $(length(viol)) (open loop, not enforced)")
    isempty(viol) || println("  first: step $(first(viol[1])) at t=$(round(res.t[first(viol[1])]; digits=3)) s: $(viol[1].second.message)")
    println("wrote ", write_csv(joinpath(@__DIR__, "output", "kinematic_open_loop.csv"), res))
end
