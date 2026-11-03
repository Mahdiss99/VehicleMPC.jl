# Optional visualization of the open-loop example. Plotting lives in its own environment so
# the VehicleMPC package itself has no plotting dependency.
#
# Run from the repository root (Julia ≥ 1.11, which honours [sources] in examples/Project.toml):
#
#     julia --project=examples -e "using Pkg; Pkg.instantiate()"
#     julia --project=examples examples/plot_open_loop.jl
#
# The figure is saved to examples/output/kinematic_open_loop.png; no window is opened.

ENV["GKSwstype"] = "100"    # headless GR backend
include("kinematic_open_loop.jl")
using Plots

function plot_open_loop(res::SimulationResult, track::Track)
    s_grid = range(0, track_length(track); length=400)
    edge(ey) = [frenet_to_cartesian(track, s, ey)[1:2] for s in s_grid]
    w = track.width / 2

    p_xy = plot(first.(edge(0.0)), last.(edge(0.0)); ls=:dot, lc=:gray, label="centreline",
        aspect_ratio=:equal, xlabel="X [m]", ylabel="Y [m]", title="Open-loop trajectory")
    for ey in (-w, w)
        plot!(p_xy, first.(edge(ey)), last.(edge(ey)); lc=:black, label="")
    end
    plot!(p_xy, [p[1] for p in res.poses], [p[2] for p in res.poses]; lw=2, label="rear axle")

    states = reduce(hcat, res.states)'
    p_ey = plot(res.t, states[:, 2]; ylabel="ey [m]", label="")
    p_epsi = plot(res.t, states[:, 3]; ylabel="epsi [rad]", label="")
    p_v = plot(res.t, states[:, 4]; ylabel="v [m/s]", xlabel="t [s]", label="")
    return plot(p_xy, plot(p_ey, p_epsi, p_v; layout=(3, 1)); layout=(1, 2), size=(1000, 500), left_margin=6Plots.mm, bottom_margin=6Plots.mm)
end

res = run_open_loop_example()
fig = plot_open_loop(res, example_track())
out = joinpath(@__DIR__, "output", "kinematic_open_loop.png")
mkpath(dirname(out))
savefig(fig, out)
println("saved ", out)
