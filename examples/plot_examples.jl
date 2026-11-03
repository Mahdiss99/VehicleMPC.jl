# Optional plot of the standalone examples A–D (Plots, in the examples environment).
#
#     julia --project=examples -e "using Pkg; Pkg.instantiate()"      # once
#     julia --project=examples examples/plot_examples.jl
#
# Reads examples/output/{linear_straight,linear_quarter,linear_obstacle,nmpc_obstacle}.csv
# (whichever exist; run the examples first) and writes examples/output/examples_*.png.

using Plots
ENV["GKSwstype"] = "100"

include("linear_straight.jl")
include("linear_quarter.jl")
include("linear_obstacle.jl")
include("nmpc_obstacle.jl")

function read_xy(path)
    lines = readlines(path)
    hdr = split(lines[1], ",")
    ix, iy, iv = findfirst(==("X"), hdr), findfirst(==("Y"), hdr), findfirst(==("v"), hdr)
    rows = [split(l, ",") for l in lines[2:end]]
    return [parse(Float64, r[ix]) for r in rows], [parse(Float64, r[iy]) for r in rows], [parse(Float64, r[iv]) for r in rows]
end

function track_edges!(p, track; smax=track_length(track))
    for (ey, lc, ls) in ((track.width / 2, :black, :solid), (-track.width / 2, :black, :solid), (0.0, :gray, :dot))
        P = [frenet_to_cartesian(track, s, ey) for s in range(0, smax; length=1500)]
        plot!(p, first.(P), [q[2] for q in P]; lc, ls, label="")
    end
    return p
end

circle(cx, cy, r) = (θ = range(0, 2π; length=100); (cx .+ r .* cos.(θ), cy .+ r .* sin.(θ)))

out = joinpath(@__DIR__, "output")
for (name, ex, smax) in (("linear_straight", example_a(), 120.0), ("linear_quarter", example_b(), nothing),
    ("linear_obstacle", example_c(), nothing), ("nmpc_obstacle", example_d(), nothing))
    path = joinpath(out, "$name.csv")
    isfile(path) || (println("skipping $name (run examples/$name.jl first)"); continue)
    X, Y, v = read_xy(path)
    p = plot(; aspect_ratio=:equal, xlabel="X [m]", ylabel="Y [m]", title=name, legend=:topright, size=(1000, 800))
    track_edges!(p, ex.track; smax=smax === nothing ? track_length(ex.track) : min(smax, maximum(X) + 5))
    if ex.obstacle !== nothing
        c = ex.config
        plot!(p, circle(ex.obstacle.x, ex.obstacle.y, ex.obstacle.radius + c.ego_radius + c.obstacle_buffer)...;
            seriestype=:shape, fillalpha=0.3, fillcolor=:orange, lc=:orange, label="required separation D")
        plot!(p, circle(ex.obstacle.x, ex.obstacle.y, ex.obstacle.radius)...; seriestype=:shape, fillcolor=:red, lc=:red, label="obstacle")
    end
    plot!(p, X, Y; line_z=v, lw=2, c=:viridis, colorbar_title="v [m/s]", label="vehicle (rear axle)")
    savefig(p, joinpath(out, "examples_$name.png"))
    println("wrote ", joinpath(out, "examples_$name.png"))
end
