# Closed-loop plots for the NMPC examples (optional; uses the examples environment).
#
#     julia --project=examples -e "using Pkg; Pkg.instantiate()"
#     julia --project=examples examples/plot_nmpc.jl
#
# Saves examples/output/nmpc_straight.png, nmpc_circle_baseline.png,
# nmpc_circle_progress_dominant.png and nmpc_quarter.png. No window is opened.

ENV["GKSwstype"] = "100"    # headless GR backend
include("nmpc_straight.jl")
include("nmpc_circle.jl")
include("nmpc_quarter.jl")
using Plots

function plot_closed_loop(title, ctrl::NMPCController, cl::ClosedLoopResult)
    track, cfg, p = ctrl.track, ctrl.config, ctrl.vehicle.params
    w = track.width / 2
    s_end = is_closed(track) ? track_length(track) : min(track_length(track), cl.states[end][1] + 2)
    sg = range(0, s_end; length=600)
    edge(ey) = [frenet_to_cartesian(track, s, ey)[1:2] for s in sg]
    xy = plot(first.(edge(0.0)), last.(edge(0.0)); ls=:dot, lc=:gray, label="centreline",
        xlabel="X [m]", ylabel="Y [m]", title=title, legend=:outertopright)
    for (ey, lab) in ((w, "track edge"), (-w, ""))
        plot!(xy, first.(edge(ey)), last.(edge(ey)); lc=:black, label=lab)
    end
    for (ey, lab) in ((ctrl.ey_max, "NMPC bound"), (-ctrl.ey_max, ""))
        plot!(xy, first.(edge(ey)), last.(edge(ey)); lc=:red, ls=:dash, label=lab)
    end
    plot!(xy, [q[1] for q in cl.poses], [q[2] for q in cl.poses]; lw=2, lc=:blue, label="rear axle")
    is_closed(track) && plot!(xy; aspect_ratio=:equal)

    t, tu = cl.t, cl.t[1:end-1]
    X = reduce(hcat, cl.states)
    U = reduce(hcat, cl.inputs)
    tf = cl.fine_t
    F = reduce(hcat, cl.fine_states)
    p_ey = plot(tf, F[2, :]; label="plant", ylabel="ey [m]")
    scatter!(p_ey, t, X[2, :]; ms=2, label="knots")
    hline!(p_ey, [-ctrl.ey_max, ctrl.ey_max]; ls=:dash, lc=:red, label="bound")
    hline!(p_ey, [-w, w]; lc=:black, label="edge")
    p_epsi = plot(tf, F[3, :]; ylabel="epsi [rad]", label="")
    p_v = plot(tf, F[4, :]; ylabel="v [m/s]", label="", ylims=(0, p.v_max * 1.1))
    hline!(p_v, [p.v_max]; ls=:dash, lc=:red, label="v_max")
    p_d = plot(tu, U[1, :]; seriestype=:steppost, ylabel="delta [rad]", label="")
    hline!(p_d, [-p.delta_max, p.delta_max]; ls=:dash, lc=:red, label="")
    p_a = plot(tu, U[2, :]; seriestype=:steppost, ylabel="a [m/s²]", label="")
    hline!(p_a, [p.a_min, p.a_max]; ls=:dash, lc=:red, label="")
    ns = (length(tf) - 1) ÷ length(cl.inputs)
    ay = [lateral_acceleration(ctrl.vehicle, F[4, j], U[1, cld(j - 1, ns)]) for j in 2:length(tf)]
    p_ay = plot(tf[2:end], ay; ylabel="ay [m/s²]", label="")
    cfg.ay_max === nothing || hline!(p_ay, [-cfg.ay_max, cfg.ay_max]; ls=:dash, lc=:red, label="")
    ms = [1e3 * r.timing.total for r in cl.controller]
    p_t = plot(tu[2:end], ms[2:end]; ylabel="solve! [ms]", xlabel="t [s]", label="", seriestype=:steppost)
    hline!(p_t, [80, 100]; ls=:dash, lc=[:orange :red], label="")
    right = plot(p_ey, p_epsi, p_v, p_d, p_a, p_ay, p_t; layout=(7, 1), xlabel="", size=(500, 1100), legend=false,
        guidefontsize=8, tickfontsize=7)
    return plot(xy, right; layout=@layout([a{0.55w} b]), size=(1300, 1100), left_margin=6Plots.mm, bottom_margin=4Plots.mm)
end

function save_nmpc_plots(out=mkpath(joinpath(@__DIR__, "output")))
    (; ctrl, cl) = run_nmpc_straight()
    savefig(plot_closed_loop("NMPC, straight track", ctrl, cl), joinpath(out, "nmpc_straight.png"))
    for (name, cfg) in pairs(CIRCLE_SCENARIOS)
        (; ctrl, cl) = run_nmpc_circle(; config=cfg)
        savefig(plot_closed_loop("NMPC, circle R = 1 m ($name)", ctrl, cl), joinpath(out, "nmpc_circle_$name.png"))
    end
    (; ctrl, cl) = run_nmpc_quarter()
    savefig(plot_closed_loop("NMPC, quarter_track() (verified_qualified)", ctrl, cl), joinpath(out, "nmpc_quarter.png"))
    println("saved plots to ", out)
end

save_nmpc_plots()
