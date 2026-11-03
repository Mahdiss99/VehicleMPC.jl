# Shared reporting helpers for the NMPC examples (no plotting).

using VehicleMPC

"Linear-interpolation quantile (no Statistics dependency)."
function quantile_sorted(xs::AbstractVector, q::Real)
    isempty(xs) && return NaN
    h = (length(xs) - 1) * q + 1
    lo, hi = floor(Int, h), ceil(Int, h)
    return xs[lo] + (h - lo) * (xs[hi] - xs[lo])
end

"Print a closed-loop summary; timing statistics exclude the first (compilation) solve."
function summarize(name, ctrl::NMPCController, cl::ClosedLoopResult)
    res = cl.controller
    acc = count(r -> r.accepted, res)
    println("== ", name)
    println("status: ", cl.status, isempty(cl.message) ? "" : " ($(cl.message))")
    println("solves: $(length(res)), accepted: $acc, rejected: $(length(res) - acc)")
    tt = sort([1e3 * r.timing.total for r in res[2:end]])
    if !isempty(tt)
        println("solve! time after the first solve [ms]: median $(round(quantile_sorted(tt, 0.5); digits=1)), ",
            "p99 $(round(quantile_sorted(tt, 0.99); digits=1)), max $(round(tt[end]; digits=1))  ",
            "(first solve incl. compilation: $(round(1e3 * res[1].timing.total; digits=0)) ms)")
    end
    its = [r.iterations for r in res]
    println("Ipopt iterations: median $(sort(its)[cld(length(its), 2)]), max $(maximum(its))")
    println("max dynamics residual $(maximum(r.max_dynamics_residual for r in res)), ",
        "max knot constraint violation $(maximum(r.max_constraint_violation for r in res))")
    xs, fs = cl.states, cl.fine_states
    println("max |ey|: knots $(round(maximum(abs(x[2]) for x in xs); digits=4)) m (bound ±$(round(ctrl.ey_max; digits=3))), ",
        "plant substeps $(round(maximum(abs(x[2]) for x in fs); digits=4)) m (track ±$(ctrl.track.width / 2))")
    if ctrl.config.ay_max !== nothing && !isempty(cl.inputs)
        ns = (length(fs) - 1) ÷ length(cl.inputs)
        ay = maximum(abs(lateral_acceleration(ctrl.vehicle, fs[j][4], cl.inputs[cld(j - 1, ns)][1])) for j in 2:length(fs))
        println("max |ay| on the plant: $(round(ay; digits=4)) m/s² (ay_max $(ctrl.config.ay_max))")
    end
    xf = xs[end]
    println("final: t=$(round(cl.t[end]; digits=2)) s, s=$(round(xf[1]; digits=3)) m, ey=$(round(xf[2]; digits=4)) m, ",
        "epsi=$(round(xf[3]; digits=4)) rad, v=$(round(xf[4]; digits=4)) m/s")
    is_closed(ctrl.track) && println("laps completed: $(round(xf[1] / track_length(ctrl.track); digits=2))")
    println("plant operating-constraint violations (tol $(ctrl.config.acceptance_tol)): $(length(cl.violations))")
end

"Write the closed loop (one row per control period) to CSV."
function write_closed_loop_csv(path, cl::ClosedLoopResult)
    mkpath(dirname(path))
    open(path, "w") do io
        println(io, "t,s,ey,epsi,v,X,Y,psi,delta,a,status,iterations,solve_ms,warm_started")
        for k in eachindex(cl.states)
            x, p = cl.states[k], cl.poses[k]
            u = k <= length(cl.inputs) ? cl.inputs[k] : (NaN, NaN)
            r = k <= length(cl.controller) ? cl.controller[k] : nothing
            meta = r === nothing ? ("", "", "", "") : (r.status, r.iterations, 1e3 * r.timing.total, r.warm_started)
            println(io, join((cl.t[k], x..., p..., u..., meta...), ","))
        end
    end
    return path
end
