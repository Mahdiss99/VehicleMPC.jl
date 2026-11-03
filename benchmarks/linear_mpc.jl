# Milestone 4A: linearized vehicle MPC (JuMP + OSQP) on a straight track.
#
#     julia --project=. benchmarks/linear_mpc.jl [n_cycles]
#
# Writes benchmarks/results/linear_mpc.md. Reference vehicle, straight track (400 m,
# width 1.0 m), dt = 0.1 s, closed loop with the zero-order-hold RK4 plant
# (simulate_closed_loop). Scenarios:
#   A nominal progression from 1 m/s on the centreline
#   B lateral and heading error recovery (ey = 0.35 m, epsi = −0.25 rad, 2 m/s)
#   C near the speed limit (4.9 m/s, ey = 0.1 m)
# Experiments:
#   D cold vs warm start (scenarios A–C, N = 10)
#   E N = 10 and N = 20
#   F Cartesian-endpoint Ipopt NMPC with the same settings, with every accepted plan passed
#     through the same nonlinear validation (validate_plan)
#   plus the :rebuild update mode (a new JuMP model per QP) to measure the update overhead,
#   the warm start with a repeated (instead of pure-pursuit) tail input, the warm start
#   without an explicit OSQP primal start,
#   and the objective of the linear MPC plan vs the NMPC plan at sampled closed-loop states.
# One closed-loop run per configuration; all cycles count. The first controller of the
# process is built and solved separately to report compilation-inclusive timing.

using VehicleMPC
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const P = VEH.params
const T = straight_track(400.0; width=1.0)
const N_CYCLES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 250
const LIM = (; dt=0.1, track_margin=0.05, epsi_max=deg2rad(60), ay_max=4.0)
const SCEN = [("A nominal progression", [0.0, 0.0, 0.0, 1.0]), ("B lateral/heading recovery", [0.0, 0.35, -0.25, 2.0]),
    ("C near the speed limit", [0.0, 0.1, 0.05, 4.9])]

quant(xs, q) = isempty(xs) ? NaN : (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))
ms(x) = 1e3 * x
mean(x) = isempty(x) ? NaN : sum(x) / length(x)
f1(x) = @sprintf("%.1f", x)
f2(x) = @sprintf("%.2f", x)
f3(x) = @sprintf("%.3g", x)

lmpc(N; kw...) = LinearMPCController(VEH, T, LinearMPCConfig(; horizon=N, dt=0.1, kw...))
nmpc(N) = NMPCController(VEH, T, NMPCConfig(; prediction_model=:cartesian_endpoint_exact, horizon=N, dt=0.1))

"Closed-loop summary; `validate` re-validates every accepted plan (used for the NMPC)."
function summarize(name, ctrl, cl; validate=false)
    rs = cl.controller
    acc = filter(r -> r.accepted, rs)
    tt = [r.timing.total for r in rs]
    ey = [x[2] for x in cl.states]
    ep = [x[3] for x in cl.states]
    ay = [abs(VEH.params.wheelbase == 0 ? 0.0 : lateral_acceleration(VEH, x[4], u[1])) for (x, u) in zip(cl.states, cl.inputs)]
    nviol = 0
    if validate
        for (k, r) in enumerate(rs)
            r.accepted || continue
            v = validate_plan(VEH, T, LIM, collect(frenet_state_to_cartesian(T, cl.states[k])), r.U; substeps=10, tol=1e-4)
            nviol += !isempty(v.violations)
        end
    end
    lin = rs isa Vector{LinearMPCResult}
    return (; name, status=cl.status, message=cl.message, cycles=length(rs), accepted=length(acc), rejected=length(rs) - length(acc),
        med=ms(quant(tt, 0.5)), p95=ms(quant(tt, 0.95)), p99=ms(quant(tt, 0.99)), max=ms(maximum(tt)),
        over80=count(>(0.08), tt), over100=count(>(0.1), tt),
        solves_mean=lin ? mean([r.qp_solves for r in rs]) : mean([r.nlp_solves for r in rs]),
        solves_max=lin ? maximum(r.qp_solves for r in rs) : maximum(r.nlp_solves for r in rs),
        solves_hist=lin ? [count(r -> r.qp_solves == j, rs) for j in 1:5] : Int[],
        iters_med=lin ? quant(Float64[i for r in rs for i in r.qp_iterations], 0.5) : quant(Float64[r.iterations for r in rs], 0.5),
        iters_max=lin ? maximum(i for r in rs for i in r.qp_iterations) : maximum(r.iterations for r in rs),
        upd_med=lin ? ms(quant([r.timing.update + r.timing.linearize for r in rs], 0.5)) : ms(quant([r.timing.update for r in rs], 0.5)),
        solve_med=lin ? ms(quant([r.timing.solve for r in rs], 0.5)) : ms(quant([r.timing.solver for r in rs], 0.5)),
        val_med=lin ? ms(quant([r.timing.validate for r in rs], 0.5)) : ms(quant([r.timing.validation for r in rs], 0.5)),
        mis_max=lin && !isempty(acc) ? maximum(maximum(r.mismatch ./ (1e-3, 1e-3, 1e-3)) for r in acc) : NaN,
        plan_viol=lin ? count(r -> !isempty(r.violations) && r.accepted, rs) : nviol,
        plant_viol=length(cl.violations), max_ey=maximum(abs, ey), rms_ey=sqrt(mean(ey .^ 2)), rms_ep=sqrt(mean(ep .^ 2)),
        final_ey=ey[end], final_v=cl.states[end][4], s_end=cl.states[end][1], max_ay=isempty(ay) ? NaN : maximum(ay),
        max_v=maximum(x[4] for x in cl.states))
end

run(name, ctrl, x0; validate=false) = summarize(name, ctrl, simulate_closed_loop(ctrl, VEH, x0, N_CYCLES); validate)

# ---- first build / first solve (compilation included), then warm timing ----
t_first_build = @elapsed (c0 = lmpc(10))
t_first_solve = @elapsed solve!(c0, SCEN[1][2])
t_warm_build = @elapsed (c1 = lmpc(10))
r_cold = solve!(c1, SCEN[1][2])
t_nmpc_first_build = @elapsed (n0 = nmpc(10))
t_nmpc_first_solve = @elapsed solve!(n0, SCEN[1][2])
builds10 = [lmpc(10).build_time for _ in 1:5]
builds20 = [lmpc(20).build_time for _ in 1:5]

rows = []
for (sname, x0) in SCEN, N in (10, 20)
    push!(rows, ("LMPC warm", N, sname, run(sname, lmpc(N), x0)))
    push!(rows, ("LMPC cold", N, sname, run(sname, lmpc(N; warm_start=false), x0)))
    if N == 10
        push!(rows, ("LMPC rebuild", N, sname, run(sname, lmpc(N; update_mode=:rebuild), x0)))
        push!(rows, ("LMPC warm, repeated tail", N, sname, run(sname, lmpc(N; warm_tail=:repeat), x0)))
        push!(rows, ("LMPC warm, no primal start", N, sname, run(sname, lmpc(N; primal_start=:none), x0)))
    end
    push!(rows, ("NMPC Ipopt", N, sname, run(sname, nmpc(N), x0; validate=true)))
    println("done ", sname, " N=", N)
end

# ---- objective of the plans at sampled closed-loop states (N = 10) ----
objrows = []
for (sname, x0) in SCEN[1:2]
    cl = simulate_closed_loop(lmpc(10), VEH, x0, 100)
    for k in (1, 5, 10, 25, 50, 100)
        k <= length(cl.states) || continue
        x, up = cl.states[k], k == 1 ? (0.0, 0.0) : Tuple(cl.inputs[k-1])
        rl = solve!(lmpc(10), x; previous_input=up)
        rn = solve!(nmpc(10), x; previous_input=up)
        push!(objrows, (sname, k, rl.status, rl.nonlinear_objective, rl.qp_solves, rn.status, rn.objective,
            rl.accepted && rn.accepted ? maximum(abs.(rl.u - rn.u)) : NaN))
    end
end

io = IOBuffer()
println(io, "# Linearized vehicle MPC (JuMP + OSQP), Milestone 4A\n")
println(io, "Generated by `benchmarks/linear_mpc.jl` ($(N_CYCLES) cycles per run; Julia $(VERSION); $(Sys.CPU_NAME), $(Sys.CPU_THREADS) threads).")
println(io, "Straight track 400 m, width 1.0 m, reference vehicle, dt = 0.1 s, zero-order-hold RK4 plant. One run per configuration; timing varies between runs and machines (≈2× on this machine). No real-time guarantee is implied.\n")
println(io, "LMPC: `LinearMPCConfig` defaults (mismatch tolerance 1e-3 m / rad / m/s, ≤ 5 QPs, 80 ms SQP budget, OSQP eps 1e-6 with polishing). NMPC: `NMPCConfig(prediction_model = :cartesian_endpoint_exact)` with the default objective weights (identical to the LMPC), Ipopt.\n")
println(io, "## Build and first-solve timing\n")
println(io, "| | time |\n|---|---:|")
println(io, "| first LMPC build in the process (incl. compilation) | $(f1(ms(t_first_build))) ms |")
println(io, "| first LMPC solve (incl. compilation) | $(f1(ms(t_first_solve))) ms |")
println(io, "| second LMPC build (N = 10) | $(f1(ms(t_warm_build))) ms |")
println(io, "| LMPC build N = 10, median of 5 | $(f1(ms(quant(builds10, 0.5)))) ms |")
println(io, "| LMPC build N = 20, median of 5 | $(f1(ms(quant(builds20, 0.5)))) ms |")
println(io, "| cold first cycle of a built LMPC (scenario A) | $(f1(ms(r_cold.timing.total))) ms, $(r_cold.qp_solves) QP(s) |")
println(io, "| first NMPC build / first solve (incl. compilation) | $(f1(ms(t_nmpc_first_build))) / $(f1(ms(t_nmpc_first_solve))) ms |\n")
println(io, "## Closed-loop results\n")
println(io, "Timing is the full control cycle (`timing.total`: nominal trajectory, linearisation, coefficient updates, QP solves, validation for the LMPC; the whole `solve!` for the NMPC). `solves`: QP solves (LMPC) or NLP solves (NMPC) per cycle. `upd`: median per-cycle linearisation + coefficient-update time (LMPC; for `rebuild` it includes building the models) or the NMPC's parameter update. `plan viol`: accepted plans that fail the independent nonlinear validation. `plant viol`: plant operating-constraint violations.\n")
println(io, "| controller | N | scenario | status | cycles | acc/rej | median | p95 | p99 | max [ms] | >80 / >100 ms | solves mean/max | iters med/max | upd | solve | validate [ms] | plan viol | plant viol |")
println(io, "|---|---:|---|---|---:|---|---:|---:|---:|---:|---|---|---|---:|---:|---:|---:|---:|")
for (cn, N, sn, r) in rows
    println(io, "| $cn | $N | $sn | $(r.status) | $(r.cycles) | $(r.accepted)/$(r.rejected) | $(f1(r.med)) | $(f1(r.p95)) | $(f1(r.p99)) | $(f1(r.max)) | $(r.over80) / $(r.over100) | $(f2(r.solves_mean)) / $(r.solves_max) | $(r.iters_med) / $(r.iters_max) | $(f2(r.upd_med)) | $(f2(r.solve_med)) | $(f2(r.val_med)) | $(r.plan_viol) | $(r.plant_viol) |")
end
println(io, "\n### Tracking\n")
println(io, "| controller | N | scenario | max abs ey [m] | RMS ey [m] | RMS epsi [rad] | final ey [m] | final v [m/s] | max v [m/s] | max abs ay [m/s²] | s at end [m] |")
println(io, "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|")
for (cn, N, sn, r) in rows
    println(io, "| $cn | $N | $sn | $(f3(r.max_ey)) | $(f3(r.rms_ey)) | $(f3(r.rms_ep)) | $(f3(r.final_ey)) | $(f3(r.final_v)) | $(f3(r.max_v)) | $(f3(r.max_ay)) | $(f1(r.s_end)) |")
end
println(io, "\n### QP solves per cycle (LMPC)\n")
println(io, "| controller | N | scenario | 1 | 2 | 3 | 4 | 5 | max accepted mismatch / tolerance |")
println(io, "|---|---:|---|---:|---:|---:|---:|---:|---:|")
for (cn, N, sn, r) in rows
    isempty(r.solves_hist) && continue
    println(io, "| $cn | $N | $sn | " * join(r.solves_hist, " | ") * " | $(f3(r.mis_max)) |")
end
failed = [(cn, N, sn, r) for (cn, N, sn, r) in rows if r.status !== :success]
if !isempty(failed)
    println(io, "\n### Failures\n")
    for (cn, N, sn, r) in failed
        println(io, "- $cn, N = $N, $sn: $(r.message)")
    end
end
println(io, "\n## Plan objective vs the Ipopt NMPC at LMPC closed-loop states (N = 10)\n")
println(io, "Both controllers solve from the same state and previous input with fresh (cold) controllers. LMPC objective: the MPC objective evaluated on the exact nonlinear rollout of its inputs. NMPC objective: Ipopt's objective (the same function on its exact-map prediction).\n")
println(io, "| scenario | cycle | LMPC status | LMPC objective | QPs | NMPC status | NMPC objective | difference | max abs Δu |")
println(io, "|---|---:|---|---:|---:|---|---:|---:|---:|")
for (sn, k, ls, lo, nq, ns, no, du) in objrows
    println(io, "| $sn | $k | $ls | $(@sprintf("%.6f", lo)) | $nq | $ns | $(@sprintf("%.6f", no)) | $(f3(lo - no)) | $(f3(du)) |")
end
out = joinpath(@__DIR__, "results", "linear_mpc.md")
write(out, String(take!(io)))
println("wrote ", out)
