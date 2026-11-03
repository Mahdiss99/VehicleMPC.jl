# Milestone 4BC: linearized MPC (JuMP + OSQP) on the multi-curvature quarter track with a
# stationary circular obstacle, compared with the Cartesian-endpoint Ipopt NMPC.
#
#     julia --project=. benchmarks/linear_multicurvature_obstacle.jl [n_cycles] [reps]          # tables
#     julia --project=examples benchmarks/linear_multicurvature_obstacle.jl [n_cycles] [reps]   # tables + figures
#
# Writes benchmarks/results/linear_multicurvature_obstacle.md (and
# docs/figures/linear_multicurvature_*.png).
#
# Common settings: reference vehicle, N = 10 (N = 20 in stage D), dt = 0.1 s, 5 cm margin,
# NMPC objective weights, obstacle radius 0.10 m, ego radius 0.05 m, buffer 0.03 m (the
# Milestone 3A/3B dimensions), event-aware plant (10 substeps per period), start
# [0, 0, 0, 0.5].
#
# Stages:
#   A quarter track without an obstacle
#   B the Milestone 3A straight-track obstacle at (8, 0). The track is 140 m long instead
#     of 80 m so that 250 cycles fit; the obstacle geometry is unchanged
#   C the Milestone 3B quarter-track scenarios (unchanged positions)
#   D N = 20 for the configurations that succeed at N = 10
#
# LMPC: LinearMPCConfig defaults, interior obstacle samples M = 3 (and M = 0), REPS runs of
# the primary configurations. NMPC: prediction_model = :cartesian_endpoint_exact, up to 8
# reassignments within the 80 ms correction budget, M = 3, with the Milestone 3B.1 cycle
# detection, branch resolution and sequential encounters, one run each. Every accepted NMPC
# plan is passed through the same validation as the LMPC plans (validate_plan_on_track).
# All cycles count, after a warm-up of every code path in the same process.

using VehicleMPC
using VehicleMPC.StaticArrays: SVector
using Printf

const VEH = KinematicBicycle(reference_test_vehicle())
const P = VEH.params
const N_CYCLES = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 250
const REPS = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 3
const Q = quarter_track()
const LQ = track_length(Q)
const TRS = straight_track(140.0; width=0.7)
const X0 = [0.0, 0.0, 0.0, 0.5]
const EGO, BUF, ROB = 0.05, 0.03, 0.10
const D_REQ, D_CON = round(EGO + ROB + BUF; digits=10), round(EGO + ROB; digits=10)
const LIM = (; dt=0.1, track_margin=0.05, epsi_max=deg2rad(60), ay_max=4.0)
const TOL = 1e-6

quant(xs, q) = isempty(xs) ? NaN : (ys = sort(xs); h = (length(ys) - 1) * q + 1; lo = floor(Int, h); ys[lo] + (h - lo) * (ys[ceil(Int, h)] - ys[lo]))
mean(x) = isempty(x) ? NaN : sum(x) / length(x)
nanmin(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : minimum(w))
nanmax(v) = (w = filter(!isnan, collect(v)); isempty(w) ? NaN : maximum(w))
ms(x) = 1e3 * x

lmpc(tr, ob; N=10, kw...) = LinearMPCController(VEH, tr, LinearMPCConfig(; horizon=N, dt=0.1, ego_radius=EGO, obstacle_buffer=BUF, kw...); obstacle=ob)
nmpc_cfg(N, M) = NMPCConfig(; prediction_model=:cartesian_endpoint_exact, horizon=N, dt=0.1, track_margin=0.05, schedule_corrections=8,
    interior_samples=0, ego_radius=EGO, obstacle_buffer=BUF, obstacle_samples=M, reassignment_cycle_detection=true, branch_resolution=true,
    encounter_strategy=:sequential)
function nmpc(tr, ob; N=10, M=3)
    c = ob === nothing ? NMPCController(VEH, tr, nmpc_cfg(N, M)) : NMPCController(VEH, tr, nmpc_cfg(N, M); obstacle=ob)
    solve!(c, X0)                                         # prime the NLP evaluator, then forget the cycle
    reset_warm_start!(c)
    ob === nothing || set_obstacle!(c, ob)
    return c
end

"Closed loop with the event-aware plant, per-cycle @timed allocation and GC statistics."
function closed_loop(ctrl, tr, n)
    h = 0.01
    x, uprev = SVector{4,Float64}(X0...), SVector(0.0, 0.0)
    states, results, inputs, alloc, gct, fine, tf = [x], Any[], SVector{2,Float64}[], Float64[], Float64[], [x], [0.0]
    status, msg = :success, ""
    for k in 1:n
        tm = @timed solve!(ctrl, x; previous_input=uprev)
        r = tm.value
        push!(results, r); push!(alloc, tm.bytes / 1e6); push!(gct, ms(tm.gctime))
        if !r.accepted
            cause = ctrl isa LinearMPCController ? r.obstacle.failure_cause : r.obstacle.failure_cause
            status, msg = :controller_failure, "cycle $k (t = $(round((k - 1) * 0.1; digits=1)) s): $(r.status) [cause $cause]: $(first(r.message, 400))"
            break
        end
        for j in 1:10
            st = integrate_on_track(VEH, tr, x, r.u, h; max_step=h)
            if !succeeded(st)
                status, msg = :plant_stopped, "cycle $k: $(st.status)"
                break
            end
            x = st.x
            push!(fine, x); push!(tf, (k - 1) * 0.1 + j * h)
        end
        status === :success || break
        push!(states, x); push!(inputs, r.u)
        uprev = r.u
    end
    return (; states, results, inputs, alloc, gct, fine, tf, status, msg)
end

"Plan of an NMPC result as a Cartesian 4×(N+1) matrix (exact-map knots)."
nmpc_cartesian(tr, X) = reduce(hcat, [collect(frenet_state_to_cartesian(tr, X[:, k])) for k in axes(X, 2)])

function runstats(ctrl, cl, tr, ob)
    rs = cl.results
    lin = ctrl isa LinearMPCController
    acc = [k for (k, r) in enumerate(rs) if r.accepted]
    tot = [ms(r.timing.total) for r in rs]
    ey = [x[2] for x in cl.fine]
    poses = [frenet_to_cartesian(tr, x[1], x[2], x[3]) for x in cl.fine]
    dmin = ob === nothing ? NaN : minimum(hypot(q[1] - ob.x, q[2] - ob.y) for q in poses)
    # Prediction errors: first step (plan knot 2 vs the plant one period later) and over the
    # horizon (plan knots vs the exact Cartesian rollout of the plan's inputs).
    fs, hz, nval, nlobj = Float64[], Float64[], 0, Float64[]
    alphas = lin ? ctrl.obs_alphas : [j / (ctrl.config.obstacle_samples + 1) for j in 1:ctrl.config.obstacle_samples]
    for k in acc
        r = rs[k]
        z0 = collect(frenet_state_to_cartesian(tr, cl.states[k]))
        Zp = lin ? r.Z : nmpc_cartesian(tr, r.X)
        if k + 1 <= length(cl.states)
            zn = frenet_state_to_cartesian(tr, cl.states[k+1])
            push!(fs, hypot(Zp[1, 2] - zn[1], Zp[2, 2] - zn[2]))
        end
        Zr = exact_rollout(VEH, z0, r.U, 0.1)
        push!(hz, maximum(hypot(Zp[1, j] - Zr[1, j], Zp[2, j] - Zr[2, j]) for j in axes(Zr, 2)))
        if !lin
            v = validate_plan_on_track(VEH, tr, LIM, z0, r.U; s0=cl.states[k][1], obstacle=ob, D=D_REQ, Dc=D_CON, alphas)
            nval += !isempty(v.violations)
            push!(nlobj, plan_objective_on_track(ctrl.config, VEH, ctrl.ey_max, v.frenet, r.U, k == 1 ? (0.0, 0.0) : cl.inputs[k-1];
                horizon=size(r.U, 2), dt=0.1))
        else
            nval += !isempty(r.violations)
            push!(nlobj, r.nonlinear_objective)
        end
    end
    s_end = cl.states[end][1]
    lap1 = findfirst(x -> x[1] >= LQ, cl.states)
    t40 = findfirst(x -> x[1] >= 40.0, cl.states)
    br(f) = lin ? [ms(getfield(r.timing, f)) for r in rs] : Float64[]
    enc = [k for (k, r) in enumerate(rs) if lin ? any(a -> a[1] in (:left, :right), r.obstacle.attempts) :
                                             any(a -> a[1] in (:left, :right), r.obstacle.attempts)]
    return (; status=cl.status, msg=cl.msg, cycles=length(rs), accepted=length(acc), laps=(s_end - X0[1]) / LQ, s_end,
        t_lap1=lap1 === nothing ? NaN : 0.1 * (lap1 - 1), t_40=t40 === nothing ? NaN : 0.1 * (t40 - 1),
        rms_ey=sqrt(mean(ey .^ 2)), max_ey=maximum(abs, ey), track_clear=tr.width / 2 - maximum(abs, ey),
        contact_clear=dmin - D_CON, buffer_clear=dmin - D_REQ,
        plant_viol=count(x -> !isempty(check_operating_constraints(VEH, tr, x; tol=TOL)), cl.fine),
        first_step=fs, horizon_err=hz, val_fail=nval, nlobj,
        solves=[lin ? r.qp_solves : r.nlp_solves for r in rs],
        iters=lin ? [i for r in rs for i in r.qp_iterations] : [r.iterations for r in rs], tot,
        solver_fail=lin ? count(r -> any(s -> !(s in (:OPTIMAL,)), r.qp_status), rs) : count(r -> r.status in (:solver_failure,), rs),
        nominal=br(:nominal), reference=br(:reference), linearize=br(:linearize), update=br(:update), solve=br(:solve),
        validate=br(:validate), guesses=br(:guesses), gct=cl.gct, alloc=cl.alloc, enc, enc_tot=[tot[k] for k in enc],
        cause=isempty(rs) || rs[end].accepted ? :none : rs[end].obstacle.failure_cause, cl)
end

function run_config(make, tr, ob, n)
    ctrl = make()
    cl = closed_loop(ctrl, tr, n)
    return runstats(ctrl, cl, tr, ob)
end

mkob(spec) = spec === nothing ? nothing : obstacle_on_track(Q, spec[1], spec[2]; radius=ROB)
const SCEN = [("B R 1 m left bend, s = 3.5", (3.5, 0.0)), ("C R 1 m right bend (S-bend), s = 6.6", (6.6, 0.0)),
    ("D R 2 m bend, s = 9.7", (9.7, 0.0)), ("E1 before straight→R 1 m, s = 1.95", (1.95, 0.0)),
    ("E2 after straight→R 1 m, s = 2.10", (2.10, 0.0)), ("E3 before R −1→R 2 m, s = 8.20", (8.20, 0.0)),
    ("E4 after R −1→R 2 m, s = 8.40", (8.40, 0.0)), ("F lap boundary, s = L − 0.3", (LQ - 0.3, 0.0)),
    ("G1 offset +0.08 on R 1 m left", (3.5, 0.08)), ("G2 offset −0.08 on R 1 m right", (6.6, -0.08))]
const OB_STRAIGHT = CircleObstacle(x=8.0, y=0.0, radius=ROB)

# ---- first build / first solve (compilation included) ----
t_first_build = @elapsed (c_first = lmpc(Q, mkob((3.5, 0.0))))
t_first_solve = @elapsed solve!(c_first, X0)
t_nmpc_build = @elapsed (n_first = NMPCController(VEH, Q, nmpc_cfg(10, 3); obstacle=mkob((3.5, 0.0))))
t_nmpc_solve = @elapsed solve!(n_first, X0)
println("first builds done")

# ---- warm-up of every code path (discarded) ----
for (tr, ob) in ((Q, nothing), (Q, mkob((3.5, 0.0))), (Q, mkob((6.6, 0.0))), (TRS, OB_STRAIGHT))
    closed_loop(lmpc(tr, ob), tr, 80)
    closed_loop(lmpc(tr, ob; obstacle_samples=0), tr, 30)
    closed_loop(lmpc(tr, ob; N=20), tr, 30)
end
closed_loop(nmpc(Q, mkob((3.5, 0.0))), Q, 60)
closed_loop(nmpc(TRS, OB_STRAIGHT), TRS, 60)
builds = (; q10=[@elapsed(lmpc(Q, nothing)) for _ in 1:5], q10o=[@elapsed(lmpc(Q, mkob((3.5, 0.0)))) for _ in 1:5],
    q20o=[@elapsed(lmpc(Q, mkob((3.5, 0.0)); N=20)) for _ in 1:5], n10o=[@elapsed(NMPCController(VEH, Q, nmpc_cfg(10, 3); obstacle=mkob((3.5, 0.0)))) for _ in 1:3])
println("warm-up done")

results = Dict{String,Vector{Any}}()
order = String[]
add!(name, stats) = (haskey(results, name) || (push!(order, name); results[name] = Any[]); push!(results[name], stats))

# Stage A: quarter track without an obstacle.
for rep in 1:REPS
    add!("A LMPC N=10 · no obstacle", run_config(() -> lmpc(Q, nothing), Q, nothing, N_CYCLES))
end
add!("A NMPC N=10 · no obstacle", run_config(() -> nmpc(Q, nothing), Q, nothing, N_CYCLES))
println("stage A done")
# Stage B: straight-track obstacle (Milestone 3A geometry).
for rep in 1:REPS
    add!("B LMPC N=10 M=3 · straight, (8, 0)", run_config(() -> lmpc(TRS, OB_STRAIGHT), TRS, OB_STRAIGHT, N_CYCLES))
end
add!("B LMPC N=10 M=0 · straight, (8, 0)", run_config(() -> lmpc(TRS, OB_STRAIGHT; obstacle_samples=0), TRS, OB_STRAIGHT, N_CYCLES))
add!("B NMPC N=10 M=3 · straight, (8, 0)", run_config(() -> nmpc(TRS, OB_STRAIGHT), TRS, OB_STRAIGHT, N_CYCLES))
add!("B NMPC N=10 M=0 · straight, (8, 0)", run_config(() -> nmpc(TRS, OB_STRAIGHT; M=0), TRS, OB_STRAIGHT, N_CYCLES))
println("stage B done")
# Stage C: quarter-track obstacle scenarios.
for (name, spec) in SCEN
    ob = mkob(spec)
    for rep in 1:REPS
        add!("C LMPC N=10 M=3 · $name", run_config(() -> lmpc(Q, ob), Q, ob, N_CYCLES))
    end
    add!("C LMPC N=10 M=0 · $name", run_config(() -> lmpc(Q, ob; obstacle_samples=0), Q, ob, N_CYCLES))
    add!("C NMPC N=10 M=3 · $name", run_config(() -> nmpc(Q, ob), Q, ob, N_CYCLES))
    println("stage C: $name done")
end
# Stage D: N = 20 for the configurations that succeeded at N = 10.
ok10(prefix) = all(r -> r.status === :success, results[prefix])
ok10("A LMPC N=10 · no obstacle") && add!("D LMPC N=20 · no obstacle", run_config(() -> lmpc(Q, nothing; N=20), Q, nothing, N_CYCLES))
ok10("B LMPC N=10 M=3 · straight, (8, 0)") && add!("D LMPC N=20 M=3 · straight, (8, 0)", run_config(() -> lmpc(TRS, OB_STRAIGHT; N=20), TRS, OB_STRAIGHT, N_CYCLES))
for (name, spec) in SCEN
    ok10("C LMPC N=10 M=3 · $name") || continue
    ob = mkob(spec)
    add!("D LMPC N=20 M=3 · $name", run_config(() -> lmpc(Q, ob; N=20), Q, ob, N_CYCLES))
end
println("stage D done")

# Objective of both controllers' plans at the same states (fresh controllers, the original
# objective on the exact rollout of each plan's inputs, true track coordinates).
objrows = []
for (label, tr, ob, key) in (("A no obstacle", Q, nothing, "A LMPC N=10 · no obstacle"), ("B straight, (8, 0)", TRS, OB_STRAIGHT, "B LMPC N=10 M=3 · straight, (8, 0)"),
    ("C R 1 m left, s = 3.5", Q, mkob((3.5, 0.0)), "C LMPC N=10 M=3 · B R 1 m left bend, s = 3.5"))
    cl = results[key][1].cl
    for k in (5, 15, 30, 60, 120)
        k + 1 <= length(cl.states) || continue
        x, up = cl.states[k], k == 1 ? (0.0, 0.0) : Tuple(cl.inputs[k-1])
        rl = solve!(lmpc(tr, ob), x; previous_input=up)
        rloop = results[key][1].cl.results[k]                # the plan the LMPC used in closed loop (warm)
        nc = nmpc(tr, ob)
        rn = solve!(nc, x; previous_input=up)
        z0 = collect(frenet_state_to_cartesian(tr, x))
        vn = rn.accepted ? validate_plan_on_track(VEH, tr, LIM, z0, rn.U; s0=x[1], obstacle=ob, D=D_REQ, Dc=D_CON, alphas=[0.25, 0.5, 0.75]) : nothing
        jn = vn === nothing ? NaN : plan_objective_on_track(nc.config, VEH, nc.ey_max, vn.frenet, rn.U, up; horizon=10, dt=0.1)
        push!(objrows, (label, k, rloop.nonlinear_objective, rl.status, rl.nonlinear_objective, rl.qp_solves, ms(rl.timing.total), rn.status, jn,
            rn.nlp_solves, ms(rn.timing.total), vn === nothing ? -1 : length(vn.violations), rl.accepted && rn.accepted ? maximum(abs.(rl.u - rn.u)) : NaN))
    end
end
println("objective comparison done")

# ---- report ----
f1(x) = isnan(x) ? "–" : @sprintf("%.1f", x)
f2(x) = isnan(x) ? "–" : @sprintf("%.2f", x)
f4(x) = isnan(x) ? "–" : @sprintf("%.4f", x)
g3(x) = isnan(x) ? "–" : @sprintf("%.3g", x)
allv(runs, f) = reduce(vcat, [getfield(r, f) for r in runs]; init=Float64[])
statusstr(r) = r.status === :success ? @sprintf("%.2f laps", r.laps) : @sprintf("stop c%d (%.2f laps)", r.cycles, r.laps)
io = IOBuffer()
println(io, "# Linearized MPC on the quarter track with a stationary obstacle (Milestone 4BC)\n")
println(io, "Generated by `benchmarks/linear_multicurvature_obstacle.jl` ($(N_CYCLES) cycles per run, $(REPS) runs of the primary LMPC configurations, one NMPC run; Julia $(VERSION); $(Sys.CPU_NAME), $(Sys.CPU_THREADS) threads, one Windows laptop). Timing varies between runs and machines (≈2× observed on this machine). No real-time guarantee is implied.\n")
println(io, "Common settings: reference vehicle, dt = 0.1 s, 5 cm margin, NMPC objective weights, obstacle radius $(ROB) m, ego radius $(EGO) m, buffer $(BUF) m (D = $(D_REQ) m, contact at $(D_CON) m), event-aware plant, start $(X0). LMPC: `LinearMPCConfig` defaults (local track model, M = 3 interior halfspace samples unless stated). NMPC: Cartesian-endpoint model, ≤ 8 reassignments in the 80 ms correction budget, M = 3, cycle detection + branch resolution + sequential encounters (Milestone 3B.1). Every accepted NMPC plan is re-validated with the same `validate_plan_on_track` as the LMPC plans. Stage B uses a 140 m straight track (instead of 80 m) so that 250 cycles fit; the obstacle geometry is the Milestone 3A one.\n")
println(io, "## Build and first-solve timing\n")
println(io, "| | time |\n|---|---:|")
println(io, "| first LMPC build in the process (incl. compilation) / first solve | $(f1(ms(t_first_build))) / $(f1(ms(t_first_solve))) ms |")
println(io, "| first NMPC build / first solve (incl. compilation) | $(f1(ms(t_nmpc_build))) / $(f1(ms(t_nmpc_solve))) ms |")
println(io, "| LMPC build after warm-up, quarter, no obstacle, N = 10 (median of 5) | $(f1(ms(quant(builds.q10, 0.5)))) ms |")
println(io, "| LMPC build after warm-up, quarter, obstacle M = 3, N = 10 / N = 20 | $(f1(ms(quant(builds.q10o, 0.5)))) / $(f1(ms(quant(builds.q20o, 0.5)))) ms |")
println(io, "| NMPC build after warm-up, quarter, obstacle (median of 3) | $(f1(ms(quant(builds.n10o, 0.5)))) ms |\n")
println(io, "## Outcomes and tracking\n")
println(io, "`laps`: progress / track length (quarter track; for stage B the distance in m is in `s end`). `lap 1 [s]`: time to complete the first lap (quarter) or to reach s = 40 m (straight). Clearances are measured on the plant (every 0.01 s). `contact`/`buffer`: minimum centre distance minus $(D_CON) m / minus D = $(D_REQ) m. `val. fail`: accepted plans that fail the independent nonlinear validation. `1st-step err`: plan knot 2 vs the plant one period later [mm]; `horizon err`: plan knots vs the exact rollout of its inputs [mm].\n")
println(io, "| configuration | outcome per run | s end [m] | lap 1 [s] | RMS / max abs ey [m] | min track clearance [m] | min contact / buffer clearance [m] | plant viol. | val. fail | 1st-step err mean / max [mm] | horizon err max [mm] | failure cause |")
println(io, "|:--|:--|--:|--:|:-:|--:|:-:|--:|--:|:-:|--:|:--|")
for name in order
    runs = results[name]
    r1 = runs[1]
    fs, hz = allv(runs, :first_step), allv(runs, :horizon_err)
    causes = unique([r.cause for r in runs if r.status !== :success])
    @printf(io, "| %s | %s | %.1f | %s | %.4f / %.3f | %.4f | %s / %s | %d | %d | %s / %s | %s | %s |\n", name, join(statusstr.(runs), "; "),
        mean([r.s_end for r in runs]), f1(occursin("straight", name) ? r1.t_40 : r1.t_lap1), mean([r.rms_ey for r in runs]), maximum(r.max_ey for r in runs),
        minimum(r.track_clear for r in runs), f4(nanmin(r.contact_clear for r in runs)), f4(nanmin(r.buffer_clear for r in runs)),
        sum(r.plant_viol for r in runs), sum(r.val_fail for r in runs), g3(1e3 * mean(fs)), g3(1e3 * nanmax(fs)), g3(1e3 * nanmax(hz)),
        isempty(causes) ? "–" : join(string.(causes), ", "))
end
println(io, "\n## Computation time (full controller cycle)\n")
println(io, "`timing.total` of every cycle of every run: initialisation, track projections, linearisation, coefficient updates, all QP (or NLP) solves, alternative passing-side attempts, rollouts and validation. `solves`: QP solves (LMPC) or NLP solves (NMPC) per cycle. `iters`: OSQP iterations per QP / Ipopt iterations per cycle. GC: total garbage-collection time in the run(s) and the largest single-cycle GC pause.\n")
println(io, "| configuration | median / p95 / p99 / max [ms] | > 80 / > 100 ms | solves mean / max | iters median / max | encounter cycles: n, median / max [ms] | GC total / max per cycle [ms] | alloc. per cycle median [MB] |")
println(io, "|:--|:-:|:-:|:-:|:-:|:-:|:-:|--:|")
for name in order
    runs = results[name]
    tot, sol, it, et, gc, al = allv(runs, :tot), allv(runs, :solves), allv(runs, :iters), allv(runs, :enc_tot), allv(runs, :gct), allv(runs, :alloc)
    @printf(io, "| %s | %s / %s / %s / %s | %d / %d | %.2f / %d | %s / %d | %d, %s | %s / %s | %.2f |\n", name, f1(quant(tot, 0.5)), f1(quant(tot, 0.95)),
        f1(quant(tot, 0.99)), f1(maximum(tot)), count(>(80), tot), count(>(100), tot), mean(sol), maximum(sol), g3(quant(it, 0.5)), maximum(it),
        length(et), isempty(et) ? "–" : @sprintf("%.1f / %.1f", quant(et, 0.5), maximum(et)), f1(sum(gc)), f1(maximum(gc)), quant(al, 0.5))
end
println(io, "\n### LMPC time breakdown (median per cycle [ms])\n")
println(io, "| configuration | nominal | track references | linearisation | JuMP/OSQP updates | OSQP solves | rollout + validation | passing guesses | total |")
println(io, "|:--|--:|--:|--:|--:|--:|--:|--:|--:|")
for name in order
    occursin("LMPC", name) || continue
    runs = results[name]
    v(f) = f2(quant(allv(runs, f), 0.5))
    println(io, "| $name | $(v(:nominal)) | $(v(:reference)) | $(v(:linearize)) | $(v(:update)) | $(v(:solve)) | $(v(:validate)) | $(v(:guesses)) | $(v(:tot)) |")
end
println(io, "\n## Plan objective at the same states\n")
println(io, "At states of the LMPC closed loop (run 1), with the same previous input. The objective is the original NMPC objective evaluated on the exact rollout of each plan's inputs, in true track coordinates (`plan_objective_on_track`); lower is better. `LMPC in loop`: the plan the LMPC actually used at that cycle (warm-started). `LMPC fresh` / `NMPC fresh`: freshly built controllers (cold start) solving from that state. `NMPC viol.`: violations of the NMPC plan in the shared validation. The QP surrogate objective is not compared.\n")
println(io, "| scenario | cycle | LMPC in loop | LMPC fresh: status, objective, QPs, [ms] | NMPC fresh: status, objective, NLP solves, [ms] | NMPC viol. | in loop − NMPC | fresh − NMPC | max abs Δu (fresh) |")
println(io, "|:--|--:|--:|:--|:--|--:|--:|--:|--:|")
for (lab, k, lin, ls, lo, nq, lt, ns, no, nn, nt, nv, du) in objrows
    @printf(io, "| %s | %d | %.5f | %s, %.5f, %d, %.1f | %s, %.5f, %d, %.1f | %d | %s | %s | %s |\n", lab, k, lin, ls, lo, nq, lt, ns, no, nn, nt, nv,
        g3(lin - no), g3(lo - no), g3(du))
end
fails = [(name, i, r) for name in order for (i, r) in enumerate(results[name]) if r.status !== :success]
println(io, "\n## Failures\n")
isempty(fails) && println(io, "None.")
for (name, i, r) in fails
    println(io, "- $name, run $i: $(r.msg)")
end
out = joinpath(@__DIR__, "results", "linear_multicurvature_obstacle.md")
write(out, String(take!(io)))
println("wrote ", out)

if Base.find_package("Plots") !== nothing
    @eval using Plots
    ENV["GKSwstype"] = "100"
    edge(ey) = (Pp = [frenet_to_cartesian(Q, s, ey) for s in range(0, LQ; length=2000)]; ([p[1] for p in Pp], [p[2] for p in Pp]))
    xy(tr, states) = (Pp = [frenet_to_cartesian(tr, x[1], x[2], x[3]) for x in states]; ([p[1] for p in Pp], [p[2] for p in Pp]))
    circle(cx, cy, r) = (θ = range(0, 2π; length=200); (cx .+ r .* cos.(θ), cy .+ r .* sin.(θ)))
    function trackplot(; kw...)
        p = plot(; aspect_ratio=:equal, xlabel="X [m]", ylabel="Y [m]", legend=:outertopright, kw...)
        for (ey, lc, ls, lab) in ((0.35, :black, :solid, "physical track edge"), (-0.35, :black, :solid, ""), (0.30, :gray, :dash, "planning corridor"),
            (-0.30, :gray, :dash, ""), (0.0, :gray, :dot, "centreline"))
            plot!(p, edge(ey)...; lc, ls, lw=ls === :solid ? 1.5 : 1, label=lab)
        end
        return p
    end
    lap1(cl) = cl.fine[1:something(findfirst(x -> x[1] > LQ, cl.fine), length(cl.fine))]
    function scene!(p, name, ob)
        plot!(p, circle(ob.x, ob.y, D_REQ)...; seriestype=:shape, fillcolor=:orange, fillalpha=0.3, lc=:orange, label="D = $(D_REQ) m")
        plot!(p, circle(ob.x, ob.y, ob.radius)...; seriestype=:shape, fillcolor=:red, lc=:red, label="obstacle")
        rl, rn = results["C LMPC N=10 M=3 · $name"][1], results["C NMPC N=10 M=3 · $name"][1]
        plot!(p, xy(Q, lap1(rn.cl))...; lc=:green, lw=2, ls=:dash, label="NMPC (Ipopt), lap 1")
        plot!(p, xy(Q, lap1(rl.cl))...; lc=:blue, lw=2, label="LMPC (OSQP), lap 1")
        return p
    end
    p1 = trackplot(; title="Quarter track, obstacle on the R = 1 m left bend (B), N = 10, M = 3")
    plot!(p1, xy(Q, lap1(results["A LMPC N=10 · no obstacle"][1].cl))...; lc=:gray, lw=1, ls=:dashdot, label="LMPC without obstacle")
    scene!(p1, SCEN[1][1], mkob(SCEN[1][2]))
    zooms = map((SCEN[1], SCEN[2], SCEN[6], SCEN[8])) do (name, spec)
        ob = mkob(spec)
        p = trackplot(; title=first(name, 34), xlims=(ob.x - 0.8, ob.x + 0.8), ylims=(ob.y - 0.8, ob.y + 0.8), legend=false)
        scene!(p, name, ob)
    end
    savefig(plot(p1, plot(zooms...; layout=(2, 2)); layout=(2, 1), size=(1500, 1900), left_margin=6Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "linear_multicurvature_trajectory.png"))
    # Timing: per-cycle computation time, LMPC vs NMPC, all stage C scenarios (run 1).
    tl = reduce(vcat, [results["C LMPC N=10 M=3 · $n"][1].tot for (n, _) in SCEN])
    tn = reduce(vcat, [results["C NMPC N=10 M=3 · $n"][1].tot for (n, _) in SCEN])
    srt(v) = (sort(v), range(0, 1; length=length(v)))
    q1 = plot(srt(tl)...; xscale=:log10, lw=2, label="LMPC (OSQP), $(length(tl)) cycles", xlabel="cycle computation time [ms]", ylabel="fraction of cycles",
        title="Stage C, all scenarios (run 1), N = 10", legend=:bottomright)
    plot!(q1, srt(tn)...; lw=2, ls=:dash, label="NMPC (Ipopt), $(length(tn)) cycles")
    vline!(q1, [80, 100]; lc=:red, ls=:dot, label="80 / 100 ms")
    rB = results["C LMPC N=10 M=3 · $(SCEN[1][1])"][1]
    q2 = plot(1:length(rB.tot), rB.tot; lw=1, label="LMPC total", xlabel="cycle", ylabel="[ms]", title="Scenario B, LMPC per-cycle breakdown (run 1)")
    plot!(q2, 1:length(rB.solve), rB.solve; lw=1, label="OSQP solves")
    plot!(q2, 1:length(rB.validate), rB.validate; lw=1, label="rollout + validation")
    plot!(q2, 1:length(rB.reference), rB.reference; lw=1, label="track references")
    savefig(plot(q1, q2; layout=(1, 2), size=(1500, 600), left_margin=6Plots.mm, bottom_margin=6Plots.mm),
        joinpath(@__DIR__, "..", "docs", "figures", "linear_multicurvature_timing.png"))
    println("saved figures")
end
