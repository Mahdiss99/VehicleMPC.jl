# Linearized vehicle MPC with JuMP and OSQP (Milestone 4A).

const LMV = KinematicBicycle(reference_test_vehicle())
const LMT = straight_track(400.0; width=1.0)
const LMP = LMV.params

lm_controller(; kw...) = LinearMPCController(LMV, LMT, LinearMPCConfig(; sqp_time_budget=Inf, kw...))

@testset "exact-map Jacobians: ForwardDiff vs finite differences, affine identity" begin
    for (z, u) in (([0.3, -0.2, 0.4, 2.0], [0.2, 1.0]), ([1.0, 2.0, -1.2, 4.5], [-0.3, -3.0]), ([0.0, 0.0, 0.0, 0.0], [0.0, 0.0]),
        ([5.0, 0.1, 0.01, 1e-3], [1e-4, 2.0]))
        A, B, c = linearize_exact_map(LMV, z, u, 0.1)
        F(w) = collect(exact_cartesian_step(LMV, w[1:4], w[5:6], 0.1))
        w, h = vcat(z, u), 1e-6
        J = hcat([(F(w + h * e) - F(w - h * e)) / 2h for e in eachcol(Matrix(1.0I, 6, 6))]...)
        @test isapprox(A, J[:, 1:4]; atol=1e-7) && isapprox(B, J[:, 5:6]; atol=1e-7)
        @test A * z + B * u + c ≈ F(w) atol = 1e-12                  # exact at the linearisation point
        # First-order accuracy: the affine model's error is quadratic in the perturbation.
        dz, du = [0.01, -0.01, 0.02, 0.05], [0.01, 0.1]
        e1 = norm(A * (z + dz) + B * (u + du) + c - F(vcat(z + dz, u + du)))
        e2 = norm(A * (z + dz / 2) + B * (u + du / 2) + c - F(vcat(z + dz / 2, u + du / 2)))
        @test e1 < 1e-3 && (e1 < 1e-12 || 3 < e1 / e2 < 5)
    end
    # Structural zeros: positions do not enter the dynamics, and the speed row is v + a dt.
    A, B, _ = linearize_exact_map(LMV, [1.0, 2.0, 0.3, 2.0], [0.1, 1.0], 0.1)
    @test A[:, 1:2] == [1 0; 0 1; 0 0; 0 0] && A[4, :] == [0, 0, 0, 1] && B[4, :] ≈ [0, 0.1]
end

@testset "lateral-acceleration linearisation" begin
    L = LMP.wheelbase
    for (v, d) in ((2.0, 0.1), (4.0, -0.05), (0.0, 0.2), (1.0, 0.0))
        g, gv, gd = linearize_lateral_acceleration(L, v, d)
        @test g ≈ v^2 * tan(d) / L
        h = 1e-6
        @test gv ≈ ((v + h)^2 - (v - h)^2) * tan(d) / (2h * L) atol = 1e-6
        @test gd ≈ v^2 * (tan(d + h) - tan(d - h)) / (2h * L) atol = 1e-5
        # Second-order error only.
        e(dv, dd) = abs((v + dv)^2 * tan(d + dd) / L - (g + gv * dv + gd * dd))
        @test e(0.02, 0.002) < 0.05 && e(0.0, 0.0) == 0
    end
end

@testset "QP construction, convexity, persistent coefficient updates" begin
    c = lm_controller()
    N = 10
    @test c.build_time > 0
    @test size(c.Z) == (4, N + 1) && size(c.U) == (2, N) && size(c.dyn) == (4, N) && size(c.ay_hi) == (N, 2)
    @test all(is_fixed, c.Z[:, 1]) && all(is_fixed, c.uprev)
    # The straight track model requires a straight track (Milestone 4BC: :auto selects the
    # local model on other tracks; see test_linear_mpc_tracks.jl).
    @test_throws ArgumentError LinearMPCController(LMV, quarter_track(), LinearMPCConfig(track_model=:straight))
    @test c.track_model === :straight
    @test_throws ArgumentError LinearMPCConfig(update_mode=:other) |> x -> LinearMPCController(LMV, LMT, x)
    @test_throws ArgumentError LinearMPCController(LMV, LMT, LinearMPCConfig(trust_region_shrink=1.5))
    @test_throws ArgumentError LinearMPCController(LMV, LMT, LinearMPCConfig(warm_tail=:zero))
    @test_throws ArgumentError LinearMPCController(LMV, LMT, LinearMPCConfig(primal_start=:shifted))
    # The objective is a convex quadratic: its Hessian is positive semidefinite.
    f = objective_function(c.model)
    vars = all_variables(c.model)
    idx = Dict(v => i for (i, v) in enumerate(vars))
    Q = zeros(length(vars), length(vars))
    for (p, coef) in f.terms
        i, j = idx[p.a], idx[p.b]
        i == j ? (Q[i, i] += 2coef) : (Q[i, j] += coef; Q[j, i] += coef)
    end
    @test eigmin(Symmetric(Q)) >= -1e-10
    # Every constraint is linear.
    @test all(F <: Union{AffExpr,VariableRef} for (F, _) in list_of_constraint_types(c.model))

    x = [0.0, 0.1, 0.05, 2.0]
    r = solve!(c, x; previous_input=(0.02, 0.5))
    @test r.status === :success
    z0 = collect(frenet_state_to_cartesian(LMT, x))
    @test fix_value.(c.Z[:, 1]) ≈ z0 && fix_value.(c.uprev) == [0.02, 0.5]       # measured state / previous input
    @test r.Z[:, 1] ≈ z0 atol = 1e-6
    # The installed coefficients are those of the last linearisation (r.Zbar, r.Ubar).
    for k in (1, 5, N)
        A, B, cc = linearize_exact_map(LMV, r.Zbar[:, k], r.Ubar[:, k], 0.1)
        for i in 1:4
            @test normalized_rhs(c.dyn[i, k]) ≈ cc[i]
            @test [normalized_coefficient(c.dyn[i, k], c.Z[j, k]) for j in 1:4] ≈ -A[i, :]
            @test [normalized_coefficient(c.dyn[i, k], c.U[j, k]) for j in 1:2] ≈ -B[i, :]
            @test normalized_coefficient(c.dyn[i, k], c.Z[i, k+1]) == 1
        end
        _, gv, gd = linearize_lateral_acceleration(LMP.wheelbase, r.Zbar[4, k], r.Ubar[1, k])
        @test normalized_coefficient(c.ay_hi[k, 1], c.Z[4, k]) ≈ gv && normalized_coefficient(c.ay_hi[k, 1], c.U[1, k]) ≈ gd
    end
    # A second measurement updates the fixed initial state without rebuilding the model.
    m = c.model
    x2 = [0.5, -0.1, 0.0, 2.2]
    r2 = solve!(c, x2; previous_input=r.u)
    @test c.model === m && r2.accepted
    @test fix_value.(c.Z[:, 1]) ≈ collect(frenet_state_to_cartesian(LMT, x2))
    # The rebuild mode gives the same control (it only differs in model construction).
    cr = lm_controller(update_mode=:rebuild)
    rr = solve!(cr, x; previous_input=(0.02, 0.5))
    @test rr.accepted && isapprox(rr.u, r.u; atol=1e-4)
    @test cr.model !== c.model
end

@testset "warm start, trust region, bounded successive linearisation" begin
    c = lm_controller()
    x = [0.0, 0.2, 0.1, 1.0]
    r1 = solve!(c, x)
    @test r1.accepted && c.last_U == r1.U
    @test r1.qp_solves == length(r1.qp_status) == length(r1.trust_scale) == length(r1.mismatch_history) <= 5
    @test r1.qp_status[end] === :OPTIMAL && all(>(0), r1.qp_iterations) && r1.primal_residual[end] < 1e-4
    # Cold start: the nominal trajectory is a dynamically consistent rollout.
    z0 = collect(frenet_state_to_cartesian(LMT, x))
    Ucold = nominal_inputs(c, z0)
    @test all(abs.(Ucold[1, :]) .<= LMP.delta_max) && all(LMP.a_min .<= Ucold[2, :] .<= LMP.a_max)
    # Warm start: the shifted previous inputs, re-rolled from the new measurement.
    x2 = [0.1, 0.19, 0.09, 1.05]
    r2 = solve!(c, x2; previous_input=r1.u)
    @test r2.accepted
    # (r2's nominal = shifted r1 inputs plus a pure-pursuit tail; check it directly.)
    z2 = collect(frenet_state_to_cartesian(LMT, x2))
    c.last_U = r1.U
    Us = shifted_inputs(c, z2)
    @test Us[:, 1:end-1] == r1.U[:, 2:end]
    @test Us[:, end] == collect(pursuit_input(c, exact_rollout(LMV, z2, Us[:, 1:end-1], 0.1)[:, end]))
    @test shifted_inputs(lm_controller(warm_tail=:repeat) |> (cc -> (cc.last_U = r1.U; cc)), z2)[:, end] == r1.U[:, end]
    if r2.qp_solves == 1
        @test r2.Ubar == Us
        @test r2.Zbar ≈ exact_rollout(LMV, z2, Us, 0.1)
    end
    c.last_U = r2.U
    # Trust region: with small radii every QP stays within ρ·radius of its nominal trajectory.
    ct = lm_controller(trust_region_u=(0.01, 0.2), trust_region_z=(0.05, 0.05, 0.02, 0.1), max_qp_solves=1)
    rt = solve!(ct, x)
    @test all(abs.(rt.U[1, :] - rt.Ubar[1, :]) .<= 0.01 + 1e-6) && all(abs.(rt.U[2, :] - rt.Ubar[2, :]) .<= 0.2 + 1e-6)
    @test all(abs.(rt.Z[4, 2:end] - rt.Zbar[4, 2:end]) .<= 0.1 + 1e-6)
    # Bounded: a far cold start that needs several QPs fails explicitly with max_qp_solves = 1.
    c1 = lm_controller(max_qp_solves=1)
    rf = solve!(c1, x)
    @test rf.qp_solves == 1 && rf.status === :not_converged && !rf.accepted && rf.u === nothing
    @test c1.last_U === nothing                                                 # failure resets the warm start
    # A zero time budget allows exactly one QP.
    rb = solve!(LinearMPCController(LMV, LMT, LinearMPCConfig(sqp_time_budget=0.0)), x)
    @test rb.qp_solves == 1 && occursin("time budget", rb.message)
    # The trust region shrinks after a mismatch rejection.
    if r1.qp_solves > 1
        @test r1.trust_scale[2] < r1.trust_scale[1]
    end
end

@testset "OSQP failure handling and invalid measurements" begin
    c = lm_controller(osqp_max_iter=1, polish=false)
    r = solve!(c, [0.0, 0.2, 0.1, 1.0])
    @test r.status === :qp_failure && !r.accepted && r.u === nothing && r.qp_solves == 1
    @test occursin("OSQP", r.message)
    cl = simulate_closed_loop(c, LMV, [0.0, 0.2, 0.1, 1.0], 5)
    @test cl.status === :controller_failure && isempty(cl.inputs)
    ri = solve!(lm_controller(), [0.0, 0.0, 0.0, -1.0])
    @test ri.status === :invalid_measurement && !ri.accepted
end

@testset "rollout consistency and independent nonlinear validation" begin
    c = lm_controller()
    x = [0.0, -0.3, -0.3, 3.0]
    r = solve!(c, x)
    @test r.accepted
    z0 = collect(frenet_state_to_cartesian(LMT, x))
    @test r.Zroll ≈ exact_rollout(LMV, z0, r.U, 0.1)
    d = r.Zroll - r.Z
    @test maximum(hypot.(d[1, :], d[2, :])) <= 1e-3 && maximum(abs, d[3, :]) <= 1e-3 && maximum(abs, d[4, :]) <= 1e-3
    @test isempty(r.violations) && r.max_abs_ey <= 0.45 && r.max_abs_ay <= 4.0 + 1e-4
    @test r.nonlinear_objective ≈ r.objective atol = 0.01
    lim = (; dt=0.1, track_margin=0.05, epsi_max=deg2rad(60), ay_max=4.0)
    v = validate_plan(LMV, LMT, lim, z0, r.U)
    @test isempty(v.violations) && v.Zroll == r.Zroll
    # Violations detected on the nonlinear rollout: corridor/track, heading, input, speed, ay.
    N = 10
    hard = hcat(fill([0.5, 0.0], N)...)
    vh = validate_plan(LMV, LMT, lim, [0.0, 0.0, 0.0, 3.0], hard)
    @test any(occursin("planning corridor", s) for s in vh.violations)
    @test any(occursin("|ay|", s) for s in vh.violations) && any(occursin("actuator", s) for s in vh.violations)
    @test any(occursin("physical track", s) for s in vh.violations) && vh.min_track_clearance < 0
    vs = validate_plan(LMV, LMT, lim, [0.0, 0.0, 0.0, 4.8], hcat(fill([0.0, 4.0], N)...))
    @test any(occursin("speed", s) for s in vs.violations)
    ve = validate_plan(LMV, LMT, (; lim..., epsi_max=0.1, ay_max=nothing), [0.0, 0.0, 0.0, 1.0], hcat(fill([0.3, 0.0], N)...))
    @test any(occursin("epsi", s) for s in ve.violations) && ve.max_abs_ay == 0
    # Straight tracks only.
    @test_throws ArgumentError validate_plan(LMV, quarter_track(), lim, z0, r.U)
end

@testset "closed loop: lateral recovery, speed limit, lateral acceleration" begin
    c = lm_controller(sqp_time_budget=0.08)
    cl = simulate_closed_loop(c, LMV, [0.0, 0.35, -0.2, 1.0], 120)
    @test succeeded(cl) && isempty(cl.violations)
    @test abs(cl.states[end][2]) < 0.02 && abs(cl.states[end][3]) < 0.02
    @test all(r.accepted && r.qp_solves <= 5 for r in cl.controller)
    @test maximum(abs(u[1]) for u in cl.inputs) <= LMP.delta_max
    @test maximum(abs(lateral_acceleration(LMV, x[4], u[1])) for (x, u) in zip(cl.states, cl.inputs)) <= 4.0 + 1e-4
    # Near the speed limit: the speed stays within [v_min, v_max].
    cv = simulate_closed_loop(lm_controller(sqp_time_budget=0.08), LMV, [0.0, 0.1, 0.0, 4.95], 60)
    @test succeeded(cv) && isempty(cv.violations)
    @test all(LMP.v_min - 1e-6 <= x[4] <= LMP.v_max + 1e-6 for x in cv.states)
    @test cv.states[end][4] > 4.9
end

@testset "objective comparability with the Cartesian-endpoint Ipopt NMPC" begin
    x, up = [0.0, 0.2, 0.1, 2.0], (0.0, 0.0)
    nc = NMPCController(LMV, LMT, NMPCConfig(prediction_model=:cartesian_endpoint_exact, horizon=10))
    rn = solve!(nc, x; previous_input=up)
    lc = lm_controller()
    rl = solve!(lc, x; previous_input=up)
    @test rn.accepted && rl.accepted
    z0 = collect(frenet_state_to_cartesian(LMT, x))
    # The same objective function: evaluated on the NMPC's exact rollout it reproduces the NMPC's objective.
    vn = validate_plan(lc, z0, rn.U)
    @test linear_plan_objective(lc, vn.Zroll, rn.U, up) ≈ rn.objective atol = 1e-6
    @test isempty(vn.violations)
    # The linear MPC's plan is close to the NLP optimum (it cannot be better beyond tolerances).
    @test rl.nonlinear_objective >= rn.objective - 1e-4
    @test rl.nonlinear_objective - rn.objective <= 0.05 * abs(rn.objective)
    @test isapprox(rl.u, rn.u; atol=0.05)
end

@testset "branch selection respects the obstacle acceptance rule (4A correction)" begin
    V, T = KinematicBicycle(reference_test_vehicle()), straight_track(60.0; width=0.7)
    cfg = NMPCConfig(prediction_model=:cartesian_endpoint_exact, schedule_corrections=8, correction_time_budget=100.0,
        ego_radius=0.05, obstacle_buffer=0.03, horizon=20, obstacle_samples=0)        # constraints at the knots only
    c = NMPCController(V, T, cfg; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.10))
    x = [4.0, 0.0, 0.0, 3.0]
    gl, gr = side_guess(c, x, :left), side_guess(c, x, :right)
    cand(seg, g, obj) = (; segment=seg, core=(; X=g.Xg, U=g.Ug, obj))
    valid = [cand(1, gl, 0.0), cand(2, gr, 1.0)]
    # With the obstacle out of reach both pass the obstacle check: the lower objective is selected.
    set_obstacle!(c, CircleObstacle(x=50.0, y=0.0, radius=0.05))
    @test all(obstacle_rejection(c, collision_check(c, x, v.core.X, v.core.U))[1] === nothing for v in valid)
    best, note = select_branch_candidate(c, x, valid)
    @test best.segment == 1 && note == ""
    # An obstacle on the left (lower-objective) path, midway between two knots: every knot
    # keeps the required separation, but the dense evaluation finds the collision. The
    # collision-valid right candidate is selected instead.
    k = argmin(abs.(gl.Xg[1, :] .- 8.0))
    pk = frenet_state_to_cartesian(T, gl.Xg[:, k])
    mid = interior_point(V.params.wheelbase, pk, gl.Xg[4, k], gl.Ug[1, k], gl.Ug[2, k], 0.05)
    set_obstacle!(c, CircleObstacle(x=mid[1], y=mid[2], radius=0.02))
    ccl = collision_check(c, x, gl.Xg, gl.Ug)
    @test ccl.knot_min_margin > 0 && ccl.collisions > 0
    @test obstacle_rejection(c, ccl)[1] === :collision_detected
    @test obstacle_rejection(c, collision_check(c, x, gr.Xg, gr.Ug))[1] === nothing
    best, note = select_branch_candidate(c, x, valid)
    @test best.segment == 2 && occursin("segment 1 rejected", note)
    # No collision-valid candidate: no selection.
    best, note = select_branch_candidate(c, x, [cand(1, gl, 0.0)])
    @test best === nothing && occursin("segment 1", note)
    @test select_branch_candidate(c, x, []) == (nothing, "")
    # Without an obstacle the rule is unchanged (lowest objective).
    c0 = NMPCController(V, T, NMPCConfig(prediction_model=:cartesian_endpoint_exact, horizon=20))
    @test select_branch_candidate(c0, x, valid)[1].segment == 1
end
