# Nonlinear MPC: construction, constraints, parameter updates, warm start, failure handling,
# residual validation, repeatability and between-knot diagnostics.

const NV = KinematicBicycle(reference_test_vehicle())
const NP = NV.params
const CIRCLE = circular_track(1.0; width=0.7)
const STRAIGHT = straight_track(200.0; width=0.7)

"Variable-value function for evaluating JuMP constraint functions at a chosen point."
function point_values(c, Xv, Uv, Pv)
    vals = Dict{VariableRef,Float64}()
    for k in 2:size(Xv, 2), i in 1:4
        vals[c.X[i, k]] = Xv[i, k]
    end
    for k in 1:size(Uv, 2), i in 1:2
        vals[c.U[i, k]] = Uv[i, k]
    end
    for idx in eachindex(c.P)
        vals[c.P[idx]] = Pv[idx]
    end
    return v -> is_parameter(v) ? parameter_value(v) : vals[v]
end

"Exact rollout with the checked model plus the RK4 stage points (computed from the definition)."
function consistent_point(c, x1, Uv)
    N, dt, L = horizon(c), c.config.dt, NP.wheelbase
    kap = curvature_values(c)
    Xv = zeros(4, N + 1)
    Xv[:, 1] = x1
    Pv = zeros(size(c.P))
    for k in 1:N
        Xv[:, k+1] = discrete_dynamics(NV, Xv[:, k], Uv[:, k], kap[k], dt, c.config.integrator)
        if c.config.integrator isa RK4
            f = z -> collect(kinematic_rhs(L, z..., Uv[1, k], Uv[2, k], kap[k]))[2:4]
            z1 = Xv[2:4, k]
            z2 = z1 + dt / 2 * f(z1)
            z3 = z1 + dt / 2 * f(z2)
            z4 = z1 + dt * f(z3)
            Pv[:, 1, k], Pv[:, 2, k], Pv[:, 3, k] = z2, z3, z4
        end
    end
    return Xv, Pv
end

constraint_residual(valfn, con) = (o = constraint_object(con); value(valfn, o.func) - MOI.constant(o.set))

@testset "optimization model construction" begin
    for integ in (RK4(), ForwardEuler()), N in (5, 12)
        c = NMPCController(NV, CIRCLE, NMPCConfig(horizon=N, integrator=integ))
        @test size(c.X) == (4, N + 1) && size(c.U) == (2, N)
        @test all(is_parameter, c.X[:, 1]) && all(is_parameter, c.u_prev) && all(is_parameter, c.kappa)
        @test !any(is_parameter, c.X[:, 2:end]) && !any(is_parameter, c.U)
        @test curvature_values(c) == fill(1.0, N)
        @test length(c.dynamics) == (integ isa RK4 ? 13N : 4N)
        @test size(c.P) == (integ isa RK4 ? (3, 3, N) : (3, 0, N))
        @test c.ey_max ≈ 0.30
        @test all(k -> lower_bound(c.X[2, k]) ≈ -0.30 && upper_bound(c.X[2, k]) ≈ 0.30, 2:N+1)
        @test all(k -> lower_bound(c.X[3, k]) ≈ -deg2rad(60) && upper_bound(c.X[3, k]) ≈ deg2rad(60), 2:N+1)
        @test all(k -> lower_bound(c.X[4, k]) == NP.v_min && upper_bound(c.X[4, k]) == NP.v_max, 2:N+1)
        @test all(k -> lower_bound(c.U[1, k]) == -NP.delta_max && upper_bound(c.U[1, k]) == NP.delta_max, 1:N)
        @test all(k -> lower_bound(c.U[2, k]) == NP.a_min && upper_bound(c.U[2, k]) == NP.a_max, 1:N)
        @test !any(has_lower_bound, c.X[1, 2:end]) && !any(has_upper_bound, c.X[1, 2:end])   # s is free
        if integ isa RK4
            # Stage points are kept Frenet-safe: 1 - kappa*ey ≥ 2 eps_geometry on the circle (kappa = 1).
            @test all(upper_bound(c.P[1, j, k]) ≈ 1 - 2 * NV.eps_geometry for j in 1:3, k in 1:N)
            @test all(lower_bound(c.P[3, j, k]) == 0.0 for j in 1:3, k in 1:N)
        end
    end
    # Two smooth inequalities per end point per interval for |ay| ≤ ay_max.
    nl(c) = num_constraints(c.model, GenericNonlinearExpr{VariableRef}, MOI.LessThan{Float64}) +
            num_constraints(c.model, GenericNonlinearExpr{VariableRef}, MOI.GreaterThan{Float64})
    @test nl(NMPCController(NV, CIRCLE, NMPCConfig(horizon=7))) == 4 * 7
    @test nl(NMPCController(NV, CIRCLE, NMPCConfig(horizon=7, ay_max=nothing))) == 0

    # Rejected configurations.
    @test NMPCController(NV, quarter_track(), NMPCConfig()).kappa_range == (-1.0, 1.0)          # multi-segment (2B.2)
    @test_throws ArgumentError NMPCController(NV, Track([TrackSegment(1.0, 0.0), TrackSegment(0.3, 4.0)]; width=0.7), NMPCConfig())
    @test_throws ArgumentError NMPCController(NV, circular_track(1.0; width=0.1), NMPCConfig())    # no lateral room
    @test_throws ArgumentError NMPCController(NV, circular_track(0.5; width=0.9), NMPCConfig(track_margin=0.0))  # Frenet margin
    @test_throws ArgumentError NMPCController(NV, CIRCLE, NMPCConfig(horizon=0))
    @test_throws ArgumentError NMPCController(NV, CIRCLE, NMPCConfig(dt=0.0))
    @test_throws ArgumentError NMPCController(NV, CIRCLE, NMPCConfig(epsi_max=π / 2))
    @test_throws ArgumentError NMPCController(NV, CIRCLE, NMPCConfig(ay_max=-1.0))
    @test_throws ArgumentError NMPCController(NV, CIRCLE, NMPCConfig(qy=-1.0))
end

@testset "dynamic constraints encode the discrete model" begin
    for integ in (RK4(), ForwardEuler()), (track, x1) in ((CIRCLE, [0.3, 0.1, -0.2, 1.5]), (STRAIGHT, [2.0, -0.2, 0.3, 3.0]))
        c = NMPCController(NV, track, NMPCConfig(horizon=6, integrator=integ))
        for i in 1:4
            set_parameter_value(c.X[i, 1], x1[i])
        end
        Uv = [0.2 -0.1 0.3 0.0 -0.25 0.1; 1.0 -2.0 0.5 0.0 3.0 -1.0]
        Xv, Pv = consistent_point(c, x1, Uv)
        valfn = point_values(c, Xv, Uv, Pv)
        @test maximum(abs(constraint_residual(valfn, con)) for con in c.dynamics) <= 1e-12
        # Perturbing one predicted state breaks exactly the constraints that involve it.
        Xp = copy(Xv)
        Xp[2, 4] += 1e-3
        res = [constraint_residual(point_values(c, Xp, Uv, Pv), con) for con in c.dynamics]
        @test maximum(abs, res) >= 1e-3 - 1e-12
        @test count(r -> abs(r) > 1e-12, res) < length(res)
    end
end

@testset "initial-state equality and parameter updates" begin
    cfg = NMPCConfig(horizon=10, warm_start=false)
    c = NMPCController(NV, CIRCLE, cfg)
    xa, xb = [0.0, 0.05, 0.0, 1.0], [3.0, -0.15, 0.2, 1.8]
    ra = solve!(c, xa)
    rb = solve!(c, xb)
    @test ra.accepted && rb.accepted
    @test ra.X[:, 1] == xa && rb.X[:, 1] == xb                  # exact: X[:,1] are parameters
    @test parameter_value.(c.X[:, 1]) == xb
    # The persistent model re-solved with new parameters equals a model built from scratch.
    fresh = solve!(NMPCController(NV, CIRCLE, cfg), xb)
    @test maximum(abs, rb.X - fresh.X) <= 1e-9 && maximum(abs, rb.U - fresh.U) <= 1e-9
    @test maximum(abs, rb.U - ra.U) > 1e-3

    # The previously applied input enters the first rate penalty.
    cr = NMPCController(NV, STRAIGHT, NMPCConfig(horizon=10, warm_start=false, rd_delta=100.0))
    rp = solve!(cr, [0.0, 0.0, 0.0, 2.0]; previous_input=(0.3, 0.0))
    rm = solve!(cr, [0.0, 0.0, 0.0, 2.0]; previous_input=(-0.3, 0.0))
    @test rp.accepted && rm.accepted
    @test parameter_value.(cr.u_prev) == [-0.3, 0.0]
    @test rp.u[1] > 0.2 && rm.u[1] < -0.2
    @test rp.u[1] ≈ -rm.u[1] atol = 1e-6                          # mirror symmetry on a straight
end

@testset "input, state and track bounds hold at the solution" begin
    c = NMPCController(NV, STRAIGHT, NMPCConfig(horizon=15))
    r = solve!(c, [0.0, 0.2, 0.1, 4.8])
    @test r.accepted
    @test all(abs.(r.X[2, 2:end]) .<= c.ey_max + 1e-9)
    @test all(abs.(r.X[3, 2:end]) .<= deg2rad(60) + 1e-9)
    @test all(NP.v_min .<= r.X[4, 2:end] .<= NP.v_max)
    @test all(abs.(r.U[1, :]) .<= NP.delta_max) && all(NP.a_min .<= r.U[2, :] .<= NP.a_max)
    @test maximum(r.X[4, :]) ≈ NP.v_max atol = 1e-6              # progress reward saturates the speed limit
    @test r.max_constraint_violation <= 1e-9
end

@testset "lateral-acceleration constraint" begin
    x0 = [0.0, 0.0, 0.0, 0.5]
    for ay in (2.0, 4.0)
        c = NMPCController(NV, CIRCLE, NMPCConfig(ay_max=ay))
        cl = simulate_closed_loop(c, NV, x0, 40)
        @test succeeded(cl)
        ays = [abs(lateral_acceleration(NV, cl.fine_states[j][4], cl.inputs[cld(j - 1, 10)][1])) for j in 2:length(cl.fine_states)]
        @test maximum(ays) <= ay + 1e-6                           # holds on the plant, between knots too
        @test maximum(ays) >= 0.98ay                              # and it is active
        # Steady state on the centreline: v²·kappa = ay ⇒ v ≈ sqrt(ay·R) (R = 1, small ey).
        @test cl.states[end][4] ≈ sqrt(ay) rtol = 0.05
    end
    # Without the constraint the speed is limited only by v_max.
    c = NMPCController(NV, CIRCLE, NMPCConfig(ay_max=nothing))
    cl = simulate_closed_loop(c, NV, x0, 30)
    @test succeeded(cl) && cl.states[end][4] > 4.0
end

@testset "warm-start shifting and cold initialisation" begin
    c = NMPCController(NV, CIRCLE, NMPCConfig(horizon=10))
    N = 10
    Xg, Ug = VehicleMPC.cold_start_guess(c, [0.0, 0.0, 0.0, 1.0])
    @test all(Ug[1, :] .≈ atan(1.0 * NP.wheelbase)) && all(Ug[2, :] .== 0)
    @test Xg[:, 1] == [0.0, 0.0, 0.0, 1.0]
    @test maximum(abs, Xg[2:3, :]) <= 1e-12                       # centreline-following guess

    r1 = solve!(c, [0.0, 0.05, 0.0, 1.0])
    @test r1.accepted && !r1.warm_started
    xnew = [r1.X[1, 2] + 0.001, r1.X[2, 2] - 0.002, r1.X[3, 2], r1.X[4, 2]]
    Xw, Uw = VehicleMPC.warm_start_guess(c, xnew)
    @test Uw[:, 1:N-1] == r1.U[:, 2:N] && Uw[:, N] == r1.U[:, N]    # shifted, last input repeated
    @test Xw[:, 1] == xnew                                           # matched to the new measurement
    for k in 1:N                                                     # regenerated by the prediction model
        @test Xw[:, k+1] ≈ discrete_dynamics(NV, Xw[:, k], Uw[:, k], 1.0, 0.1, RK4()) atol = 1e-14
    end
    VehicleMPC.apply_start_values!(c, Xw, Uw, curvature_values(c))
    @test start_value(c.U[1, 1]) == Uw[1, 1] && start_value(c.X[4, 3]) == Xw[4, 3]
    # Start values are projected into the variable bounds (never the state itself).
    Xbad = copy(Xw)
    Xbad[2, 5] = 10.0
    VehicleMPC.apply_start_values!(c, Xbad, Uw, curvature_values(c))
    @test start_value(c.X[2, 5]) == c.ey_max

    r2 = solve!(c, xnew)
    @test r2.accepted && r2.warm_started
    reset_warm_start!(c)
    @test !solve!(c, xnew).warm_started
end

@testset "solver termination handling" begin
    for cfg in (NMPCConfig(max_iter=1), NMPCConfig(max_wall_time=1e-6))
        c = NMPCController(NV, CIRCLE, cfg)
        r = solve!(c, [0.0, 0.1, 0.1, 1.0])
        @test r.status === :solver_failure && !r.accepted && r.u === nothing
        @test r.termination_status in (MOI.ITERATION_LIMIT, MOI.TIME_LIMIT)
        @test c.last_U === nothing                                   # no warm start from a rejected solve
        @test occursin("termination status", r.message)
    end
end

@testset "intentionally infeasible and invalid cases" begin
    c = NMPCController(NV, STRAIGHT, NMPCConfig(horizon=10))
    # Above v_max: even full braking cannot reach v ≤ 5 m/s after one 0.1 s step.
    r = solve!(c, [0.0, 0.0, 0.0, 6.0])
    @test r.status === :solver_failure && r.u === nothing
    @test r.termination_status in (MOI.LOCALLY_INFEASIBLE, MOI.INFEASIBLE, MOI.NORM_LIMIT)
    # Far outside the track corridor: cannot return within one step.
    r = solve!(c, [0.0, 0.6, 0.0, 0.5])
    @test r.status === :solver_failure && r.u === nothing
    # The controller recovers on the next feasible measurement.
    @test solve!(c, [0.0, 0.0, 0.0, 1.0]).accepted

    # Invalid measurements never reach the solver.
    cc = NMPCController(NV, CIRCLE, NMPCConfig(horizon=10))
    for x in ([NaN, 0.0, 0.0, 1.0], [0.0, 0.95, 0.0, 1.0], [0.0, 0.0, 0.0, -1.0])
        r = solve!(cc, x)
        @test r.status === :invalid_measurement && r.termination_status == MOI.OPTIMIZE_NOT_CALLED && r.u === nothing
    end
    @test solve!(cc, [0.0, 0.0, 0.0, 1.0]; previous_input=(Inf, 0.0)).status === :invalid_measurement
end

@testset "dynamics and constraint residuals" begin
    for integ in (RK4(), ForwardEuler())
        c = NMPCController(NV, CIRCLE, NMPCConfig(integrator=integ))
        r = solve!(c, [0.0, 0.1, -0.1, 1.5])
        @test r.accepted
        @test r.max_dynamics_residual <= 1e-6 && r.max_constraint_violation <= 1e-6
        dyn, viol, problems = validate_prediction(c, r.X, r.U, r.X[:, 1], curvature_values(c))
        @test dyn == r.max_dynamics_residual && isempty(problems)
        # Validation is independent of the solver: a corrupted prediction is detected.
        Xc = copy(r.X)
        Xc[3, 6] += 0.01
        @test validate_prediction(c, Xc, r.U, r.X[:, 1], curvature_values(c))[1] >= 0.01 - 1e-9
        Uc = copy(r.U)
        Uc[1, 3] = 0.5
        @test validate_prediction(c, r.X, Uc, r.X[:, 1], curvature_values(c))[2] > 0.05
    end
end

@testset "repeatability" begin
    cfg = NMPCConfig(warm_start=false)
    x = [1.0, 0.1, 0.05, 1.2]
    r1 = solve!(NMPCController(NV, CIRCLE, cfg), x)
    c = NMPCController(NV, CIRCLE, cfg)
    r2, r3 = solve!(c, x), solve!(c, x)
    @test r1.X == r2.X == r3.X && r1.U == r2.U == r3.U
    @test r1.iterations == r2.iterations == r3.iterations
end

@testset "between-knot diagnostics" begin
    # Prediction integration error: RK4 is far more accurate than Euler at dt = 0.1.
    errs = map((RK4(), ForwardEuler())) do integ
        c = NMPCController(NV, CIRCLE, NMPCConfig(integrator=integ))
        r = solve!(c, [0.0, 0.1, 0.1, 1.5])
        d = between_knot_diagnostics(c, r)
        @test d.status === :success && isempty(d.violations)
        d.max_knot_error
    end
    @test errs[1] <= 1e-4 && errs[2] >= 10 * errs[1]

    # Knots inside the corridor while the continuous path leaves it (single 0.2 s interval).
    # (ay_max disabled so that only the track boundary is examined here.)
    c = NMPCController(NV, STRAIGHT, NMPCConfig(horizon=1, dt=0.2, track_margin=0.0, ay_max=nothing))
    x1, U = [0.0, 0.28, 0.5, 4.0], reshape([-0.3, 0.0], 2, 1)
    X = hcat(x1, discrete_dynamics(NV, x1, U[:, 1], 0.0, 0.2, RK4()))
    @test all(abs.(X[2, :]) .< 0.35)
    d = between_knot_diagnostics(c, X, U; substeps=100)
    @test d.max_abs_ey > 0.37
    @test !isempty(d.violations) && all(p -> p.second.constraint === :track_boundary, d.violations)
    @test all(p -> 0 < p.first < 0.2, d.violations)

    # Lateral acceleration rising inside an interval is caught both between knots and by the
    # end-point check in validate_prediction.
    cc = NMPCController(NV, CIRCLE, NMPCConfig(horizon=1, dt=0.1, ay_max=4.0))
    delta = atan(4.0 * NP.wheelbase / 1.9^2)                       # ay = 4 exactly at v = 1.9
    x1, U = [0.0, 0.0, 0.0, 1.9], reshape([delta, 5.0], 2, 1)
    X = hcat(x1, discrete_dynamics(NV, x1, U[:, 1], 1.0, 0.1, RK4()))
    d = between_knot_diagnostics(cc, X, U)
    @test any(p -> p.second.constraint === :lateral_acceleration, d.violations)
    @test validate_prediction(cc, X, U, x1, curvature_values(cc))[2] > 1.0
end
