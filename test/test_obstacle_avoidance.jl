# Stationary-obstacle collision avoidance (Milestone 3A, experimental).

const OBV = KinematicBicycle(reference_test_vehicle())
const OBT = straight_track(60.0; width=0.7)
const OBOB = CircleObstacle(x=8.0, y=0.0, radius=0.10)
const OBCFG = (prediction_model=:cartesian_endpoint_exact, schedule_corrections=8, correction_time_budget=100.0, ego_radius=0.05,
    obstacle_buffer=0.03)

@testset "obstacle geometry and separation" begin
    @test required_separation(OBOB, 0.05, 0.03) ≈ 0.18
    @test required_separation(OBOB, 0.05, 0.0) ≈ 0.15                      # zero buffer
    @test_throws ArgumentError required_separation(OBOB, -0.01, 0.0)
    @test_throws ArgumentError CircleObstacle(x=0.0, y=0.0, radius=-1.0)
    @test_throws ArgumentError CircleObstacle(x=NaN, y=0.0, radius=1.0)
    D = 0.18
    @test obstacle_margin(8.0 + D, 0.0, 8.0, 0.0, D) ≈ 0.0 atol = 1e-12          # on the boundary (up to rounding)
    @test obstacle_margin(8.0, D + 0.01, 8.0, 0.0, D) > 0                      # outside
    @test obstacle_margin(8.0 + 0.1, 0.05, 8.0, 0.0, D) < 0                    # inside
    @test obstacle_margin(8.0 + D + 1e-3, 0.0, 8.0, 0.0, D) ≈ 1e-3 atol = 1e-5   # ≈ metric near the boundary
    # Automatic differentiation of the margin through the exact interior position.
    f = w -> obstacle_margin(interior_point(0.25, (w[1], w[2], w[3]), w[4], w[5], w[6], 0.05)..., 8.0, 0.0, D)
    w0 = [7.6, 0.1, 0.05, 2.0, 0.2, 1.0]
    g = VehicleMPC.ForwardDiff.gradient(f, w0)
    gfd = [(f(w0 + 1e-6 * e) - f(w0 - 1e-6 * e)) / 2e-6 for e in eachcol(Matrix(1.0I, 6, 6))]
    @test maximum(abs.(g - gfd)) <= 1e-8
end

@testset "NLP constraints, Parameters and updates" begin
    c = NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=10, obstacle_samples=3); obstacle=OBOB)
    c0 = NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=10))
    ncon(m) = num_constraints(m; count_variable_in_set_constraints=false)
    @test length(c.obs_cons) == 10 + 3 * 10 && ncon(c.model) == ncon(c0.model) + 40
    @test parameter_value.(c.obs_par) ≈ [8.0, 0.0, 0.18]
    @test c.obs_alphas == [0.25, 0.5, 0.75] && isempty(c0.obs_cons) && c0.obstacle === nothing
    # The constraint functions equal obstacle_margin at the knot poses.
    x = [5.0, 0.0, 0.0, 2.0]
    r = solve!(c, x)
    @test r.accepted && r.obstacle.present
    vals = Dict{VariableRef,Float64}()
    for k in 2:11, i in 1:4
        vals[c.X[i, k]] = r.X[i, k]
    end
    for k in 1:10, i in 1:2
        vals[c.U[i, k]] = r.U[i, k]
    end
    valfn = v -> is_parameter(v) ? parameter_value(v) : vals[v]
    for k in 2:11
        p = frenet_state_to_cartesian(OBT, r.X[:, k])
        @test value(valfn, constraint_object(c.obs_cons[k-1]).func) ≈ obstacle_margin(p[1], p[2], 8.0, 0.0, 0.18) atol = 1e-12
    end
    @test minimum(value(valfn, constraint_object(con).func) for con in c.obs_cons) >= -1e-7
    # Moving the obstacle updates the Parameters only (same model).
    m0 = c.model
    set_obstacle!(c, CircleObstacle(x=9.0, y=0.1, radius=0.05))
    @test c.model === m0 && parameter_value.(c.obs_par) ≈ [9.0, 0.1, 0.13] && c.obstacle_side === :none
    @test_throws ArgumentError set_obstacle!(c0, OBOB)
    @test_throws ArgumentError NMPCController(OBV, OBT, NMPCConfig(); obstacle=OBOB)       # Frenet mode: not supported
    @test_throws ArgumentError NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., obstacle_acceptance=:none); obstacle=OBOB)
end

@testset "left/right initialisations are dynamically consistent" begin
    c = NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=20, obstacle_samples=3); obstacle=OBOB)
    x = [4.0, 0.0, 0.0, 3.0]
    @test passage_sides(c, x[1]) == (true, true)
    gl, gr = side_guess(c, x, :left), side_guess(c, x, :right)
    @test gl.converged && gr.converged
    k = argmin(abs.(gl.Xg[1, :] .- 8.0))
    @test gl.Xg[2, k] > 0.1 && gr.Xg[2, argmin(abs.(gr.Xg[1, :] .- 8.0))] < -0.1          # the guesses pass on their sides
    p = OBV.params
    for g in (gl, gr)
        @test all(abs.(g.Ug[1, :]) .<= p.delta_max) && all(abs.(g.Xg[2, :]) .< OBT.width / 2)
        @test all(abs(lateral_acceleration(OBV, g.Xg[4, k], g.Ug[1, k])) <= c.config.ay_max + 1e-9 for k in 1:20)
        # The guess is a trajectory of the vehicle model: re-simulating its inputs reproduces it.
        sim = simulate_on_track(OBV, OBT, x, [g.Ug[:, k] for k in 1:20], 0.1; max_step=0.02)
        @test maximum(maximum(abs.(sim.states[k] - g.Xg[:, k])) for k in 1:21) <= 1e-12
    end
    @test_throws ArgumentError side_guess(c, x, :up)
end

@testset "candidate selection, side retention, closed-loop avoidance" begin
    cfg = NMPCConfig(; OBCFG..., horizon=20, obstacle_samples=3)
    c = NMPCController(OBV, OBT, cfg; obstacle=OBOB)
    cl = simulate_closed_loop(c, OBV, [0.0, 0.0, 0.0, 0.5], 60)
    @test succeeded(cl) && cl.states[end][1] > 20.0                                   # passes and keeps going
    rs = cl.controller
    first2 = findfirst(r -> length(r.obstacle.attempts) == 2, rs)
    @test first2 !== nothing && Set(a[1] for a in rs[first2].obstacle.attempts) == Set([:left, :right])
    @test count(r -> length(r.obstacle.attempts) >= 2, rs) <= 2                       # two solves only at the encounter
    sides = unique(r.obstacle.side for r in rs if r.obstacle.side !== :none)
    @test length(sides) == 1                                                          # no switching
    @test all(r -> r.obstacle.collisions == 0 && r.obstacle.knot_min_clearance >= -1e-6, rs)
    D, Dc = 0.18, 0.15
    poses = [frenet_to_cartesian(OBT, x[1], x[2], x[3]) for x in cl.fine_states]
    dmin = minimum(hypot(p[1] - 8.0, p[2]) for p in poses)
    @test dmin > Dc                                                                   # no collision of the plant disk
    @test maximum(abs(x[2]) for x in cl.fine_states) <= OBT.width / 2                # inside the physical track
    @test isempty(cl.violations)
    # Same MPC without the obstacle drives straight through the obstacle's location.
    cl0 = simulate_closed_loop(NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=20)), OBV, [0.0, 0.0, 0.0, 0.5], 60)
    @test minimum(hypot(p[1] - 8.0, p[2]) for p in (frenet_to_cartesian(OBT, x[1], x[2], x[3]) for x in cl0.fine_states)) < 0.05
end

@testset "between-knot collisions are detected (knot-only constraints)" begin
    c = NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=20); obstacle=OBOB)
    cl = simulate_closed_loop(c, OBV, [0.0, 0.0, 0.0, 0.5], 10)
    bad = findfirst(r -> !r.accepted, cl.controller)
    @test bad !== nothing
    r = cl.controller[bad]
    @test r.status === :collision_detected && r.obstacle.failure_cause === :collision && r.u === nothing
    @test r.obstacle.violation_between_knots && r.obstacle.collisions > 0 && isfinite(r.obstacle.first_violation_time)
    # The imposed (knot) constraints themselves were satisfied by the rejected plan.
    @test r.obstacle.knot_min_clearance >= -1e-6
    # collision_check on a hand-made plan: knots outside the disk, straight motion through it.
    X = zeros(4, 21); U = zeros(2, 20)
    X[1, :] = 7.5 .+ 0.5 .* (0:20)
    X[4, :] .= 5.0
    U[2, :] .= 0.0
    cc = collision_check(c, X[:, 1], X, U)
    @test cc.collisions > 0 && cc.between && cc.min_distance < 1e-9
end

@testset "initial collision, infeasible geometry, solver failure" begin
    c = NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=20, obstacle_samples=3); obstacle=OBOB)
    r = solve!(c, [7.9, 0.05, 0.0, 1.0])
    @test r.status === :initial_collision && r.nlp_solves == 0 && r.obstacle.failure_cause === :initial_collision && r.u === nothing
    @test solve!(c, [7.81, 0.0, 0.0, 0.0]).status !== :initial_collision            # 0.19 m > D = 0.18: not an initial collision
    # An obstacle covering the whole corridor: no passage; at speed close to it, braking is impossible.
    blk = CircleObstacle(x=8.0, y=0.0, radius=0.30)
    cb = NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=20, obstacle_samples=3); obstacle=blk)
    @test passage_sides(cb, 6.0) == (false, false)
    rb = solve!(cb, [6.6, 0.0, 0.0, 5.0])
    @test !rb.accepted && rb.u === nothing && rb.obstacle.failure_cause in (:infeasible_geometry, :collision, :constraint_violation)
    @test rb.obstacle.passage_left == false && rb.obstacle.passage_right == false
    # With room to brake, stopping would be feasible, but neither side initialisation leads
    # Ipopt there (documented limitation: no braking candidate). The cycle is rejected, not
    # accepted with an invalid control.
    rs = solve!(NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=20, obstacle_samples=3); obstacle=blk), [0.0, 0.0, 0.0, 1.0])
    @test !rs.accepted && rs.u === nothing && rs.obstacle.failure_cause === :infeasible_geometry && length(rs.obstacle.attempts) == 2
    # Solver failure is reported as such (iteration limit), with every attempt recorded.
    rf = solve!(NMPCController(OBV, OBT, NMPCConfig(; OBCFG..., horizon=20, obstacle_samples=3, max_iter=1); obstacle=OBOB), [4.0, 0.0, 0.0, 3.0])
    @test rf.status === :solver_failure && !rf.accepted && length(rf.obstacle.attempts) == 2
    @test rf.obstacle.failure_cause === :solver_convergence && rf.nlp_solves == 2
end

@testset "no-obstacle regression" begin
    cfg = NMPCConfig(; OBCFG..., horizon=10)
    a = simulate_closed_loop(NMPCController(OBV, OBT, cfg), OBV, [0.0, 0.2, 0.1, 0.5], 30)
    b = simulate_closed_loop(NMPCController(OBV, OBT, cfg), OBV, [0.0, 0.2, 0.1, 0.5], 30)
    @test a.states == b.states && all(r -> !r.obstacle.present, a.controller)
    # The obstacle settings alone (without an obstacle) do not change the controller.
    d = simulate_closed_loop(NMPCController(OBV, OBT, NMPCConfig(prediction_model=:cartesian_endpoint_exact, schedule_corrections=8,
        correction_time_budget=100.0, horizon=10)), OBV, [0.0, 0.2, 0.1, 0.5], 30)
    @test d.states == a.states
    @test NMPCConfig().ego_radius == 0.0 && NMPCConfig().obstacle_samples == 0
end
