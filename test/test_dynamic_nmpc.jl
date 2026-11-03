# Dynamic-bicycle NMPC with multiple stationary obstacles (final milestone).

const DNP = showcase_dynamic_parameters()
const DNV = DynamicBicycle(DNP)
const DNQ = quarter_track()
const DNS = straight_track(60.0; width=0.7)

"CoM state with the rear axle at Frenet `(s, ey, epsi)` and speed `v` (no lateral speed or yaw rate)."
function dn_state(track, s, ey, epsi, v)
    Xr, Yr, psi = frenet_to_cartesian(track, s, ey, epsi)
    Xg, Yg, _ = cg_pose_from_rear_axle(DNP, Xr, Yr, psi)
    return [Xg, Yg, psi, v, 0.0, 0.0]
end
dn_ctrl(track; obstacles=CircleObstacle[], buffers=nothing, kw...) = DynamicNMPCController(DNV, track, DynamicNMPCConfig(; time_budget=Inf,
    ego_radius=0.05, obstacle_buffer=0.03, kw...); obstacles, buffers)

@testset "construction and configuration" begin
    c = dn_ctrl(DNQ)
    N, S = 10, 4
    @test size(c.F) == (6, N) && size(c.W) == (6, N * (S - 1)) && size(c.G) == (6, 3, N * S) && size(c.U) == (2, N)
    @test isempty(c.obstacles) && size(c.op) == (3, 0)
    @test_throws ArgumentError DynamicNMPCController(DNV, DNQ, DynamicNMPCConfig(substeps=0))
    @test_throws ArgumentError DynamicNMPCController(DNV, DNQ, DynamicNMPCConfig(plant_step=0.1))
    @test_throws ArgumentError DynamicNMPCController(DNV, DNQ, DynamicNMPCConfig(); obstacles=[CircleObstacle(x=0.0, y=0.0, radius=0.1)], buffers=[0.1, 0.2])
    @test_throws ArgumentError DynamicNMPCController(DNV, DNQ, DynamicNMPCConfig(); obstacles=[CircleObstacle(x=100.0, y=100.0, radius=0.1)])
end

@testset "single-interval validation with and without obstacles" begin
    z0 = dn_state(DNS, 2.0, 0.0, 0.0, 1.0)
    U = zeros(2, 1)
    Z, _ = dynamic_rollout(DNV, z0, U, 0.1; substeps=4)
    far_obstacle = obstacle_on_track(DNS, 20.0, 0.0; radius=0.1)
    for obstacles in (CircleObstacle[], [far_obstacle])
        c = dn_ctrl(DNS; horizon=1, obstacles)
        v = validate_dynamic_plan(c, z0, z0[1], U, Z)
        @test isempty(v.violations)
        @test all(isfinite, v.frenet)
        @test v.frenet ≈ chain_frenet(DNS, Z, z0[1])
        @test v.collisions == 0
    end
end

@testset "solve, acceptance and independent validation" begin
    c = dn_ctrl(DNQ)
    z0 = dn_state(DNQ, 0.0, 0.0, 0.0, 1.0)
    r = solve!(c, z0; s_hint=DNP.lr)
    @test r.accepted && r.status === :success && r.termination_status === :LOCALLY_SOLVED
    @test abs(r.u[1]) <= DNP.delta_max && DNP.a_min <= r.u[2] <= DNP.a_max
    @test r.Z[:, 1] == z0 && size(r.path) == (2, 10 * 4 + 1) && r.path[:, end] ≈ r.Z[1:2, end]
    # The plan is consistent with the MPC's own RK4 rollout (dynamics residual) and the fine rollout.
    Zr, _ = dynamic_rollout(DNV, z0, r.U, 0.1; substeps=4)
    @test maximum(abs.(Zr - r.Z)) < 1e-5
    v = r.validation
    @test all(v.mismatch .< 5e-3) && v.min_track_clearance > 0 && v.max_slip <= DNP.alpha_max + 1e-2
    @test isempty(r.violations) && r.timing.total >= r.timing.solve > 0
    # Frenet coordinates of the plan agree with the true track projection.
    fr = chain_frenet(DNQ, r.Z, DNP.lr)
    @test maximum(abs.(fr[:, 2:end] - r.frenet[:, 2:end])) < 1e-6
    # Invalid measurements: no solve, no control.
    for bad in ([NaN, 0, 0, 1.0, 0, 0], dn_state(DNQ, 0.0, 0.0, 0.0, 0.2))
        rb = solve!(dn_ctrl(DNQ), bad; s_hint=0.0)
        @test rb.status === :invalid_measurement && !rb.accepted && rb.u === nothing && rb.nlp_solves == 0
    end
    rb = solve!(dn_ctrl(DNQ), [50.0, 50.0, 0.0, 1.0, 0.0, 0.0]; s_hint=0.0)               # no track projection
    @test rb.status === :invalid_measurement && rb.u === nothing
    # Solver failure (iteration limit 1): rejected, no control, warm start cleared.
    cf = dn_ctrl(DNQ; max_iter=1)
    cf.last_U = zeros(2, 10)
    rf = solve!(cf, z0; s_hint=DNP.lr)
    @test !rf.accepted && rf.u === nothing && rf.status in (:solver_failure,) && cf.last_U === nothing
    @test rf.termination_status !== :LOCALLY_SOLVED
end

@testset "closed loop on the quarter track (curvature signs, lap boundary)" begin
    c = dn_ctrl(DNQ)
    z0 = dn_state(DNQ, 0.0, 0.0, 0.0, 1.0)
    cl = simulate_dynamic_closed_loop(c, DNV, z0, 120; s0=DNP.lr)
    @test succeeded(cl) && isempty(cl.violations)
    @test all(r.accepted for r in cl.controller)
    @test cl.frenet[end][1] - cl.frenet[1][1] > track_length(DNQ)                         # past the lap boundary, unwrapped
    @test maximum(abs(f[2]) for f in cl.fine_frenet) < DNQ.width / 2
    @test all(z[4] >= DNP.vx_min - 1e-6 for z in cl.states)
    @test maximum(r.validation.mismatch[1] for r in cl.controller) < 5e-3
end

@testset "assignment cycle at a segment boundary (regression)" begin
    # The state of the dynamic race at cycle 225 (2.26 laps), where knot 10 sat 4–7 mm from
    # the straight → R = 2 m boundary and plain reassignment alternated between the two
    # segments until max_reassignments (:segment_assignment_invalid). Recorded with the
    # showcase configuration (time_budget = Inf) and the warm start of cycle 224.
    z = [2.670478811459989, 4.37857803272023, 13.147075995629782, 3.3930057641460323, -0.08642391234067412, 1.4596403586863413]
    s = 78.76649881886556
    u = [0.12252063026340716, 0.19297817807059983]
    U = [0.12252063026340716 0.09256122384967004 0.09256122040359388 0.09256122031392636 0.08075864415012775 0.0650734280258429 0.05155857012381587 0.04281825689628098 0.03897262262559634 0.03897262008934222;
        0.19297817807059983 -1.0787760215265918 0.3482585831335509 2.1550090859627034 3.910325566500675 4.902015799204961 4.590976091982374 3.129826966546384 1.2985809558832628 0.3977229186060519]
    c = dn_ctrl(DNQ)
    c.last_U = copy(U)
    r = solve!(c, z; previous_input=u, s_hint=s)
    @test r.accepted && r.status === :success && isempty(r.violations)
    asg = VehicleMPC.cartesian_assignment(DNQ, r.frenet[1, :])
    @test all(VehicleMPC.frame_valid(DNQ, asg.frames[j], r.frenet[1, j], 1e-6) for j in 2:11)
    # Pins never outlive their attempt: the next cycle starts with free knot progress.
    c.last_U = nothing
    r2 = solve!(c, dn_state(DNQ, 0.0, 0.0, 0.0, 1.0); s_hint=DNP.lr)
    @test r2.accepted && r2.nlp_solves == 1                                              # straight start: no reassignment, no pin
    @test !any(has_lower_bound, c.F[1, :]) && !any(has_upper_bound, c.F[1, :])
end

@testset "multiple obstacles: simultaneous constraints, separation radii, sides" begin
    # Two offset obstacles 1.8 m apart on a straight: both constrained in the same horizon.
    obs = [obstacle_on_track(DNS, 8.0, 0.10; radius=0.10), obstacle_on_track(DNS, 9.8, -0.10; radius=0.08)]
    c = dn_ctrl(DNS; obstacles=obs, buffers=[0.03, 0.05])
    @test required_separation(c, 1) ≈ 0.18 && required_separation(c, 2) ≈ 0.05 + 0.08 + 0.05
    z0 = dn_state(DNS, 5.5, 0.0, 0.0, 2.5)
    r = solve!(c, z0; s_hint=5.5 + DNP.lr)
    @test r.accepted
    @test sort(r.active_obstacles) == [1, 2]
    @test [parameter_value(c.op[3, i]) for i in 1:2] ≈ [0.18^2, 0.18^2]
    @test length(relevant_obstacles(c, 5.5)) == 2
    # Both obstacles are respected by the plan at every constrained point, and on the rollout.
    for (i, o) in enumerate(obs)
        D = required_separation(c, i)
        @test minimum(hypot(r.path[1, j] - o.x, r.path[2, j] - o.y) for j in axes(r.path, 2)) >= D - 1e-5
    end
    @test r.validation.collisions == 0
    # Passing decisions for both obstacles, consistent with where the plan goes.
    @test haskey(r.sides, 1) && haskey(r.sides, 2)
    for (i, o) in enumerate(obs)                                                      # straight track along +X: left = larger Y
        j = argmin([hypot(r.path[1, q] - o.x, r.path[2, q] - o.y) for q in axes(r.path, 2)])
        j < size(r.path, 2) && @test (r.path[2, j] > o.y) == (r.sides[i] === :left)
    end
    # Closed loop through both: no contact, opposite offsets handled, progress continues.
    c2 = dn_ctrl(DNS; obstacles=obs, buffers=[0.03, 0.05])
    cl = simulate_dynamic_closed_loop(c2, DNV, dn_state(DNS, 3.0, 0.0, 0.0, 1.5), 50; s0=3.0 + DNP.lr)
    @test succeeded(cl) && isempty(cl.violations)
    for (i, o) in enumerate(obs)
        @test minimum(hypot(z[1] - o.x, z[2] - o.y) for z in cl.fine_states) > 0.05 + o.radius      # no contact
    end
    @test cl.frenet[end][1] > 12.0
    # The last reported side of each obstacle is the side on which the plant actually passed it.
    sides = Dict(i => s for r in cl.controller for (i, s) in r.sides)
    for (i, o) in enumerate(obs)
        zc = cl.fine_states[argmin([hypot(z[1] - o.x, z[2] - o.y) for z in cl.fine_states])]
        @test (zc[2] > o.y) == (sides[i] === :left)
    end
end

@testset "obstacle slots: no stale constraints; transitions; blocked layout" begin
    # Near a curvature transition (straight → R = 1 m) on the quarter track.
    ob = obstacle_on_track(DNQ, 2.1, 0.0; radius=0.10)
    far = obstacle_on_track(DNQ, 25.0, 0.0; radius=0.10)
    c = dn_ctrl(DNQ; obstacles=[ob, far])
    cl = simulate_dynamic_closed_loop(c, DNV, dn_state(DNQ, 0.0, 0.0, 0.0, 1.0), 40; s0=DNP.lr)
    @test succeeded(cl) && isempty(cl.violations)
    @test minimum(hypot(z[1] - ob.x, z[2] - ob.y) for z in cl.fine_states) > 0.15
    r = cl.controller[end]
    @test 2 ∉ r.active_obstacles && parameter_value(c.op[3, 2]) == 0.0                   # far obstacle: inactive slot
    @test 1 ∉ r.active_obstacles || parameter_value(c.op[3, 1]) ≈ 0.18^2
    # A centred obstacle of radius 0.30 leaves no passage: explicit rejection, never contact.
    blk = obstacle_on_track(DNS, 6.0, 0.0; radius=0.30)
    cb = dn_ctrl(DNS; obstacles=[blk])
    clb = simulate_dynamic_closed_loop(cb, DNV, dn_state(DNS, 0.0, 0.0, 0.0, 1.0), 40; s0=DNP.lr)
    @test clb.status === :controller_failure && !clb.controller[end].accepted && clb.controller[end].u === nothing
    @test minimum(hypot(z[1] - blk.x, z[2] - blk.y) for z in clb.fine_states) > 0.05 + 0.30
    # Initial collision: no solve.
    ri = solve!(dn_ctrl(DNS; obstacles=[CircleObstacle(x=0.2, y=0.0, radius=0.1)]), dn_state(DNS, 0.0, 0.0, 0.0, 1.0); s_hint=DNP.lr)
    @test ri.status === :initial_collision && ri.u === nothing
end

@testset "independent clearance evaluation; collisions between constrained samples" begin
    o = obstacle_on_track(DNS, 4.0, 0.0; radius=0.10)
    c = dn_ctrl(DNS; obstacles=[o])
    z0 = dn_state(DNS, 2.0, 0.0, 0.0, 2.0)
    U = zeros(2, 10)                                                                   # straight through the obstacle
    Zp, _ = dynamic_rollout(DNV, z0, U, 0.1; substeps=4)
    v = validate_dynamic_plan(c, z0, 2.0 + DNP.lr, U, Zp)
    @test v.collisions > 0 && any(occursin("contact", s) for s in v.violations)
    fine = reduce(vcat, [integrate_dynamic(DNV, k == 1 ? z0 : Zp[:, k], U[:, k], 0.1) for k in 1:10])
    @test v.min_obstacle_distance ≈ minimum(hypot(z[1] - o.x, z[2] - o.y) for z in fine) atol = 1e-3
    # An obstacle placed midway between two constrained sample times on the path: the
    # constrained points may keep the separation, the dense evaluation finds the contact.
    o2 = CircleObstacle(x=Zp[1, 3] + 0.5 * 0.025 * 2.0, y=Zp[2, 3] + 0.13, radius=0.10)
    c2 = dn_ctrl(DNS; obstacles=[o2])
    v2 = validate_dynamic_plan(c2, z0, 2.0 + DNP.lr, U, Zp)
    @test v2.min_contact_clearance < 0.0 ? v2.collisions > 0 : v2.collisions == 0
end
