# Stationary-obstacle avoidance on the multi-curvature quarter track (Milestone 3B).

const MCV = KinematicBicycle(reference_test_vehicle())
const MCQ = quarter_track()
const MCL = track_length(MCQ)
mccfg(; kw...) = NMPCConfig(; prediction_model=:cartesian_endpoint_exact, horizon=10, dt=0.1, track_margin=0.05, schedule_corrections=8,
    correction_time_budget=100.0, interior_samples=0, strict_corridor=false, ego_radius=0.05, obstacle_buffer=0.03, obstacle_samples=3, kw...)
mcctrl(ob; kw...) = NMPCController(MCV, MCQ, mccfg(; kw...); obstacle=ob)
mcdist(ob, states) = minimum(hypot(p[1] - ob.x, p[2] - ob.y) for p in (frenet_to_cartesian(MCQ, x[1], x[2], x[3]) for x in states))

@testset "placement and associations on curved segments" begin
    for (s, ey) in ((3.5, 0.0), (3.5, 0.08), (6.6, -0.08), (9.7, 0.05), (1.95, 0.0), (8.4, -0.1))
        ob = obstacle_on_track(MCQ, s, ey; radius=0.10)
        @test collect(frenet_to_cartesian(MCQ, s, ey)[1:2]) ≈ [ob.x, ob.y]
        c = mcctrl(ob)
        as = obstacle_associations(c, s - 1.0, s + 1.0)
        @test length(as) == 1 && isapprox(as[1].s, s; atol=1e-9) && isapprox(as[1].ey, ey; atol=1e-9) && isapprox(as[1].dist, abs(ey); atol=1e-9)
    end
    # Positive and negative curvature segments are both present.
    @test curvature(MCQ, 3.5) > 0 && curvature(MCQ, 6.6) < 0 && curvature(MCQ, 9.7) ≈ 0.5
end

@testset "relevance across boundaries and laps; ambiguity; centres off the track" begin
    ob = obstacle_on_track(MCQ, 2.1, 0.0; radius=0.10)
    c = mcctrl(ob)
    @test obstacle_relevant(c, [0.0, 0.0, 0.0, 0.5])                 # across the straight → arc boundary
    @test !obstacle_relevant(c, [12.0, 0.0, 0.0, 2.0])               # behind (and the next lap is out of reach)
    @test obstacle_relevant(c, [MCL - 1.0, 0.0, 0.0, 2.0])            # next lap: same physical obstacle
    so, _ = obstacle_frenet(c, [MCL - 1.0, 0.0, 0.0, 2.0])
    @test so ≈ MCL + 2.1 atol = 1e-9                                  # unwrapped forward progress
    # Lap boundary: the obstacle near s = L appears at L - 0.3 and, one lap later, at 2L - 0.3.
    cf = mcctrl(obstacle_on_track(MCQ, MCL - 0.3, 0.0; radius=0.10))
    @test obstacle_frenet(cf, [MCL - 2.0, 0.0, 0.0, 2.0])[1] ≈ MCL - 0.3 atol = 1e-9
    @test obstacle_frenet(cf, [2MCL - 2.0, 0.0, 0.0, 2.0])[1] ≈ 2MCL - 0.3 atol = 1e-9
    @test obstacle_frenet(cf, [MCL + 0.1, 0.0, 0.0, 2.0])[1] ≈ MCL - 0.3 atol = 1e-9   # just passed: still within D + E behind
    # An obstacle whose centre is outside the physical track but whose disk reaches the corridor is relevant.
    c42 = mcctrl(obstacle_on_track(MCQ, 9.7, 0.42; radius=0.10))
    @test obstacle_relevant(c42, [8.0, 0.0, 0.0, 2.0]) && passage_sides(c42, [8.0, 0.0, 0.0, 2.0]) == (false, true)
    c60 = mcctrl(obstacle_on_track(MCQ, 9.7, 0.60; radius=0.10))
    @test !obstacle_relevant(c60, [8.0, 0.0, 0.0, 2.0])              # 0.60 m > ey_max + D = 0.48 m
    # Ambiguous geometry: two parallel straights 0.5 m apart (hairpin); both passes are reported.
    hp = Track([TrackSegment(3.0, 0.0), TrackSegment(π * 0.25, 4.0), TrackSegment(3.0, 0.0)]; width=0.4)
    ch = NMPCController(MCV, hp, NMPCConfig(prediction_model=:cartesian_endpoint_exact, track_margin=0.05, ego_radius=0.05,
        obstacle_buffer=0.05, obstacle_samples=1); obstacle=CircleObstacle(x=1.5, y=0.25, radius=0.05))   # reach E + D = 0.30 m
    as = obstacle_associations(ch, 0.0, track_length(hp))
    @test length(as) == 2 && isapprox(as[1].s, 1.5; atol=1e-9) && isapprox(as[2].s, 3.0 + π * 0.25 + 1.5; atol=1e-9)
end

@testset "side guesses on both curvature signs" begin
    p = MCV.params
    for (s, x) in ((3.5, [2.2, 0.0, 0.0, 1.8]), (6.6, [5.3, 0.0, 0.0, 1.8]))
        c = mcctrl(obstacle_on_track(MCQ, s, 0.0; radius=0.10))
        for side in (:left, :right)
            g = side_guess(c, x, side)
            @test g.converged && g.rollout_ok
            k = argmin(abs.(g.Xg[1, :] .- s))
            @test side === :left ? g.Xg[2, k] > 0.05 : g.Xg[2, k] < -0.05
            @test all(abs.(g.Ug[1, :]) .<= p.delta_max) && g.track_clearance > 0
            @test all(abs(lateral_acceleration(MCV, g.Xg[4, k], g.Ug[1, k])) <= c.config.ay_max + 1e-9 for k in 1:10)
            sim = simulate_on_track(MCV, MCQ, x, [g.Ug[:, k] for k in 1:10], 0.1; max_step=0.02)
            @test maximum(maximum(abs.(sim.states[k] - g.Xg[:, k])) for k in 1:11) <= 1e-12     # a model trajectory
        end
    end
end

@testset "closed-loop avoidance on both curvature signs" begin
    for s in (3.5, 6.6)
        ob = obstacle_on_track(MCQ, s, 0.0; radius=0.10)
        c = mcctrl(ob)
        cl = simulate_closed_loop(c, MCV, [0.0, 0.0, 0.0, 0.5], 60)
        @test succeeded(cl) && cl.states[end][1] > s + 1.0
        @test mcdist(ob, cl.fine_states) > 0.15                      # no contact of the disks
        @test isempty(cl.violations)                                 # physical track, speed and inputs
        rs = cl.controller
        @test all(r -> r.obstacle.collisions == 0 && r.obstacle.knot_min_clearance >= -1e-6, rs)
        @test any(r -> length(r.obstacle.attempts) == 2, rs)         # an encounter with two candidates
        @test all(r -> r.max_dynamics_residual <= 1e-6, rs)
        # Segment reassignment happened during the run while the constraints stayed valid.
        @test any(r -> r.nlp_solves > count(a -> a[3] > 0, r.obstacle.attempts), rs)
    end
end

@testset "collision constraints with updated segment frames; dense check across a transition" begin
    ob = obstacle_on_track(MCQ, 2.0, 0.0; radius=0.10)             # on the straight → R 1 m boundary
    c = mcctrl(ob)
    x = [0.8, 0.0, 0.0, 2.0]
    r = solve!(c, x)
    @test r.accepted
    # Two candidates were solved; reinstall the chosen result's segment assignment first.
    install_schedule!(c, (; kappas=r.schedule.kappas, s_nominal=r.schedule.s_nominal, Xg=r.X, Ug=r.U))
    vals = Dict{VariableRef,Float64}()
    for k in 2:11, i in 1:4
        vals[c.X[i, k]] = r.X[i, k]
    end
    for k in 1:10, i in 1:2
        vals[c.U[i, k]] = r.U[i, k]
    end
    valfn = v -> is_parameter(v) ? parameter_value(v) : vals[v]
    D = 0.18
    for k in 2:11                                                    # knot constraints use the current frames
        q = frenet_state_to_cartesian(MCQ, r.X[:, k])
        @test value(valfn, constraint_object(c.obs_cons[k-1]).func) ≈ obstacle_margin(q[1], q[2], ob.x, ob.y, D) atol = 1e-9
    end
    # A no-obstacle plan crossing the boundary, with an obstacle placed on it mid-interval:
    # the dense evaluation finds the collision between the knots.
    c0 = NMPCController(MCV, MCQ, mccfg())
    r0 = solve!(c0, x)
    k = findfirst(j -> r0.X[1, j] < 2.0 <= r0.X[1, j+1], 1:10)
    pk = frenet_state_to_cartesian(MCQ, r0.X[:, k])
    q = interior_point(MCV.params.wheelbase, pk, r0.X[4, k], r0.U[1, k], r0.U[2, k], 0.05)
    set_obstacle!(c, CircleObstacle(x=q[1], y=q[2], radius=0.10))
    cc = collision_check(c, x, r0.X, r0.U)
    @test cc.collisions > 0 && cc.min_distance < 1e-6 && cc.between
end

@testset "repeated encounter of the same obstacle on the next lap" begin
    ob = obstacle_on_track(MCQ, MCL - 0.3, 0.0; radius=0.10)
    c = mcctrl(ob)
    cl = simulate_closed_loop(c, MCV, [0.0, 0.0, 0.0, 0.5], 200)
    @test succeeded(cl) && cl.states[end][1] > 2MCL - 0.3 + 1.0      # passed it twice
    enc = [r for r in cl.controller if length(r.obstacle.attempts) == 2]
    ss = [r.obstacle.association_s for r in enc]
    @test any(s -> abs(s - (MCL - 0.3)) < 1e-6, ss) && any(s -> abs(s - (2MCL - 0.3)) < 1e-6, ss)
    @test !any(s -> s < 0, ss)                                       # the obstacle just behind the start is not an encounter
    @test mcdist(ob, cl.fine_states) > 0.15 && isempty(cl.violations)
end

@testset "failure classification: geometric block, dynamic infeasibility" begin
    rb = solve!(mcctrl(obstacle_on_track(MCQ, 3.5, 0.0; radius=0.30)), [2.4, 0.0, 0.0, 2.0])
    @test !rb.accepted && rb.u === nothing && rb.obstacle.failure_cause === :infeasible_geometry
    @test rb.obstacle.passage_left == false && rb.obstacle.passage_right == false
    # Geometrically open, but 0.9 m ahead at 5 m/s: no guess can clear it.
    rd = solve!(mcctrl(obstacle_on_track(MCQ, 25.9, 0.0; radius=0.10)), [25.0, 0.0, 0.0, 5.0])
    @test !rd.accepted && rd.u === nothing && rd.obstacle.passage_left && rd.obstacle.passage_right
    @test rd.obstacle.failure_cause in (:dynamically_infeasible, :collision, :constraint_violation)
    @test length(rd.obstacle.guesses) == 2 && all(g -> g[2] < 0, rd.obstacle.guesses)
    @test_throws ArgumentError NMPCConfig(obstacle_ramp=0.1, obstacle_ramp_min=0.3) |> c -> NMPCController(MCV, MCQ, c)
end

@testset "no-obstacle quarter-track regression" begin
    a = simulate_closed_loop(NMPCController(MCV, MCQ, mccfg()), MCV, [0.0, 0.0, 0.0, 0.5], 40)
    b = simulate_closed_loop(NMPCController(MCV, MCQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact, horizon=10,
        schedule_corrections=8, correction_time_budget=100.0)), MCV, [0.0, 0.0, 0.0, 0.5], 40)
    @test a.states == b.states && all(r -> !r.obstacle.present, a.controller)
end
