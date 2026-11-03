# Segment-reassignment reliability and encounter strategies (Milestone 3B.1).

const RRV = KinematicBicycle(reference_test_vehicle())
const RRQ = quarter_track()
const RRL = track_length(RRQ)
rrcfg(; kw...) = NMPCConfig(; prediction_model=:cartesian_endpoint_exact, horizon=10, dt=0.1, track_margin=0.05, schedule_corrections=8,
    correction_time_budget=1e3, interior_samples=0, strict_corridor=false, ego_radius=0.05, obstacle_buffer=0.03, obstacle_samples=3, kw...)
const RROB = obstacle_on_track(RRQ, 8.2, 0.0; radius=0.10)          # scenario E3

# The recorded E3 failure: the original controller fails at the second-lap cycle 118.
const RRE3 = let c = NMPCController(RRV, RRQ, rrcfg(); obstacle=RROB)
    cl = simulate_closed_loop(c, RRV, [0.0, 0.0, 0.0, 0.5], 118)
    (; cl, x=cl.states[end], u=cl.inputs[end])
end

@testset "cycle detection on assignment vectors" begin
    A, B, C = [1, 1, 2, 2], [1, 1, 1, 2], [1, 2, 2, 2]
    cyc = detect_assignment_cycle([A, B], A)                           # A, B, A: period two
    @test cyc.length == 2 && cyc.knots == [3] && cyc.first == 1
    @test detect_assignment_cycle([A], A).length == 1                  # an immediate repeat
    c3 = detect_assignment_cycle([A, B, C], B)                         # B, C, B (A not in the cycle)
    @test c3.length == 2 && c3.first == 2 && c3.knots == [2, 3]
    @test detect_assignment_cycle([A, B], C) === nothing
    @test detect_assignment_cycle(Vector{Int}[], A) === nothing
    # Lap-aware ids: the same segment on the next lap is a different configuration.
    n = num_segments(RRQ)
    ids1 = cartesian_assignment(RRQ, [8.0, 8.5]).seg
    ids2 = cartesian_assignment(RRQ, [8.0 + RRL, 8.5 + RRL]).seg
    @test ids2 == ids1 .+ n && detect_assignment_cycle([ids1], ids2) === nothing
end

@testset "temporary progress bounds are restored" begin
    c = NMPCController(RRV, RRQ, rrcfg(); obstacle=RROB)
    v = c.X[1, 5]
    @test !has_lower_bound(v) && !has_upper_bound(v)
    VehicleMPC.with_progress_bounds(v, 1.0, 2.0) do
        @test lower_bound(v) == 1.0 && upper_bound(v) == 2.0
    end
    @test !has_lower_bound(v) && !has_upper_bound(v)
    @test_throws ErrorException VehicleMPC.with_progress_bounds(() -> error("inside"), v, 1.0, 2.0)
    @test !has_lower_bound(v) && !has_upper_bound(v)                   # restored after an exception
    set_lower_bound(v, -5.0); set_upper_bound(v, 50.0)                 # pre-existing bounds come back
    VehicleMPC.with_progress_bounds(() -> nothing, v, 1.0, 2.0)
    @test lower_bound(v) == -5.0 && upper_bound(v) == 50.0
end

@testset "explicit branch assignment installs the exact geometry" begin
    c = NMPCController(RRV, RRQ, rrcfg(); obstacle=RROB)
    x = [7.6, 0.02, 0.0, 2.0]
    g = side_guess(c, x, :left)
    ids = cartesian_assignment(RRQ, g.Xg[1, :]).seg
    ids[end] += 1                                                      # a deliberate branch choice for the last knot
    s = install_cartesian_ids!(c, (; g..., kappas=g.kappas), ids)
    @test s.asg.seg == ids
    for k in eachindex(ids)
        f = segment_frame(RRQ, ids[k])
        @test parameter_value.(c.cframe[:, k]) == [f.s0, f.x0, f.y0, f.psi0, f.kappa]
    end
end

@testset "E3 replay: oscillation, detection, branch resolution" begin
    x, up = RRE3.x, RRE3.u
    @test RRE3.cl.status === :controller_failure && length(RRE3.cl.controller) == 118
    # Original: 9 solves alternating knot 10 across the boundary s = 43.133 (lap 2).
    c0 = NMPCController(RRV, RRQ, rrcfg(); obstacle=RROB)
    r0 = solve_once!(c0, x; previous_input=up, initial=side_guess(c0, x, :left))
    @test r0.status === :segment_assignment_invalid && r0.nlp_solves == 9 && r0.reassignment.cycles == 0
    tr = r0.reassignment.trace
    @test all(e -> e.outside == [10], tr[2:end])
    bnd = RRQ.s_start[4] + RRL
    @test all(isodd(e.solve) ? e.outside_s[1] < bnd : e.outside_s[1] > bnd for e in tr[2:end])     # alternating sides
    # Detection: stops after 3 solves with a period-two cycle in knot 10.
    cd = NMPCController(RRV, RRQ, rrcfg(reassignment_cycle_detection=true); obstacle=RROB)
    rd = solve_once!(cd, x; previous_input=up, initial=side_guess(cd, x, :left))
    @test rd.status === :segment_assignment_invalid && rd.nlp_solves == 3
    @test rd.reassignment.cycles == 1 && rd.reassignment.cycle_length == 2 && rd.reassignment.cycle_knots == [10]
    # Branch resolution: knot 10 bounded to the R -1 m arc gives a valid solution.
    cb = NMPCController(RRV, RRQ, rrcfg(reassignment_cycle_detection=true, branch_resolution=true); obstacle=RROB)
    rb = solve_once!(cb, x; previous_input=up, initial=side_guess(cb, x, :left))
    seg_m1 = segment_id(RRQ, bnd - 0.01, :right)
    @test rb.accepted && rb.status === :success && rb.reassignment.branch_selected == seg_m1
    @test length(rb.reassignment.branch) == 2 && rb.nlp_solves == 5
    @test any(b -> b.segment == seg_m1 && b.status === :success, rb.reassignment.branch)
    @test rb.X[1, 10] <= bnd + 1e-6 && rb.max_dynamics_residual <= 1e-6
    @test !has_lower_bound(cb.X[1, 10]) && !has_upper_bound(cb.X[1, 10])   # the temporary bound is gone
    # No contamination: a later solve on this controller equals one on a fresh controller.
    y = [10.0, 0.0, 0.0, 2.0]
    ra = solve_once!(cb, y)
    rf = solve_once!(NMPCController(RRV, RRQ, rrcfg(reassignment_cycle_detection=true, branch_resolution=true); obstacle=RROB), y)
    @test ra.accepted && rf.accepted && maximum(abs.(ra.X - rf.X)) <= 1e-9
    # Through the obstacle wrapper (both candidates), the cycle is accepted without collision.
    rw = solve!(NMPCController(RRV, RRQ, rrcfg(reassignment_cycle_detection=true, branch_resolution=true); obstacle=RROB), x; previous_input=up)
    @test rw.accepted && rw.obstacle.collisions == 0 && rw.obstacle.check_samples == 50 && rw.reassignment.cycles >= 1
end

@testset "branch resolution: infeasible candidates, budget, non-applicable cycles" begin
    x, up = RRE3.x, RRE3.u
    cd = NMPCController(RRV, RRQ, rrcfg(reassignment_cycle_detection=true); obstacle=RROB)
    rd = solve_once!(cd, x; previous_input=up, initial=side_guess(cd, x, :left))
    tr = rd.reassignment.trace
    g = side_guess(cd, x, :left)
    hist = [(; ids=e.ids, X=g.Xg, U=g.Ug, s=g.Xg[1, :], status=e.status, objective=e.objective) for e in tr[2:3]]
    cyc = detect_assignment_cycle([h.ids for h in hist], tr[2].ids)
    @test cyc.knots == [10]
    sched = (; g..., kappas=g.kappas, asg=cartesian_assignment(RRQ, g.Xg[1, :]))
    # Infeasible: with one Ipopt iteration both candidates fail; no solution, bound restored.
    ci = NMPCController(RRV, RRQ, rrcfg(reassignment_cycle_detection=true, branch_resolution=true, max_iter=1); obstacle=RROB)
    set_parameter_value.(ci.X[:, 1], x)
    bi = VehicleMPC.resolve_branch!(ci, SVector{4}(x...), sched, hist, cyc, time_ns(), time_ns(), 0.0)
    @test bi.chosen === nothing && length(bi.attempts) == 2 && all(a -> a.status !== :success, bi.attempts)
    @test occursin("no valid candidate", bi.message) && !has_lower_bound(ci.X[1, 10])
    # Budget exhausted: no candidate is started.
    bb = VehicleMPC.resolve_branch!(ci, SVector{4}(x...), sched, hist, cyc, time_ns(), time_ns() - 10^12, 0.0)
    @test bb.chosen === nothing && bb.solves == 0 && all(a -> a.status === :not_attempted_budget, bb.attempts)
    # Several knots, or non-adjacent segments: reported, not searched.
    multi = (; length=2, knots=[9, 10], first=1)
    @test occursin("involves 2 knots", VehicleMPC.resolve_branch!(ci, SVector{4}(x...), sched, hist, multi, time_ns(), time_ns(), 0.0).message)
    far = [(; h..., ids=(i == 2 ? (v = copy(h.ids); v[10] += 2; v) : h.ids)) for (i, h) in enumerate(hist)]
    @test occursin("not two adjacent", VehicleMPC.resolve_branch!(ci, SVector{4}(x...), sched, far, cyc, time_ns(), time_ns(), 0.0).message)
    @test_throws ArgumentError NMPCController(RRV, RRQ, rrcfg(branch_resolution=true); obstacle=RROB)
end

@testset "sequential encounters and fallback" begin
    ob = obstacle_on_track(RRQ, 3.5, 0.0; radius=0.10)
    x = [1.2, 0.0, 0.0, 2.0]
    rb = solve!(NMPCController(RRV, RRQ, rrcfg(); obstacle=ob), x)
    rs = solve!(NMPCController(RRV, RRQ, rrcfg(encounter_strategy=:sequential); obstacle=ob), x)
    @test rb.accepted && rs.accepted && length(rb.obstacle.guesses) == 2 && length(rs.obstacle.guesses) == 2
    @test count(a -> a[3] > 0, rs.obstacle.attempts) == 1 && any(a -> a[2] === :not_attempted_sequential, rs.obstacle.attempts)
    @test rs.nlp_solves < rb.nlp_solves && rs.obstacle.collisions == 0 && rs.obstacle.check_samples == 50
    # Fallback: the first candidate is rejected, so the second is solved.
    ri = solve!(NMPCController(RRV, RRQ, rrcfg(encounter_strategy=:sequential); obstacle=obstacle_on_track(RRQ, 25.9, 0.0; radius=0.10)),
        [25.0, 0.0, 0.0, 5.0])
    @test !ri.accepted && count(a -> a[3] > 0, ri.obstacle.attempts) == 2
    # A tight cycle budget skips the optional second candidate.
    rt = solve!(NMPCController(RRV, RRQ, rrcfg(cycle_budget=1e-6); obstacle=ob), x)
    @test any(a -> a[2] === :not_attempted_budget, rt.obstacle.attempts) && count(a -> a[3] > 0, rt.obstacle.attempts) == 1
    @test all(isfinite, rt.obstacle.phase_times) && isapprox(sum(rt.obstacle.phase_times), rt.timing.total; rtol=0.05)
    @test_throws ArgumentError NMPCController(RRV, RRQ, rrcfg(encounter_strategy=:random); obstacle=ob)
end

@testset "regressions: defaults unchanged, straight track unaffected" begin
    @test NMPCConfig().reassignment_cycle_detection == false && NMPCConfig().branch_resolution == false
    @test NMPCConfig().encounter_strategy === :both && NMPCConfig().cycle_budget == Inf
    T = straight_track(60.0; width=0.7)
    ob = CircleObstacle(x=8.0, y=0.0, radius=0.10)
    a = simulate_closed_loop(NMPCController(RRV, T, rrcfg(horizon=20); obstacle=ob), RRV, [0.0, 0.0, 0.0, 0.5], 40)
    b = simulate_closed_loop(NMPCController(RRV, T, rrcfg(horizon=20, reassignment_cycle_detection=true, branch_resolution=true); obstacle=ob),
        RRV, [0.0, 0.0, 0.0, 0.5], 40)
    @test succeeded(a) && a.states == b.states
end
