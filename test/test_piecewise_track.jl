# Event-aware integration on multi-segment tracks, checked against exact solutions and the
# independent Cartesian model.

const PV = KinematicBicycle(reference_test_vehicle())
const PL = PV.params.wheelbase
# Open track: straight 2 m -> left quarter circle R = 1 -> straight 2 m.
const BEND = Track([TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0)]; width=0.7)
# Closed rounded rectangle that starts with an arc, so the lap boundary is a straight -> arc jump.
const LOOP = Track([TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0),
        TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0)]; width=0.7, closed=true)

"Cartesian distance and heading difference between a Frenet state and a Cartesian state."
function cart_gap(track, x, z)
    p = frenet_state_to_cartesian(track, x)
    return hypot(p[1] - z[1], p[2] - z[2]), abs(p[3] - z[3]), abs(p[4] - z[4])
end

@testset "locate_crossing" begin
    tau, ok = locate_crossing(t -> t - 0.3, 1.0)
    @test ok && isapprox(tau, 0.3; atol=1e-12) && tau - 0.3 >= 0   # returned point is on/after the root
    tau, ok = locate_crossing(t -> exp(t) - 2, 1.0; tol=1e-14)
    @test ok && isapprox(tau, log(2); atol=1e-13)
    @test !locate_crossing(t -> t + 1, 1.0)[2]                     # no sign change: reported, not guessed
    @test !locate_crossing(t -> t - 2, 1.0)[2]
end

@testset "no boundary crossed: identical to the constant-curvature RK4" begin
    for (track, x) in ((BEND, [0.2, 0.1, 0.05, 1.5]), (BEND, [2.3, -0.1, 0.1, 2.0]), (LOOP, [3.0, 0.0, 0.0, 1.0]))
        u = [0.1, 0.4]
        r = integrate_on_track(PV, track, x, u, 0.1; max_step=0.1)
        @test succeeded(r) && isempty(r.crossings)
        @test r.x == discrete_dynamics(PV, x, u, curvature(track, x[1]), 0.1, RK4())
        # With substeps it is RK4 with a smaller step: agreement to integration accuracy.
        r2 = integrate_on_track(PV, track, x, u, 0.1; max_step=0.01)
        @test maximum(abs, r2.x - r.x) <= 1e-5                     # RK4 truncation at dt = 0.1
    end
end

@testset "event time, curvature after the crossing, continuity" begin
    # Centreline, heading aligned, delta = 0, a = 0: sdot = v on the straight, so the
    # crossing time is exact: (2 - s0)/v.
    x0, u = [1.9, 0.0, 0.0, 2.0], [0.0, 0.0]
    r = integrate_on_track(PV, BEND, x0, u, 0.1; max_step=0.1)
    @test succeeded(r) && length(r.crossings) == 1
    c = r.crossings[1]
    @test c.t ≈ 0.05 atol = 1e-12
    @test c.s == 2.0 && c.from == 1 && c.to == 2 && c.lap == 0
    @test c.x[1] >= 2.0 && c.x[1] - 2.0 <= 1e-12                   # state on/just past the boundary
    @test c.x ≈ [2.0, 0.0, 0.0, 2.0] atol = 1e-12                   # continuous Frenet state
    # After the crossing the arc's curvature is used: equal to composing two RK4 steps.
    manual = discrete_dynamics(PV, discrete_dynamics(PV, x0, u, 0.0, c.t, RK4()), u, 1.0, 0.1 - c.t, RK4())
    @test maximum(abs, r.x - manual) <= 1e-12
    # Exact truth: delta = 0 drives a straight line, X = s0 + v t, Y = 0.
    rf = integrate_on_track(PV, BEND, x0, u, 0.1; max_step=1e-3)
    X, Y, _ = frenet_to_cartesian(BEND, rf.x[1], rf.x[2], rf.x[3])
    @test isapprox(X, 2.1; atol=1e-12) && abs(Y) <= 1e-12
    # Splitting the interval into two calls gives the same state (state continuity).
    ra = integrate_on_track(PV, BEND, x0, u, 0.04; max_step=0.01)
    rb = integrate_on_track(PV, BEND, ra.x, u, 0.06; max_step=0.01)
    rc = integrate_on_track(PV, BEND, x0, u, 0.1; max_step=0.01)
    @test maximum(abs, rb.x - rc.x) <= 1e-12

    # Constant steering, a = 0: the exact path is a circle of radius L/tan(delta).
    delta = 0.3
    Rv = PL / tan(delta)
    x0 = [1.7, 0.0, 0.0, 1.5]
    r = integrate_on_track(PV, BEND, x0, [delta, 0.0], 0.4; max_step=0.005)
    @test succeeded(r) && length(r.crossings) == 1
    th = 1.5 * 0.4 / Rv
    Xe, Ye, psie = 1.7 + Rv * sin(th), Rv * (1 - cos(th)), th
    X, Y, psi = frenet_to_cartesian(BEND, r.x[1], r.x[2], r.x[3])
    @test hypot(X - Xe, Y - Ye) <= 1e-9 && abs(psi - psie) <= 1e-9
end

@testset "multiple crossings, negative curvature, Cartesian agreement" begin
    lab = legacy_track(:lab)                  # open; segments of 0.37–1.95 m, curvature ±1/0.7, 1/0.66, 1/1.44
    i0 = 5                                    # 0.6 m straight, then 0.37 m arc, 0.41 m straight, 0.37 m arc with κ < 0
    x0 = [lab.s_start[i0] + 0.1, 0.0, 0.0, 4.0]
    u = [0.05, 0.0]
    r = integrate_on_track(PV, lab, x0, u, 0.5; max_step=1e-3)
    @test succeeded(r)
    @test length(r.crossings) >= 3
    @test [c.from for c in r.crossings] == i0 .+ (0:length(r.crossings)-1)
    @test [c.to for c in r.crossings] == i0 .+ (1:length(r.crossings))
    @test issorted([c.t for c in r.crossings]) && issorted([c.s for c in r.crossings])
    @test any(c -> lab.segments[c.to].curvature < 0, r.crossings)   # enters the negative-curvature arc
    z = integrate_cartesian(PV, frenet_state_to_cartesian(lab, x0), u, 0.5; max_step=1e-4)
    gap = cart_gap(lab, r.x, z)
    @test gap[1] <= 1e-9 && gap[2] <= 1e-9 && gap[3] <= 1e-12

    # Quarter track: straight -> R=1 left -> R=1 right (kappa +1 -> -1) over several intervals.
    tr = quarter_track()
    x0 = [1.5, 0.05, -0.02, 2.0]
    snom = 1.5 .+ 2.0 .* (0:30) .* 0.1
    inputs = [[atan(PL * average_curvature(tr, snom[k], snom[k+1])) + 0.05 * sin(0.5k), 0.2] for k in 1:30]
    sim = simulate_on_track(PV, tr, x0, inputs, 0.1; max_step=1e-3)
    @test succeeded(sim) && length(sim.crossings) >= 2
    @test any(c -> c.from == 2 && c.to == 3, sim.crossings)         # +1 -> -1 transition
    zs = simulate_cartesian(PV, frenet_state_to_cartesian(tr, x0), inputs, 0.1; max_step=1e-4)
    @test maximum(cart_gap(tr, sim.states[k], zs[k])[1] for k in eachindex(zs)) <= 1e-8
    @test maximum(cart_gap(tr, sim.states[k], zs[k])[2] for k in eachindex(zs)) <= 1e-8
end

@testset "closed-track lap boundary keeps progress unwrapped" begin
    L = track_length(LOOP)
    x0 = [3L - 0.05, 0.0, 0.0, 1.0]           # end of lap 2, last segment (straight)
    r = integrate_on_track(PV, LOOP, x0, [0.0, 0.0], 0.1; max_step=0.01)
    @test succeeded(r) && length(r.crossings) == 1
    c = r.crossings[1]
    @test c.from == num_segments(LOOP) && c.to == 1 && c.lap == 3
    @test c.s ≈ 3L && isapprox(c.t, 0.05; atol=1e-10)
    # delta = 0: the vehicle continues along the tangent while the R = 1 arc curves away, so
    # its progress after d = 0.05 m is exactly R·atan(d/R) (not d). Unwrapped, not reset.
    @test isapprox(r.x[1], 3L + atan(0.05); atol=1e-7)
    @test r.x[1] > 3L
    # Several laps with the Cartesian cross-check (projection needs no hint here).
    x0 = [0.3, 0.0, 0.0, 2.0]
    inputs = [[atan(PL * average_curvature(LOOP, 0.3 + 0.2(k - 1), 0.3 + 0.2k)), 0.0] for k in 1:80]
    sim = simulate_on_track(PV, LOOP, x0, inputs, 0.1; max_step=1e-3)
    @test succeeded(sim) && sim.states[end][1] > L && any(c -> c.lap >= 1, sim.crossings)
    zs = simulate_cartesian(PV, frenet_state_to_cartesian(LOOP, x0), inputs, 0.1; max_step=1e-4)
    @test maximum(cart_gap(LOOP, sim.states[k], zs[k])[1] for k in eachindex(zs)) <= 1e-8
    # Back-projection of the Cartesian reference with progress hints recovers the Frenet state.
    for k in (10, 40, 80)
        s, ey, epsi = cartesian_to_frenet(LOOP, zs[k][1], zs[k][2], zs[k][3]; s_hint=sim.states[k][1])
        @test abs(s - sim.states[k][1]) <= 1e-8 && abs(ey - sim.states[k][2]) <= 1e-8
        @test abs(rem2pi(epsi - sim.states[k][3], RoundNearest)) <= 1e-8
    end
end

@testset "behaviour on and near boundaries" begin
    u = [0.0, 0.0]
    # Exactly on a boundary: belongs to the next segment (half-open); no event is generated.
    r = integrate_on_track(PV, BEND, [2.0, 0.0, 0.0, 1.0], u, 0.1; max_step=0.1)
    @test succeeded(r) && isempty(r.crossings)
    @test r.x == discrete_dynamics(PV, [2.0, 0.0, 0.0, 1.0], u, 1.0, 0.1, RK4())
    # One ulp before the boundary: one crossing at (almost) zero time, no loop.
    r = integrate_on_track(PV, BEND, [prevfloat(2.0), 0.0, 0.0, 1.0], u, 0.1)
    @test succeeded(r) && length(r.crossings) == 1 && r.crossings[1].t <= 1e-12
    # Standing still on a boundary: nothing happens, no infinite loop.
    r = integrate_on_track(PV, BEND, [2.0, 0.0, 0.0, 0.0], u, 0.5)
    @test succeeded(r) && isempty(r.crossings) && r.x == [2.0, 0.0, 0.0, 0.0]
    # Driving backwards relative to the centreline (epsi = π): crossing in the -s direction.
    # On the straight that follows the arc, sdot = -v exactly, so the crossing time is exact.
    sb = 2.0 + π / 2
    r = integrate_on_track(PV, BEND, [sb + 0.05, 0.0, π, 1.0], u, 0.1; max_step=0.01)
    @test succeeded(r) && length(r.crossings) == 1
    @test r.crossings[1].from == 3 && r.crossings[1].to == 2 && isapprox(r.crossings[1].t, 0.05; atol=1e-12)
    @test r.x[1] < sb
end

@testset "open-track ends and failure statuses" begin
    st = straight_track(1.0)
    r = integrate_on_track(PV, st, [0.95, 0.0, 0.0, 1.0], [0.0, 0.0], 0.1)
    @test r.status === :end_of_track && isapprox(r.t, 0.05; atol=1e-10) && isapprox(r.x[1], 1.0; atol=1e-12)
    r = integrate_on_track(PV, st, [0.05, 0.0, π, 1.0], [0.0, 0.0], 0.1)
    @test r.status === :start_of_track && isapprox(r.t, 0.05; atol=1e-10)
    sim = simulate_on_track(PV, st, [0.5, 0.0, 0.0, 1.0], fill([0.0, 0.0], 10), 0.1)
    @test sim.status === :end_of_track && isapprox(sim.t[end], 0.5; atol=1e-9) && length(sim.states) == 6
    @test_throws DomainError integrate_on_track(PV, st, [1.5, 0.0, 0.0, 1.0], [0.0, 0.0], 0.1)
    # Model domain: steering into the centre of a tight arc reaches 1 - kappa*ey ≤ eps_geometry.
    tight = Track([TrackSegment(0.5, 0.0), TrackSegment(π / 2, 2.0)])
    # Start at 1 - kappa*ey = 0.12 (limit 0.1) heading inwards: ey grows by 0.01 m within ~4 ms.
    r = integrate_on_track(PV, tight, [0.6, 0.44, 1.2, 3.0], [0.4, 0.0], 1.0)
    @test r.status === :domain_error && r.t < 1.0
    # Safeguard against unbounded event counts.
    lab = legacy_track(:lab)
    r = integrate_on_track(PV, lab, [lab.s_start[5] + 0.1, 0.0, 0.0, 4.0], [0.05, 0.0], 0.5; max_events=1)
    @test r.status === :too_many_events
    @test_throws ArgumentError integrate_on_track(PV, BEND, [0.0, 0.0, 0.0, 1.0], [0.0, 0.0], 0.1; max_step=0.0)
end

@testset "integration error decreases with resolution (4th order through events)" begin
    tr = quarter_track()
    x0 = [1.6, 0.05, 0.0, 2.0]
    u = [0.2, 0.5]
    zref = integrate_cartesian(PV, frenet_state_to_cartesian(tr, x0), u, 1.0; max_step=1e-5)
    errs = map((0.1, 0.05, 0.025, 0.0125)) do h
        r = integrate_on_track(PV, tr, x0, u, 1.0; max_step=h)
        @test succeeded(r) && length(r.crossings) >= 1
        cart_gap(tr, r.x, zref)[1]
    end
    ratios = errs[1:end-1] ./ errs[2:end]
    @test all(>(10), ratios)                  # RK4 order 4 would give 16
    @test errs[end] <= 1e-7
end
