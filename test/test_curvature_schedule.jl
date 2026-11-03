# Average curvature, curvature schedules and the stagewise prediction methods.

const SV = KinematicBicycle(reference_test_vehicle())

"Independent reference: (1/Δ)∫κ ds from segment overlaps (no headings involved)."
function overlap_average(track, s0, s1)
    a, b = minmax(s0, s1)
    L, total = track_length(track), 0.0
    laps = track.closed ? (floor(Int, a / L):floor(Int, b / L)) : (0:0)
    for lap in laps, (j, seg) in enumerate(track.segments)
        lo, hi = lap * L + track.s_start[j], lap * L + track.s_start[j+1]
        total += seg.curvature * max(0.0, min(b, hi) - max(a, lo))
    end
    return total / (b - a)
end

const LOOP2 = Track([TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0),
        TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0)]; closed=true)

@testset "segment boundaries" begin
    tr = quarter_track()
    @test segment_boundaries(tr, 1.0, 3.0) == [2.0]
    @test segment_boundaries(tr, 3.0, 1.0) == [2.0]                 # order-independent
    @test segment_boundaries(tr, 2.0, 3.0) == Float64[]             # half-open (s0, s1]
    @test segment_boundaries(tr, 1.0, 2.0) == [2.0]
    L = track_length(tr)
    b = segment_boundaries(tr, L - 1.0, L + 2.5)
    @test b ≈ [L, L + 2.0]                                          # lap boundary and next one
    @test isempty(segment_boundaries(circular_track(1.0), 0.0, 20.0))
    lab = legacy_track(:lab)
    @test segment_boundaries(lab, 0.0, track_length(lab)) ≈ lab.s_start[2:end-1]
    @test_throws DomainError segment_boundaries(lab, 0.0, track_length(lab) + 1)
end

@testset "average curvature" begin
    tr = quarter_track()
    @test average_curvature(tr, 2.5, 3.5) ≈ 1.0                     # inside one segment
    @test average_curvature(tr, 0.2, 1.7) == 0.0
    @test average_curvature(tr, 1.5, 2.5) ≈ 0.5                     # one boundary, half/half
    @test average_curvature(tr, 2.5, 1.5) ≈ 0.5                     # symmetric
    # Arbitrary intervals across several boundaries, laps and negative curvature, vs overlaps.
    L = track_length(tr)
    for (s0, s1) in ((0.3, 7.9), (4.0, 6.0), (L - 3.0, L + 2.6), (1.0, 2L + 1.0), (-1.2, 0.8), (2.0 + π - 0.4, 2.0 + π + 0.7))
        @test average_curvature(tr, s0, s1) ≈ overlap_average(tr, s0, s1) atol = 1e-12
    end
    @test average_curvature(tr, 0.7, 0.7 + 3L) ≈ 2π / L             # whole laps: total turning / length
    lab = legacy_track(:lab)
    for (s0, s1) in ((0.1, 13.9), (5.0, 6.3), (6.0, 6.5))
        @test average_curvature(lab, s0, s1) ≈ overlap_average(lab, s0, s1) atol = 1e-12
    end
    @test average_curvature(LOOP2, track_length(LOOP2) - 0.5, track_length(LOOP2) + 0.5) ≈ 0.5   # straight -> arc at the lap boundary

    # Small intervals: one-sided limits; zero length at a jump is ambiguous.
    @test average_curvature(tr, 2.0 - 1e-8, 2.0 - 0.5e-8) == 0.0
    @test average_curvature(tr, 2.0, 2.0 + 1e-8) == 1.0              # right limit at the boundary
    @test average_curvature(tr, 2.0, 2.0 - 1e-8) == 0.0              # left limit at the boundary
    @test average_curvature(tr, 3.0, 3.0) == 1.0                     # interior point
    @test_throws ArgumentError average_curvature(tr, 2.0, 2.0)
    @test average_curvature(tr, 1.0, 1.0 + 2e-6) == 0.0              # just above ds_min: quotient
    @test curvature_left(tr, 2.0) == 0.0 && curvature_left(tr, 2.5) == 1.0
    @test curvature_left(tr, 0.0) == 0.0                             # closed: last segment (straight)
    @test curvature_left(LOOP2, 0.0) == 0.0 && curvature(LOOP2, 0.0) == 1.0
    @test_throws DomainError curvature_left(straight_track(1.0), 0.0)
    @test_throws DomainError average_curvature(lab, 13.0, 15.0)      # beyond an open track
end

@testset "curvature schedule" begin
    tr = quarter_track()
    s = [1.5, 1.7, 1.9, 2.1, 2.3]
    @test curvature_schedule(tr, s; method=:start) == [0.0, 0.0, 0.0, 1.0]
    @test curvature_schedule(tr, s) ≈ [0.0, 0.0, 0.5, 1.0]
    @test length(curvature_schedule(tr, collect(0.0:0.1:2.0))) == 20
    @test_throws ArgumentError curvature_schedule(tr, [1.0])
    @test_throws ArgumentError curvature_schedule(tr, s; method=:smooth)
end

@testset "prediction methods" begin
    tr = quarter_track()
    dt = 0.1
    U = repeat([0.1, 0.2], 1, 5)
    # Entirely inside one segment: A, B and D coincide with each other and with the
    # event-aware integrator (no boundary to resolve).
    x0 = [2.3, 0.05, 0.0, 1.0]
    A = predict_start_curvature(SV, tr, x0, U, dt)
    sim = simulate_on_track(SV, tr, x0, [U[:, k] for k in 1:5], dt; max_step=dt)
    snom = [x[1] for x in A.states]
    B = predict_average_curvature(SV, tr, x0, U, dt, snom)
    D = predict_split_curvature(SV, tr, x0, U, dt, snom)
    @test succeeded(A) && succeeded(B) && succeeded(D)
    @test maximum(maximum(abs, A.states[k] - sim.states[k]) for k in 1:6) <= 1e-12
    @test maximum(maximum(abs, B.states[k] - A.states[k]) for k in 1:6) <= 1e-12
    @test D.states == A.states

    # Across a boundary. D with the exact crossing times takes the same RK4 sub-steps as the
    # event-aware integrator at max_step = dt, so the two agree to root-finding precision.
    # Against a fine reference the remaining difference is ordinary RK4 truncation error.
    # A (start-of-step curvature) is far off.
    x0 = [1.85, 0.0, 0.0, 2.0]
    U = repeat([atan(SV.params.wheelbase * 1.0), 0.0], 1, 3)
    useq = [U[:, k] for k in 1:3]
    same = simulate_on_track(SV, tr, x0, useq, dt; max_step=dt)
    fine = simulate_on_track(SV, tr, x0, useq, dt; max_step=dt / 50)
    times = [Float64[c.t - (k - 1) * dt for c in same.crossings if (k - 1) * dt <= c.t < k * dt] for k in 1:3]
    @test sum(length, times) == 1
    Dx = predict_split_curvature(SV, tr, x0, U, dt, [x[1] for x in same.states]; crossing_times=times)
    @test maximum(prediction_errors(tr, Dx.states, same.states).position) <= 1e-12
    @test maximum(prediction_errors(tr, Dx.states, fine.states).position) <= 1e-5
    errA = prediction_errors(tr, predict_start_curvature(SV, tr, x0, U, dt).states, fine.states)
    @test maximum(errA.heading) > 1e-2
    @test errA.position[1] == 0 && errA.s[1] == 0                   # knot 1 is the shared initial state
end
