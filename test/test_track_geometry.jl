# Track geometry: known positions, signed curvature, segment continuity, closure, progress
# wrapping, Frenet <-> Cartesian round trips, and regression against the legacy
# Environment.jl track construction.

pose_close(p, q; atol) = hypot(p[1] - q[1], p[2] - q[2]) <= atol && abs(p[3] - q[3]) <= atol

@testset "straight track" begin
    x0, y0, psi0 = 1.0, 2.0, π / 6
    tr = straight_track(5.0; start_pose=(x0, y0, psi0))
    @test track_length(tr) == 5.0
    @test !is_closed(tr)
    @test curvature(tr, 2.0) == 0
    @test pose_close(centerline_pose(tr, 3.0), (x0 + 3cos(psi0), y0 + 3sin(psi0), psi0); atol=1e-14)
    # ey is measured along the left normal (-sin psi, cos psi).
    X, Y, psi = frenet_to_cartesian(tr, 3.0, 0.2, 0.1)
    @test X ≈ x0 + 3cos(psi0) - 0.2sin(psi0) atol = 1e-14
    @test Y ≈ y0 + 3sin(psi0) + 0.2cos(psi0) atol = 1e-14
    @test psi ≈ psi0 + 0.1
    # Open track: no wrapping, no extrapolation.
    @test split_progress(tr, 5.0) == (0, 5.0)
    @test_throws DomainError curvature(tr, 5.1)
    @test_throws DomainError centerline_pose(tr, -0.1)
end

@testset "circular track" begin
    R = 2.0
    ccw = circular_track(R)
    cw = circular_track(R; direction=-1)
    @test is_closed(ccw) && is_closed(cw)
    @test track_length(ccw) ≈ 2π * R
    @test curvature(ccw, 1.0) == 1 / R
    @test curvature(cw, 1.0) == -1 / R
    @test total_turning(ccw) ≈ 2π
    @test total_turning(cw) ≈ -2π

    # Counter-clockwise circle centred at (0, R).
    @test pose_close(centerline_pose(ccw, π * R / 2), (R, R, π / 2); atol=1e-12)
    @test pose_close(centerline_pose(ccw, π * R), (0.0, 2R, π); atol=1e-12)
    # Positive ey points to the left, i.e. towards the centre on a left turn.
    @test pose_close(frenet_to_cartesian(ccw, π * R / 2, 0.3), (R - 0.3, R, π / 2); atol=1e-12)
    # Clockwise circle centred at (0, -R); positive ey now points away from the centre.
    @test pose_close(centerline_pose(cw, π * R / 2), (R, -R, -π / 2); atol=1e-12)
    @test pose_close(frenet_to_cartesian(cw, π * R / 2, 0.3), (R + 0.3, -R, -π / 2); atol=1e-12)

    # The Frenet frame is singular at the centre of curvature.
    @test_throws DomainError frenet_to_cartesian(ccw, 1.0, R)
    @test_throws DomainError frenet_to_cartesian(cw, 1.0, -R - 0.1)
end

@testset "signed curvature and differential geometry" begin
    # Quarter track (legacy): radius-1 left semicircle, radius-1 right semicircle, radius-2 left quarters.
    tr = quarter_track()
    expected_kappa = [0, 1, -1, 0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0]
    mids = [(tr.s_start[i] + tr.s_start[i+1]) / 2 for i in 1:num_segments(tr)]
    @test curvature.(Ref(tr), mids) == expected_kappa

    # Inside every segment: d(psi)/ds = kappa and d(x, y)/ds = (cos psi, sin psi).
    h = 1e-5
    for tr in (quarter_track(), legacy_track(:rectangular), legacy_track(:lab), circular_track(3.0; direction=-1))
        for i in 1:num_segments(tr), frac in (0.25, 0.5, 0.75)
            s = tr.s_start[i] + frac * tr.segments[i].length
            p_plus, p_minus = centerline_pose(tr, s + h), centerline_pose(tr, s - h)
            _, _, psi = centerline_pose(tr, s)
            @test (p_plus[3] - p_minus[3]) / 2h ≈ tr.segments[i].curvature atol = 1e-8
            @test (p_plus[1] - p_minus[1]) / 2h ≈ cos(psi) atol = 1e-8
            @test (p_plus[2] - p_minus[2]) / 2h ≈ sin(psi) atol = 1e-8
        end
    end
end

@testset "segment transitions are continuous" begin
    h = 1e-9
    for tr in (quarter_track(), legacy_track(:rectangular), legacy_track(:lab), legacy_track(:circular))
        for i in 2:num_segments(tr)
            s = tr.s_start[i]
            @test segment_index(tr, s) == i            # half-open segments [s_i, s_{i+1})
            @test segment_index(tr, s - h) == i - 1
            @test pose_close(centerline_pose(tr, s - h), centerline_pose(tr, s + h); atol=1e-8)
            # Offset curves are continuous too (ey = ±width/2).
            for ey in (-tr.width / 2, tr.width / 2)
                @test pose_close(frenet_to_cartesian(tr, s - h, ey), frenet_to_cartesian(tr, s + h, ey); atol=1e-8)
            end
        end
    end
end

@testset "closure and progress wrapping" begin
    for name in (:quarter, :circular, :rectangular)
        tr = legacy_track(name)
        L = track_length(tr)
        @test is_closed(tr)
        @test total_turning(tr) ≈ 2π
        x0, y0, p0 = centerline_pose(tr, 0.0)
        x1, y1, p1 = centerline_pose(tr, L - 1e-9)
        @test hypot(x1 - x0, y1 - y0) <= 1e-8
        @test p1 ≈ p0 + 2π atol = 1e-8

        for s in (0.1, 0.37L, 0.9L), k in (-2, 1, 3)
            @test curvature(tr, s + k * L) == curvature(tr, s)
            P0, Pk = frenet_to_cartesian(tr, s, 0.1, 0.2), frenet_to_cartesian(tr, s + k * L, 0.1, 0.2)
            @test hypot(P0[1] - Pk[1], P0[2] - Pk[2]) <= 1e-9
            # Heading stays continuous in unwrapped progress: one extra turn per lap.
            @test Pk[3] ≈ P0[3] + k * total_turning(tr) atol = 1e-9
            @test split_progress(tr, s + k * L)[1] == k
            @test split_progress(tr, s + k * L)[2] ≈ s atol = 1e-9
        end
        @test split_progress(tr, 2L) == (2, 0.0)
        @test split_progress(tr, -0.5)[1] == -1
        @test split_progress(tr, -0.5)[2] ≈ L - 0.5
        @test wrap_progress(tr, -1e-18) < L
    end

    # The legacy lab track does not close (≈ 5.5 cm gap), so it is treated as an open track.
    lab = legacy_track(:lab)
    @test !is_closed(lab)
    @test_throws ArgumentError legacy_track(:lab; closed=true)
    @test_throws DomainError curvature(lab, track_length(lab) + 1.0)
    @test_throws ArgumentError Track([TrackSegment(1.0, 0.0)]; closed=true)
end

@testset "Frenet -> Cartesian -> Frenet round trips" begin
    for tr in (straight_track(8.0; width=1.0, start_pose=(-1.0, 3.0, -0.4)), circular_track(2.0; width=0.7),
        circular_track(1.0; direction=-1, width=0.5), quarter_track(), legacy_track(:rectangular, width=0.5))
        L = track_length(tr)
        for frac in range(0.0, 0.999; length=37), ey in (-0.2, 0.0, 0.15), epsi in (-0.6, 0.0, 0.5)
            s = frac * L
            X, Y, psi = frenet_to_cartesian(tr, s, ey, epsi)
            s2, ey2, epsi2 = cartesian_to_frenet(tr, X, Y, psi)
            @test (is_closed(tr) ? abs(rem(s2 - s, L, RoundNearest)) : abs(s2 - s)) <= 1e-9
            @test ey2 ≈ ey atol = 1e-9
            @test epsi2 ≈ epsi atol = 1e-9
        end
    end

    # With a hint, progress on a closed track is unwrapped to the lap nearest the hint.
    tr = circular_track(2.0; width=0.7)
    L = track_length(tr)
    X, Y, psi = frenet_to_cartesian(tr, 3L + 1.0, 0.1, 0.0)
    s, ey, _ = cartesian_to_frenet(tr, X, Y, psi; s_hint=3L + 0.9)
    @test s ≈ 3L + 1.0 atol = 1e-9
    @test ey ≈ 0.1 atol = 1e-12
    s, _, _ = cartesian_to_frenet(tr, X, Y, psi)
    @test s ≈ 1.0 atol = 1e-9
end

@testset "Cartesian -> Frenet rejects missing and ambiguous projections" begin
    # Two parallel straights joined by a U-turn: the mid-line point projects onto both.
    hairpin = Track([TrackSegment(2.0, 0.0), TrackSegment(π * 0.5, 2.0), TrackSegment(2.0, 0.0)])
    @test_throws ArgumentError cartesian_to_frenet(hairpin, 1.0, 0.5; max_lateral=1.0)
    s, ey, _ = cartesian_to_frenet(hairpin, 1.0, 0.5; max_lateral=1.0, s_hint=1.0)
    @test s ≈ 1.0 && ey ≈ 0.5
    s, ey, _ = cartesian_to_frenet(hairpin, 1.0, 0.5; max_lateral=1.0, s_hint=4.0)
    @test s ≈ 2.0 + π * 0.5 + 1.0 && ey ≈ 0.5      # heading -X: the point is again on the left
    # Outside the corridor or beyond the ends of an open track: explicit error, no placeholder.
    @test_throws ArgumentError cartesian_to_frenet(straight_track(2.0; width=0.5), 1.0, 0.5)
    @test_throws ArgumentError cartesian_to_frenet(straight_track(2.0), 3.0, 0.0)
    @test_throws ArgumentError cartesian_to_frenet(circular_track(1.0), 0.0, 1.0; max_lateral=Inf)  # centre
end

@testset "invalid track construction" begin
    @test_throws ArgumentError TrackSegment(0.0, 0.0)
    @test_throws ArgumentError TrackSegment(-1.0, 0.0)
    @test_throws ArgumentError TrackSegment(1.0, NaN)
    @test_throws ArgumentError TrackSegment(7.0, 1.0)          # more than one revolution
    @test_throws ArgumentError Track(TrackSegment{Float64}[])
    @test_throws ArgumentError circular_track(0.0)
    @test_throws ArgumentError straight_track(1.0; width=-0.1)
end

# The legacy Environment.jl construction is mathematically correct for the forward map
# (s, ey, epsi) -> (x, y, psi); the new implementation must reproduce it exactly.
@testset "regression against legacy Environment.jl" begin
    legacy = Module(:LegacyEnvironment)
    Base.include(legacy, joinpath(@__DIR__, "..", "Environment.jl"))
    for (name, opts) in ((:quarter, legacy.Quarter_trackoption()), (:rectangular, legacy.Rectangular_trackoption()),
        (:lab, legacy.Lab_trackoption()), (:circular, legacy.Circular_trackoption()))

        key_pts = legacy.build_track_keypoint(opts)
        tr = legacy_track(name)
        @test track_length(tr) ≈ key_pts[end][4] rtol = 1e-14
        for i in 1:num_segments(tr)
            @test tr.s_start[i+1] ≈ key_pts[i+1][4]
            @test tr.segments[i].curvature ≈ key_pts[i+1][6]
            @test hypot(tr.poses[i+1][1] - key_pts[i+1][1], tr.poses[i+1][2] - key_pts[i+1][2]) <= 1e-12
            @test abs(rem2pi(tr.poses[i+1][3] - key_pts[i+1][3], RoundNearest)) <= 1e-12
        end
        L = track_length(tr)
        for s in range(0.0, L; length=301)[1:end-1], ey in (-0.25, 0.0, 0.25)
            any(b -> abs(s - b) < 1e-9, tr.s_start) && continue    # skip exact boundaries
            X, Y, psi = frenet_to_cartesian(tr, s, ey, 0.1)
            Xl, Yl, psil = legacy.local_to_global(key_pts, (s, ey, 0.1))
            @test hypot(X - Xl, Y - Yl) <= 1e-12
            @test abs(rem2pi(psi - psil, RoundNearest)) <= 1e-12
            @test curvature(tr, s) == legacy.get_curvature(key_pts, s)
        end
    end
end
