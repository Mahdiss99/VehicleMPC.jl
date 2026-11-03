# Between-knot corridor constraints for the Cartesian-endpoint NMPC (Milestone 2B.5).

const ISV = KinematicBicycle(reference_test_vehicle())
const ISL = ISV.params.wheelbase
const ISQ = quarter_track()
const ISCART = (prediction_model=:cartesian_endpoint_exact, schedule_corrections=8, correction_time_budget=100.0)
const ISE = ISQ.width / 2 - 0.05

@testset "exact intermediate propagation" begin
    for (pose, v, u) in (((0.3, -0.2, 0.4), 1.7, (0.2, 1.0)), ((0.0, 0.0, 0.0), 2.0, (0.0, 0.0)), ((1.0, 2.0, 3.0), 0.5, (-0.4, -5.0)))
        @test all(interior_point(ISL, pose, v, u..., 0.0) .== pose[1:2])
        zend = exact_cartesian_step(ISV, [pose..., v], [u...], 0.1)
        @test maximum(abs.(interior_point(ISL, pose, v, u..., 0.1) .- zend[1:2])) <= 1e-15
        for a in (0.25, 0.5, 0.75)
            zr = integrate_cartesian(ISV, [pose..., v], [u...], a * 0.1; max_step=1e-5)
            @test maximum(abs.(interior_point(ISL, pose, v, u..., a * 0.1) .- zr[1:2])) <= 1e-12
        end
    end
    # Stationary: the interior point is the knot.
    @test interior_point(ISL, (1.0, 2.0, 0.3), 0.0, 0.3, 0.0, 0.05) == (1.0, 2.0)
    @test interior_alphas(0) == Float64[] && interior_alphas(1) == [0.5] && interior_alphas(3) == [0.25, 0.5, 0.75]
    @test_throws ArgumentError interior_alphas(2)
end

@testset "straight and annular corridor inequalities" begin
    for (tr, ids) in ((ISQ, 1:num_segments(ISQ)), (legacy_track(:lab), 1:num_segments(legacy_track(:lab))))
        E = tr.width / 2 - 0.05
        for id in ids
            f = segment_frame(tr, id)
            par = corridor_parameters(f, E)
            @test par[1] == (f.kappa == 0 ? 0.0 : 1.0) && all(isfinite, par)
            if f.kappa != 0                                        # the centre is at distance R from the centreline
                for s in range(f.s0, f.s1; length=7)
                    x, y = centerline_position(tr, s)
                    @test hypot(x - par[2], y - par[3]) ≈ 1 / abs(f.kappa) atol = 1e-12
                end
            end
            for s in range(f.s0 + 1e-3, f.s1 - 1e-3; length=5), ey in (-E - 0.01, -E + 0.01, 0.0, E - 0.01, E + 0.01)
                X, Y, _ = frenet_to_cartesian(tr, s, ey)
                lo, hi = corridor_margins(par, X, Y, E)
                @test (min(lo, hi) >= 0) == (abs(ey) <= E)                     # inside ⇔ both margins ≥ 0
                if f.kappa == 0
                    @test lo ≈ ey + E atol = 1e-12
                    @test hi ≈ E - ey atol = 1e-12
                else                                                         # ≈ metric; exact up to (E ∓ ey)²/(2R)
                    @test abs(min(lo, hi) - (E - abs(ey))) <= (E + abs(ey))^2 * abs(f.kappa) / 2 + 1e-12
                end
            end
        end
    end
    # Both curvature signs exist on the quarter track (R = 1 m left, then right).
    @test ISQ.segments[2].curvature > 0 && ISQ.segments[3].curvature < 0
end

@testset "projections: missing, ambiguous, outside the assigned arc" begin
    pr = sample_projection(ISQ, 100.0, 100.0, 1.0; max_lateral=0.7)
    @test pr.status === :missing && isnan(pr.s)
    # Hairpin: two parallel straights 0.5 m apart; the mid-point projects onto both.
    hp = Track([TrackSegment(3.0, 0.0), TrackSegment(π * 0.25, 4.0), TrackSegment(3.0, 0.0)]; width=0.4)
    @test sample_projection(hp, 1.5, 0.25, 3.0; max_lateral=0.3, window=10.0).status === :ambiguous
    @test sample_projection(hp, 1.5, 0.1, 1.5; max_lateral=0.3, window=1.0).status === :ok
    # A point inside the annulus of the R = 1 m arc (segment 2) but beyond the arc's end
    # satisfies the curved inequalities, yet its projection is in segment 3.
    f2 = segment_frame(ISQ, 2)
    par = corridor_parameters(f2, ISE)
    O = (par[2], par[3])
    ang = atan(f2.y0 - O[2], f2.x0 - O[1]) - 0.3                          # 0.3 rad *before* the arc's start, same circle
    Xa, Ya = O[1] + cos(ang), O[2] + sin(ang)
    @test min(corridor_margins(par, Xa, Ya, ISE)...) > 0                    # inside the full-circle annulus
    pa = sample_projection(ISQ, Xa, Ya, f2.s0; max_lateral=0.69)
    @test pa.status !== :ok || !VehicleMPC.frame_valid(ISQ, f2, pa.s, 1e-6)   # but not on the assigned arc
end

@testset "interior sample assignment: transitions, opposite signs, laps" begin
    c = NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10, interior_samples=3))
    @test c.alphas == [0.25, 0.5, 0.75] && size(c.iframe) == (8, 30) && length(c.interior) == 60
    N = 10
    for s0 in (1.2, 4.6, track_length(ISQ) - 0.9)                       # straight→arc, κ +1 → −1, lap boundary
        Xn = vcat(reshape(collect(s0 .+ 0.25 .* (0:N)), 1, :), zeros(2, N + 1), fill(2.5, 1, N + 1))
        Un = zeros(2, N)
        Un[1, :] .= [atan(curvature(ISQ, Xn[1, k] + 0.1) * ISL) for k in 1:N]
        asg = cartesian_assignment(ISQ, Xn[1, :])
        samp = interior_assignment(c, Xn, Un, asg.frames)
        @test length(samp.ids) == 30 && samp.fallbacks == 0
        @test length(unique(samp.ids)) >= 2                             # assignments change along the horizon
        @test all(samp.ids[i] <= samp.ids[i+1] for i in 1:29)           # monotone in progress
        for (col, f) in enumerate(samp.frames)
            @test f.s0 - 0.05 <= samp.hints[col] <= f.s1 + 0.05
        end
        s0 > track_length(ISQ) - 1 && @test(maximum(samp.ids) > num_segments(ISQ))   # next lap is lap-aware
    end
end

@testset "NLP: Parameters, constraint values, derivatives, preserved constraints" begin
    c0 = NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10))
    c = NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10, interior_samples=1))
    @test isempty(c0.alphas) && isempty(c0.iframe) && isempty(c0.interior)
    ncon(m) = num_constraints(m; count_variable_in_set_constraints=false)
    @test ncon(c.model) == ncon(c0.model) + 2 * 10
    m0 = c.model
    r1 = solve!(c, [1.0, 0.0, 0.0, 2.0])
    @test r1.accepted && r1.corridor.interior_samples == 10 && r1.corridor.interior_failures == 0
    p1 = parameter_value.(c.iframe)
    sched = (; asg=(; frames=cartesian_assignment(ISQ, r1.schedule.s_nominal).frames))
    r2 = solve!(c, [1.25, 0.01, 0.0, 2.2]; previous_input=r1.u)
    @test c.model === m0 && r2.accepted && parameter_value.(c.iframe) != p1
    # The constraint functions equal the Float64 corridor margins at the solution.
    vals = Dict{VariableRef,Float64}()
    for k in 2:11, i in 1:4
        vals[c.X[i, k]] = r2.X[i, k]
    end
    for k in 1:10, i in 1:2
        vals[c.U[i, k]] = r2.U[i, k]
    end
    valfn = v -> is_parameter(v) ? parameter_value(v) : vals[v]
    cv = [value(valfn, constraint_object(con).func) for con in c.interior]
    @test minimum(cv) >= -1e-7                                         # satisfied at the solution
    frames = cartesian_assignment(ISQ, r2.schedule.s_nominal).frames
    for k in 1:10
        par = Tuple(parameter_value.(c.iframe[:, k]))
        pose = frenet_pose_local(frames[k], r2.X[1, k], r2.X[2, k], r2.X[3, k])
        px, py = interior_point(ISL, pose, r2.X[4, k], r2.U[1, k], r2.U[2, k], 0.05)
        @test collect(corridor_margins(par, px, py, c.ey_max)) ≈ cv[2k-1:2k] atol = 1e-9
    end
    # Automatic differentiation of the interior corridor margin vs central differences.
    par = corridor_parameters(segment_frame(ISQ, 2), ISE)
    f = w -> collect(corridor_margins(par, interior_point(ISL, (w[1], w[2], w[3]), w[4], w[5], w[6], 0.05)..., ISE))
    w0 = [frenet_to_cartesian(ISQ, 3.0, 0.1, 0.05)..., 2.0, 0.3, 1.0]
    J = VehicleMPC.ForwardDiff.jacobian(f, w0)
    Jfd = reduce(hcat, [(f(w0 + 1e-6 * e) - f(w0 - 1e-6 * e)) / 2e-6 for e in eachcol(Matrix(1.0I, 6, 6))])
    @test maximum(abs.(J - Jfd)) <= 1e-7
    # Existing state, input and lateral-acceleration constraints still hold.
    p = ISV.params
    @test all(abs.(r2.U[1, :]) .<= p.delta_max + 1e-9) && all(p.a_min - 1e-9 .<= r2.U[2, :] .<= p.a_max + 1e-9)
    @test all(abs.(r2.X[2, 2:end]) .<= c.ey_max + 1e-7) && all(abs.(r2.X[3, 2:end]) .<= c.config.epsi_max + 1e-7)
    @test r2.max_constraint_violation <= 1e-6
end

@testset "between-sample violations, strict rejection, analytic clearance" begin
    # Midpoint constraints hold, yet the dense check finds the corridor left between samples.
    cfg = NMPCConfig(; ISCART..., horizon=10, interior_samples=1, dense_check_samples=20)
    c = NMPCController(ISV, ISQ, cfg)
    x = [0.0, 0.0, 0.0, 0.5]
    r = solve!(c, x)
    @test r.accepted && r.corridor.interior_min_margin >= -1e-7 && r.corridor.interior_failures == 0
    @test r.corridor.dense_samples == 20 && r.corridor.violations > 0 && r.corridor.min_corridor_clearance < -1e-3
    tw = r.corridor.worst_time
    @test minimum(abs(tw - ((k - 1) + 0.5) * 0.1) for k in 1:10) > 1e-3       # not at a constrained sample
    @test r.corridor.min_track_clearance > 0 && r.corridor.interior_prediction_error <= 1e-6
    # Strict corridor: the same candidate is rejected as a corridor violation (not physical).
    rs = solve!(NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10, interior_samples=1, dense_check_samples=20,
        strict_corridor=true)), x)
    @test rs.status === :corridor_violation && !rs.accepted && rs.reference.passed && occursin("dense check", rs.message)
    # Analytic continuous-time clearance: never above any sampled minimum, and close to the dense one.
    an = analytic_corridor_clearance(c, x, r.X, r.U)
    d100 = corridor_check(c, x, r.X, r.U; samples=100)
    @test an.failures == 0 && an.min_corridor <= d100.min_corridor_clearance + 1e-12
    @test an.min_corridor >= d100.min_corridor_clearance - 1e-4
    @test an.min_track ≈ an.min_corridor + 0.05 atol = 1e-12
    # Mode 0 with the dense check enabled (non-strict) is the 2B.4 controller: identical closed loop.
    a = simulate_closed_loop(NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10)), ISV, x, 30)
    b = simulate_closed_loop(NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10, dense_check_samples=20)), ISV, x, 30)
    @test a.states == b.states && all(r -> r.corridor.dense_samples == 20, b.controller)
end

@testset "low speed, correction budget, solver failure, configuration errors" begin
    r = solve!(NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10, interior_samples=3)), [ISQ.s_start[2], 0.0, 0.0, 0.0])
    @test r.accepted && r.corridor.interior_failures == 0 && r.corridor.interior_min_margin >= -1e-7 && r.X[4, end] > 0.5
    rb = solve!(NMPCController(ISV, ISQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact, horizon=20, interior_samples=3,
        schedule_corrections=8, correction_time_budget=1e-6)), [0.0, 0.0, 0.0, 0.5])
    @test !rb.accepted && rb.nlp_solves == 1 && occursin("time budget", rb.message)
    @test rb.status in (:segment_assignment_invalid, :interior_assignment_invalid)
    rf = solve!(NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., horizon=10, interior_samples=3, max_iter=1)), [0.0, 0.0, 0.0, 0.5])
    @test rf.status === :solver_failure && rf.corridor.interior_samples == 0 && rf.nlp_solves == 1
    @test_throws ArgumentError NMPCController(ISV, ISQ, NMPCConfig(interior_samples=1))
    @test_throws ArgumentError NMPCController(ISV, ISQ, NMPCConfig(; ISCART..., interior_samples=2))
    @test NMPCConfig().interior_samples == 0 && NMPCConfig().dense_check_samples == 0
    # A corridor at least as wide as a radius of curvature is rejected (annulus invalid).
    narrow = Track([TrackSegment(1.0, 0.0), TrackSegment(π / 4, 4.0), TrackSegment(1.0, 0.0)]; width=0.6)
    @test_throws ArgumentError NMPCController(ISV, narrow, NMPCConfig(; ISCART..., interior_samples=1))
end
