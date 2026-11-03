# Experimental exact segment-pair curvature (Milestone 2B.3B).

const SPV = KinematicBicycle(reference_test_vehicle())
const SPQ = quarter_track()
const SPLOOP = Track([TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0),
        TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0)]; closed=true)
const SPTRANS = Track([TrackSegment(3.0, 0.0), TrackSegment(1.5π, 0.5), TrackSegment(6.0, 0.0)]; width=0.7)
const SPCFG = NMPCConfig(curvature_model=:segment_pair_exact)

"The segment-pair right-hand side for actual end points (a, b) under the assignment of stage k."
sp_rhs(sp, s, k, a, b) = sp.dpsi[k] + sp.kj[k] * (b - s[k+1]) - sp.ki[k] * (a - s[k])

@testset "heading increment: local curvature integral" begin
    # Independent check against differences of centerline_heading, including laps, negative
    # curvature, opposite-sign transitions, several transitions and tiny intervals.
    for (tr, pairs) in ((SPQ, [(0.3, 1.9), (1.5, 2.5), (4.9, 5.3), (1.0, 9.0), (2.0, 2.0 + π), (5.0, 5.0 + 1e-6)]),
        (legacy_track(:lab), [(5.7, 7.2), (0.1, 13.9), (6.224, 6.590)]),
        (SPLOOP, [(track_length(SPLOOP) - 0.4, track_length(SPLOOP) + 0.3), (0.2, 3 * track_length(SPLOOP) - 0.1), (25.0, 26.0)]))
        for (a, b) in pairs
            ref = centerline_heading(tr, b) - centerline_heading(tr, a)
            @test isapprox(heading_increment(tr, a, b), ref; atol=1e-12)
            @test heading_increment(tr, b, a) ≈ -heading_increment(tr, a, b) atol = 1e-15
        end
    end
    @test heading_increment(SPQ, 3.0, 3.0) == 0.0
    @test heading_increment(SPLOOP, 0.0, track_length(SPLOOP)) ≈ 2π atol = 1e-12     # one lap
    @test_throws DomainError heading_increment(legacy_track(:lab), 13.0, 15.0)
end

@testset "segment-pair identity: exact inside the assigned segments, wrong outside" begin
    for (tr, s) in ((SPQ, [1.6, 2.4, 2.9, 5.0, 5.4]),            # 0 → +1, same segment, +1 → −1
        (SPQ, [1.5, 5.6]),                                         # two transitions inside one stage
        (SPLOOP, [track_length(SPLOOP) - 0.3, track_length(SPLOOP) + 0.4]))   # lap boundary
        sp = segment_pair_assignment(tr, s)
        for k in 1:length(s)-1
            for (da, db) in ((0.0, 0.0), (0.05, -0.03), (-0.04, 0.06))
                a, b = s[k] + da, s[k+1] + db
                inside = sp.lo[k] <= a <= sp.hi[k] && sp.lo[k+1] <= b <= sp.hi[k+1]
                inside || continue
                exact = heading_increment(tr, a, b)
                model = sp.w[k] == 1 ? sp_rhs(sp, s, k, a, b) : sp.kc[k] * (b - a)
                @test isapprox(model, exact; atol=1e-12)
            end
        end
    end
    # Moving an end point into the next segment breaks the identity, and this is detectable.
    s = [1.6, 1.9]                                                   # same segment (straight), nominal
    sp = segment_pair_assignment(SPQ, s)
    @test sp.w == [0.0] && sp.kc == [0.0]
    b = 2.2                                                          # optimised end in the κ = 1 arc
    @test !(sp.lo[2] <= b <= sp.hi[2])
    @test abs(sp.kc[1] * (b - 1.6) - heading_increment(SPQ, 1.6, b)) > 0.1
end

@testset "same-segment, short, stationary and boundary-knot stages" begin
    sp = segment_pair_assignment(SPQ, [2.5, 2.8, 3.1])
    @test sp.w == [0.0, 0.0] && sp.kc == [1.0, 1.0] && sp.kbar == [1.0, 1.0]
    # Stationary exactly on a boundary: entering segment (κ = 1), no free curvature.
    sp = segment_pair_assignment(SPQ, fill(2.0, 4))
    @test sp.w == zeros(3) && sp.kc == ones(3) && sp.short_intervals == 3 && sp.boundary_knots == 4
    # Short crossing interval: constant form with the entering curvature, flagged.
    sp = segment_pair_assignment(SPQ, [1.9997, 2.0004])
    @test sp.w == [0.0] && sp.kc == [0.0] && sp.short_intervals == 1
    # Knot exactly on a boundary belongs to the entering segment.
    sp = segment_pair_assignment(SPQ, [1.7, 2.0, 2.3])
    @test sp.lo[2] == 2.0 && sp.kj[1] == 1.0 && sp.w[1] == 1.0 && sp.w[2] == 0.0 && sp.boundary_knots == 1
    @test sp_rhs(sp, [1.7, 2.0, 2.3], 1, 1.7, 2.0) ≈ heading_increment(SPQ, 1.7, 2.0) atol = 1e-12
end

@testset "NLP structure, bounds, parameters and differentiation" begin
    c = NMPCController(SPV, SPQ, SPCFG)
    N = horizon(c)
    @test curvature_model(c) === :segment_pair_exact
    @test length(c.sp_w) == N && all(is_parameter, c.sp_w) && all(is_parameter, c.sp_dpsi)
    @test all(k -> lower_bound(k) == -1.0 && upper_bound(k) == 1.0, c.kappa_eff)
    s = collect(1.1 .+ 0.25 .* (0:N))
    sched = (; method=:test, iterations=1, converged=true, change=0.0, kappas=stage_curvatures(SPQ, s), s_nominal=s,
        Xg=zeros(4, N + 1), Ug=zeros(2, N))
    install_schedule!(c, sched)
    sp = segment_pair_assignment(SPQ, s)
    @test parameter_value.(c.sp_w) == sp.w && parameter_value.(c.sp_dpsi) == sp.dpsi && parameter_value.(c.sp_kc) == sp.kc
    @test parameter_value.(c.sp_ki) == sp.ki && parameter_value.(c.sp_kj) == sp.kj && parameter_value.(c.lin_sbar) == s
    @test count(==(1.0), sp.w) == 2
    @test !any(has_lower_bound, c.X[1, 2:end])                       # no membership bounds by default
    # The persistent equality evaluates the documented model for both forms.
    cons = [con for con in all_constraints(c.model; include_variable_in_set_constraints=false)
            if occursin("kappa_eff", string(constraint_object(con).func)) && occursin("sp_w", string(constraint_object(con).func))]
    @test length(cons) == N
    sstar = s .+ 0.003                                              # uniform shift: stays inside the assigned segments
    @test all(sp.lo[k] <= sstar[k] <= sp.hi[k] for k in 1:N+1)
    keff = [sp.w[k] == 1 ? sp_rhs(sp, s, k, sstar[k], sstar[k+1]) / (sstar[k+1] - sstar[k]) : sp.kc[k] for k in 1:N]
    @test maximum(abs.(keff .- [heading_increment(SPQ, sstar[k], sstar[k+1]) / (sstar[k+1] - sstar[k]) for k in 1:N])) <= 1e-12
    vals = Dict{VariableRef,Float64}(c.X[1, k] => sstar[k] for k in 2:N+1)
    for k in 1:N
        vals[c.kappa_eff[k]] = keff[k]
    end
    set_parameter_value(c.X[1, 1], sstar[1])
    valfn = v -> is_parameter(v) ? parameter_value(v) : get(vals, v, 0.0)
    res(con) = value(valfn, constraint_object(con).func) - MOI.constant(constraint_object(con).set)
    @test maximum(abs ∘ res, cons) <= 1e-12
    # Zero progress on a w = 0 stage: kappa_eff is pinned (no free curvature).
    vals[c.kappa_eff[1]] = sp.kc[1] + 0.3
    @test maximum(abs ∘ res, cons) > 0.1
    # Membership bounds (approach B) are set from the assignment.
    cb = NMPCController(SPV, SPQ, NMPCConfig(curvature_model=:segment_pair_exact, segment_membership=true))
    install_schedule!(cb, sched)
    @test all(k -> lower_bound(cb.X[1, k]) == sp.lo[k] && upper_bound(cb.X[1, k]) == sp.hi[k], 2:N+1)
end

@testset "one solve: exact consistency, reference check and stationary start" begin
    x = [3.0 - 0.5 * 2.0 * 0.1, 0.0, 0.0, 2.0]
    c = NMPCController(SPV, SPTRANS, SPCFG)
    r = solve!(c, x)
    @test r.accepted && r.status === :success && r.nlp_solves == 1
    @test r.schedule.curvature_model === :segment_pair_exact
    # Exact identity held; the remaining residual is at the solver's tolerance (tol = 1e-8).
    @test r.schedule.configuration_changes == 0 && r.schedule.stage_residual <= 1e-7
    @test r.max_dynamics_residual <= 1e-6 && r.reference.passed
    @test r.reference.max_position_error < 0.02
    @test isfinite(r.reference.min_corridor_distance) && r.reference.min_boundary_distance > 0
    @test r.schedule.min_progress_increment > 0
    # Bilinear equality holds at the solution (differentiated correctly by the solver).
    sp = segment_pair_assignment(SPTRANS, r.schedule.s_nominal)
    keff = value.(c.kappa_eff)
    s = r.X[1, :]
    for k in 1:20
        lhs = keff[k] * (s[k+1] - s[k])
        rhs = sp.w[k] == 1 ? sp_rhs(sp, r.schedule.s_nominal, k, s[k], s[k+1]) : sp.kc[k] * (s[k+1] - s[k])
        @test abs(lhs - rhs) <= 1e-7
    end
    # Starting at rest exactly on a boundary: no exception, the vehicle can accelerate.
    rs = solve!(NMPCController(SPV, SPQ, NMPCConfig(curvature_model=:segment_pair_exact, schedule_corrections=8,
        correction_time_budget=100.0)), [2.0, 0.0, 0.0, 0.0])
    @test rs.status in (:success, :schedule_inconsistent, :reference_violation, :solver_failure, :scheduling_failure)
    rs.accepted && @test(rs.X[4, end] > 0.1)
end

@testset "invalid segment assignments and membership changes are detected" begin
    c = NMPCController(SPV, SPQ, NMPCConfig(curvature_model=:segment_pair_exact, segment_change_policy=:reject))
    N = horizon(c)
    s = collect(1.1 .+ 0.25 .* (0:N))
    sp = segment_pair_assignment(SPQ, s)
    lin = (; kbar=sp.kbar, A=zeros(N), B=zeros(N), boundary_knots=0, short_intervals=0, tr=fill(Inf, N + 1), sp)
    sched = (; method=:test, iterations=1, converged=true, change=0.0, kappas=sp.kbar, s_nominal=s, lin)
    # Knot 4 (nominal 1.85, straight) optimised to 2.05 (κ = 1 arc): outside its segment.
    sstar = copy(s)
    sstar[4] = 2.05
    keff = [heading_increment(SPQ, sstar[k], sstar[k+1]) / (sstar[k+1] - sstar[k]) for k in 1:N]
    rep = schedule_consistency(c, sched, (; X=vcat(reshape(sstar, 1, :), zeros(3, N + 1)), kappas_used=keff), true)
    @test rep.configuration_changes == 2 && !rep.consistent && occursin("assigned segments", rep.message)
    # With exact κ_eff the residual is zero, so the residual policy alone would accept it.
    c2 = NMPCController(SPV, SPQ, SPCFG)
    rep2 = schedule_consistency(c2, sched, (; X=vcat(reshape(sstar, 1, :), zeros(3, N + 1)), kappas_used=keff), true)
    @test rep2.configuration_changes == 2 && rep2.consistent && rep2.stage_residual <= 1e-12
    # A deliberately wrong assignment (wrong curvature) produces a residual that is caught.
    wrong = copy(keff)
    wrong[3] += 0.2
    rep3 = schedule_consistency(c2, sched, (; X=vcat(reshape(s, 1, :), zeros(3, N + 1)), kappas_used=wrong), true)
    @test !rep3.consistent && rep3.stage_residual > 0.04
end

@testset "single-segment tracks and default unchanged" begin
    for (tr, x0) in ((circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5]), (straight_track(100.0; width=0.7), [0.0, 0.2, 0.1, 0.5]))
        a = simulate_closed_loop(NMPCController(SPV, tr, NMPCConfig()), SPV, x0, 15)
        cb = NMPCController(SPV, tr, NMPCConfig(curvature_model=:segment_pair_exact, segment_membership=true))
        b = simulate_closed_loop(cb, SPV, x0, 15)
        @test curvature_model(cb) === :constant && a.states == b.states && a.inputs == b.inputs
    end
    @test NMPCConfig().curvature_model === :stagewise_average
    @test_throws ArgumentError NMPCController(SPV, SPQ, NMPCConfig(curvature_model=:segment_pair_exact, segment_change_policy=:ignore))
end
