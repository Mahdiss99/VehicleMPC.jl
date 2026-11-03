# Experimental progress-linearized curvature (Milestone 2B.3A): sensitivities, boundary
# handling, the augmented NLP and its validation.

const PLV = KinematicBicycle(reference_test_vehicle())
const PLQ = quarter_track()
const PLLOOP = Track([TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0),
        TrackSegment(π / 2, 1.0), TrackSegment(2.0, 0.0), TrackSegment(π / 2, 1.0), TrackSegment(1.0, 0.0)]; closed=true)
const PLTRANS = Track([TrackSegment(3.0, 0.0), TrackSegment(1.5π, 0.5), TrackSegment(6.0, 0.0)]; width=0.7)
const LINCFG = NMPCConfig(curvature_model=:progress_linearized)

"Central finite differences of the exact average curvature with respect to both end points."
function fd_sens(track, a, b; h=1e-6)
    K(x, y) = average_curvature(track, x, y)
    return ((K(a + h, b) - K(a - h, b)) / 2h, (K(a, b + h) - K(a, b - h)) / 2h)
end

@testset "sensitivities vs finite differences away from boundaries" begin
    L = track_length(PLLOOP)
    for (track, a, b) in ((PLQ, 1.5, 2.4),            # straight → +1 arc
        (PLQ, 4.9, 5.3),                              # +1 → −1 (negative curvature)
        (PLQ, 1.5, 5.5),                              # two transitions in one stage
        (PLQ, 8.0, 8.6),                              # −1 → +0.5
        (legacy_track(:lab), 5.7, 7.2),               # several short segments, κ = ±1/0.7
        (PLLOOP, L - 0.4, L + 0.3))                   # closed-track lap boundary (straight → arc)
        lin = curvature_linearization(track, [a, b])
        A_fd, B_fd = fd_sens(track, a, b)
        @test lin.kbar[1] ≈ average_curvature(track, a, b) atol = 1e-12
        @test isapprox(lin.A[1], A_fd; atol=1e-6, rtol=1e-6)
        @test isapprox(lin.B[1], B_fd; atol=1e-6, rtol=1e-6)
        @test lin.boundary_knots == 0 && lin.short_intervals == 0
        @test lin.A[1] != 0 || lin.B[1] != 0
    end
end

@testset "constant-curvature intervals have exactly zero sensitivity" begin
    lin = curvature_linearization(PLQ, [2.3, 2.6, 2.9, 3.4, 0.5, 1.0][1:4])
    @test lin.A == zeros(3) && lin.B == zeros(3) && lin.kbar ≈ ones(3)
    @test curvature_linearization(circular_track(2.0), collect(0.0:0.3:3.0)).A == zeros(10)
    lin = curvature_linearization(straight_track(10.0), [1.0, 2.0])
    @test lin.A == [0.0] && lin.B == [0.0] && lin.kbar == [0.0]
end

@testset "short, stationary and boundary-knot intervals" begin
    # Short (|Δs| < 1 mm) and stationary intervals: constant local model, no division.
    lin = curvature_linearization(PLQ, [1.9996, 2.0003])
    @test lin.A == [0.0] && lin.B == [0.0] && lin.short_intervals == 1
    lin = curvature_linearization(PLQ, fill(2.0, 4))                 # stationary exactly on a boundary
    @test lin.kbar == [1.0, 1.0, 1.0] && lin.A == zeros(3) && lin.short_intervals == 3
    # Knot on a boundary: mean of the one-sided derivatives (documented fallback).
    a, b = 2.0, 2.3
    lin = curvature_linearization(PLQ, [a, b])
    @test lin.boundary_knots == 1
    K(x, y) = average_curvature(PLQ, x, y)
    h = 1e-7
    left = (K(a, b) - K(a - h, b)) / h
    right = (K(a + h, b) - K(a, b)) / h
    @test right ≈ 0 atol = 1e-6                                       # K = 1 for any a ≥ 2
    @test lin.A[1] ≈ (left + right) / 2 rtol = 1e-4
    lin = curvature_linearization(PLQ, [1.7, 2.0])                    # end knot on the boundary
    @test lin.boundary_knots == 1 && lin.B[1] ≈ (0.5 - 0.0) / 0.3
end

@testset "augmented NLP: variables, bounds, parameters" begin
    c = NMPCController(PLV, PLQ, NMPCConfig(curvature_model=:progress_linearized, linearization_trust_region=0.1))
    N = horizon(c)
    @test curvature_model(c) === :progress_linearized
    @test length(c.kappa_eff) == N && length(c.lin_sbar) == N + 1 && length(c.lin_tr) == N + 1
    @test all(k -> lower_bound(k) == -1.0 && upper_bound(k) == 1.0, c.kappa_eff)
    @test all(is_parameter, c.lin_kbar) && all(is_parameter, c.lin_A) && all(is_parameter, c.lin_B) && all(is_parameter, c.lin_sbar)
    @test !any(is_parameter, c.kappa_eff)
    # Frenet safety holds for every value kappa_eff can take (its bounds = the track range).
    lo, hi = c.stage_ey_bounds
    @test all(1 - k * e >= 2 * PLV.eps_geometry - 1e-12 for k in (-1.0, 1.0), e in (lo, hi))

    # Parameters follow the schedule; the trust region is set on knots of crossing stages.
    s = collect(1.1 .+ 0.25 .* (0:N))                                  # no knot on a boundary
    sched = (; method=:test, iterations=1, converged=true, change=0.0, kappas=stage_curvatures(PLQ, s), s_nominal=s,
        Xg=zeros(4, N + 1), Ug=zeros(2, N))
    sch = install_schedule!(c, sched)
    lin = curvature_linearization(PLQ, s)
    @test parameter_value.(c.lin_kbar) == lin.kbar && parameter_value.(c.lin_A) == lin.A && parameter_value.(c.lin_B) == lin.B
    @test parameter_value.(c.lin_sbar) == s
    crossing = findall(k -> lin.A[k] != 0 || lin.B[k] != 0, 1:N)
    @test length(crossing) == 2
    for k in 2:N+1
        adj = (k - 1 in crossing) || (k <= N && k in crossing)
        @test parameter_value(c.lin_tr[k]) ≈ (adj ? 0.1 * 0.25 : 1e6)
    end
    @test sch.lin.tr[1] == 0.1 * 0.25 || !(1 in crossing)
    # The κ_eff constraints evaluate the documented affine model.
    # The κ_eff definitions are the quadratic (Parameter × variable) constraints.
    cons = [con for con in all_constraints(c.model; include_variable_in_set_constraints=false)
            if constraint_object(con).func isa QuadExpr && occursin("kappa_eff", string(constraint_object(con).func))]
    @test length(cons) == N
    sstar = s .+ 0.01 .* (0:N)
    vals = Dict{VariableRef,Float64}()
    for k in 2:N+1
        vals[c.X[1, k]] = sstar[k]
    end
    for k in 1:N
        vals[c.kappa_eff[k]] = lin.kbar[k] + lin.A[k] * (sstar[k] - s[k]) + lin.B[k] * (sstar[k+1] - s[k+1])
    end
    set_parameter_value(c.X[1, 1], sstar[1])
    valfn = v -> is_parameter(v) ? parameter_value(v) : get(vals, v, 0.0)
    @test maximum(abs(value(valfn, constraint_object(con).func) - MOI.constant(constraint_object(con).set)) for con in cons) <= 1e-12
end

@testset "one solve through the augmented NLP: consistency and reference" begin
    # Straight → arc, cold start: the stagewise model is stale, the linearized one is not.
    x = [3.0 - 0.5 * 2.0 * 0.1, 0.0, 0.0, 2.0]
    rs = solve!(NMPCController(PLV, PLTRANS, NMPCConfig()), x)
    rl = solve!(NMPCController(PLV, PLTRANS, NMPCConfig(curvature_model=:progress_linearized, linearization_trust_region=0.1)), x)
    @test rs.status === :schedule_inconsistent
    @test rl.status === :success && rl.nlp_solves == 1 && rl.schedule.curvature_model === :progress_linearized
    @test rl.schedule.stage_residual <= 5e-3 && rl.reference.passed
    @test rl.reference.max_position_error < 0.02 && rl.reference.first_step_position_error < 0.01
    @test rl.max_dynamics_residual <= 1e-6                            # validated with the optimised κ_eff
    # The reported model error is |κ_eff − exact average| along the optimised progress.
    c = NMPCController(PLV, PLTRANS, NMPCConfig(curvature_model=:progress_linearized, linearization_trust_region=0.1))
    r = solve!(c, x)
    keff = value.(c.kappa_eff)
    @test r.schedule.model_curvature_error ≈ maximum(abs, keff - stage_curvatures(PLTRANS, r.X[1, :])) atol = 1e-12
    # κ_eff obeys its affine model at the solution.
    lin = curvature_linearization(PLTRANS, r.schedule.s_nominal)
    sn = r.schedule.s_nominal
    @test maximum(abs, keff - [lin.kbar[k] + lin.A[k] * (r.X[1, k] - sn[k]) + lin.B[k] * (r.X[1, k+1] - sn[k+1]) for k in 1:20]) <= 1e-6
    @test minimum(keff) >= -1e-12 - 0.0 && maximum(keff) <= 0.5 + 1e-12   # within [κ_min, κ_max] = [0, 0.5]
end

@testset "invalid local models are detected and reported" begin
    c = NMPCController(PLV, PLQ, LINCFG)
    N = horizon(c)
    s = collect(1.0 .+ 0.25 .* (0:N))
    lin = (; curvature_linearization(PLQ, s)..., tr=fill(Inf, N + 1))
    sched = (; method=:test, iterations=1, converged=true, change=0.0, kappas=lin.kbar, s_nominal=s, lin)
    # The optimised progress moves stage 3's end knot (nominally 1.75) across the boundary at
    # s = 2: the assumed segment configuration changed.
    sstar = copy(s)
    sstar[4] = 2.05
    core = (; X=vcat(reshape(sstar, 1, :), zeros(3, N + 1)), kappas_used=copy(lin.kbar))
    rep = schedule_consistency(c, sched, core, true)
    @test rep.configuration_changes >= 1 && rep.curvature_model === :progress_linearized
    @test rep.model_curvature_error > 0.1 && !rep.consistent
    # κ_eff driven onto its bound while the nominal is interior: saturated stage.
    k = findfirst(i -> lin.A[i] != 0 || lin.B[i] != 0, 1:N)
    core2 = (; X=vcat(reshape(s, 1, :), zeros(3, N + 1)), kappas_used=[i == k ? -1.0 : lin.kbar[i] for i in 1:N])
    rep2 = schedule_consistency(c, sched, core2, true)
    @test rep2.saturated_stages == 1 && !rep2.consistent
end

@testset "measured state checked for every curvature the NLP may use" begin
    # Inside the κ = +1 arc with ey = -0.95: valid for the actual κ = +1 (and for the fixed
    # first-stage κ̄ = 1), but singular-adjacent for κ = -1, which κ_eff may take.
    x = [2.5, -0.95, 0.0, 0.5]
    rl = solve!(NMPCController(PLV, PLQ, LINCFG), x)
    @test rl.status === :invalid_measurement && occursin("first-stage curvature", rl.message)
    rs = solve!(NMPCController(PLV, PLQ, NMPCConfig()), x)
    @test rs.status !== :invalid_measurement                         # fixed κ̄_1 = 1 is valid there
end

@testset "single-segment tracks: identical to the stagewise mode" begin
    for (tr, x0) in ((circular_track(1.0; width=0.7), [0.0, 0.0, 0.0, 0.5]), (straight_track(100.0; width=0.7), [0.0, 0.2, 0.1, 0.5]))
        cl1 = simulate_closed_loop(NMPCController(PLV, tr, NMPCConfig()), PLV, x0, 15)
        c2 = NMPCController(PLV, tr, LINCFG)
        cl2 = simulate_closed_loop(c2, PLV, x0, 15)
        @test curvature_model(c2) === :constant && isempty(c2.kappa_eff)
        @test cl1.states == cl2.states && cl1.inputs == cl2.inputs
    end
end

@testset "configuration validation" begin
    @test_throws ArgumentError NMPCController(PLV, PLQ, NMPCConfig(curvature_model=:smooth))
    @test_throws ArgumentError NMPCController(PLV, PLQ, NMPCConfig(curvature_model=:progress_linearized, linearization_trust_region=0.0))
    @test_throws ArgumentError NMPCController(PLV, PLQ, NMPCConfig(curvature_model=:progress_linearized, linearization_min_interval=-1.0))
    @test curvature_model(NMPCController(PLV, PLQ, NMPCConfig())) === :stagewise_average  # default unchanged
end
