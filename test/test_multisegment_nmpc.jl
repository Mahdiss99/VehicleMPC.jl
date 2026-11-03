# Multi-segment NMPC (Milestone 2B.2): curvature schedule, Frenet safety, consistency and
# reference checks, failure classification. Correction budgets are set large wherever a
# result could otherwise depend on wall-clock time, so outcomes are deterministic.

const MV = KinematicBicycle(reference_test_vehicle())
const MQ = quarter_track()
const ML = MV.params.wheelbase
# Open straight → left arc (R = 2 m) → straight, with room for a 2 s horizon.
const MTRANS = Track([TrackSegment(3.0, 0.0), TrackSegment(1.5π, 0.5), TrackSegment(6.0, 0.0)]; width=0.7)
const NOBUDGET = 100.0

"Fake post-solve result with a given progress column (for schedule_consistency)."
fake_core(s) = (; X=vcat(reshape(collect(s), 1, :), zeros(3, length(s))))

@testset "Frenet safety on the whole curvature range" begin
    c = NMPCController(MV, MQ, NMPCConfig())
    @test c.kappa_range == (-1.0, 1.0)
    lo, hi = c.stage_ey_bounds
    @test lo ≈ -0.8 && hi ≈ 0.8
    m = 2 * MV.eps_geometry
    # Every legal curvature and every stage point inside its bounds keeps 1 - kappa*ey ≥ 2 eps.
    @test all(1 - k * e >= m - 1e-12 for k in range(-1, 1; length=41), e in range(lo, hi; length=41))
    @test all(1 - k * e >= m for k in (-1.0, 1.0), e in (-c.ey_max, c.ey_max))
    @test all(upper_bound(c.P[1, j, k]) ≈ hi && lower_bound(c.P[1, j, k]) ≈ lo for j in 1:3, k in 1:horizon(c))
    # Curvature updates are range-checked: illegal values never reach the NLP.
    set_curvature_schedule!(c, fill(1.0, 20))
    set_curvature_schedule!(c, fill(-1.0, 20))
    @test_throws ArgumentError set_curvature_schedule!(c, fill(1.2, 20))
    @test_throws ArgumentError set_curvature_schedule!(c, [NaN; zeros(19)])
    @test_throws DimensionMismatch set_curvature_schedule!(c, zeros(19))
    # Geometry without a Frenet-safe corridor is rejected (kappa = ±4 on a 0.7 m track).
    @test_throws ArgumentError NMPCController(MV, Track([TrackSegment(1.0, 0.0), TrackSegment(0.3, 4.0)]; width=0.7), NMPCConfig())
    @test_throws ArgumentError NMPCController(MV, Track([TrackSegment(1.0, 0.0), TrackSegment(0.3, -4.0)]; width=0.7), NMPCConfig())
    # Single-segment tracks keep their Milestone 2A bounds.
    cc = NMPCController(MV, circular_track(1.0; width=0.7), NMPCConfig())
    @test cc.kappa_range == (1.0, 1.0) && cc.stage_ey_bounds[1] == -Inf && cc.stage_ey_bounds[2] ≈ 0.8
end

@testset "single-segment tracks: exact constant curvature" begin
    for (tr, k) in ((circular_track(1.0; width=0.7), 1.0), (straight_track(100.0; width=0.7), 0.0))
        c = NMPCController(MV, tr, NMPCConfig())
        r = solve!(c, [0.5, 0.05, 0.0, 1.0])
        @test r.accepted && r.schedule.method === :constant && r.schedule.iterations == 0
        @test r.schedule.kappas == fill(k, 20) && curvature_values(c) == fill(k, 20)
        @test r.schedule.stage_residual == 0.0 && r.schedule.consistent && r.reference.passed
        @test r.nlp_solves == 1
    end
    # The averaging formula itself also reduces to the constant curvature on a circle.
    tr = circular_track(1.0)
    @test maximum(abs.(stage_curvatures(tr, collect(0.0:0.37:7.4)) .- 1.0)) <= 1e-12
end

@testset "curvature parameters update between solves and reach the dynamics" begin
    c = NMPCController(MV, MQ, NMPCConfig(schedule_corrections=10, correction_time_budget=NOBUDGET))
    r1 = solve!(c, [0.5, 0.0, 0.0, 1.0])
    k1 = curvature_values(c)
    @test k1 == r1.schedule.kappas
    reset_warm_start!(c)
    r2 = solve!(c, [3.0, 0.0, 0.0, 1.5])                      # inside the κ = +1 arc, κ = -1 ahead
    k2 = curvature_values(c)
    @test k2 == r2.schedule.kappas && k2 != k1
    @test any(k -> k < 0, k2) && any(k -> k > 0.5, k2)
    # The parameters genuinely enter the dynamics: at a rollout consistent with the current
    # schedule every dynamics constraint has zero residual, and with another schedule not.
    U = r2.U
    X = VehicleMPC.strict_rollout(c, r2.X[:, 1], U, k2)
    vals = Dict{VariableRef,Float64}()
    for k in 2:21, i in 1:4
        vals[c.X[i, k]] = X[i, k]
    end
    for k in 1:20, i in 1:2
        vals[c.U[i, k]] = U[i, k]
    end
    L = MV.params.wheelbase
    for k in 1:20
        f = z -> collect(kinematic_rhs(L, z..., U[1, k], U[2, k], k2[k]))[2:4]
        z1 = X[2:4, k]
        z2 = z1 + 0.05 * f(z1)
        z3 = z1 + 0.05 * f(z2)
        z4 = z1 + 0.1 * f(z3)
        for (j, z) in enumerate((z2, z3, z4)), i in 1:3
            vals[c.P[i, j, k]] = z[i]
        end
    end
    valfn = v -> is_parameter(v) ? parameter_value(v) : vals[v]
    resid() = maximum(abs(value(valfn, constraint_object(con).func) - MOI.constant(constraint_object(con).set)) for con in c.dynamics)
    @test resid() <= 1e-12
    set_curvature_schedule!(c, fill(0.0, 20))
    @test resid() > 1e-3
end

@testset "schedule initialisation and fixed point" begin
    c = NMPCController(MV, MQ, NMPCConfig())
    x = [0.0, 0.0, 0.0, 0.5]
    s_nom, U = VehicleMPC.quasi_steady_guess(c, x)
    @test s_nom[1] == 0.0 && issorted(s_nom)
    @test all(abs.(U[1, :]) .<= MV.params.delta_max) && all(MV.params.a_min .<= U[2, :] .<= MV.params.a_max)
    @test s_nom[end] > 2.0                                     # the guess reaches the first curve
    fp = curvature_fixed_point(c, x, U, s_nom)
    @test fp.converged && fp.iterations >= 2
    # At convergence the schedule was computed from s_nominal, and the rollout with it
    # reproduces s_nominal within schedule_tol.
    @test fp.kappas ≈ stage_curvatures(MQ, fp.s_nominal) atol = 1e-15
    @test maximum(abs, VehicleMPC.strict_rollout(c, x, U, fp.kappas)[1, :] - fp.s_nominal) <= c.config.schedule_tol
    # 2B.1 regression: a nominal 10 % or 20 % too slow or too fast is corrected by the fixed point.
    for scale in (0.8, 0.9, 1.1, 1.2)
        fps = curvature_fixed_point(c, x, U, x[1] .+ scale .* (s_nom .- x[1]))
        @test fps.converged && maximum(abs, fps.kappas - fp.kappas) <= 1e-6
    end
    # Warm start after an accepted solve uses the shifted solution.
    cw = NMPCController(MV, MQ, NMPCConfig(schedule_corrections=10, correction_time_budget=NOBUDGET))
    r1 = solve!(cw, x)
    @test r1.accepted && r1.schedule.method in (:cold, :cold_corrected, Symbol("cold_corrected_corrected"),
        Symbol("cold_corrected_corrected_corrected"), Symbol("cold_corrected_corrected_corrected_corrected"),
        Symbol("cold_corrected_corrected_corrected_corrected_corrected"))
    r2 = solve!(cw, r1.X[:, 2]; previous_input=r1.u)
    @test startswith(string(r2.schedule.method), "warm") && r2.warm_started
end

@testset "fixed-point non-convergence is reported, never hidden" begin
    c = NMPCController(MV, MQ, NMPCConfig(schedule_max_iter=1))
    x = [0.0, 0.0, 0.0, 0.5]
    s_nom, U = VehicleMPC.quasi_steady_guess(c, x)
    fp = curvature_fixed_point(c, x, U, s_nom)
    @test !fp.converged && fp.change > c.config.schedule_tol && occursin("not converged", fp.message)
    r = solve!(c, x)
    @test r.status === :scheduling_failure && !r.accepted && r.u === nothing && r.nlp_solves == 0
    @test r.termination_status == MOI.OPTIMIZE_NOT_CALLED && !r.schedule.converged
    # Open track: a horizon running past the end is a scheduling failure, not an extrapolation.
    ct = NMPCController(MV, MTRANS, NMPCConfig())
    r = solve!(ct, [track_length(MTRANS) - 1.0, 0.0, 0.0, 2.0])
    @test r.status === :scheduling_failure && occursin("leaves the track", r.message)
end

@testset "stationary state exactly on a curvature boundary" begin
    @test_throws ArgumentError average_curvature(MQ, 2.0, 2.0)          # the raw limit is ambiguous
    @test stage_curvatures(MQ, fill(2.0, 21)) == fill(1.0, 20)           # right limit: segment entered
    c = NMPCController(MV, MQ, NMPCConfig(schedule_corrections=10, correction_time_budget=NOBUDGET))
    r = solve!(c, [2.0, 0.0, 0.0, 0.0])                                   # no exception
    @test r.status in (:success, :schedule_inconsistent, :reference_violation, :scheduling_failure, :solver_failure)
    @test r.schedule.method !== :none
end

@testset "post-solve schedule residuals" begin
    c = NMPCController(MV, MQ, NMPCConfig())
    s = collect(1.0 .+ 0.25 .* (0:20))                                    # crosses s = 2 and s = 2 + π
    kap = stage_curvatures(MQ, s)
    sched = (; method=:test, iterations=1, converged=true, change=0.0, kappas=kap, s_nominal=s)
    rep = schedule_consistency(c, sched, fake_core(s), true)
    @test rep.stage_residual <= 1e-12 && rep.cumulative_residual <= 1e-12 && rep.consistent
    @test rep.crossing_stages == 2 && rep.progress_mismatch == 0
    # Optimised progress 2 cm ahead of the nominal: the residual equals the curvature-integral
    # mismatch computed independently from the segment curvatures.
    s2 = copy(s)
    s2[2:end] .+= 0.02
    rep2 = schedule_consistency(c, sched, fake_core(s2), true)
    r = [centerline_heading(MQ, s2[k+1]) - centerline_heading(MQ, s2[k]) - kap[k] * (s2[k+1] - s2[k]) for k in 1:20]
    @test rep2.stage_residual ≈ maximum(abs, r) && rep2.cumulative_residual ≈ maximum(abs, cumsum(r))
    @test rep2.progress_mismatch ≈ 0.02
    @test !rep2.consistent && occursin("residual", rep2.message)
    # A deliberately stale schedule (all straight) on curved progress is detected.
    stale = (; sched..., kappas=zeros(20))
    rep3 = schedule_consistency(c, stale, fake_core(s), true)
    @test !rep3.consistent && isapprox(rep3.stage_residual, 0.25; atol=1e-12)   # a full stage on the κ = 1 arc
end

@testset "stale schedules: rejected by default, qualified only if verified" begin
    x = [0.0, 0.0, 0.0, 0.5]
    # Cold start with a single solve: the optimiser accelerates past the nominal, so the
    # schedule is stale (documented behaviour on the quarter track).
    r = solve!(NMPCController(MV, MQ, NMPCConfig()), x)
    @test r.status === :schedule_inconsistent && !r.accepted && r.u === nothing && r.nlp_solves == 1
    @test !r.schedule.consistent && r.schedule.stage_residual > 0.005
    @test occursin("stale curvature schedule", r.message)
    @test r.reference.checked                                            # reported, even when rejected
    # The opt-in qualified policy accepts it only because the reference check passed.
    rq = solve!(NMPCController(MV, MQ, NMPCConfig(stale_schedule_policy=:accept_if_reference_valid)), x)
    @test rq.status === :success_stale_schedule && rq.accepted && rq.reference.passed
    # Corrections remove the staleness (deterministic: no time limit).
    rc = solve!(NMPCController(MV, MQ, NMPCConfig(schedule_corrections=10, correction_time_budget=NOBUDGET)), x)
    @test rc.status === :success && rc.schedule.consistent && rc.nlp_solves > 1
    @test rc.schedule.stage_residual <= 0.005 && rc.schedule.cumulative_residual <= 0.01
end

@testset "independent reference validation on the actual track" begin
    c = NMPCController(MV, MQ, NMPCConfig(schedule_corrections=10, correction_time_budget=NOBUDGET))
    r = solve!(c, [0.0, 0.0, 0.0, 0.5])
    @test r.accepted && r.reference.passed && r.reference.status === :success
    @test r.reference.crossings >= 1 && r.schedule.crossing_stages >= 1
    @test r.reference.max_position_error <= 0.05 && r.reference.max_heading_error <= 0.02
    @test r.reference.max_abs_ey <= MQ.width / 2
    # Inputs that drive off the actual track are rejected by the reference check.
    Ubad = copy(r.U)
    Ubad[1, :] .= -MV.params.delta_max
    bad = reference_check(c, r.X[:, 1], r.X, Ubad)
    @test !bad.passed && any(p -> p.second.constraint === :track_boundary, bad.violations) ||
          (!bad.passed && bad.status !== :success)
    # Over-speed between knots is detected too.
    Ufast = copy(r.U)
    Ufast[2, :] .= MV.params.a_max
    fast = reference_check(c, r.X[:, 1], r.X, Ufast)
    @test !fast.passed
end

@testset "multiple transitions and negative curvature in one horizon" begin
    lab = legacy_track(:lab)
    c = NMPCController(MV, lab, NMPCConfig(schedule_corrections=10, correction_time_budget=NOBUDGET))
    r = solve!(c, [lab.s_start[5] + 0.1, 0.0, 0.0, 1.0])
    @test r.schedule.converged
    @test any(<(0), r.schedule.kappas) && any(>(1), r.schedule.kappas)    # κ = ±1/0.7 inside the horizon
    @test count(k -> 0 < abs(k) < 1 / 0.7 - 1e-6, r.schedule.kappas) >= 2 # averaged crossing stages
    if r.accepted
        @test r.schedule.crossing_stages >= 2 && r.reference.passed
    else
        @test r.status in (:schedule_inconsistent, :reference_violation, :solver_failure)
    end
end

@testset "closed loop: event-aware plant, lap wrapping with unwrapped progress" begin
    L = track_length(MQ)
    cfg = NMPCConfig(track_margin=0.10, schedule_corrections=8, correction_time_budget=NOBUDGET,
        stale_schedule_policy=:accept_if_reference_valid)
    c = NMPCController(MV, MQ, cfg)
    cl = simulate_closed_loop(c, MV, [L - 1.5, 0.0, 0.0, 1.5], 20)
    @test cl.plant_integration === :event_aware
    @test cl.states[end][1] > L                                          # unwrapped, not reset
    @test any(cr -> cr.to == 1 && cr.lap == 1, cl.crossings)             # lap boundary resolved
    @test all(r -> r.accepted ? r.reference.passed : true, cl.controller)
    @test isempty(cl.violations)
    # Single-segment tracks keep the Milestone 2A plant.
    cc = NMPCController(MV, circular_track(1.0; width=0.7), NMPCConfig())
    @test simulate_closed_loop(cc, MV, [0.0, 0.0, 0.0, 0.5], 2).plant_integration === :zero_order_hold
end

@testset "solver failures on multi-segment tracks are classified as such" begin
    r = solve!(NMPCController(MV, MQ, NMPCConfig(max_iter=1)), [0.0, 0.0, 0.0, 0.5])
    @test r.status === :solver_failure && r.u === nothing && r.nlp_solves == 1
    @test !r.schedule.consistent && occursin("not evaluated", r.schedule.message)
    # Invalid measurement checked with the actual curvature at its position (κ = +1 arc).
    r = solve!(NMPCController(MV, MQ, NMPCConfig()), [3.0, 0.95, 0.0, 1.0])
    @test r.status === :invalid_measurement && r.nlp_solves == 0
end
