# Exact Cartesian discretisation and the experimental Cartesian-endpoint NMPC (Milestone 2B.4).

const CEV = KinematicBicycle(reference_test_vehicle())
const CEL = CEV.params.wheelbase
const CEQ = quarter_track()
const CETRANS = Track([TrackSegment(3.0, 0.0), TrackSegment(1.5π, 0.5), TrackSegment(6.0, 0.0)]; width=0.7)
const CECFG = NMPCConfig(prediction_model=:cartesian_endpoint_exact, schedule_corrections=8, correction_time_budget=100.0)

"Classical RK4 of the Cartesian bicycle in BigFloat with m substeps (independent reference)."
function ce_rk4_big(z0, u, dt; m=1000)
    setprecision(BigFloat, 256) do
        L, d, a, h = big(CEL), big(u[1]), big(u[2]), big(dt) / m
        f(z) = [z[4] * cos(z[3]), z[4] * sin(z[3]), z[4] * tan(d) / L, a]
        z = big.(collect(Float64, z0))
        for _ in 1:m
            k1 = f(z); k2 = f(z .+ h / 2 .* k1); k3 = f(z .+ h / 2 .* k2); k4 = f(z .+ h .* k3)
            z = z .+ h / 6 .* (k1 .+ 2k2 .+ 2k3 .+ k4)
        end
        Float64.(z)
    end
end

"Inputs from a simple tracking feedback, so that reference trajectories stay on the track."
function ce_feedback(track, x0, n, dt; a=k -> 0.0)
    x, us = SVector{4}(Float64.(x0)...), SVector{2,Float64}[]
    for k in 1:n
        u = SVector(clamp(atan(curvature(track, x[1] + 0.3) * CEL) - 1.5x[2] - 1.2x[3], -0.43, 0.43), a(k))
        push!(us, u)
        x = integrate_on_track(CEV, track, x, u, dt; max_step=1e-3).x
    end
    return us
end

@testset "sinc_un: convention, accuracy and derivatives" begin
    @test sinc_un(0.0) == 1.0 && sinc_un_d1(0.0) == 0.0 && sinc_un_d2(0.0) ≈ -1 / 3
    @test sinc_un(0.3) ≈ sin(0.3) / 0.3 && !(sinc(0.3) ≈ sin(0.3) / 0.3)      # Base.sinc is the normalised one
    zs = vcat(0.0, [s * 10.0^e for e in -12:0.5:0.5 for s in (-1, 1)], prevfloat(0.5), 0.5, nextfloat(0.5), 1.3, -2.7)
    big_sinc(z) = setprecision(() -> z == 0 ? big(1.0) : sin(big(z)) / big(z), BigFloat, 256)
    big_d1(z) = setprecision(() -> (zb = big(z); zb == 0 ? big(0.0) : (zb * cos(zb) - sin(zb)) / zb^2), BigFloat, 256)
    big_d2(z) = setprecision(() -> (zb = big(z); ((2 - zb^2) * sin(zb) - 2zb * cos(zb)) / zb^3), BigFloat, 256)
    @test maximum(abs(sinc_un(z) - big_sinc(z)) for z in zs) <= 2e-16
    @test maximum(abs(sinc_un_d1(z) - big_d1(z)) for z in zs) <= 5e-16
    @test maximum(abs(sinc_un_d2(z) - big_d2(z)) for z in zs if abs(z) > 1e-6) <= 1e-15
    # Automatic differentiation through sinc_un and its derivative.
    D = VehicleMPC.ForwardDiff.derivative
    @test maximum(abs(D(sinc_un, z) - sinc_un_d1(z)) for z in zs) <= 5e-16
    @test maximum(abs(D(sinc_un_d1, z) - sinc_un_d2(z)) for z in zs) <= 2e-15
    # Smooth across the series/closed-form switch.
    @test abs(sinc_un(prevfloat(0.5)) - sinc_un(0.5)) <= 1e-15 && abs(sinc_un_d2(prevfloat(0.5)) - sinc_un_d2(0.5)) <= 1e-14
end

@testset "exact Cartesian step vs independent high-accuracy integration" begin
    cases = [([0.0, 0, 0, 2.0], [0.0, 0.0]),            # straight
        ([1.0, -2, 0.3, 2.5], [0.4, 0.0]),              # left
        ([1.0, -2, 0.3, 2.5], [-0.4, 0.0]),             # right
        ([0.0, 0, 1.0, 1.0], [0.2, 5.0]),               # acceleration, constant steering
        ([0.0, 0, -2.0, 3.0], [-0.3, -5.0]),            # braking, constant steering
        ([0.0, 0, 0.7, 1.0], [0.0, 3.0]),               # zero steering
        ([0.0, 0, 0.7, 2.0], [1e-9, 1.0]),              # near-zero steering
        ([0.0, 0, 0.0, 0.0], [0.43, 5.0]),              # from rest
        ([5.0, 3, 40.0, 4.0], [0.3, -2.0]),             # large unwrapped heading
        ([0.0, 0, 0.0, 5.0], [deg2rad(25), 5.0]),       # largest steering and speed
        ([0.0, 0, 0.5, 0.5], [0.3, -5.0])]              # braking to rest exactly at dt
    for (z0, u) in cases
        ze = exact_cartesian_step(CEV, z0, u, 0.1)
        zr = ce_rk4_big(z0, u, 0.1)
        @test maximum(abs.(ze .- zr)) <= 5e-15 * max(1, abs(z0[3]))
        @test maximum(abs.(ze .- integrate_cartesian(CEV, z0, u, 0.1; max_step=1e-4))) <= 1e-12
    end
    # Zero steering reduces to the straight-line formula exactly.
    z = exact_cartesian_step(CEV, [1.0, 2.0, 0.3, 2.0], [0.0, -1.0], 0.1)
    d = 2.0 * 0.1 - 0.5 * 0.01
    @test z[1] == 1.0 + d * cos(0.3) && z[2] == 2.0 + d * sin(0.3) && z[3] == 0.3 && z[4] == 2.0 - 0.1
    # Continuity as the steering tends to zero (no cancellation).
    zz = [exact_cartesian_step(CEV, [0.0, 0.0, 0.3, 2.0], [δ, 1.0], 0.1) for δ in (1e-14, 1e-10, 1e-6)]
    @test maximum(abs.(zz[1] .- exact_cartesian_step(CEV, [0.0, 0.0, 0.3, 2.0], [0.0, 1.0], 0.1))) <= 1e-13   # true change ≈ 8e-15 rad
    @test all(isfinite, zz[3]) && maximum(abs.(zz[3] .- zz[2])) <= 1e-6
    @test_throws DomainError exact_cartesian_step(CEV, zeros(4), [π / 2, 0.0], 0.1)
    @test_throws ArgumentError exact_cartesian_step(CEV, zeros(4), [0.0, 0.0], 0.0)
end

@testset "automatic differentiation through the exact map" begin
    J = VehicleMPC.ForwardDiff.jacobian
    for (z0, u) in (([0.3, -0.2, 0.4, 1.7], [0.2, 1.0]), ([0.0, 0.0, 0.0, 2.0], [0.0, 0.0]), ([0.0, 0.0, 1.0, 0.5], [1e-9, -2.0]))
        w = vcat(z0, u)
        f = v -> exact_cartesian_step(CEV, v[1:4], v[5:6], 0.1)
        Ja = J(f, w)
        Jfd = similar(Ja)
        for j in 1:6
            e = zeros(6)
            e[j] = 1e-6
            Jfd[:, j] = (f(w + e) - f(w - e)) / 2e-6
        end
        @test maximum(abs.(Ja - Jfd)) <= 1e-8
    end
end

@testset "segment-local centreline and Frenet formulas" begin
    for tr in (CEQ, legacy_track(:lab), circular_track(2.0; direction=-1, width=0.7), CETRANS)
        L = track_length(tr)
        top = tr.closed ? 2.5L : L
        for s in range(0.0, top; length=301), ey in (-0.3, 0.0, 0.25), epsi in (0.0, -0.4)
            f = segment_frame(tr, segment_id(tr, s, :right))
            @test maximum(abs.(segment_local_pose(f.x0, f.y0, f.psi0, f.kappa, s - f.s0) .- centerline_pose(tr, s))) <= 2e-14
            @test maximum(abs.(frenet_pose_local(f, s, ey, epsi) .- frenet_to_cartesian(tr, s, ey, epsi))) <= 2e-14
        end
    end
    # Both curvature signs: R = 1 m left and right arcs of the quarter track.
    @test any(seg.curvature > 0 for seg in CEQ.segments) && any(seg.curvature < 0 for seg in CEQ.segments)
    # Unwrapped heading: the lap-2 frame of segment 1 continues the lap-1 last segment.
    n, L = num_segments(CEQ), track_length(CEQ)
    flast, fnext = segment_frame(CEQ, n), segment_frame(CEQ, n + 1)
    @test fnext.psi0 ≈ flast.psi0 + flast.kappa * (flast.s1 - flast.s0) atol = 1e-13
    @test fnext.psi0 - segment_frame(CEQ, 1).psi0 ≈ total_turning(CEQ) atol = 1e-14
    @test maximum(abs.(frenet_pose_local(flast, L, 0.1, 0.2) .- frenet_pose_local(fnext, L, 0.1, 0.2))) <= 1e-13
    @test frenet_pose_local(fnext, L + 1.0, 0.0, 0.0)[3] ≈ centerline_heading(CEQ, L + 1.0) atol = 1e-13
end

@testset "segment assignment: boundary knots, laps, open-track ends" begin
    b = CEQ.s_start[2]                                                # straight → R = 1 m boundary at s = 2
    asg = cartesian_assignment(CEQ, [b - 0.1, b, b + 0.1, track_length(CEQ) + b])
    @test asg.seg[2] == 2 && asg.frames[2].s0 == b                     # a knot on a boundary enters the next segment
    @test asg.boundary_knots == 2 && asg.seg[4] == num_segments(CEQ) + 2
    # On the boundary both adjacent frames give the same pose (continuity), and both are valid there.
    f1, f2 = segment_frame(CEQ, 1), segment_frame(CEQ, 2)
    @test maximum(abs.(frenet_pose_local(f1, b, 0.2, 0.1) .- frenet_pose_local(f2, b, 0.2, 0.1))) <= 1e-15
    @test VehicleMPC.frame_valid(CEQ, f1, b, 1e-6) && VehicleMPC.frame_valid(CEQ, f2, b, 1e-6)
    @test !VehicleMPC.frame_valid(CEQ, f1, b + 1e-3, 1e-6)
    # A circle's analytic continuation is the track itself: valid on every lap.
    circ = circular_track(1.0; width=0.7)
    @test VehicleMPC.frame_valid(circ, segment_frame(circ, 1), 20.0, 1e-6)
    # Open track: the end point belongs to the last segment; beyond it is an error.
    a = cartesian_assignment(CETRANS, [0.0, track_length(CETRANS)])
    @test a.seg == [1, 3]
    @test_throws DomainError cartesian_assignment(CETRANS, [track_length(CETRANS) + 0.1])
end

@testset "NLP structure, residuals and Parameter updates" begin
    c = NMPCController(CEV, CEQ, CECFG)
    N = horizon(c)
    @test prediction_model(c) === :cartesian_endpoint_exact && curvature_model(c) === :none
    @test size(c.cframe) == (5, N + 1) && isempty(c.P) && isempty(c.kappa_eff) && length(c.dynamics) == 4N
    # A trajectory of the true dynamics (event-aware, small steps) satisfies the NLP equalities.
    x0 = [1.2, 0.05, 0.02, 2.0]
    us = ce_feedback(CEQ, x0, N, 0.1; a=k -> 0.5)
    sim = simulate_on_track(CEV, CEQ, x0, us, 0.1; max_step=1e-4)
    @test succeeded(sim) && !isempty(sim.crossings)
    X, U = reduce(hcat, sim.states), reduce(hcat, us)
    sched = (; method=:test, iterations=0, converged=true, change=0.0, kappas=stage_curvatures(CEQ, X[1, :]), Xg=X, Ug=U,
        s_nominal=X[1, :], message="")
    sched = install_schedule!(c, sched)
    asg = cartesian_assignment(CEQ, X[1, :])
    @test parameter_value.(c.cframe[1, :]) == [f.s0 for f in asg.frames]
    @test parameter_value.(c.cframe[4, :]) == [f.psi0 for f in asg.frames]
    @test parameter_value.(c.cframe[5, :]) == [f.kappa for f in asg.frames]
    @test !any(has_lower_bound, c.X[1, 2:end])                       # no membership bounds by default
    for i in 1:4
        set_parameter_value(c.X[i, 1], X[i, 1])
    end
    vals = Dict{VariableRef,Float64}()
    for k in 2:N+1, i in 1:4
        vals[c.X[i, k]] = X[i, k]
    end
    for k in 1:N, i in 1:2
        vals[c.U[i, k]] = U[i, k]
    end
    valfn = v -> is_parameter(v) ? parameter_value(v) : vals[v]
    res(con) = value(valfn, constraint_object(con).func) - MOI.constant(constraint_object(con).set)
    @test maximum(abs ∘ res, c.dynamics) <= 1e-10
    @test maximum(VehicleMPC.cartesian_stage_residual(c, X, U, asg.frames, k) for k in 1:N) <= 1e-10
    # The same trajectory evaluated with frames shifted by one segment violates the equalities.
    wrong = cartesian_assignment(CEQ, X[1, :] .+ 0.8)
    @test maximum(VehicleMPC.cartesian_stage_residual(c, X, U, wrong.frames, k) for k in 1:N) > 1e-3
    # Membership bounds (optional) are set from the assignment.
    cm = NMPCController(CEV, CEQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact, segment_membership=true))
    install_schedule!(cm, sched)
    @test all(k -> lower_bound(cm.X[1, k]) == asg.frames[k].s0 && upper_bound(cm.X[1, k]) == asg.frames[k].s1, 2:N+1)
    # Consecutive solves update the Parameters in place (no rebuild).
    m0 = c.model
    r1 = solve!(c, [1.0, 0.0, 0.0, 2.0])
    p1 = parameter_value.(c.cframe)
    r2 = solve!(c, [1.0 + 0.2, 0.01, 0.0, 2.1]; previous_input=r1.accepted ? r1.u : (0.0, 0.0))
    @test c.model === m0 && r1.accepted && r2.accepted
    @test parameter_value.(c.cframe) == reduce(hcat, [[f.s0, f.x0, f.y0, f.psi0, f.kappa]
                                                     for f in cartesian_assignment(CEQ, r2.schedule.s_nominal).frames])
    @test parameter_value.(c.cframe) != p1
end

@testset "one control cycle across a transition: exact prediction" begin
    c = NMPCController(CEV, CETRANS, CECFG)
    r = solve!(c, [2.9, 0.0, 0.0, 2.0])
    @test r.accepted && r.status === :success && r.schedule.curvature_model === :cartesian_endpoint_exact
    @test r.schedule.configuration_changes == 0 && r.schedule.crossing_stages >= 1
    @test r.max_dynamics_residual <= 1e-6 && r.exact.checked && r.exact.status === :success
    @test r.exact.knot_residual <= 1e-6 && r.exact.geometry_residual <= 1e-12
    @test r.exact.max_position_error <= 1e-6 && r.exact.max_progress_error <= 1e-6 && r.exact.max_ey_error <= 1e-6
    # Independent event-aware reference: the systematic averaging error is gone.
    @test r.reference.passed && r.reference.max_position_error <= 1e-5 && r.schedule.stage_residual <= 1e-6
    @test r.timing.reference > 0 && !deadline_missed(r, Inf) && deadline_missed(r, 0.0)
end

@testset "invalid segment assignments are detected and rejected" begin
    c = NMPCController(CEV, CEQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact))
    N = horizon(c)
    s = collect(1.1 .+ 0.25 .* (0:N))
    sched = install_schedule!(c, (; method=:test, iterations=0, converged=true, change=0.0, kappas=stage_curvatures(CEQ, s),
        Xg=zeros(4, N + 1), Ug=zeros(2, N), s_nominal=s, message=""))
    sstar = copy(s)
    sstar[4] = 2.05                                                   # nominal 1.85 (straight) moved onto the R = 1 m arc
    X = vcat(reshape(sstar, 1, :), zeros(2, N + 1), fill(2.0, 1, N + 1))
    rep = cartesian_consistency(c, sched, (; X, U=zeros(2, N)), true)
    @test rep.configuration_changes == 1 && !rep.consistent && occursin("assigned segments", rep.message)
    rep0 = cartesian_consistency(c, sched, (; X=vcat(reshape(s, 1, :), X[2:4, :]), U=zeros(2, N)), true)
    @test rep0.configuration_changes == 0 && rep0.consistent
    # The geometric mapping residual exposes the wrong frame (formula extrapolated past its end).
    e = exact_map_check(c, X[:, 1], X, zeros(2, N); frames=sched.asg.frames)
    @test e.geometry_residual > 1e-3
    # One solve from a cold start whose optimised knot leaves its segment: rejected, not accepted.
    c1 = NMPCController(CEV, CEQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact))
    r = solve!(c1, [0.0, 0.0, 0.0, 0.5])
    @test r.status === :segment_assignment_invalid && !r.accepted && r.u === nothing && r.nlp_solves == 1
    @test r.schedule.configuration_changes >= 1
    # With corrections the same cycle is reassigned and accepted.
    r8 = solve!(NMPCController(CEV, CEQ, CECFG), [0.0, 0.0, 0.0, 0.5])
    @test r8.accepted && r8.nlp_solves >= 2 && r8.schedule.configuration_changes == 0
    @test endswith(string(r8.schedule.method), "_corrected")
end

@testset "correction budget and solver failure" begin
    r = solve!(NMPCController(CEV, CEQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact, schedule_corrections=8,
        correction_time_budget=1e-6)), [0.0, 0.0, 0.0, 0.5])
    @test r.status === :segment_assignment_invalid && r.nlp_solves == 1 && occursin("time budget", r.message)
    rf = solve!(NMPCController(CEV, CEQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact, schedule_corrections=8,
        max_iter=1)), [0.0, 0.0, 0.0, 0.5])
    @test rf.status === :solver_failure && rf.nlp_solves == 1 && !rf.accepted && !rf.exact.checked
end

@testset "acceleration from rest on a boundary; open-track end" begin
    r = solve!(NMPCController(CEV, CEQ, CECFG), [CEQ.s_start[2], 0.0, 0.0, 0.0])
    @test r.accepted && r.X[4, end] > 0.5 && r.schedule.min_progress_increment >= 0
    @test r.exact.max_position_error <= 1e-6
    # Near the end of an open track the nominal horizon leaves the track: no exception, no solve.
    re = solve!(NMPCController(CEV, CETRANS, CECFG), [track_length(CETRANS) - 0.3, 0.0, 0.0, 2.0])
    @test re.status === :scheduling_failure && re.nlp_solves == 0
end

@testset "between-knot physical and corridor checks" begin
    # Finer reference sampling can only find equal or smaller clearances (nested sample times).
    c = NMPCController(CEV, CEQ, CECFG)
    r = solve!(c, [3.0, 0.1, 0.0, 2.5])
    @test r.accepted
    r5 = reference_check(c, [3.0, 0.1, 0.0, 2.5], r.X, r.U)
    c50 = NMPCController(CEV, CEQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact, reference_substeps=50))
    r50 = reference_check(c50, [3.0, 0.1, 0.0, 2.5], r.X, r.U)
    @test r50.min_boundary_distance <= r5.min_boundary_distance + 1e-9
    @test r50.min_corridor_distance ≈ r50.min_boundary_distance - c.config.track_margin atol = 1e-12
    # Strict corridor: accepted plans never leave the corridor in the re-simulation, and
    # rejections are reported as corridor violations (not physical ones).
    cfg = (; prediction_model=:cartesian_endpoint_exact, schedule_corrections=8, correction_time_budget=100.0, horizon=10)
    loose = simulate_closed_loop(NMPCController(CEV, CEQ, NMPCConfig(; cfg...)), CEV, [0.0, 0.0, 0.0, 0.5], 60)
    strict = simulate_closed_loop(NMPCController(CEV, CEQ, NMPCConfig(; cfg..., strict_corridor=true)), CEV, [0.0, 0.0, 0.0, 0.5], 60)
    tol = 1e-6
    @test any(r -> r.accepted && r.reference.min_corridor_distance < -tol, loose.controller)
    @test all(r -> r.reference.min_corridor_distance >= -tol, filter(r -> r.accepted, strict.controller))
    @test all(r -> r.reference.passed, filter(r -> r.status === :corridor_violation, strict.controller))
    @test NMPCConfig().strict_corridor == false
end

@testset "single-segment track and defaults unchanged" begin
    # Closed circle: several laps without segment changes (the frame is valid on every lap).
    cl = simulate_closed_loop(NMPCController(CEV, circular_track(1.0; width=0.7), CECFG), CEV, [0.0, 0.0, 0.0, 0.5], 60)
    @test succeeded(cl) && cl.states[end][1] > 2π && all(r -> r.schedule.configuration_changes == 0, cl.controller)
    @test maximum(r.exact.max_position_error for r in cl.controller) <= 1e-6
    # Defaults: Frenet prediction, no exact-map check, unchanged statuses.
    @test NMPCConfig().prediction_model === :frenet
    cd = NMPCController(CEV, CEQ, NMPCConfig())
    @test prediction_model(cd) === :frenet && isempty(cd.cframe) && curvature_model(cd) === :stagewise_average
    rd = solve!(cd, [0.0, 0.0, 0.0, 0.5])
    @test !rd.exact.checked && rd.exact.status === :not_run
    @test_throws ArgumentError NMPCConfig(prediction_model=:cartesian) |> c -> NMPCController(CEV, CEQ, c)
    @test_throws ArgumentError NMPCController(CEV, CEQ, NMPCConfig(prediction_model=:cartesian_endpoint_exact,
        curvature_model=:segment_pair_exact))
end
