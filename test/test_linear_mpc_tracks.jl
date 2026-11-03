# Linearized MPC on multi-curvature tracks with a stationary obstacle (Milestone 4BC).

const LTV = KinematicBicycle(reference_test_vehicle())
const LTP = LTV.params
const LTQ = quarter_track()
const LTS = straight_track(80.0; width=0.7)
const LTOB = (ego_radius=0.05, obstacle_buffer=0.03)

lt_controller(track; obstacle=nothing, kw...) = LinearMPCController(LTV, track, LinearMPCConfig(; horizon=10, sqp_time_budget=Inf, kw...); obstacle)

"True track coordinates of `(X, Y, psi)` near progress `hint`."
lt_true(tr, X, Y, psi, hint) = (r = track_reference(tr, X, Y, psi; s_hint=hint, min_frenet_factor=1e-3); (r.s, r.ey, r.epsi))

@testset "affine track coordinates: exactness, Jacobians, both curvature signs" begin
    for (tr, s, ey, ep) in ((circular_track(2.0; width=0.7), 1.0, 0.2, 0.1), (circular_track(1.0; direction=-1, width=0.7), 2.0, -0.25, -0.2),
        (LTQ, 4.0, 0.15, 0.05), (LTQ, 7.0, -0.2, 0.1), (LTQ, 10.0, 0.1, -0.1), (LTS, 3.0, 0.2, 0.3))
        X, Y, psi = frenet_to_cartesian(tr, s, ey, ep)
        r = track_reference(tr, X, Y, psi; s_hint=s + 0.05)
        # Exact at the nominal point.
        @test r.s ≈ s atol = 1e-9
        @test r.ey ≈ ey atol = 1e-9
        @test r.epsi ≈ ep atol = 1e-9
        @test r.factor ≈ 1 - r.kappa * ey && r.kappa == curvature(tr, s)
        # The affine model's gradients (∂s/∂p = t/(1-κ ey), ∂ey/∂p = n, ∂epsi/∂p = -κ t/(1-κ ey),
        # ∂epsi/∂psi = 1) match central finite differences of the true projection (away from
        # segment boundaries).
        h = 1e-6
        fd(dX, dY, dpsi) = (collect(lt_true(tr, X + dX, Y + dY, psi + dpsi, s)) - collect(lt_true(tr, X - dX, Y - dY, psi - dpsi, s))) / 2h
        gx, gy, gp = fd(h, 0, 0), fd(0, h, 0), fd(0, 0, h)
        a = (r.t[1] / r.factor, r.t[2] / r.factor)
        @test isapprox(gx, [a[1], r.n[1], -r.kappa * a[1]]; atol=1e-6)
        @test isapprox(gy, [a[2], r.n[2], -r.kappa * a[2]]; atol=1e-6)
        @test isapprox(gp, [0, 0, 1]; atol=1e-6)
        # First-order accuracy: the error of the affine coordinates is quadratic in the offset.
        aff(dX, dY) = (r.s + a[1] * dX + a[2] * dY, r.ey + r.n[1] * dX + r.n[2] * dY, r.epsi - r.kappa * (a[1] * dX + a[2] * dY))
        err(d) = maximum(abs.(collect(aff(d, -d)) - collect(lt_true(tr, X + d, Y - d, psi, s))))
        @test err(0.02) < 2e-3 && (err(0.02) < 1e-12 || 3 < err(0.02) / err(0.01) < 5)
    end
end

@testset "track references: lap unwrapping, invalid and ambiguous projections" begin
    L = track_length(LTQ)
    X, Y, psi = frenet_to_cartesian(LTQ, 1.0, 0.1, 0.0)
    @test track_reference(LTQ, X, Y, psi; s_hint=1.0).s ≈ 1.0
    @test track_reference(LTQ, X, Y, psi; s_hint=L + 1.1).s ≈ L + 1.0               # one lap later
    @test track_reference(LTQ, X, Y, psi; s_hint=2L + 0.8).s ≈ 2L + 1.0             # two laps later
    @test track_reference(LTQ, X, Y, psi + 2π; s_hint=L + 1.0).epsi ≈ 0 atol = 1e-12  # unwrapped heading
    # Near the lap boundary: the projection follows the hint across it.
    Xb, Yb, pb = frenet_to_cartesian(LTQ, L - 0.05, 0.0, 0.0)
    @test track_reference(LTQ, Xb, Yb, pb; s_hint=L - 0.1).s ≈ L - 0.05
    Xa, Ya, pa = frenet_to_cartesian(LTQ, 0.05, 0.0, 0.0)
    @test track_reference(LTQ, Xa, Ya, pa; s_hint=L - 0.1).s ≈ L + 0.05
    # No candidate within max_jump of the hint: an explicit error, never a placeholder.
    @test_throws TrackReferenceError track_reference(LTQ, X, Y, psi; s_hint=5.0)
    @test_throws TrackReferenceError track_reference(LTQ, 100.0, 100.0, 0.0; s_hint=1.0)
    # Too close to a centre of curvature (1 - κ ey below the minimum).
    Xc, Yc, pc = frenet_to_cartesian(LTQ, 3.5, 0.6, 0.0)                              # R = 1 m arc, ey = 0.6
    @test_throws TrackReferenceError track_reference(LTQ, Xc, Yc, pc; s_hint=3.5, min_frenet_factor=0.5)
    @test track_reference(LTQ, Xc, Yc, pc; s_hint=3.5, min_frenet_factor=0.2).factor ≈ 0.4
    # Ambiguous: a U-turn track whose two straights are both within reach of the hint.
    U = Track([TrackSegment(1.0, 0.0), TrackSegment(π * 0.3, 1 / 0.3), TrackSegment(1.0, 0.0)]; width=0.7)
    @test_throws TrackReferenceError track_reference(U, 0.5, 0.3, 0.0; s_hint=1.5, max_jump=1.5)
    @test track_reference(U, 0.5, 0.3, 0.0; s_hint=0.5, max_jump=0.5).s ≈ 0.5
    # Curvature convention at a segment boundary: downstream (half-open segments).
    Xs, Ys, ps = frenet_to_cartesian(LTQ, 2.0, 0.0, 0.0)
    @test track_reference(LTQ, Xs, Ys, ps; s_hint=2.0).kappa == 1.0
end

@testset "interior-time partial-map linearisation" begin
    L = LTP.wheelbase
    for (z, u, tau) in (([0.3, -0.2, 0.4, 2.0], [0.2, 1.0], 0.025), ([1.0, 2.0, -1.2, 4.5], [-0.3, -3.0], 0.075), ([0.0, 0.0, 0.0, 0.0], [0.1, 2.0], 0.05))
        pbar, J = interior_jacobian(L, z, u, tau)
        @test pbar ≈ collect(interior_point(L, z, z[4], u[1], u[2], tau))
        f(w) = collect(interior_point(L, w[1:3], w[4], w[5], w[6], tau))
        w, h = vcat(z, u), 1e-6
        Jfd = hcat([(f(w + h * e) - f(w - h * e)) / 2h for e in eachcol(Matrix(1.0I, 6, 6))]...)
        @test isapprox(J, Jfd; atol=1e-7)
        @test J[:, 1:2] == [1 0; 0 1]                                                    # a position offset shifts the sample
    end
    # tau = dt reproduces the exact step's position.
    z, u = [0.3, -0.2, 0.4, 2.0], [0.2, 1.0]
    @test interior_jacobian(L, z, u, 0.1)[1] ≈ collect(exact_cartesian_step(LTV, z, u, 0.1))[1:2]
end

@testset "obstacle halfspaces: safety and degeneracy" begin
    ob = CircleObstacle(x=1.0, y=2.0, radius=0.1)
    D = 0.18
    n, deg = obstacle_direction((1.5, 2.3), ob, (0.0, 1.0), :left; min_distance=1e-3)
    @test !deg && hypot(n...) ≈ 1 && isapprox(n[1] * 0.3 - n[2] * 0.5, 0; atol=1e-12)        # radial
    # Any point of the halfspace n·(p - o) ≥ D lies outside the disk of radius D.
    for θ in range(0, 2π; length=13), d in (0.0, 0.05, 0.4), t in (-1.0, 0.0, 2.0)
        nn = (cos(θ), sin(θ))
        p = (ob.x + (D + d) * nn[1] - t * nn[2], ob.y + (D + d) * nn[2] + t * nn[1])
        @test hypot(p[1] - ob.x, p[2] - ob.y) >= D - 1e-12
    end
    # Degenerate nominal point (at the centre): the passing-side track normal, never NaN.
    for (side, sg) in ((:left, 1), (:right, -1), (:none, 1))
        n, deg = obstacle_direction((ob.x, ob.y), ob, (0.0, 1.0), side; min_distance=1e-3)
        @test deg && n == (0.0, sg * 1.0) && all(isfinite, n)
    end
    n, deg = obstacle_direction((ob.x + 1e-4, ob.y), ob, (-1.0, 0.0), :right; min_distance=1e-3)
    @test deg && n == (1.0, -0.0)
    # Controller rows: one per knot and per interior sample, with the documented right-hand side.
    c = lt_controller(LTS; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.10), LTOB..., obstacle_samples=3)
    @test size(c.obs) == (4, 10) && c.obs_alphas == [0.25, 0.5, 0.75] && c.track_model === :local
    @test size(lt_controller(LTS; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.10), LTOB..., obstacle_samples=0).obs) == (1, 10)
    x = [4.0, 0.0, 0.0, 3.0]
    r = solve!(c, x)
    @test r.accepted && r.obstacle.present
    z0 = collect(frenet_state_to_cartesian(LTS, x))
    Zb, Ub = r.Zbar, r.Ubar
    Dq = 0.18 + c.config.mismatch_tol[1] + c.config.local_backoff.obstacle
    for k in (1, 5, 10)
        n, _ = obstacle_direction(Zb[1:2, k+1], c.obstacle, (0.0, 1.0), :none; min_distance=1e-3)
        @test [normalized_coefficient(c.obs[1, k], c.Z[i, k+1]) for i in 1:2] ≈ collect(n)
        @test normalized_rhs(c.obs[1, k]) ≈ Dq + n[1] * 8.0 + n[2] * 0.0
    end
    # The QP prediction satisfies every halfspace, so its knots keep the separation.
    @test all(hypot(r.Z[1, k] - 8.0, r.Z[2, k]) >= Dq - 1e-4 for k in 2:11)
    @test_throws ArgumentError set_obstacle!(lt_controller(LTS), CircleObstacle(x=8.0, y=0.0, radius=0.1))
    @test_throws ArgumentError LinearMPCController(LTV, LTS, LinearMPCConfig(track_model=:straight); obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.1))
end

@testset "local QP: construction, convexity, persistent updates across curvature changes" begin
    c = lt_controller(LTQ)
    @test c.track_model === :local && length(c.S) == 10 && length(c.con_ep) == 10
    # Convex objective: assemble the Hessian of the quadratic objective.
    f = objective_function(c.model)
    vars = all_variables(c.model)
    idx = Dict(v => i for (i, v) in enumerate(vars))
    H = zeros(length(vars), length(vars))
    for (p, coef) in f.terms
        i, j = idx[p.a], idx[p.b]
        i == j ? (H[i, i] += 2coef) : (H[i, j] += coef; H[j, i] += coef)
    end
    @test eigmin(Symmetric(H)) >= -1e-10
    @test all(F <: Union{AffExpr,VariableRef} for (F, _) in list_of_constraint_types(c.model))
    # Cycles across the straight → R +1 → R −1 → R 2 sequence: curvature signs change and the
    # reference headings rotate, all with the same OSQP model (persistent updates).
    m = c.model
    x, up = [0.0, 0.0, 0.0, 0.5], (0.0, 0.0)
    kappas = Float64[]
    for k in 1:45
        r = solve!(c, x; previous_input=up)
        @test r.accepted
        r.accepted || break
        append!(kappas, r.references[4, :])
        # The installed rows are those of the last linearisation.
        refs = trajectory_references(c, r.Zbar, x[1])
        for i in (1, 6, 10)
            ref = refs[i+1]
            @test normalized_coefficient(c.con_ep[i], c.S[i]) == ref.kappa
            @test [normalized_coefficient(c.con_ey[i], c.Z[j, i+1]) for j in 1:2] ≈ -collect(ref.n)
            @test [normalized_coefficient(c.con_s[i], c.Z[j, i+1]) for j in 1:2] ≈ -collect(ref.t) ./ ref.factor
            @test normalized_rhs(c.con_ey[i]) ≈ ref.ey - ref.n[1] * r.Zbar[1, i+1] - ref.n[2] * r.Zbar[2, i+1]
        end
        sim = simulate_on_track(LTV, LTQ, x, [r.u], 0.1; max_step=0.01)
        x, up = sim.states[end], r.u
    end
    @test c.model === m
    @test any(==(0.0), kappas) && any(==(1.0), kappas) && any(==(-1.0), kappas) && any(==(0.5), kappas)
    # Rebuild mode: a new model per QP, the same kind of result.
    cr = lt_controller(LTQ; update_mode=:rebuild)
    rr = solve!(cr, [0.0, 0.0, 0.0, 0.5])
    @test rr.accepted && cr.model !== m
    # The local model on a straight track reproduces the straight model.
    st = straight_track(400.0; width=1.0)
    a = solve!(LinearMPCController(LTV, st, LinearMPCConfig(sqp_time_budget=Inf)), [0.0, 0.2, 0.1, 1.0])
    b = solve!(LinearMPCController(LTV, st, LinearMPCConfig(sqp_time_budget=Inf, track_model=:local, step_policy=:rollout,
        local_backoff=(corridor=0.0, heading=0.0, ay=0.0, obstacle=0.0, speed=0.0), local_eps=1e-6, local_osqp_max_iter=20000)), [0.0, 0.2, 0.1, 1.0])
    @test a.track_model === :straight && b.track_model === :local
    @test a.accepted && b.accepted && isapprox(a.u, b.u; atol=1e-5) && isapprox(a.nonlinear_objective, b.nonlinear_objective; atol=1e-6)
    @test maximum(abs, collect(b.track_error)) < 1e-12                                 # exact on a straight track
end

@testset "successive relinearisation, trust region and budget failures" begin
    c = lt_controller(LTQ)
    r = solve!(c, [0.0, 0.0, 0.0, 0.5])                                                 # cold start: a far nominal
    @test r.accepted && r.qp_solves > 1
    @test r.mismatch_history[end][1] < r.mismatch_history[1][1]
    @test all(diff(r.trust_scale) .<= 0) && r.trust_scale[end] < 1
    @test all(isfinite, r.track_error) && r.track_error[2] < 0.01
    @test r.timing.reference > 0 && r.timing.total >= r.timing.solve
    # Bounded: a single QP is not enough from the cold start.
    r1 = solve!(lt_controller(LTQ; max_qp_solves=1), [0.0, 0.0, 0.0, 0.5])
    @test r1.qp_solves == 1 && !r1.accepted && r1.u === nothing && r1.status === :not_converged
    # A zero time budget allows exactly one QP.
    r0 = solve!(LinearMPCController(LTV, LTQ, LinearMPCConfig(sqp_time_budget=0.0)), [0.0, 0.0, 0.0, 0.5])
    @test r0.qp_solves == 1 && occursin("time budget", r0.message)
    # Hard constraints (no slack) can make a QP infeasible; this is reported, not hidden.
    @test LinearMPCConfig().slack_penalty === :quadratic
    # The trust-region memory: a following cycle may start below scale 1, a rejected cycle resets it.
    @test c.trust_memory <= 1
    reset_warm_start!(c)
    @test c.trust_memory == 1 && c.last_U === nothing
    # Initial collision and blocked passage (Milestone 3A geometry).
    ob = CircleObstacle(x=8.0, y=0.0, radius=0.10)
    ci = lt_controller(LTS; obstacle=ob, LTOB...)
    ri = solve!(ci, [7.9, 0.0, 0.0, 1.0])
    @test ri.status === :initial_collision && !ri.accepted && ri.obstacle.failure_cause === :initial_collision
    cb = lt_controller(LTS; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.30), LTOB...)
    cl = simulate_closed_loop(cb, LTV, [0.0, 0.0, 0.0, 0.5], 60)
    @test cl.status === :controller_failure && isempty(cl.violations)
    @test cl.controller[end].obstacle.failure_cause === :infeasible_geometry
    @test minimum(hypot(p[1] - 8.0, p[2]) for p in (frenet_to_cartesian(LTS, x[1], x[2], x[3]) for x in cl.fine_states)) > 0.35
end

@testset "nonlinear validation on the actual track, with the obstacle" begin
    lim = (; dt=0.1, track_margin=0.05, epsi_max=deg2rad(60), ay_max=4.0)
    z0 = collect(frenet_state_to_cartesian(LTS, [4.0, 0.0, 0.0, 2.0]))
    ob = CircleObstacle(x=5.0, y=0.0, radius=0.10)
    straightU = zeros(2, 10)
    v = validate_plan_on_track(LTV, LTS, lim, z0, straightU; s0=4.0, obstacle=ob, D=0.18, Dc=0.15, alphas=[0.25, 0.5, 0.75])
    @test any(occursin("obstacle: contact", s) for s in v.violations) && any(occursin("constrained-point", s) for s in v.violations)
    @test v.obstacle.collisions > 0 && v.obstacle.min_distance < 0.01 && v.excess > 0
    @test v.frenet[1, :] ≈ 4.0 .+ 0.2 .* (0:10) && maximum(abs, v.frenet[2, :]) < 1e-12
    # Passing between constrained points: the dense evaluation finds the contact that the knots miss.
    obm = CircleObstacle(x=4.1, y=0.0, radius=0.10)                                      # midway between knots 1 and 2
    vk = validate_plan_on_track(LTV, LTS, lim, z0, straightU; s0=4.0, obstacle=obm, D=0.18, Dc=0.15, alphas=Float64[])
    @test vk.obstacle.collisions > 0 && any(occursin("obstacle: contact", s) for s in vk.violations)
    # Separation acceptance: a pass inside the buffer but without contact.
    obb = CircleObstacle(x=5.0, y=0.16, radius=0.10)
    vb = validate_plan_on_track(LTV, LTS, lim, z0, straightU; s0=4.0, obstacle=obb, D=0.18, Dc=0.15, check_samples=50)
    @test vb.obstacle.collisions == 0 && vb.obstacle.separation_violations > 0
    @test !any(occursin("dense evaluation", s) for s in vb.violations)
    vs = validate_plan_on_track(LTV, LTS, lim, z0, straightU; s0=4.0, obstacle=obb, D=0.18, Dc=0.15, acceptance=:separation)
    @test any(occursin("required separation", s) for s in vs.violations)
    # Off the track: a plan that leaves the track has no valid projection, which is a violation.
    zq = collect(frenet_state_to_cartesian(LTQ, [1.5, 0.0, 0.0, 3.0]))
    vq = validate_plan_on_track(LTV, LTQ, lim, zq, zeros(2, 10); s0=1.5)              # straight on into the R = 1 m bend
    @test !isempty(vq.violations) && vq.min_track_clearance < 0
    # Objective: the original NMPC objective on the true coordinates.
    @test plan_objective_on_track(NMPCConfig(), LTV, 0.3, v.frenet, straightU, (0.0, 0.0); horizon=10, dt=0.1) ≈ -10 * 2.0 / 5.0
end

@testset "passing-side guesses for the linear controller" begin
    ob = CircleObstacle(x=8.0, y=0.0, radius=0.10)
    c = lt_controller(LTS; obstacle=ob, LTOB...)
    x = [4.0, 0.0, 0.0, 3.0]
    @test passage_sides(c, x) == (true, true) && obstacle_relevant(c, x)
    for (side, sg) in ((:left, 1), (:right, -1))
        g = side_guess(c, x, side; a=zeros(10))
        @test g.converged && g.rollout_ok
        k = argmin(abs.(g.Xg[1, :] .- 8.0))
        @test sg * g.Xg[2, k] > 0.1
        # Re-rolled with the exact Cartesian map, the inputs reproduce the guess (dynamically
        # consistent) and pass the obstacle on the same side.
        Z = exact_rollout(LTV, collect(frenet_state_to_cartesian(LTS, x)), g.Ug, 0.1)
        @test maximum(abs.(Z[1, :] .- g.Xg[1, :])) < 1e-6 && maximum(abs.(Z[2, :] .- g.Xg[2, :])) < 1e-6
        @test sg * Z[2, k] > 0.1
    end
end

@testset "closed loop: quarter track without obstacle" begin
    c = lt_controller(LTQ)
    cl = simulate_closed_loop(c, LTV, [0.0, 0.0, 0.0, 0.5], 130)
    @test succeeded(cl) && isempty(cl.violations)
    @test cl.states[end][1] > track_length(LTQ)                                           # at least one lap
    @test maximum(abs(x[2]) for x in cl.fine_states) <= 0.3 + 1e-3                          # within the planning corridor
    @test all(r.accepted && r.qp_solves <= 5 && isempty(r.violations) for r in cl.controller)
    @test maximum(abs(lateral_acceleration(LTV, x[4], u[1])) for (x, u) in zip(cl.states, cl.inputs)) <= 4.0 + 1e-3
    @test all(r.mismatch[1] <= 1e-3 for r in cl.controller)
end

@testset "closed loop: stationary obstacle (straight and quarter track)" begin
    for (tr, ob, n) in ((LTS, CircleObstacle(x=8.0, y=0.0, radius=0.10), 60), (LTQ, obstacle_on_track(LTQ, 3.5, 0.0; radius=0.10), 70))
        c = lt_controller(tr; obstacle=ob, LTOB...)
        cl = simulate_closed_loop(c, LTV, [0.0, 0.0, 0.0, 0.5], n)
        @test succeeded(cl) && isempty(cl.violations)
        d = minimum(hypot(p[1] - ob.x, p[2] - ob.y) for p in (frenet_to_cartesian(tr, x[1], x[2], x[3]) for x in cl.fine_states))
        @test d > 0.15                                                                     # no contact
        @test d > 0.18 - 1e-3                                                              # the buffer is kept (to the tolerance)
        enc = [r for r in cl.controller if any(a -> a[1] in (:left, :right), r.obstacle.attempts)]
        @test !isempty(enc) && enc[1].obstacle.side in (:left, :right)
        @test all(r.obstacle.collisions == 0 for r in cl.controller)
        @test cl.states[end][1] > ob.x - 0.5 || tr !== LTS                                 # progress past the obstacle
        acc = [r for r in cl.controller if r.accepted && r.obstacle.relevant]
        @test all(r.obstacle.constrained_clearance >= -1e-4 for r in acc)
    end
end

@testset "reliability: obstacle updates reach the QP, failures leave no state" begin
    # set_obstacle! changes the obstacle used by the next QP's halfspace rows (no stale data).
    c = lt_controller(LTS; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.10), LTOB...)
    x = [4.0, 0.0, 0.0, 3.0]
    @test solve!(c, x).accepted
    ob2 = CircleObstacle(x=9.0, y=0.1, radius=0.05)
    set_obstacle!(c, ob2)
    @test c.obstacle === ob2 && c.obstacle_side === :none && isnan(c.obstacle_encounter)
    r = solve!(c, x)
    @test r.accepted && r.obstacle.required ≈ 0.05 + 0.05 + 0.03
    Dq = 0.13 + c.config.mismatch_tol[1] + c.config.local_backoff.obstacle
    for k in (1, 6, 10)
        n, _ = obstacle_direction(r.Zbar[1:2, k+1], ob2, (0.0, 1.0), :none; min_distance=1e-3)
        @test normalized_rhs(c.obs[1, k]) ≈ Dq + n[1] * ob2.x + n[2] * ob2.y
    end
    # A rejected cycle returns no control and resets the warm start and the trust-region memory.
    cb = lt_controller(LTS; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.30), LTOB...)
    cl = simulate_closed_loop(cb, LTV, [0.0, 0.0, 0.0, 0.5], 60)
    @test cl.status === :controller_failure
    rf = cl.controller[end]
    @test !rf.accepted && rf.u === nothing && rf.status !== :success
    @test cb.last_U === nothing && cb.last_Z === nothing && cb.trust_memory == 1.0
    @test all(r.accepted == (r.u !== nothing) for r in cl.controller)
    # The next solve after a failure starts cold (no stale warm start).
    rn = solve!(cb, [0.0, 0.0, 0.0, 0.5])
    @test rn.trust_scale[1] == 1.0
    # Measured states outside the model domain are rejected without a solve.
    ri = solve!(lt_controller(LTQ), [0.0, 0.0, 0.0, -1.0])
    @test ri.status === :invalid_measurement && ri.u === nothing && ri.qp_solves == 0
    ro = solve!(lt_controller(LTS; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.10), LTOB...), [0.0, 0.0, 0.0, -1.0])
    @test ro.status === :invalid_measurement && ro.obstacle.present && ro.obstacle.failure_cause === :invalid_measurement
end
