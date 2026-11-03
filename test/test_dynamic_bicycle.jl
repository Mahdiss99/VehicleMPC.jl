# Six-state dynamic bicycle model (final milestone).

const DBP = showcase_dynamic_parameters()
const DBV = DynamicBicycle(DBP)

@testset "parameters and validation" begin
    @test DBP.lf + DBP.lr ≈ 0.25 && DBP.m == 1.98 && DBP.Iz == 0.024 && DBP.Cf == 40.0
    lp = legacy_dynamic_parameters()
    @test lp.Cf == 2 * 1.25 && lp.Cr == 2 * 1.25 && lp.m == 1.98 && lp.Iz == 0.024 && lp.lf == lp.lr == 0.125
    base = (m=1.98, Iz=0.024, lf=0.125, lr=0.125, Cf=40.0, Cr=40.0, delta_max=0.4, a_min=-5.0, a_max=5.0, vx_min=0.8, vx_max=5.0, alpha_max=0.12)
    @test DynamicBicycleParameters(; base...) isa DynamicBicycleParameters
    for bad in ((m=-1.0,), (Cf=0.0,), (vx_min=0.0,), (vx_min=6.0,), (delta_max=2.0,), (a_min=1.0,), (alpha_max=0.0,), (Iz=NaN,))
        @test_throws ArgumentError DynamicBicycleParameters(; base..., bad...)
    end
end

@testset "equations, units and sign conventions" begin
    # Straight, zero steering: Ẋ = vx, v̇x = a, no lateral or yaw motion.
    f = dynamic_rhs(DBP, (0.0, 0.0, 0.0, 2.0, 0.0, 0.0), (0.0, 1.0))
    @test collect(f) ≈ [2.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    # Heading π/2: motion along +Y; lateral velocity vy moves along −X.
    f = dynamic_rhs(DBP, (0.0, 0.0, π / 2, 2.0, 0.3, 0.0), (0.0, 0.0))
    @test isapprox(f[1], -0.3; atol=1e-12) && f[2] ≈ 2.0
    # Positive (left) steering gives positive front slip, positive lateral force, positive
    # lateral acceleration and positive yaw acceleration.
    z, u = (0.0, 0.0, 0.0, 2.0, 0.0, 0.0), (0.1, 0.0)
    af, ar = slip_angles(DBP, z, u)
    @test af ≈ 0.1 && ar == 0
    Fyf, Fyr = tyre_forces(DBP, z, u)
    @test Fyf ≈ DBP.Cf * 0.1 && Fyr == 0
    f = dynamic_rhs(DBP, z, u)
    @test f[5] ≈ Fyf * cos(0.1) / DBP.m && f[6] ≈ DBP.lf * Fyf * cos(0.1) / DBP.Iz
    @test f[4] ≈ -Fyf * sin(0.1) / DBP.m                       # front lateral force projected on the body x-axis
    # Slip angles with body velocities and yaw rate (exact atan form).
    z = (0.0, 0.0, 0.0, 2.0, 0.1, 0.4)
    af, ar = slip_angles(DBP, z, (0.05, 0.0))
    @test af ≈ 0.05 - atan((0.1 + DBP.lf * 0.4) / 2.0) && ar ≈ -atan((0.1 - DBP.lr * 0.4) / 2.0)
    # Lateral acceleration identity: a_y = v̇y + r vx.
    f = dynamic_rhs(DBP, z, (0.05, 0.3))
    @test dynamic_lateral_acceleration(DBP, z, (0.05, 0.3)) ≈ f[5] + z[6] * z[4]
    # Units: doubling mass halves the force-induced accelerations.
    p2 = DynamicBicycleParameters(; m=2DBP.m, Iz=DBP.Iz, lf=DBP.lf, lr=DBP.lr, Cf=DBP.Cf, Cr=DBP.Cr, delta_max=DBP.delta_max, a_min=DBP.a_min,
        a_max=DBP.a_max, vx_min=DBP.vx_min, vx_max=DBP.vx_max, alpha_max=DBP.alpha_max)
    zz, uu = (0.0, 0.0, 0.0, 2.0, 0.0, 0.0), (0.1, 0.0)
    @test dynamic_rhs(p2, zz, uu)[5] ≈ dynamic_rhs(DBP, zz, uu)[5] / 2
end

@testset "zero steering, steady turn, low-speed domain" begin
    # Zero steering: straight line, constant acceleration.
    traj = integrate_dynamic(DBV, [0.0, 0.0, 0.3, 1.5, 0.0, 0.0], [0.0, 0.5], 1.0)
    zf = traj[end]
    @test isapprox(zf[1], (1.5 + 0.25) * cos(0.3); atol=1e-9) && isapprox(zf[2], (1.5 + 0.25) * sin(0.3); atol=1e-9)
    @test zf[3] ≈ 0.3 && zf[4] ≈ 2.0 && abs(zf[5]) < 1e-12 && abs(zf[6]) < 1e-12
    # Steady turn: constant δ = L/R at moderate speed converges to r ≈ v/R (neutral steer).
    R, v = 2.0, 1.5
    ss = steady_state_cornering(DBP, R, v)
    @test isapprox(ss.understeer_gradient, 0; atol=1e-12) && ss.delta ≈ 0.25 / R
    z = SVector{6,Float64}(0, 0, 0, v, 0, 0)
    for _ in 1:30
        z = integrate_dynamic(DBV, z, [ss.delta, 0.0], 0.1)[end]
    end
    vtot = hypot(z[4], z[5])
    @test isapprox(z[6], vtot / R; rtol=0.03)                      # yaw rate of the steady circle
    @test isapprox(dynamic_lateral_acceleration(DBP, z, (ss.delta, 0.0)), vtot^2 / R; rtol=0.05)
    @test maximum(abs, slip_angles(DBP, z, (ss.delta, 0.0))) < DBP.alpha_max
    # Feasibility of the bends: the legacy tyres would need ≈1.6 rad of slip at 4 m/s²;
    # the showcase parameters stay in the linear range.
    lp = legacy_dynamic_parameters()
    @test steady_state_cornering(lp, 1.0, 2.0).alpha_f > 1.5
    @test max_cornering_speed(lp, 1.0) < 0.6 && max_cornering_speed(DBP, 1.0; ay_max=4.0) ≈ 2.0
    # Low-speed domain: undefined at vx ≤ 0, stiff as vx → 0.
    @test_throws DomainError check_dynamic_domain(DBV, [0.0, 0, 0, 0.0, 0, 0], [0.0, 0.0])
    @test_throws DomainError check_dynamic_domain(DBV, [0.0, 0, 0, -1.0, 0, 0], [0.0, 0.0])
    @test_throws DomainError check_dynamic_domain(DBV, [0.0, 0, 0, NaN, 0, 0], [0.0, 0.0])
    @test_throws DomainError integrate_dynamic(DBV, [0.0, 0, 0, 0.2, 0, 0], [0.0, -5.0], 0.1)     # brakes through vx = 0
    lat, yaw = lateral_mode_rates(DBP, 1.0)
    @test all(isapprox.(lateral_mode_rates(DBP, 0.5), (2lat, 2yaw)))
    h = 0.1 / 4
    @test maximum(lateral_mode_rates(DBP, DBP.vx_min)) * h < 2.0                      # inside RK4's stability region
    # At the operating-domain boundary the MPC's RK4 (h = 0.025) stays within the acceptance
    # test's plan/plant consistency tolerance (5e-3) of the fine integration.
    z0 = [0.0, 0.0, 0.0, DBP.vx_min, 0.05, 0.3]
    zc = dynamic_step(DBV, z0, [0.15, 0.0], 0.1; substeps=4)
    zr = integrate_dynamic(DBV, z0, [0.15, 0.0], 0.1; h=1e-4)[end]
    @test maximum(abs.(zc - zr)) < 2e-3
    @test isempty(dynamic_operating_violations(DBV, [0.0, 0, 0, 2.0, 0, 0], [0.05, 1.0]))
    @test !isempty(dynamic_operating_violations(DBV, [0.0, 0, 0, 0.5, 0, 0], [0.0, 0.0]))          # below vx_min
    @test !isempty(dynamic_operating_violations(DBV, [0.0, 0, 0, 2.0, 0, 0], [0.3, 0.0]))          # slip beyond alpha_max
end

@testset "RK4 convergence and automatic differentiation" begin
    z0 = [0.3, -0.2, 0.4, 2.0, 0.1, 0.6]
    u = [0.12, 1.0]
    ref = integrate_dynamic(DBV, z0, u, 0.1; h=1e-5)[end]
    e(n) = maximum(abs.(dynamic_step(DBV, z0, u, 0.1; substeps=n) - ref))
    @test e(4) < 1e-3
    @test 12 < e(8) / e(16) < 20                                    # fourth order (asymptotic regime)
    @test 12 < e(16) / e(32) < 20
    # ForwardDiff Jacobians of the right-hand side agree with central finite differences.
    f(w) = collect(dynamic_rhs(DBP, w[1:6], w[7:8]))
    w = vcat(z0, u)
    J = VehicleMPC.ForwardDiff.jacobian(f, w)
    h = 1e-6
    Jfd = hcat([(f(w + h * ei) - f(w - h * ei)) / 2h for ei in eachcol(Matrix(1.0I, 8, 8))]...)
    @test isapprox(J, Jfd; atol=1e-6)
    # Jacobian of the discrete map (as used by Ipopt through JuMP's AD) vs finite differences.
    g(w) = collect(dynamic_step(DBV, w[1:6], w[7:8], 0.1))
    Jg = VehicleMPC.ForwardDiff.jacobian(g, w)
    Jgfd = hcat([(g(w + h * ei) - g(w - h * ei)) / 2h for ei in eachcol(Matrix(1.0I, 8, 8))]...)
    @test isapprox(Jg, Jgfd; atol=1e-5)
end

@testset "reference point and Frenet/Cartesian consistency" begin
    Q = quarter_track()
    Xr, Yr, psi = frenet_to_cartesian(Q, 1.0, 0.1, 0.05)
    Xg, Yg, pg = cg_pose_from_rear_axle(DBP, Xr, Yr, psi)
    @test hypot(Xg - Xr, Yg - Yr) ≈ DBP.lr && pg == psi
    @test collect(rear_axle_pose(DBP, [Xg, Yg, psi, 1.0, 0.0, 0.0])) ≈ [Xr, Yr, psi]
    # The CoM's own track coordinates: projection consistent with frenet_pose_local.
    f = chain_frenet(Q, reshape([Xg, Yg, pg], 3, 1), 1.1)
    Xc, Yc, pc = frenet_to_cartesian(Q, f[1, 1], f[2, 1], f[3, 1])
    @test hypot(Xc - Xg, Yc - Yg) < 1e-9 && abs(pc - pg) < 1e-9
end
