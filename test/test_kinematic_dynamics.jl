# Kinematic bicycle dynamics: formulation, turning direction, steady-state cornering and
# domain handling. All discrete-time checks use forward Euler, except the clearly separated
# "Frenet vs Cartesian" testset, which integrates *both* models with RK4.

const P = reference_test_vehicle()
const MODEL = KinematicBicycle(P)
const L = P.wheelbase

function euler_rollout(model, x0, inputs, kappa, dt)
    xs = [SVector{4}(x0)]
    for u in inputs
        push!(xs, discrete_dynamics(model, xs[end], u, kappa, dt))
    end
    return xs
end

@testset "parameters" begin
    @test P.wheelbase == 0.25
    @test P.delta_max < π / 2 - STEERING_SINGULARITY_MARGIN + eps()
    @test_throws ArgumentError VehicleParameters(wheelbase=0.0, delta_max=0.3, a_min=-1, a_max=1, v_max=2)
    @test_throws ArgumentError VehicleParameters(wheelbase=-1.0, delta_max=0.3, a_min=-1, a_max=1, v_max=2)
    @test_throws ArgumentError VehicleParameters(wheelbase=1.0, delta_max=π / 2, a_min=-1, a_max=1, v_max=2)
    @test_throws ArgumentError VehicleParameters(wheelbase=NaN, delta_max=0.3, a_min=-1, a_max=1, v_max=2)
    @test_throws ArgumentError VehicleParameters(wheelbase=1.0, delta_max=0.3, a_min=-1, a_max=1, v_min=-1, v_max=2)
    @test_throws ArgumentError KinematicBicycle(P; eps_geometry=0.0)
    @test within_input_limits(P, [P.delta_max, P.a_max])
    @test !within_input_limits(P, [P.delta_max + 1e-6, 0.0])
    @test !within_input_limits(P, [0.0, P.a_min - 1e-6])
end

@testset "continuous dynamics match the documented equations" begin
    for (x, u, kappa) in (([1.0, 0.1, 0.2, 2.0], [0.1, 0.5], 0.4),
        ([0.0, -0.2, -0.3, 1.5], [-0.2, -1.0], -0.8),
        ([5.0, 0.0, 0.0, 0.0], [0.0, 0.0], 0.0))
        s, ey, epsi, v = x
        delta, a = u
        sdot = v * cos(epsi) / (1 - kappa * ey)
        expected = [sdot, v * sin(epsi), v * tan(delta) / L - kappa * sdot, a]
        @test continuous_dynamics(MODEL, x, u, kappa) ≈ expected atol = 1e-15
    end
end

@testset "Euler discretization agrees with the discrete equations" begin
    x, u, kappa, dt = [2.0, 0.15, -0.1, 3.0], [0.2, -0.7], 0.5, 0.05
    s, ey, epsi, v = x
    delta, a = u
    sdot = v * cos(epsi) / (1 - kappa * ey)
    expected = [s + dt * sdot,
        ey + dt * v * sin(epsi),
        epsi + dt * (v * tan(delta) / L - kappa * sdot),
        v + dt * a]
    @test discrete_dynamics(MODEL, x, u, kappa, dt) ≈ expected atol = 1e-15
    @test discrete_dynamics(MODEL, x, u, kappa, dt, ForwardEuler()) == discrete_dynamics(MODEL, x, u, kappa, dt)
    @test_throws ArgumentError discrete_dynamics(MODEL, x, u, kappa, 0.0)
    @test_throws ArgumentError discrete_dynamics(MODEL, x, u, kappa, -0.1)
    @test_throws ArgumentError discrete_dynamics(MODEL, x, u, kappa, Inf)
end

@testset "straight line" begin
    dt, N, v0 = 0.1, 50, 1.5

    # Zero steering and heading error preserve zero lateral error; constant speed ⇒ s = v t.
    xs = euler_rollout(MODEL, [0.0, 0.0, 0.0, v0], fill([0.0, 0.0], N), 0.0, dt)
    @test all(x -> x[2] == 0 && x[3] == 0 && x[4] == v0, xs)
    @test xs[end][1] ≈ v0 * N * dt rtol = 1e-14

    # Constant acceleration: v_k = v0 + a k dt exactly, and Euler progress
    # s_N = v0 N dt + a dt² N(N-1)/2.
    a = 0.8
    xs = euler_rollout(MODEL, [0.0, 0.0, 0.0, v0], fill([0.0, a], N), 0.0, dt)
    @test [x[4] for x in xs] ≈ [v0 + a * k * dt for k in 0:N] rtol = 1e-14
    @test xs[end][1] ≈ v0 * N * dt + a * dt^2 * N * (N - 1) / 2 rtol = 1e-13
    @test all(x -> x[2] == 0, xs)
end

@testset "turning direction" begin
    dt, N, delta = 0.05, 40, 0.2
    left = euler_rollout(MODEL, [0.0, 0.0, 0.0, 1.0], fill([delta, 0.0], N), 0.0, dt)
    right = euler_rollout(MODEL, [0.0, 0.0, 0.0, 1.0], fill([-delta, 0.0], N), 0.0, dt)

    # Positive steering turns left: heading error and lateral offset become positive.
    @test all(x -> x[3] > 0, left[2:end])
    @test all(x -> x[2] > 0, left[3:end])
    # Negative steering is the exact mirror image on a straight reference.
    @test [x[2] for x in right] ≈ -[x[2] for x in left] atol = 1e-15
    @test [x[3] for x in right] ≈ -[x[3] for x in left] atol = 1e-15
    @test [x[1] for x in right] ≈ [x[1] for x in left] atol = 1e-15

    # The same direction convention holds in Cartesian coordinates on a straight track
    # heading along +X: positive steering moves the vehicle to +Y (its left).
    track = straight_track(10.0)
    @test frenet_to_cartesian(track, left[end][1], left[end][2], left[end][3])[2] > 0
    @test frenet_to_cartesian(track, right[end][1], right[end][2], right[end][3])[2] < 0

    # Yaw rate v tan(delta)/L enters epsidot with a positive sign on a curved reference too.
    @test continuous_dynamics(MODEL, [0.0, 0.0, 0.0, 1.0], [0.1, 0.0], 0.5)[3] >
          continuous_dynamics(MODEL, [0.0, 0.0, 0.0, 1.0], [0.0, 0.0], 0.5)[3]
end

@testset "steady-state cornering: delta = atan(kappa L) keeps the vehicle aligned" begin
    dt, N, v = 0.01, 500, 2.0
    for R in (1.0, 2.0, 10.0), dir in (1, -1)
        kappa = dir / R
        delta = atan(kappa * L)
        xs = euler_rollout(MODEL, [0.0, 0.0, 0.0, v], fill([delta, 0.0], N), kappa, dt)
        @test maximum(abs(x[2]) for x in xs) <= 1e-12
        @test maximum(abs(x[3]) for x in xs) <= 1e-12
        @test xs[end][1] ≈ v * N * dt rtol = 1e-12

        # The rear axle stays on the reference circle in Cartesian coordinates.
        track = circular_track(R; direction=dir)
        centre = (0.0, dir * R)
        radii = [hypot((frenet_to_cartesian(track, x[1], x[2])[1:2] .- centre)...) for x in xs]
        @test maximum(abs.(radii .- R)) <= 1e-9
    end
end

# Cartesian rear-axle kinematic bicycle, used as an independent reference.
cartesian_f(z, u, L) = SVector(z[4] * cos(z[3]), z[4] * sin(z[3]), z[4] * tan(u[1]) / L, u[2])
function cartesian_rk4(z, u, L, dt)
    k1 = cartesian_f(z, u, L)
    k2 = cartesian_f(z + dt / 2 * k1, u, L)
    k3 = cartesian_f(z + dt / 2 * k2, u, L)
    k4 = cartesian_f(z + dt * k3, u, L)
    return z + dt / 6 * (k1 + 2k2 + 2k3 + k4)
end

@testset "Frenet dynamics agree with the Cartesian model (both RK4)" begin
    # Single-segment tracks only, so the curvature is genuinely constant over every step.
    dt, N = 1e-3, 2000
    inputs = [SVector(0.15 * sin(k * dt * 2), 0.3) for k in 0:N-1]
    for track in (circular_track(2.0), circular_track(1.5; direction=-1),
        straight_track(20.0; start_pose=(1.0, -2.0, 0.7)))
        x = SVector(0.3, 0.1, 0.05, 1.0)
        X, Y, psi = frenet_to_cartesian(track, x[1], x[2], x[3])
        z = SVector(X, Y, psi, x[4])
        for u in inputs
            x = discrete_dynamics(MODEL, x, u, curvature(track, x[1]), dt, RK4())
            z = cartesian_rk4(z, u, L, dt)
        end
        Xf, Yf, psif = frenet_to_cartesian(track, x[1], x[2], x[3])
        @test hypot(Xf - z[1], Yf - z[2]) <= 1e-9
        @test abs(rem2pi(psif - z[3], RoundNearest)) <= 1e-9
        @test x[4] ≈ z[4] rtol = 1e-12
    end
end

@testset "finite outputs on valid states" begin
    for kappa in (-2.0, -0.5, 0.0, 0.5, 2.0), ey in (-0.3, 0.0, 0.3), epsi in (-1.0, 0.0, 1.0),
        v in (0.0, 0.5, 4.0), delta in (-P.delta_max, 0.0, P.delta_max), a in (P.a_min, P.a_max)

        x, u = [1.0, ey, epsi, v], [delta, a]
        1 - kappa * ey > MODEL.eps_geometry || continue
        @test all(isfinite, continuous_dynamics(MODEL, x, u, kappa))
        @test all(isfinite, discrete_dynamics(MODEL, x, u, kappa, 0.1))
        A, B = discrete_jacobians(MODEL, x, u, kappa, 0.1)
        @test all(isfinite, A) && all(isfinite, B)
    end
end

@testset "invalid configurations are rejected" begin
    good_x, good_u = [0.0, 0.0, 0.0, 1.0], [0.0, 0.0]
    # Frenet singularity: 1 - kappa*ey must exceed eps_geometry.
    @test_throws DomainError continuous_dynamics(MODEL, [0.0, 1.0, 0.0, 1.0], good_u, 1.0)     # = 0
    @test_throws DomainError continuous_dynamics(MODEL, [0.0, 2.0, 0.0, 1.0], good_u, 1.0)     # < 0
    @test_throws DomainError continuous_dynamics(MODEL, [0.0, 0.95, 0.0, 1.0], good_u, 1.0)    # 0.05 < eps
    @test_throws DomainError continuous_dynamics(MODEL, [0.0, -0.95, 0.0, 1.0], good_u, -1.0)
    @test continuous_dynamics(MODEL, [0.0, 0.85, 0.0, 1.0], good_u, 1.0) isa AbstractVector    # 0.15 > eps
    # Non-finite values.
    @test_throws DomainError continuous_dynamics(MODEL, [NaN, 0.0, 0.0, 1.0], good_u, 0.0)
    @test_throws DomainError continuous_dynamics(MODEL, good_x, [Inf, 0.0], 0.0)
    @test_throws DomainError continuous_dynamics(MODEL, good_x, good_u, NaN)
    # Reverse driving is outside the model.
    @test_throws DomainError continuous_dynamics(MODEL, [0.0, 0.0, 0.0, -0.1], good_u, 0.0)
    # Steering at or near the tan singularity.
    @test_throws DomainError continuous_dynamics(MODEL, good_x, [π / 2, 0.0], 0.0)
    @test_throws DomainError continuous_dynamics(MODEL, good_x, [-(π / 2 - STEERING_SINGULARITY_MARGIN / 2), 0.0], 0.0)
    # Wrong dimensions.
    @test_throws DimensionMismatch continuous_dynamics(MODEL, [0.0, 0.0, 1.0], good_u, 0.0)
    @test_throws DimensionMismatch continuous_dynamics(MODEL, good_x, [0.0], 0.0)
    # Jacobians apply the same checks.
    @test_throws DomainError discrete_jacobians(MODEL, [0.0, 1.0, 0.0, 1.0], good_u, 1.0, 0.1)
end
