# Discrete-time Jacobians A = ∂f_d/∂x and B = ∂f_d/∂u, with the curvature passed as an
# external constant (so no derivative is ever taken across a track-segment boundary).
# Analytical (forward Euler) Jacobians are compared with ForwardDiff and with independent
# central finite differences.

const JMODEL = KinematicBicycle(reference_test_vehicle())

"Central-difference Jacobian with per-coordinate step `rel_step*max(1, |z_j|)`."
function central_difference_jacobian(f, z::AbstractVector; rel_step=1e-6)
    z = collect(float.(z))
    f0 = f(z)
    J = zeros(length(f0), length(z))
    for j in eachindex(z)
        h = rel_step * max(1.0, abs(z[j]))
        zp, zm = copy(z), copy(z)
        zp[j] += h
        zm[j] -= h
        J[:, j] = (f(zp) - f(zm)) / (2h)
    end
    return J
end

"Element-wise |J1 - J2| ≤ atol + rtol |J2|."
close_elementwise(J1, J2; rtol, atol) = all(abs.(J1 .- J2) .<= atol .+ rtol .* abs.(J2))

# Straight, positive and negative curvature; zero and nonzero steering / heading error;
# representative speeds. Every case is kept away from the Frenet and tan singularities.
const JACOBIAN_CASES = [
    (x=[s, ey, epsi, v], u=[delta, a], kappa=kappa)
    for kappa in (0.0, 0.5, -0.5, 1.0, -1.0)
    for ey in (0.0, 0.2, -0.25)
    for epsi in (0.0, 0.3, -0.4)
    for v in (0.5, 2.0, 4.5)
    for delta in (0.0, 0.2, -0.35)
    for (s, a) in ((3.0, 0.7),)
]

@testset "analytical vs ForwardDiff vs finite differences (Euler)" begin
    dt = 0.05
    nfail = 0
    for c in JACOBIAN_CASES
        A, B = discrete_jacobians(JMODEL, c.x, c.u, c.kappa, dt)
        A_ad, B_ad = discrete_jacobians_ad(JMODEL, c.x, c.u, c.kappa, dt)
        A_fd = central_difference_jacobian(z -> discrete_dynamics(JMODEL, z, c.u, c.kappa, dt), c.x)
        B_fd = central_difference_jacobian(w -> discrete_dynamics(JMODEL, c.x, w, c.kappa, dt), c.u)
        ok = close_elementwise(A, A_ad; rtol=1e-12, atol=1e-14) &&
             close_elementwise(B, B_ad; rtol=1e-12, atol=1e-14) &&
             close_elementwise(A, A_fd; rtol=1e-5, atol=1e-8) &&
             close_elementwise(B, B_fd; rtol=1e-5, atol=1e-8)
        ok || (nfail += 1; @info "Jacobian mismatch" c)
    end
    @test nfail == 0
    @test length(JACOBIAN_CASES) == 5 * 3 * 3 * 3 * 3
end

@testset "structure of the Jacobians" begin
    x, u, kappa, dt = [1.0, 0.1, 0.2, 2.0], [0.1, 0.3], 0.5, 0.1
    A, B = discrete_jacobians(JMODEL, x, u, kappa, dt)
    Ac, Bc = continuous_jacobians(JMODEL, x, u, kappa)
    @test A ≈ I + dt * Ac
    @test B ≈ dt * Bc
    # Curvature is an external parameter: f does not depend on s.
    @test A[:, 1] == [1, 0, 0, 0]
    @test B[:, 2] ≈ [0, 0, 0, dt]
end

@testset "RK4 Jacobians (ForwardDiff) vs finite differences" begin
    dt = 0.1
    for c in JACOBIAN_CASES[1:7:end]
        A, B = discrete_jacobians(JMODEL, c.x, c.u, c.kappa, dt, RK4())
        A_fd = central_difference_jacobian(z -> discrete_dynamics(JMODEL, z, c.u, c.kappa, dt, RK4()), c.x)
        B_fd = central_difference_jacobian(w -> discrete_dynamics(JMODEL, c.x, w, c.kappa, dt, RK4()), c.u)
        @test close_elementwise(A, A_fd; rtol=1e-5, atol=1e-8)
        @test close_elementwise(B, B_fd; rtol=1e-5, atol=1e-8)
    end
end

@testset "first-order Taylor remainder is second order" begin
    x, u, kappa, dt = SVector(1.0, 0.1, 0.2, 2.0), SVector(0.1, 0.3), -0.6, 0.1
    A, B = discrete_jacobians(JMODEL, x, u, kappa, dt)
    dx, du = SVector(0.3, -0.5, 0.7, 1.0), SVector(-0.4, 1.0)
    remainder(h) = norm(discrete_dynamics(JMODEL, x + h * dx, u + h * du, kappa, dt) -
                        (discrete_dynamics(JMODEL, x, u, kappa, dt) + h * (A * dx + B * du)))
    ratios = [remainder(h) / remainder(h / 2) for h in (1e-2, 5e-3, 2.5e-3)]
    @test all(r -> 3.8 < r < 4.2, ratios)
end
