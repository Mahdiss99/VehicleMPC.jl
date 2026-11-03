# Exact zero-order-hold discretisation of the rear-axle kinematic bicycle in Cartesian
# coordinates (Milestone 2B.4). See docs/cartesian_endpoint_nmpc.md.

# |z| below this uses the Taylor series; above it the closed forms. At |z| = 0.5 the first
# omitted series terms are below 1e-19 (value and both derivatives), and the closed forms
# lose at most ~25 ulp to cancellation.
const SINC_SERIES_RADIUS = 0.5

# sin(z)/z = Σ c_n z^(2n), c_n = (-1)^n/(2n+1)!, n = 0..8.
const SINC_C0 = Tuple((-1)^n / factorial(big(2n + 1)) for n in 0:8) .|> Float64
# d/dz:   Σ_{n≥1} c_n·2n·z^(2n-1)          = z·Σ_{m≥0} c_{m+1}·2(m+1)·z^(2m)
const SINC_C1 = Tuple((-1)^n * 2n / factorial(big(2n + 1)) for n in 1:8) .|> Float64
# d²/dz²: Σ_{n≥1} c_n·2n(2n-1)·z^(2n-2)    = Σ_{m≥0} c_{m+1}·2(m+1)(2m+1)·z^(2m)
const SINC_C2 = Tuple((-1)^n * 2n * (2n - 1) / factorial(big(2n + 1)) for n in 1:8) .|> Float64

"""
    sinc_un(z)

Unnormalised sinc `sin(z)/z`, with the continuous value `1` at `z = 0`.

**Not** `Base.sinc`, which is the normalised `sin(πz)/(πz)`.

For `|z| < $(SINC_SERIES_RADIUS)` it evaluates the Taylor series through `z^16`; otherwise
`sin(z)/z`. The branch is taken on the value, so the function works with ForwardDiff duals.
[`sinc_un_d1`](@ref) and [`sinc_un_d2`](@ref) are its first and second derivatives
(used by the NLP operator).
"""
sinc_un(z) = abs(z) < SINC_SERIES_RADIUS ? evalpoly(z * z, SINC_C0) : sin(z) / z

"First derivative of [`sinc_un`](@ref): `(z cos z - sin z)/z²` (series near zero)."
sinc_un_d1(z) = abs(z) < SINC_SERIES_RADIUS ? z * evalpoly(z * z, SINC_C1) : (z * cos(z) - sin(z)) / z^2

"Second derivative of [`sinc_un`](@ref): `((2 - z²) sin z - 2z cos z)/z³` (series near zero)."
sinc_un_d2(z) = abs(z) < SINC_SERIES_RADIUS ? evalpoly(z * z, SINC_C2) : ((2 - z^2) * sin(z) - 2z * cos(z)) / z^3

"""
    exact_bicycle_increment(L, psi, v, delta, a, dt, sincf=sinc_un) -> (ΔX, ΔY, Δpsi, Δv)

Exact increment of the rear-axle kinematic bicycle

    Xdot = v cos(psi),  Ydot = v sin(psi),  psidot = v tan(delta)/L,  vdot = a

over `dt` with `delta` and `a` held constant:

    d   = v dt + a dt²/2                   (signed arc length of the rear axle)
    phi = tan(delta) d / L                 (heading change)
    ΔX  = d sinc_un(phi/2) cos(psi + phi/2)
    ΔY  = d sinc_un(phi/2) sin(psi + phi/2)
    Δpsi = phi,  Δv = a dt

This is the **single definition** of the exact map. It is used with `Float64`, ForwardDiff
duals, and (with `sincf` an NLP operator) JuMP expressions. It never branches and never
divides. The derivation is in `docs/cartesian_endpoint_nmpc.md`. The map is exact only for
piecewise-constant inputs.
"""
function exact_bicycle_increment(L, psi, v, delta, a, dt, sincf=sinc_un)
    d = v * dt + a * (dt^2 / 2)
    phi = tan(delta) * d / L
    c = d * sincf(phi / 2)
    return (c * cos(psi + phi / 2), c * sin(psi + phi / 2), phi, a * dt)
end

"""
    exact_cartesian_step(model, z, u, dt) -> SVector{4}

Exact step of the Cartesian state `z = [X, Y, psi, v]` under the constant input
`u = [delta, a]` (see [`exact_bicycle_increment`](@ref)). Compare
[`integrate_cartesian`](@ref), which approximates the same ODE with RK4.

The heading is unwrapped. The formula is exact for the ODE for any sign of `v`; agreement
with the forward-driving Frenet plant additionally needs `v(t) ≥ 0` on the interval.
"""
function exact_cartesian_step(model::KinematicBicycle, z::AbstractVector, u::AbstractVector, dt::Real)
    check_timestep(dt)
    abs(u[1]) <= π / 2 - STEERING_SINGULARITY_MARGIN || throw(DomainError(u[1], "steering too close to ±π/2"))
    dX, dY, dpsi, dv = exact_bicycle_increment(model.params.wheelbase, z[3], z[4], u[1], u[2], dt)
    return SVector(z[1] + dX, z[2] + dY, z[3] + dpsi, z[4] + dv)
end

"""
    simulate_exact_cartesian(model, z0, inputs, dt) -> Vector{SVector{4}}

Cartesian states `[X, Y, psi, v]` at the control instants, propagated with
[`exact_cartesian_step`](@ref).
"""
function simulate_exact_cartesian(model::KinematicBicycle, z0::AbstractVector, inputs::AbstractVector, dt::Real)
    zs = [SVector{4}(promote(z0...)...)]
    for u in inputs
        push!(zs, exact_cartesian_step(model, zs[end], u, dt))
    end
    return zs
end
