"""
Time-discretization scheme for `discrete_dynamics`. Curvature is held constant over one
step (zero-order hold); a step that crosses a track-segment boundary therefore uses the
curvature at the start of the step.
"""
abstract type AbstractIntegrator end

"Forward Euler: `x⁺ = x + dt*f(x, u, kappa)`."
struct ForwardEuler <: AbstractIntegrator end

"Classical fourth-order Runge–Kutta with `u` and `kappa` held constant over the step."
struct RK4 <: AbstractIntegrator end

function check_timestep(dt::Real)
    (isfinite(dt) && dt > 0) || throw(ArgumentError("time step dt must be positive and finite, got $dt"))
    return nothing
end

"""
    discrete_dynamics(model, x, u, kappa, dt, integrator=ForwardEuler()) -> SVector

One discrete step `x⁺ = f_d(x, u, kappa)`.
"""
function discrete_dynamics(model, x::AbstractVector, u::AbstractVector, kappa::Real, dt::Real,
    ::ForwardEuler=ForwardEuler())
    check_timestep(dt)
    return as_state(model, x) + dt * continuous_dynamics(model, x, u, kappa)
end

function discrete_dynamics(model, x::AbstractVector, u::AbstractVector, kappa::Real, dt::Real, ::RK4)
    check_timestep(dt)
    x0 = as_state(model, x)
    k1 = continuous_dynamics(model, x0, u, kappa)
    k2 = continuous_dynamics(model, x0 + dt / 2 * k1, u, kappa)
    k3 = continuous_dynamics(model, x0 + dt / 2 * k2, u, kappa)
    k4 = continuous_dynamics(model, x0 + dt * k3, u, kappa)
    return x0 + dt / 6 * (k1 + 2k2 + 2k3 + k4)
end

"""
    discrete_jacobians(model, x, u, kappa, dt, integrator=ForwardEuler()) -> (A, B)

`A = ∂f_d/∂x`, `B = ∂f_d/∂u` with `kappa` held fixed. Forward Euler uses the analytical
continuous Jacobians (`A = I + dt*Ac`, `B = dt*Bc`); other integrators fall back to
[`discrete_jacobians_ad`](@ref).
"""
function discrete_jacobians(model, x::AbstractVector, u::AbstractVector, kappa::Real, dt::Real,
    ::ForwardEuler=ForwardEuler())
    check_timestep(dt)
    Ac, Bc = continuous_jacobians(model, x, u, kappa)
    return one(Ac) + dt * Ac, dt * Bc
end

discrete_jacobians(model, x::AbstractVector, u::AbstractVector, kappa::Real, dt::Real, integrator::AbstractIntegrator) =
    discrete_jacobians_ad(model, x, u, kappa, dt, integrator)

"""
    discrete_jacobians_ad(model, x, u, kappa, dt, integrator=ForwardEuler()) -> (A, B)

Reference Jacobians of `discrete_dynamics` computed with ForwardDiff.
"""
function discrete_jacobians_ad(model, x::AbstractVector, u::AbstractVector, kappa::Real, dt::Real,
    integrator::AbstractIntegrator=ForwardEuler())
    x0, u0 = as_state(model, x), as_input(model, u)
    A = ForwardDiff.jacobian(xx -> discrete_dynamics(model, xx, u0, kappa, dt, integrator), x0)
    B = ForwardDiff.jacobian(uu -> discrete_dynamics(model, x0, uu, kappa, dt, integrator), u0)
    return A, B
end
