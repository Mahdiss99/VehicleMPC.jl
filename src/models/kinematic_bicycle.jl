"""
Default lower bound on the Frenet metric factor `1 - kappa*ey`. A value of 0.1 means the
vehicle may not get closer to the centre of curvature than 10 % of the local radius.
"""
const DEFAULT_EPS_GEOMETRY = 0.1

"""
    KinematicBicycle(params; eps_geometry=DEFAULT_EPS_GEOMETRY)

Four-state kinematic bicycle model in Frenet coordinates, referenced to the rear axle.

State `x = [s, ey, epsi, v]`, input `u = [delta, a]`, curvature `kappa` supplied
externally (see `docs/model_conventions.md`):

    sdot    = v*cos(epsi) / (1 - kappa*ey)
    eydot   = v*sin(epsi)
    epsidot = v*tan(delta)/L - kappa*sdot
    vdot    = a
"""
struct KinematicBicycle{T<:AbstractFloat}
    params::VehicleParameters{T}
    eps_geometry::T

    function KinematicBicycle(params::VehicleParameters{T}; eps_geometry=DEFAULT_EPS_GEOMETRY) where {T}
        0 < eps_geometry < 1 || throw(ArgumentError("eps_geometry must lie in (0, 1), got $eps_geometry"))
        return new{T}(params, T(eps_geometry))
    end
end

KinematicBicycle(; kwargs...) = KinematicBicycle(reference_test_vehicle(); kwargs...)

state_dim(::KinematicBicycle) = 4
input_dim(::KinematicBicycle) = 2

"""
    check_domain(model, x, u, kappa)

Throw a `DomainError` unless `(x, u, kappa)` lies in the domain where the model is valid:
finite values, `1 - kappa*ey > eps_geometry`, `v ≥ 0`, and
`|delta| ≤ π/2 - STEERING_SINGULARITY_MARGIN`. Actuator limits are *not* checked here;
see [`within_input_limits`](@ref).
"""
function check_domain(model::KinematicBicycle, x::AbstractVector, u::AbstractVector, kappa::Real)
    length(x) == 4 || throw(DimensionMismatch("state must have length 4, got $(length(x))"))
    length(u) == 2 || throw(DimensionMismatch("input must have length 2, got $(length(u))"))
    (all(isfinite, x) && all(isfinite, u) && isfinite(kappa)) ||
        throw(DomainError((x, u, kappa), "state, input and curvature must be finite"))
    metric = 1 - kappa * x[2]
    metric > model.eps_geometry ||
        throw(DomainError(metric, "invalid Frenet geometry: 1 - kappa*ey = $metric ≤ eps_geometry = $(model.eps_geometry)"))
    x[4] >= 0 || throw(DomainError(x[4], "speed must be nonnegative for the forward-driving model"))
    abs(u[1]) <= π / 2 - STEERING_SINGULARITY_MARGIN ||
        throw(DomainError(u[1], "steering angle too close to the tan singularity"))
    return nothing
end

"""
    kinematic_rhs(L, ey, epsi, v, delta, a, kappa) -> (sdot, eydot, epsidot, vdot)

Unchecked right-hand side of the kinematic bicycle. This is the **single definition of the
equations**. [`continuous_dynamics`](@ref) wraps it with [`check_domain`](@ref), and the
nonlinear MPC traces it into JuMP expressions. It never throws and never branches, so it
works with `Float64`, ForwardDiff duals and JuMP variables or expressions alike. The
caller must keep the arguments inside the model domain (see `docs/nmpc_formulation.md`
for how the optimizer does this). The right-hand side does not depend on `s`.
"""
function kinematic_rhs(L, ey, epsi, v, delta, a, kappa)
    sdot = v * cos(epsi) / (1 - kappa * ey)
    eydot = v * sin(epsi)
    epsidot = v * tan(delta) / L - kappa * sdot
    return (sdot, eydot, epsidot, a)
end

"""
    continuous_dynamics(model, x, u, kappa) -> SVector{4}

Time derivative `f(x, u, kappa)` after [`check_domain`](@ref). Generic in the element type,
so it can be differentiated with ForwardDiff.
"""
function continuous_dynamics(model::KinematicBicycle, x::AbstractVector, u::AbstractVector, kappa::Real)
    check_domain(model, x, u, kappa)
    return SVector(kinematic_rhs(model.params.wheelbase, x[2], x[3], x[4], u[1], u[2], kappa))
end

"""
    continuous_jacobians(model, x, u, kappa) -> (Ac, Bc)

Analytical `Ac = ∂f/∂x` and `Bc = ∂f/∂u` with `kappa` held fixed (no `∂kappa/∂s` term).
"""
function continuous_jacobians(model::KinematicBicycle, x::AbstractVector, u::AbstractVector, kappa::Real)
    check_domain(model, x, u, kappa)
    L = model.params.wheelbase
    _, ey, epsi, v = x
    delta = u[1]
    D = 1 - kappa * ey
    c, s = cos(epsi), sin(epsi)

    dsdot_dey = v * c * kappa / D^2
    dsdot_depsi = -v * s / D
    dsdot_dv = c / D

    T = promote_type(eltype(x), eltype(u), typeof(kappa), typeof(L))
    z = zero(T)
    Ac = @SMatrix T[
        z  dsdot_dey            dsdot_depsi            dsdot_dv
        z  z                    v*c                    s
        z  -kappa*dsdot_dey     -kappa*dsdot_depsi     tan(delta)/L-kappa*dsdot_dv
        z  z                    z                      z
    ]
    Bc = @SMatrix T[
        z                        z
        z                        z
        v*(1+tan(delta)^2)/L     z
        z                        one(T)
    ]
    return Ac, Bc
end

as_state(::KinematicBicycle, x::AbstractVector) = SVector{4}(x)
as_input(::KinematicBicycle, u::AbstractVector) = SVector{2}(u)
