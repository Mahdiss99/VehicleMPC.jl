# Six-state nonlinear dynamic bicycle model (final milestone). It integrates the dynamic
# model of the legacy `New_model_Dynamic.jl` into the package, with the corrections listed
# in docs/dynamic_bicycle_mpc.md.

"""
    DynamicBicycleParameters(; m, Iz, lf, lr, Cf, Cr, delta_max, a_min, a_max, vx_min, vx_max, alpha_max)

Parameters of the six-state dynamic bicycle model [`DynamicBicycle`](@ref). SI units.

**Mass and geometry.**
- `m` [kg], `Iz` [kg m²]: mass and yaw moment of inertia about the centre of mass (CoM).
- `lf`, `lr` [m]: distances from the CoM to the front and rear axle. The wheelbase is
  `lf + lr`.

**Tyres.**
- `Cf`, `Cr` [N/rad]: cornering stiffness **per axle** (both tyres of an axle together),
  in the linear tyre law `F_y = C α`. The legacy model wrote `2 c_f` with `c_f` per tyre,
  so `Cf = 2 c_f`.

**Inputs.**
- `delta_max` [rad]: front steering limit `|δ| ≤ delta_max`.
- `a_min`, `a_max` [m/s²]: longitudinal acceleration command (net longitudinal tyre force
  divided by `m`).

**Operating domain.**
- `vx_min`, `vx_max` [m/s]: the longitudinal speed range in which the model is used.
  `vx_min > 0` is required, because the slip angles divide by `v_x`.
- `alpha_max` [rad]: the slip-angle range in which the linear tyre law is considered
  valid. The MPC constrains `|α_f|, |α_r| ≤ alpha_max`.

These are synthetic test parameters of a small (about 1/10-scale) car, not calibrated
measurements; see [`legacy_dynamic_parameters`](@ref) and
[`showcase_dynamic_parameters`](@ref).
"""
struct DynamicBicycleParameters{T<:AbstractFloat}
    m::T
    Iz::T
    lf::T
    lr::T
    Cf::T
    Cr::T
    delta_max::T
    a_min::T
    a_max::T
    vx_min::T
    vx_max::T
    alpha_max::T

    function DynamicBicycleParameters{T}(m, Iz, lf, lr, Cf, Cr, delta_max, a_min, a_max, vx_min, vx_max, alpha_max) where {T<:AbstractFloat}
        vals = (m, Iz, lf, lr, Cf, Cr, delta_max, a_min, a_max, vx_min, vx_max, alpha_max)
        all(isfinite, vals) || throw(ArgumentError("dynamic bicycle parameters must be finite"))
        m > 0 && Iz > 0 && lf > 0 && lr > 0 || throw(ArgumentError("m, Iz, lf and lr must be positive"))
        Cf > 0 && Cr > 0 || throw(ArgumentError("cornering stiffnesses Cf and Cr must be positive"))
        0 < delta_max < π / 2 || throw(ArgumentError("delta_max must lie in (0, π/2)"))
        a_min < 0 < a_max || throw(ArgumentError("a_min < 0 < a_max required"))
        0 < vx_min < vx_max || throw(ArgumentError("0 < vx_min < vx_max required (the slip angles divide by vx)"))
        0 < alpha_max < π / 2 || throw(ArgumentError("alpha_max must lie in (0, π/2)"))
        return new{T}(vals...)
    end
end

DynamicBicycleParameters(; m, Iz, lf, lr, Cf, Cr, delta_max, a_min, a_max, vx_min, vx_max, alpha_max) =
    DynamicBicycleParameters{Float64}(m, Iz, lf, lr, Cf, Cr, delta_max, a_min, a_max, vx_min, vx_max, alpha_max)

"""
    legacy_dynamic_parameters() -> DynamicBicycleParameters

The parameters of the legacy `NonlinearDynamicBicycleGame` (`New_model_Dynamic.jl`).

| parameter | value |
|---|---|
| mass `m` | 1.98 kg |
| yaw inertia `Iz` | 0.024 kg m² |
| axle distances `lf`, `lr` | 0.125 m each |
| cornering stiffness `Cf = Cr` | 2·1.25 = 2.5 N/rad per axle |

The limits are those of [`reference_test_vehicle`](@ref): 25° steering, ±5 m/s², 5 m/s.
The added operating domain is `vx ≥ 0.3 m/s` and `|α| ≤ 0.12 rad`.

**Limitation.** With these tyres the car is limited to very low lateral acceleration,
about 0.3 m/s² at `|α| ≤ 0.12 rad`. It cannot negotiate the R = 1 m bends of
`quarter_track()` faster than about 0.55 m/s ([`steady_state_cornering`](@ref)). The legacy
dynamic MPC prototype indeed used a 0.65 m/s speed cap.
"""
legacy_dynamic_parameters() = DynamicBicycleParameters(; m=1.98, Iz=0.024, lf=0.125, lr=0.125, Cf=2 * 1.25, Cr=2 * 1.25,
    delta_max=deg2rad(25), a_min=-5.0, a_max=5.0, vx_min=0.3, vx_max=5.0, alpha_max=0.12)

"""
    showcase_dynamic_parameters() -> DynamicBicycleParameters

Synthetic parameter set used by the racing showcase. It keeps the legacy mass, inertia and
axle distances (wheelbase 0.25 m, as in [`reference_test_vehicle`](@ref)), and uses
physically plausible tyres for a 2 kg car.

**Tyres.** `Cf = Cr = 40 N/rad` per axle, a normalised stiffness of about 4 /rad per unit
axle load.
- The steady-state slip angle at the 4 m/s² lateral-acceleration limit is then about
  0.1 rad, inside the linear range `|α| ≤ 0.12 rad`.
- The legacy value 2.5 N/rad per axle would need about 1.6 rad there (see
  [`steady_state_cornering`](@ref)).

**Operating domain.** `vx ∈ [0.8, 5] m/s`. Above 0.8 m/s, the stiff lateral and yaw modes
(about `(Cf + Cr)/(m vx)` and `(lf² Cf + lr² Cr)/(Iz vx)` [1/s]) stay inside the
stability region of the MPC's RK4 sub-steps (`dt/4`).

**Limits.** Steering, acceleration and speed limits are those of
[`reference_test_vehicle`](@ref).

Not a calibrated vehicle.
"""
showcase_dynamic_parameters() = DynamicBicycleParameters(; m=1.98, Iz=0.024, lf=0.125, lr=0.125, Cf=40.0, Cr=40.0,
    delta_max=deg2rad(25), a_min=-5.0, a_max=5.0, vx_min=0.8, vx_max=5.0, alpha_max=0.12)

"""
    DynamicBicycle(params)

Six-state nonlinear dynamic bicycle, referenced at the centre of mass, in global Cartesian
coordinates:

    z = [X, Y, ψ, vx, vy, r]      u = [δ, a]

**State and input.**
- `(X, Y)`: CoM position [m]; `ψ`: heading of the body x-axis (unwrapped) [rad].
- `vx`, `vy`: CoM velocity in the body frame (x forward, y left) [m/s].
- `r = ψ̇`: yaw rate, counter-clockwise positive [rad/s].
- `δ`: front-wheel steering angle (positive to the left) [rad].
- `a`: longitudinal acceleration command [m/s²].

**Equations:**

    Ẋ  = vx cos ψ − vy sin ψ
    Ẏ  = vx sin ψ + vy cos ψ
    ψ̇  = r
    v̇x = a − F_yf sin δ / m + r vy
    v̇y = (F_yf cos δ + F_yr)/m − r vx
    ṙ  = (lf F_yf cos δ − lr F_yr)/Iz

**Tyres** (linear, per axle):

    α_f = δ − atan((vy + lf r)/vx)      α_r = −atan((vy − lr r)/vx)
    F_yf = Cf α_f                      F_yr = Cr α_r

The model is defined for `vx > 0` only; see [`check_dynamic_domain`](@ref).
"""
struct DynamicBicycle{T<:AbstractFloat}
    params::DynamicBicycleParameters{T}
end

"Front and rear slip angles `(α_f, α_r)` [rad] of the state `z` with input `u` (`vx > 0`)."
function slip_angles(p::DynamicBicycleParameters, z, u)
    vx, vy, r = z[4], z[5], z[6]
    return u[1] - atan((vy + p.lf * r) / vx), -atan((vy - p.lr * r) / vx)
end

"Lateral axle forces `(F_yf, F_yr)` [N] of the linear tyre law."
function tyre_forces(p::DynamicBicycleParameters, z, u)
    af, ar = slip_angles(p, z, u)
    return p.Cf * af, p.Cr * ar
end

"""
    dynamic_rhs(params, z, u) -> NTuple{6}

Time derivative of the dynamic bicycle state (see [`DynamicBicycle`](@ref)). Generic
(floats, dual numbers, JuMP expressions); no domain check.
"""
function dynamic_rhs(p::DynamicBicycleParameters, z, u)
    psi, vx, vy, r = z[3], z[4], z[5], z[6]
    delta, a = u[1], u[2]
    Fyf, Fyr = tyre_forces(p, z, u)
    return (vx * cos(psi) - vy * sin(psi), vx * sin(psi) + vy * cos(psi), r,
        a - Fyf * sin(delta) / p.m + r * vy,
        (Fyf * cos(delta) + Fyr) / p.m - r * vx,
        (p.lf * Fyf * cos(delta) - p.lr * Fyr) / p.Iz)
end

"""
    dynamic_lateral_acceleration(params, z, u)

Body-frame lateral acceleration of the CoM, `a_y = (F_yf cos δ + F_yr)/m = v̇y + r vx`
[m/s²]. This is the dynamic model's own quantity; the kinematic expression `v² tan δ / L` is
not used for this model.
"""
function dynamic_lateral_acceleration(p::DynamicBicycleParameters, z, u)
    Fyf, Fyr = tyre_forces(p, z, u)
    return (Fyf * cos(u[1]) + Fyr) / p.m
end

"""
    check_dynamic_domain(model, z, u)

Throw a `DomainError` unless `z` and `u` are finite and `vx > 0`, where the model is
defined (the slip angles divide by `vx`). The narrower *operating* domain
`vx ∈ [vx_min, vx_max]` and `|α| ≤ alpha_max` is a constraint of the controller and is
checked separately ([`dynamic_operating_violations`](@ref)).
"""
function check_dynamic_domain(model::DynamicBicycle, z::AbstractVector, u::AbstractVector)
    length(z) == 6 && length(u) == 2 || throw(DimensionMismatch("dynamic bicycle: z must have 6 and u 2 entries"))
    all(isfinite, z) && all(isfinite, u) || throw(DomainError((z, u), "non-finite dynamic bicycle state or input"))
    z[4] > 0 || throw(DomainError(z[4], "the dynamic bicycle model needs vx > 0 (slip angles divide by vx)"))
    return nothing
end

"One classical RK4 step of length `h` (generic)."
function dynamic_rk4_step(p::DynamicBicycleParameters, z, u, h)
    k1 = dynamic_rhs(p, z, u)
    k2 = dynamic_rhs(p, z .+ (h / 2) .* k1, u)
    k3 = dynamic_rhs(p, z .+ (h / 2) .* k2, u)
    k4 = dynamic_rhs(p, z .+ h .* k3, u)
    return z .+ (h / 6) .* (k1 .+ 2 .* k2 .+ 2 .* k3 .+ k4)
end

"""
    dynamic_step(model, z, u, dt; substeps=4) -> SVector{6}

RK4 integration over `dt` with `substeps` equal steps, with the input held constant (zero-
order hold). This is the discretisation used inside the MPC. It is an approximation: the
dynamic model has no closed-form step.
"""
function dynamic_step(model::DynamicBicycle, z::AbstractVector, u::AbstractVector, dt::Real; substeps::Integer=4)
    check_dynamic_domain(model, z, u)
    T = float(promote_type(eltype(z), eltype(u), typeof(dt)))     # generic, e.g. for ForwardDiff
    h = dt / substeps
    zz = Tuple(T.(z))
    for _ in 1:substeps
        zz = dynamic_rk4_step(model.params, zz, u, h)
        zz[4] > 0 || throw(DomainError(zz[4], "vx left the model domain during integration"))
    end
    return SVector{6,T}(zz)
end

"""
    integrate_dynamic(model, z0, u, dt; h=1e-3) -> Vector{SVector{6}}

Fine-resolution RK4 integration over one period (`h` = step, `dt` a multiple of `h` up to
rounding). Returns the states at every step, starting with `z0`. This is the plant and the
independent validation model (100 steps per 0.1 s period by default, against the MPC's 4).
"""
function integrate_dynamic(model::DynamicBicycle, z0::AbstractVector, u::AbstractVector, dt::Real; h::Real=1e-3)
    check_dynamic_domain(model, z0, u)
    n = max(1, round(Int, dt / h))
    hh = dt / n
    out = Vector{SVector{6,Float64}}(undef, n + 1)
    zz = Tuple(Float64.(z0))
    out[1] = SVector(zz)
    for j in 1:n
        zz = dynamic_rk4_step(model.params, zz, u, hh)
        zz[4] > 0 || throw(DomainError(zz[4], "vx left the model domain during integration"))
        out[j+1] = SVector(zz)
    end
    return out
end

"""
    dynamic_operating_violations(model, z, u; tol=1e-6) -> Vector{String}

Operating-domain and limit checks of one state/input pair:
- `vx ∈ [vx_min, vx_max]`;
- `|δ| ≤ delta_max` and `a ∈ [a_min, a_max]`;
- `|α_f|, |α_r| ≤ alpha_max`.
"""
function dynamic_operating_violations(model::DynamicBicycle, z, u; tol::Real=1e-6)
    p, out = model.params, String[]
    p.vx_min - tol <= z[4] <= p.vx_max + tol || push!(out, "vx = $(fmt(z[4])) m/s outside [$(p.vx_min), $(p.vx_max)]")
    abs(u[1]) <= p.delta_max + tol || push!(out, "|δ| = $(fmt(abs(u[1]))) rad > $(fmt(p.delta_max))")
    p.a_min - tol <= u[2] <= p.a_max + tol || push!(out, "a = $(fmt(u[2])) m/s² outside [$(p.a_min), $(p.a_max)]")
    af, ar = slip_angles(p, z, u)
    max(abs(af), abs(ar)) <= p.alpha_max + tol || push!(out, "slip angle $(fmt(max(abs(af), abs(ar)))) rad > $(fmt(p.alpha_max))")
    return out
end

"""
    lateral_mode_rates(params, vx) -> (lateral, yaw)

Magnitudes [1/s] of the lateral and yaw damping rates of the linearised dynamic bicycle,
`(Cf + Cr)/(m vx)` and `(lf² Cf + lr² Cr)/(Iz vx)`. They grow as `1/vx`, which is why the
model becomes stiff at low speed. RK4 with step `h` is stable for `rate·h ≲ 2.78` (real
eigenvalues).
"""
lateral_mode_rates(p::DynamicBicycleParameters, vx::Real) = ((p.Cf + p.Cr) / (p.m * vx), (p.lf^2 * p.Cf + p.lr^2 * p.Cr) / (p.Iz * vx))

"""
    steady_state_cornering(params, R, v) -> (; ay, alpha_f, alpha_r, delta, understeer_gradient)

Linear steady-state cornering of the dynamic bicycle on a circle of radius `R` at speed `v`
(small-angle bicycle theory):
- `a_y = v²/R`;
- axle forces `F_yf = m a_y lr/L` and `F_yr = m a_y lf/L`;
- slip angles `α = F_y/C`;
- `δ = L/R + K a_y` with the understeer gradient `K = (m/L)(lr/Cf − lf/Cr)`.

Used to check whether a parameter set can negotiate a bend inside its linear tyre range.
"""
function steady_state_cornering(p::DynamicBicycleParameters, R::Real, v::Real)
    L = p.lf + p.lr
    ay = v^2 / R
    af = p.m * ay * p.lr / L / p.Cf
    ar = p.m * ay * p.lf / L / p.Cr
    K = p.m / L * (p.lr / p.Cf - p.lf / p.Cr)
    return (; ay, alpha_f=af, alpha_r=ar, delta=L / R + K * ay, understeer_gradient=K)
end

"""
    max_cornering_speed(params, R; ay_max=Inf) -> Float64

Largest steady-state speed on radius `R` with both slip angles within `alpha_max` (and
`a_y ≤ ay_max`), from [`steady_state_cornering`](@ref).
"""
function max_cornering_speed(p::DynamicBicycleParameters, R::Real; ay_max::Real=Inf)
    L = p.lf + p.lr
    ay_tyre = p.alpha_max * min(p.Cf * L / (p.m * p.lr), p.Cr * L / (p.m * p.lf))
    return sqrt(min(ay_tyre, ay_max) * R)
end

"""
    cg_pose_from_rear_axle(params, X, Y, psi) -> (Xg, Yg, psi)
    rear_axle_pose(params, z) -> (Xr, Yr, psi)

Rigid-body transformation between the rear-axle reference point (used by the kinematic
model) and the CoM (used by the dynamic model): `p_cg = p_rear + lr (cos ψ, sin ψ)`.
"""
cg_pose_from_rear_axle(p::DynamicBicycleParameters, X, Y, psi) = (X + p.lr * cos(psi), Y + p.lr * sin(psi), psi)
rear_axle_pose(p::DynamicBicycleParameters, z) = (z[1] - p.lr * cos(z[3]), z[2] - p.lr * sin(z[3]), z[3])
