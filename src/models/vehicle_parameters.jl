"""
Margin [rad] kept between any admissible steering angle and the `tan` singularity at
`±π/2`. Every `VehicleParameters` must satisfy `delta_max ≤ π/2 - STEERING_SINGULARITY_MARGIN`.
"""
const STEERING_SINGULARITY_MARGIN = 0.1

"""
    VehicleParameters(; wheelbase, delta_max, a_min, a_max, v_min=0.0, v_max)

Vehicle parameters in SI units.

| field       | unit   | meaning                                            |
|:------------|:-------|:---------------------------------------------------|
| `wheelbase` | m      | rear-axle to front-axle distance `L` (> 0)         |
| `delta_max` | rad    | symmetric steering limit, `|delta| ≤ delta_max`    |
| `a_min`     | m/s²   | minimum longitudinal acceleration (braking, ≤ 0)   |
| `a_max`     | m/s²   | maximum longitudinal acceleration (≥ 0)            |
| `v_min`     | m/s    | minimum speed (≥ 0 for the forward-driving model)  |
| `v_max`     | m/s    | maximum speed (> `v_min`)                          |

The constructor rejects non-finite values, a non-positive wheelbase, and steering limits
that are not bounded away from the `tan` singularity.
"""
struct VehicleParameters{T<:AbstractFloat}
    wheelbase::T
    delta_max::T
    a_min::T
    a_max::T
    v_min::T
    v_max::T

    function VehicleParameters{T}(wheelbase, delta_max, a_min, a_max, v_min, v_max) where {T<:AbstractFloat}
        vals = (wheelbase, delta_max, a_min, a_max, v_min, v_max)
        all(isfinite, vals) || throw(ArgumentError("vehicle parameters must be finite, got $vals"))
        wheelbase > 0 || throw(ArgumentError("wheelbase must be positive, got $wheelbase"))
        0 < delta_max <= π / 2 - STEERING_SINGULARITY_MARGIN ||
            throw(ArgumentError("delta_max must lie in (0, π/2 - $(STEERING_SINGULARITY_MARGIN)] rad, got $delta_max"))
        a_min <= 0 <= a_max || throw(ArgumentError("require a_min ≤ 0 ≤ a_max, got a_min=$a_min, a_max=$a_max"))
        0 <= v_min < v_max || throw(ArgumentError("require 0 ≤ v_min < v_max, got v_min=$v_min, v_max=$v_max"))
        return new{T}(wheelbase, delta_max, a_min, a_max, v_min, v_max)
    end
end

function VehicleParameters(; wheelbase, delta_max, a_min, a_max, v_min=0.0, v_max)
    T = float(promote_type(typeof.((wheelbase, delta_max, a_min, a_max, v_min, v_max))...))
    return VehicleParameters{T}(wheelbase, delta_max, a_min, a_max, v_min, v_max)
end

"""
    reference_test_vehicle()

Internally consistent **test configuration** used by the tests and examples.
This is *not* a calibrated physical vehicle.

- `wheelbase = 0.25 m`: equals `lf + lr = 0.125 + 0.125` of the legacy small-scale
  dynamic model, which is the scale the legacy tracks (radius 1–2 m, width 0.5–0.7 m)
  were built for.
- `delta_max = 25° ≈ 0.436 rad`: chosen so that the steady-state steering
  `atan(L/R) ≈ 14.0°` for the tightest legacy radius `R = 1 m` is feasible.
- `a ∈ [-5, 5] m/s²`, `v ∈ [0, 5] m/s`: bounds used in the legacy JuMP/OSQP scripts.
"""
reference_test_vehicle() = VehicleParameters(;
    wheelbase=0.25,
    delta_max=deg2rad(25.0),
    a_min=-5.0,
    a_max=5.0,
    v_min=0.0,
    v_max=5.0,
)

"""
    within_input_limits(params, u) -> Bool

True when `u = [delta, a]` satisfies the actuator limits in `params`.
"""
function within_input_limits(p::VehicleParameters, u::AbstractVector)
    length(u) == 2 || throw(DimensionMismatch("input must have length 2, got $(length(u))"))
    return abs(u[1]) <= p.delta_max && p.a_min <= u[2] <= p.a_max
end

"""
Vehicle parameter sets found in the legacy scripts, kept verbatim for later comparison.
They are **not** mutually consistent and must not be mixed. Only values that appear in the
legacy code are listed; nothing is inferred. Angles are converted to radians where the
legacy code stated degrees.
"""
const LEGACY_PARAMETER_SETS = (
    # New_model_Dynamic.jl :: NonlinearKinematicBicycleGame defaults; main.jl, Simple_MPC.jl
    kinematic_full_scale=(lf=1.5, lr=1.5, delta_max=deg2rad(14.0), a_bounds=(-5.0, 5.0),
        source="New_model_Dynamic.jl, main.jl (player 1)"),
    # MPC with JuMP/main.jl
    kinematic_jump=(lf=0.1, lr=0.1, delta_max=deg2rad(90.0), a_bounds=(-5.0, 5.0), v_bounds=(0.0, 5.0),
        source="MPC with JuMP/main.jl"),
    # New_model_Dynamic.jl :: NonlinearDynamicBicycleGame defaults (bounds from OSQP_comparison.jl)
    dynamic_small_scale=(m=1.98, Iz=0.024, lf=0.125, lr=0.125, cf=1.25, cr=1.25,
        delta_max=deg2rad(30.0), a_bounds=(-10.0, 10.0), v_bounds=(0.0, 5.0),
        source="New_model_Dynamic.jl, MPC with JuMP/OSQP_comparison.jl"),
)
