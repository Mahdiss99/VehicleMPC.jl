"""
    ConstraintViolation(constraint, value, limit, message)

One violated *operating* constraint. Operating constraints are physical or operational
limits: track boundary, speed range, actuator range. A state that violates them is still
mathematically valid, so the model can be evaluated there. This is different from an
invalid Frenet/model domain (`1 - kappa*ey ≤ eps_geometry`, `v < 0`, steering near the
`tan` singularity, non-finite values), which [`check_domain`](@ref) reports by throwing a
`DomainError`.

`constraint` is one of `:track_boundary`, `:speed_max`, `:speed_min`, `:steering`,
`:acceleration_min`, `:acceleration_max`. `value` is the offending quantity and `limit` the
bound it crossed, both in SI units.
"""
struct ConstraintViolation
    constraint::Symbol
    value::Float64
    limit::Float64
    message::String
end

Base.show(io::IO, v::ConstraintViolation) = print(io, "ConstraintViolation(:", v.constraint, ": ", v.message, ")")

"""
    check_operating_constraints(model, track, x, u=nothing; tol=0.0) -> Vector{ConstraintViolation}

Check state `x = [s, ey, epsi, v]` (and input `u = [delta, a]` if given) against the
operating limits. An empty vector means every checked limit holds. The function reports
violations; it never throws for them and never modifies or clamps `x` or `u`.

- Track boundary `|ey| ≤ track.width/2`. Checked only when `track.width > 0`; a width of 0
  means "no boundary defined".
- Speed `params.v_min ≤ v ≤ params.v_max`.
- Steering `|delta| ≤ params.delta_max`; acceleration `params.a_min ≤ a ≤ params.a_max`.

`tol ≥ 0` is an absolute numerical tolerance added to every limit (default 0: strict). It
is useful for solver output that sits on a bound up to round-off.

Throws `DomainError` for non-finite values and `DimensionMismatch` for wrong lengths. It
does not test the Frenet metric `1 - kappa*ey`; use `check_domain` for that.
"""
function check_operating_constraints(model::KinematicBicycle, track::Track, x::AbstractVector,
    u::Union{Nothing,AbstractVector}=nothing; tol::Real=0.0)
    (isfinite(tol) && tol >= 0) || throw(ArgumentError("tol must be finite and nonnegative, got $tol"))
    length(x) == 4 || throw(DimensionMismatch("state must have length 4, got $(length(x))"))
    all(isfinite, x) || throw(DomainError(x, "state must be finite"))
    p = model.params
    violations = ConstraintViolation[]
    ey, v = x[2], x[4]

    half_width = track.width / 2
    if half_width > 0 && abs(ey) > half_width + tol
        side = ey > 0 ? "left" : "right"
        push!(violations, ConstraintViolation(:track_boundary, ey, copysign(half_width, ey),
            "lateral offset ey = $(fmt(ey)) m is beyond the $side track boundary (|ey| ≤ $(fmt(half_width)) m)"))
    end
    v > p.v_max + tol && push!(violations, ConstraintViolation(:speed_max, v, p.v_max,
        "speed v = $(fmt(v)) m/s exceeds v_max = $(fmt(p.v_max)) m/s"))
    v < p.v_min - tol && push!(violations, ConstraintViolation(:speed_min, v, p.v_min,
        "speed v = $(fmt(v)) m/s is below v_min = $(fmt(p.v_min)) m/s"))

    u === nothing && return violations
    length(u) == 2 || throw(DimensionMismatch("input must have length 2, got $(length(u))"))
    all(isfinite, u) || throw(DomainError(u, "input must be finite"))
    delta, a = u
    abs(delta) > p.delta_max + tol && push!(violations, ConstraintViolation(:steering, delta, copysign(p.delta_max, delta),
        "steering delta = $(fmt(delta)) rad exceeds |delta| ≤ $(fmt(p.delta_max)) rad"))
    a < p.a_min - tol && push!(violations, ConstraintViolation(:acceleration_min, a, p.a_min,
        "acceleration a = $(fmt(a)) m/s² is below a_min = $(fmt(p.a_min)) m/s²"))
    a > p.a_max + tol && push!(violations, ConstraintViolation(:acceleration_max, a, p.a_max,
        "acceleration a = $(fmt(a)) m/s² exceeds a_max = $(fmt(p.a_max)) m/s²"))
    return violations
end

"True when [`check_operating_constraints`](@ref) finds no violation."
satisfies_operating_constraints(model::KinematicBicycle, track::Track, x::AbstractVector, u=nothing) =
    isempty(check_operating_constraints(model, track, x, u))

"""
    trajectory_violations(model, track, result::SimulationResult) -> Vector{Pair{Int,ConstraintViolation}}

Check every recorded state of a simulation, together with the input applied from it, and
return `k => violation` pairs, where `k` indexes `result.states`. Diagnostic only: the
simulation itself is not affected.
"""
function trajectory_violations(model::KinematicBicycle, track::Track, res::SimulationResult)
    out = Pair{Int,ConstraintViolation}[]
    for (k, x) in enumerate(res.states)
        u = k <= length(res.inputs) ? res.inputs[k] : nothing
        for v in check_operating_constraints(model, track, x, u)
            push!(out, k => v)
        end
    end
    return out
end

fmt(x) = string(round(x; sigdigits=4))
