# Event-aware integration of the Frenet kinematic bicycle on a piecewise constant-curvature
# track, and an independent Cartesian reference model. See docs/multisegment_prediction.md.

"""
One resolved segment-boundary crossing.

- `t`: time since the start of the integration call [s].
- `s`: unwrapped progress of the boundary [m].
- `from`, `to`: segment indices left and entered.
- `lap`: lap index of the segment entered (always 0 on an open track).
- `x`: Frenet state at the crossing.
"""
struct SegmentCrossing
    t::Float64
    s::Float64
    from::Int
    to::Int
    lap::Int
    x::SVector{4,Float64}
end

"""
Result of [`integrate_on_track`](@ref).

- `x`: state at time `t`.
- `status`:
  - `:success`: `t == dt`;
  - `:end_of_track`: an open track's end was reached at `t < dt`;
  - `:start_of_track`: an open track's start was reached at `t < dt`;
  - `:domain_error`: the model domain was left;
  - `:event_failure`: a crossing could not be located reliably;
  - `:too_many_events`: the `max_events` safeguard triggered.
- `crossings`: every boundary crossing in time order.
"""
struct TrackIntegrationResult
    x::SVector{4,Float64}
    t::Float64
    status::Symbol
    crossings::Vector{SegmentCrossing}
    message::String
end

succeeded(r::TrackIntegrationResult) = r.status === :success

"Unwrapped progress interval `[lo, hi)` of segment `i` on lap `lap`."
segment_bounds(tr::Track, lap::Integer, i::Integer) =
    (lap * track_length(tr) + tr.s_start[i], lap * track_length(tr) + tr.s_start[i+1])

"""
    locate_crossing(g, h; tol=1e-12, maxiter=200) -> (tau, converged)

Bracketed root search for `g(tau) = 0` on `[0, h]`, given `g(0) < 0 ≤ g(h)`. It uses the
Illinois variant of regula falsi, with a bisection fallback when an update leaves the
bracket. The bracket `[a, b]` always keeps `g(a) < 0 ≤ g(b)`. The returned `tau` is the
right end `b`, so the state at `tau` is on the boundary or just past it, never before it.
Convergence: `g(b) ≤ tol`, or the bracket is narrower than `64·eps(h)`.
"""
function locate_crossing(g, h; tol=1e-12, maxiter=200)
    a, b = 0.0, Float64(h)
    ga, gb = g(a), g(b)
    ga < 0 <= gb || return (NaN, false)
    wa, wb = ga, gb                          # secant weights (Illinois halves the stagnant one)
    side = 0
    for _ in 1:maxiter
        gb <= tol && return (b, true)        # gb is always the true value g(b) ≥ 0
        b - a <= 64 * eps(Float64(h)) && return (b, true)
        c = (a * wb - b * wa) / (wb - wa)
        (a < c < b) || (c = (a + b) / 2)
        gc = g(c)
        if gc < 0
            a, ga, wa = c, gc, gc
            side == -1 && (wb /= 2)
            side = -1
        else
            b, gb, wb = c, gc, gc
            side == 1 && (wa /= 2)
            side = 1
        end
    end
    return (NaN, false)
end

"""
    integrate_on_track(model, track, x, u, dt; max_step=0.01, root_tol=1e-12, max_events=10_000)
        -> TrackIntegrationResult

Integrate the Frenet kinematic bicycle over `dt` with the input `u` held constant. The
**actual piecewise-constant curvature along the trajectory** is used, not a curvature
frozen at the start of the step.

Algorithm (RK4 with event location):

1. The current segment `(lap, i)` is tracked explicitly; the segment index is never looked
   up from `s` after the start. The segment has the unwrapped interval `[lo, hi)` and
   curvature `kappa_i`.
2. One RK4 step of size `h = min(max_step, time left)` is taken with `kappa_i`.
3. If the step leaves `[lo, hi)` (`s ≥ hi` forward, `s < lo` backward), the crossing time
   `tau ∈ [0, h]` is located as the root of `tau -> s(RK4_tau(x)) - boundary`, the RK4 step
   map of variable length (`locate_crossing`). The state is advanced to the boundary
   with `kappa_i`, and integration continues in the neighbouring segment for the remaining
   time.
4. Several crossings within one call are handled by repeating step 3.
5. On a closed track, leaving the last segment enters the first segment of the next lap.
   `s` is never reset, so progress stays unwrapped. On an open track, reaching either end
   stops with `:end_of_track` or `:start_of_track` at the boundary state; there is no
   extrapolation.

The Frenet state is continuous at a boundary, because the centreline position, heading and
normal are continuous. Only the right-hand side changes.

Safeguards:

- **No ping-pong at a boundary.** The segment is tracked explicitly, so a state lying on a
  boundary up to round-off is assigned to the segment it just entered, and the next step
  starts there.
- **Bounded work.** `max_events` bounds the number of crossings.
- **Failures are reported, not hidden.** A failed root search returns `:event_failure`. A
  `DomainError` from the checked dynamics returns `:domain_error` with the last valid
  state.
- **Invalid start.** An initial state outside an open track throws a `DomainError`.
"""
function integrate_on_track(model::KinematicBicycle, track::Track, x0::AbstractVector, u::AbstractVector, dt::Real;
    max_step::Real=0.01, root_tol::Real=1e-12, max_events::Integer=10_000)
    check_timestep(dt)
    (isfinite(max_step) && max_step > 0) || throw(ArgumentError("max_step must be positive, got $max_step"))
    x = SVector{4,Float64}(x0...)
    uu = SVector{2,Float64}(u...)
    lap, sw = split_progress(track, x[1])
    i = segment_index(track, sw)
    n = num_segments(track)
    t = 0.0
    crossings = SegmentCrossing[]
    result(status, msg="") = TrackIntegrationResult(x, t, status, crossings, msg)
    step(xs, tau, kappa) = tau <= 0 ? xs : discrete_dynamics(model, xs, uu, kappa, tau, RK4())

    while dt - t > 64 * eps(Float64(dt))
        length(crossings) >= max_events && return result(:too_many_events, "more than $max_events crossings in one call")
        lo, hi = segment_bounds(track, lap, i)
        kappa = track.segments[i].curvature
        h = min(Float64(max_step), dt - t)
        xn = try
            step(x, h, kappa)
        catch err
            err isa DomainError || rethrow()
            return result(:domain_error, sprint(showerror, err))
        end
        forward = xn[1] >= hi
        backward = xn[1] < lo
        if !(forward || backward)
            x, t = xn, t + h
            continue
        end
        boundary = forward ? hi : lo
        sgn = forward ? 1.0 : -1.0           # g < 0 inside the segment, ≥ 0 beyond the boundary
        g = tau -> sgn * (step(x, tau, kappa)[1] - boundary)
        tau = if g(0.0) >= 0
            0.0                               # already on the boundary (round-off)
        else
            try
                r, ok = locate_crossing(g, h; tol=root_tol)
                ok || return result(:event_failure, "crossing of s = $boundary could not be located within [0, $h] s")
                r
            catch err
                err isa DomainError || rethrow()
                return result(:domain_error, "during event location: " * sprint(showerror, err))
            end
        end
        x = step(x, tau, kappa)
        t += tau
        from = i
        if forward
            if i < n
                i += 1
            elseif track.closed
                i, lap = 1, lap + 1
            else
                return result(:end_of_track, "reached the end of the open track (s = $boundary) at t = $t s")
            end
        else
            if i > 1
                i -= 1
            elseif track.closed
                i, lap = n, lap - 1
            else
                return result(:start_of_track, "reached the start of the open track (s = $boundary) at t = $t s")
            end
        end
        push!(crossings, SegmentCrossing(t, boundary, from, i, lap, x))
    end
    return result(:success)
end

"""
Result of [`simulate_on_track`](@ref): states at the control instants `t`, all crossings
(with absolute times), and a status/message as in [`TrackIntegrationResult`](@ref). On
failure, the last entry of `states` is the state where integration stopped, at time
`t[end] < N·dt`.
"""
struct TrackSimulationResult
    t::Vector{Float64}
    states::Vector{SVector{4,Float64}}
    crossings::Vector{SegmentCrossing}
    status::Symbol
    message::String
end

succeeded(r::TrackSimulationResult) = r.status === :success

"""
    simulate_on_track(model, track, x0, inputs, dt; kwargs...) -> TrackSimulationResult

Apply the piecewise-constant `inputs` (one per interval of length `dt`) with
[`integrate_on_track`](@ref). Keyword arguments are passed on.
"""
function simulate_on_track(model::KinematicBicycle, track::Track, x0::AbstractVector, inputs::AbstractVector, dt::Real; kwargs...)
    x = SVector{4,Float64}(x0...)
    t, states, crossings = [0.0], [x], SegmentCrossing[]
    for (k, u) in enumerate(inputs)
        r = integrate_on_track(model, track, x, u, dt; kwargs...)
        t0 = (k - 1) * dt
        append!(crossings, [SegmentCrossing(t0 + c.t, c.s, c.from, c.to, c.lap, c.x) for c in r.crossings])
        push!(t, t0 + r.t)
        push!(states, r.x)
        succeeded(r) || return TrackSimulationResult(t, states, crossings, r.status, "interval $k: " * r.message)
        x = r.x
    end
    return TrackSimulationResult(t, states, crossings, :success, "")
end

# ---------------------------------------------------------------------------------------
# Independent Cartesian reference model (no track information at all)

"""
    cartesian_rhs(L, psi, v, delta, a) -> (Xdot, Ydot, psidot, vdot)

Rear-axle kinematic bicycle in Cartesian coordinates. This is an independent reference: it
uses no track geometry and no Frenet quantities.
"""
cartesian_rhs(L, psi, v, delta, a) = (v * cos(psi), v * sin(psi), v * tan(delta) / L, a)

"""
    integrate_cartesian(model, z, u, dt; max_step=1e-3) -> SVector{4}

Integrate the Cartesian state `z = [X, Y, psi, v]` over `dt` with classical RK4 in equal
substeps no longer than `max_step`.
"""
function integrate_cartesian(model::KinematicBicycle, z0::AbstractVector, u::AbstractVector, dt::Real; max_step::Real=1e-3)
    check_timestep(dt)
    L = model.params.wheelbase
    delta, a = Float64(u[1]), Float64(u[2])
    abs(delta) <= π / 2 - STEERING_SINGULARITY_MARGIN || throw(DomainError(delta, "steering too close to ±π/2"))
    f(z) = SVector(cartesian_rhs(L, z[3], z[4], delta, a))
    m = max(1, ceil(Int, dt / max_step))
    h = dt / m
    z = SVector{4,Float64}(z0...)
    for _ in 1:m
        k1 = f(z)
        k2 = f(z + h / 2 * k1)
        k3 = f(z + h / 2 * k2)
        k4 = f(z + h * k3)
        z = z + h / 6 * (k1 + 2k2 + 2k3 + k4)
    end
    return z
end

"""
    simulate_cartesian(model, z0, inputs, dt; max_step=1e-3) -> Vector{SVector{4}}

Cartesian states `[X, Y, psi, v]` at the control instants.
"""
function simulate_cartesian(model::KinematicBicycle, z0::AbstractVector, inputs::AbstractVector, dt::Real; max_step::Real=1e-3)
    zs = [SVector{4,Float64}(z0...)]
    for u in inputs
        push!(zs, integrate_cartesian(model, zs[end], u, dt; max_step))
    end
    return zs
end

"Cartesian state `[X, Y, psi, v]` of a Frenet state on `track`."
function frenet_state_to_cartesian(track::Track, x::AbstractVector)
    X, Y, psi = frenet_to_cartesian(track, x[1], x[2], x[3])
    return SVector(X, Y, psi, Float64(x[4]))
end
