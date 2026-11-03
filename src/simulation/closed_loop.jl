"""
Result of [`simulate_closed_loop`](@ref).

- `t[k]`, `states[k]`, `poses[k]`: time, plant state and Cartesian pose at the k-th
  control instant (`k = 1` is the initial state).
- `inputs[k]`: control applied over `[t[k], t[k+1])`.
- `controller[k]`: the [`NMPCResult`](@ref) (or [`LinearMPCResult`](@ref) for a
  [`LinearMPCController`](@ref)) computed at `t[k]`, including a final failed solve if
  there was one.
- `fine_t`, `fine_states`: the plant trajectory at every integration substep.
- `violations`: `t => ConstraintViolation` for every plant substep that violates an
  operating constraint (actual track width, vehicle speed and input limits), beyond the
  numerical tolerance `config.acceptance_tol`. Recording them never changes the
  simulation.
- `crossings`: segment-boundary crossings of the plant, with absolute times (event-aware
  plant only).
- `plant_integration`: `:event_aware` or `:zero_order_hold` (see `simulate_closed_loop`).
- `status`:
  - `:success`;
  - `:controller_failure`: a solve was rejected, and no control was applied for that
    period;
  - `:plant_domain_error`: the plant left the model domain;
  - `:plant_left_track`: the plant reached an end of an open track.
"""
struct ClosedLoopResult{R}
    t::Vector{Float64}
    states::Vector{SVector{4,Float64}}
    inputs::Vector{SVector{2,Float64}}
    poses::Vector{NTuple{3,Float64}}
    controller::Vector{R}
    fine_t::Vector{Float64}
    fine_states::Vector{SVector{4,Float64}}
    violations::Vector{Pair{Float64,ConstraintViolation}}
    crossings::Vector{SegmentCrossing}
    plant_integration::Symbol
    status::Symbol
    message::String
end

succeeded(r::ClosedLoopResult) = r.status === :success

"""
    simulate_closed_loop(controller, plant, x0, n_steps; initial_input=(0.0, 0.0),
                         plant_substeps=10, plant_integration=:auto, cycle_times=nothing)

Closed loop with the controller and the plant kept separate. At every control period:

1. read the measured plant state;
2. `solve!` the controller (NMPC or linear MPC) with the previously applied input;
3. stop with `:controller_failure` if the solution is not accepted (there is no fallback
   input);
4. apply the first control and integrate the `plant` model over `dt` in `plant_substeps`
   substeps;
5. record states, inputs, crossings, solver diagnostics and operating-constraint
   violations.

**Plant integration.**
- `:event_aware`: [`integrate_on_track`](@ref) with the actual piecewise curvature and
  located boundary crossings (Milestone 2B.1).
- `:zero_order_hold`: the Milestone 2A plant. Each RK4 substep uses the curvature at its
  start; this is exact on a single-segment track.
- `:auto` (default): `:event_aware` on multi-segment tracks and `:zero_order_hold` on
  single-segment tracks. On single-segment tracks the Milestone 2A results therefore stay
  numerically unchanged.

The plant never uses the controller's averaged curvature.

Pass an empty `Vector{Float64}` as `cycle_times` to collect wall-clock seconds around
each complete `solve!` call, including a rejected call. Plant integration and result
export are excluded. The vector is appended to; the controller and its result type are
unchanged.
"""
function simulate_closed_loop(ctrl::Union{NMPCController,LinearMPCController}, plant::KinematicBicycle, x0::AbstractVector, n_steps::Integer;
    initial_input=(0.0, 0.0), plant_substeps::Integer=10, plant_integration::Symbol=:auto,
    cycle_times::Union{Nothing,Vector{Float64}}=nothing)
    n_steps >= 0 && plant_substeps >= 1 || throw(ArgumentError("n_steps ≥ 0 and plant_substeps ≥ 1 required"))
    plant_integration in (:auto, :event_aware, :zero_order_hold) ||
        throw(ArgumentError("plant_integration must be :auto, :event_aware or :zero_order_hold"))
    track, dt = ctrl.track, ctrl.config.dt
    mode = plant_integration === :auto ? (num_segments(track) == 1 ? :zero_order_hold : :event_aware) : plant_integration
    h = dt / plant_substeps
    x = SVector{4,Float64}(x0...)
    check_domain(plant, x, SVector{2,Float64}(initial_input...), curvature(track, x[1]))
    uprev = SVector{2,Float64}(initial_input...)

    t, states, poses = [0.0], [x], [frenet_to_cartesian(track, x[1], x[2], x[3])]
    inputs, results = SVector{2,Float64}[], ctrl isa NMPCController ? NMPCResult[] : LinearMPCResult[]
    fine_t, fine_states = [0.0], [x]
    violations = Pair{Float64,ConstraintViolation}[]
    crossings = SegmentCrossing[]
    tol = ctrl.config.acceptance_tol
    record!(tt, xx, uu) = for v in check_operating_constraints(plant, track, xx, uu; tol)
        push!(violations, tt => v)
    end
    finish(status, msg) = ClosedLoopResult(t, states, inputs, poses, results, fine_t, fine_states, violations,
        crossings, mode, status, msg)

    for k in 1:n_steps
        tsolve = cycle_times === nothing ? UInt64(0) : time_ns()
        r = solve!(ctrl, x; previous_input=uprev)
        cycle_times === nothing || push!(cycle_times, (time_ns() - tsolve) / 1e9)
        push!(results, r)
        r.accepted || return finish(:controller_failure, "control period $k (t = $(t[end]) s): $(r.status): $(r.message)")
        u = r.u
        within_input_limits(plant.params, u) ||
            return finish(:controller_failure, "control period $k: accepted input $u violates the plant's actuator limits")
        record!(t[end], x, u)
        try
            for j in 1:plant_substeps
                tj0 = t[end] + (j - 1) * h
                if mode === :event_aware
                    step = integrate_on_track(plant, track, x, u, h; max_step=h)
                    append!(crossings, [SegmentCrossing(tj0 + c.t, c.s, c.from, c.to, c.lap, c.x) for c in step.crossings])
                    if !succeeded(step)
                        x = step.x
                        status = step.status in (:end_of_track, :start_of_track) ? :plant_left_track : :plant_domain_error
                        return finish(status, "control period $k: plant integration stopped with $(step.status): $(step.message)")
                    end
                    x = step.x
                else
                    x = discrete_dynamics(plant, x, u, curvature(track, x[1]), h, RK4())
                end
                tj = t[end] + j * h
                push!(fine_t, tj)
                push!(fine_states, x)
                j < plant_substeps && record!(tj, x, u)
            end
            push!(poses, frenet_to_cartesian(track, x[1], x[2], x[3]))
        catch err
            err isa DomainError || rethrow()
            return finish(:plant_domain_error, "control period $k: " * sprint(showerror, err))
        end
        push!(t, k * dt)
        push!(states, x)
        push!(inputs, u)
        uprev = u
    end
    record!(t[end], x, nothing)
    return finish(:success, "")
end
