"""
Result of [`simulate_open_loop`](@ref).

- `t[k]`, `states[k]`, `poses[k]`: time, Frenet state and Cartesian pose `(X, Y, psi)` of
  the k-th recorded state (`k = 1` is the initial state).
- `inputs[k]`, `curvature[k]`: input and curvature applied on the step from `states[k]`.
- `status`: `:success`, `:input_limit` (an input violated the actuator limits),
  or `:domain_error` (the next state left the model/track domain). On failure the
  offending step is not recorded and `message` explains why.
"""
struct SimulationResult{T<:AbstractFloat}
    t::Vector{T}
    states::Vector{SVector{4,T}}
    inputs::Vector{SVector{2,T}}
    curvature::Vector{T}
    poses::Vector{NTuple{3,T}}
    status::Symbol
    message::String
end

succeeded(r::SimulationResult) = r.status === :success

"""
    simulate_open_loop(model, track, x0, inputs, dt; integrator=ForwardEuler()) -> SimulationResult

Apply the input sequence `inputs` (vector of `[delta, a]`) to `model` on `track`. On each
step the curvature is looked up at the current (unwrapped) progress and held constant over
the step. Progress is never wrapped in the state; only the geometry look-ups wrap.

An invalid initial state or time step throws. A failure during the rollout stops the
simulation and is reported through `status`/`message` instead of being clamped.
"""
function simulate_open_loop(model::KinematicBicycle, track::Track, x0::AbstractVector, inputs::AbstractVector, dt::Real;
    integrator::AbstractIntegrator=ForwardEuler())
    check_timestep(dt)
    x = as_state(model, float.(x0))
    check_domain(model, x, SVector(0.0, 0.0), curvature(track, x[1]))

    t = [0.0]
    states = [x]
    poses = [frenet_to_cartesian(track, x[1], x[2], x[3])]
    applied = SVector{2,Float64}[]
    kappas = Float64[]
    for (k, u_raw) in enumerate(inputs)
        u = as_input(model, float.(u_raw))
        if !within_input_limits(model.params, u)
            return SimulationResult(t, states, applied, kappas, poses, :input_limit,
                "step $k: input $u violates actuator limits")
        end
        try
            kappa = curvature(track, x[1])
            x_next = discrete_dynamics(model, x, u, kappa, dt, integrator)
            check_domain(model, x_next, u, curvature(track, x_next[1]))
            pose = frenet_to_cartesian(track, x_next[1], x_next[2], x_next[3])
            push!(applied, u)
            push!(kappas, kappa)
            push!(states, x_next)
            push!(poses, pose)
            push!(t, k * dt)
            x = x_next
        catch err
            err isa DomainError || rethrow()
            return SimulationResult(t, states, applied, kappas, poses, :domain_error,
                "step $k: " * sprint(showerror, err))
        end
    end
    return SimulationResult(t, states, applied, kappas, poses, :success, "")
end
