# Closed loop of the dynamic-bicycle NMPC with an independent fine-resolution plant (final
# milestone). See docs/dynamic_bicycle_mpc.md.

"""
Result of [`simulate_dynamic_closed_loop`](@ref).

**Per control instant** (`k = 1` is the initial state):
- `t[k]`, `states[k]`: the plant's Cartesian CoM state `[X, Y, ψ, vx, vy, r]`;
- `frenet[k]`: its track coordinates `(s, ey, eψ)`, lap-unwrapped;
- `inputs[k]`: the control applied over `[t[k], t[k+1])`;
- `controller[k]`: the [`DynamicNMPCResult`](@ref) computed at `t[k]`, including a final
  rejected one.

**Plant trajectory.** `fine_t`, `fine_states`, `fine_frenet`, every `record_every` plant
steps.

**Plant violations.** `violations` lists `t => description` for:
- the track edge (`|ey| > width/2`);
- obstacle contact (centre distance below `ego_radius + radius`);
- the operating domain at the control instants.

Recording them never changes the simulation.

**Outcome.**
- `status`:
  - `:success`;
  - `:controller_failure` (a cycle was not accepted, so no control was applied);
  - `:plant_domain_error` (the plant left `vx > 0`);
  - `:plant_left_track` (no valid track projection);
- `message`.
"""
struct DynamicClosedLoopResult
    t::Vector{Float64}
    states::Vector{SVector{6,Float64}}
    frenet::Vector{NTuple{3,Float64}}
    inputs::Vector{SVector{2,Float64}}
    controller::Vector{DynamicNMPCResult}
    fine_t::Vector{Float64}
    fine_states::Vector{SVector{6,Float64}}
    fine_frenet::Vector{NTuple{3,Float64}}
    violations::Vector{Pair{Float64,String}}
    status::Symbol
    message::String
end

succeeded(r::DynamicClosedLoopResult) = r.status === :success

"""
    simulate_dynamic_closed_loop(ctrl::DynamicNMPCController, plant::DynamicBicycle, z0, n_steps;
                                 initial_input=(0, 0), s0=NaN, record_every=10,
                                 cycle_times=nothing) -> DynamicClosedLoopResult

Closed loop with the controller and the plant kept separate. At every control period:
1. measure the plant state;
2. `solve!` with the previously applied input;
3. stop with `:controller_failure` if the cycle is not accepted;
4. apply `r.u` and integrate the plant with fine RK4 steps (`ctrl.config.plant_step`,
   100 per 0.1 s period by default; the NLP uses 4 substeps by default);
5. record states, track coordinates, inputs and plant violations.

`s0` is the progress hint of the initial projection (for example on a closed track near
the start line). The plant may use different parameters from the controller's model
(robustness studies); by default pass `ctrl.vehicle`.

Pass an empty `Vector{Float64}` as `cycle_times` to collect wall-clock seconds around
each complete `solve!` call, including a rejected call. Plant integration and result
export are excluded; the vector is appended to.
"""
function simulate_dynamic_closed_loop(ctrl::DynamicNMPCController, plant::DynamicBicycle, z0::AbstractVector, n_steps::Integer;
    initial_input=(0.0, 0.0), s0::Real=NaN, record_every::Integer=10,
    cycle_times::Union{Nothing,Vector{Float64}}=nothing)
    n_steps >= 0 && record_every >= 1 || throw(ArgumentError("n_steps ≥ 0 and record_every ≥ 1 required"))
    c, tr = ctrl.config, ctrl.track
    dt = c.dt
    z = SVector{6,Float64}(z0...)
    uprev = SVector{2,Float64}(initial_input...)
    proj(zz, hint) = (r = track_reference(tr, zz[1], zz[2], zz[3]; s_hint=hint, max_jump=1.0, min_frenet_factor=1e-3); (r.s, r.ey, r.epsi))
    f0 = isfinite(s0) ? proj(z, s0) : cartesian_to_frenet(tr, z[1], z[2], z[3])
    t, states, frenet, inputs, results = [0.0], [z], [f0], SVector{2,Float64}[], DynamicNMPCResult[]
    fine_t, fine_states, fine_frenet = [0.0], [z], [f0]
    violations = Pair{Float64,String}[]
    finish(st, msg) = DynamicClosedLoopResult(t, states, frenet, inputs, results, fine_t, fine_states, fine_frenet, violations, st, msg)
    ctrl.s_prev = f0[1]
    W2 = tr.width / 2
    function record!(tt, zz, ff, u)
        abs(ff[2]) > W2 + c.acceptance_tol && push!(violations, tt => "track edge: |ey| = $(fmt(abs(ff[2]))) m > $(W2) m")
        for (i, o) in enumerate(ctrl.obstacles)
            d = hypot(zz[1] - o.x, zz[2] - o.y)
            d < c.ego_radius + o.radius - c.acceptance_tol && push!(violations, tt => "obstacle $i contact: distance $(fmt(d)) m")
        end
        if u !== nothing
            for v in dynamic_operating_violations(plant, zz, u; tol=c.validation_tol)
                push!(violations, tt => "operating domain: $v")
            end
        end
    end
    record!(0.0, z, f0, nothing)
    for k in 1:n_steps
        tsolve = cycle_times === nothing ? UInt64(0) : time_ns()
        r = solve!(ctrl, z; previous_input=uprev, s_hint=frenet[end][1])
        cycle_times === nothing || push!(cycle_times, (time_ns() - tsolve) / 1e9)
        push!(results, r)
        r.accepted || return finish(:controller_failure, "control period $k (t = $(round(t[end]; digits=2)) s): $(r.status): $(r.message)")
        u = r.u
        record!(t[end], z, frenet[end], u)
        seg = try
            integrate_dynamic(plant, z, u, dt; h=c.plant_step)
        catch err
            err isa DomainError || rethrow()
            return finish(:plant_domain_error, "control period $k: " * sprint(showerror, err))
        end
        nfine = length(seg) - 1
        hint = frenet[end][1]
        lastp = (z[1], z[2])
        try
            for j in record_every:record_every:nfine
                zz = seg[j+1]
                ff = proj(zz, hint + hypot(zz[1] - lastp[1], zz[2] - lastp[2]))
                hint, lastp = ff[1], (zz[1], zz[2])
                push!(fine_t, t[end] + j * dt / nfine)
                push!(fine_states, zz)
                push!(fine_frenet, ff)
                record!(fine_t[end], zz, ff, nothing)
            end
            z = seg[end]
            fz = nfine % record_every == 0 ? fine_frenet[end] : proj(z, hint + hypot(z[1] - lastp[1], z[2] - lastp[2]))
            push!(t, k * dt)
            push!(states, z)
            push!(frenet, fz)
            push!(inputs, u)
        catch err
            err isa TrackReferenceError || rethrow()
            return finish(:plant_left_track, "control period $k: " * err.msg)
        end
        uprev = u
    end
    return finish(:success, "")
end
