# Segment-reassignment reliability for the Cartesian-endpoint NMPC (Milestone 3B.1): solve
# traces, cycle detection in the reassignment, local branch resolution of an oscillating
# knot, and the optional cycle budget. See docs/segment_reassignment_reliability.md.

"""
    cycle_budget_allows(config, cycle_start_ns, longest_solve) -> Bool

Best-effort guard for **optional** solves within one control cycle. It returns true unless
`time used so far + 1.25 × longest solve so far in this cycle > cycle_budget`. With the
default `cycle_budget = Inf` it is always true. This is not a real-time guarantee: a
single Ipopt solve can take longer than estimated.
"""
cycle_budget_allows(c::NMPCConfig, cycle_start, longest) =
    !isfinite(c.cycle_budget) || (time_ns() - cycle_start) / 1e9 + 1.25 * longest <= c.cycle_budget

"""
    detect_assignment_cycle(history, proposed) -> nothing | (; length, knots, first)

`history` holds the assignment vectors already solved in this call (lap-aware segment ids
of all knots), and `proposed` is the next one. If `proposed` equals an earlier vector, the
reassignment is cycling:
- `length` is the number of solves in the cycle (2 for an A, B, A alternation);
- `knots` are the knots whose segment differs within the cycle;
- `first` is the index in `history` of the first member.

An exact repeat is sufficient evidence of a cycle, because the reassignment maps an
assignment to its next one deterministically, given the same warm start structure.
"""
function detect_assignment_cycle(history::AbstractVector, proposed::AbstractVector)
    j = findlast(h -> h == proposed, history)
    j === nothing && return nothing
    members = vcat(history[j:end], [proposed])
    knots = sort!(unique([k for k in eachindex(proposed) if any(m[k] != proposed[k] for m in members)]))
    return (; length=length(history) - j + 1, knots, first=j)
end

"One trace entry of [`ReassignmentReport`](@ref) for a finished solve."
function reassignment_entry(ctrl::NMPCController, x, sched, core, status, solve, previds, branch)
    tr, N, tol = ctrl.track, horizon(ctrl), ctrl.config.boundary_knot_tol
    ids = copy(sched.asg.seg)
    changed = previds === nothing ? Int[] : findall(ids .!= previds)
    outside, outside_s = Int[], Float64[]
    oc = NaN
    if all(isfinite, core.X)
        for k in 2:N+1
            frame_valid(tr, sched.asg.frames[k], core.X[1, k], tol) || (push!(outside, k); push!(outside_s, core.X[1, k]))
        end
        if ctrl.obstacle !== nothing
            ob, D = ctrl.obstacle, required_separation(ctrl)
            oc = try
                minimum(hypot(p[1] - ob.x, p[2] - ob.y) for p in (frenet_state_to_cartesian(tr, core.X[:, k]) for k in 2:N+1)) - D
            catch err
                err isa DomainError || rethrow()
                NaN
            end
        end
    end
    return (; solve, ids, changed, outside, outside_s, status, termination=core.ts, iterations=core.iterations,
        dyn_res=core.dyn_res, viol=core.viol, objective=core.obj, solver_time=core.solver_time, obstacle_clearance=oc, branch)
end

"""
    install_cartesian_ids!(controller, sched, ids) -> sched

Install an explicit lap-aware segment assignment `ids` (one id per knot) into the
Cartesian-endpoint NLP Parameters. With interior corridor samples, their assignment is
recomputed from `sched.Xg` and `sched.Ug`. No membership bounds are set.
"""
function install_cartesian_ids!(ctrl::NMPCController, sched, ids::AbstractVector{<:Integer})
    tr = ctrl.track
    frames = [segment_frame(tr, id) for id in ids]
    for (k, f) in enumerate(frames), (r, v) in enumerate((f.s0, f.x0, f.y0, f.psi0, f.kappa))
        set_parameter_value(ctrl.cframe[r, k], v)
    end
    asg = (; seg=collect(Int, ids), frames, boundary_knots=0)
    if !isempty(ctrl.alphas)
        samples = interior_assignment(ctrl, sched.Xg, sched.Ug, frames)
        install_interior!(ctrl, samples)
        asg = (; asg..., samples)
    end
    return (; sched..., lin=nothing, asg)
end

"""
    with_progress_bounds(f, variable, lo, hi)

Run `f()` with the variable temporarily bounded to `[lo, hi]`. The previous bounds (or
their absence) are restored afterwards, also when `f` throws.
"""
function with_progress_bounds(f, v::VariableRef, lo, hi)
    oldlo = has_lower_bound(v) ? lower_bound(v) : nothing
    oldhi = has_upper_bound(v) ? upper_bound(v) : nothing
    try
        set_lower_bound(v, lo)
        set_upper_bound(v, hi)
        return f()
    finally
        oldlo === nothing ? delete_lower_bound(v) : set_lower_bound(v, oldlo)
        oldhi === nothing ? delete_upper_bound(v) : set_upper_bound(v, oldhi)
    end
end

"""
    resolve_branch!(controller, x, sched, history, cycle, cycle_start, call_start, longest)

Local branch resolution of a reassignment cycle (Milestone 3B.1).

**When it applies:** only if the cycle involves exactly one knot `k`, alternating between
two adjacent segments `A` and `B`. Otherwise it reports why it was not attempted. There is
no combinatorial search over several knots.

**For each segment in `(A, B)`:**
1. Take the cycle member whose assignment had knot `k` in that segment: its full
   assignment vector, and its solution as the warm start.
2. Install that assignment exactly.
3. Bound **only** `s_k` to the segment's interval `[s0, s1]`. The knot is not fixed to the
   boundary.
4. Solve, within the remaining correction and cycle budgets (conservative estimate:
   1.25 × the longest solve so far).
5. Read the solution while the bound is in place, then run the full post-solve checks: assignments, dynamics, track, inputs and corridor, as
   in the ordinary loop. The obstacle checks follow in the obstacle wrapper.
6. Always restore the bound.

**Selection:** the valid candidate with the lower objective. If neither is valid, there is
no solution (`chosen === nothing`).

For a controller with an obstacle, a candidate also has to pass the obstacle acceptance
rule ([`obstacle_rejection`](@ref) on its [`collision_check`](@ref)) before it is
eligible. A lower-objective branch that fails the dense collision evaluation therefore
does not displace a collision-valid one (Milestone 4A correction; see
[`select_branch_candidate`](@ref)).

Returns `(; attempts, trace, solves, t_opt, t_ipopt, message, chosen)`.
"""
function resolve_branch!(ctrl::NMPCController, x, sched, history, cycle, cycle_start, call_start, longest)
    c, tr = ctrl.config, ctrl.track
    attempts, trace = Any[], Any[]
    none(msg) = (; attempts, trace, solves=0, t_opt=0.0, t_ipopt=NaN, message=msg, chosen=nothing)
    length(cycle.knots) == 1 || return none("branch resolution not attempted: the cycle involves $(length(cycle.knots)) knots $(cycle.knots)")
    k = cycle.knots[1]
    members = history[cycle.first:end]
    segs = sort!(unique([m.ids[k] for m in members]))
    (length(segs) == 2 && segs[2] - segs[1] == 1) ||
        return none("branch resolution not attempted: knot $k alternates between segments $segs (not two adjacent segments)")
    c.segment_membership && return none("branch resolution not attempted: segment_membership already bounds every knot")
    solves, topt, tip = 0, 0.0, 0.0
    valid = Any[]
    for sid in segs
        f = segment_frame(tr, sid)
        if (time_ns() - call_start) / 1e9 + 1.25 * longest > c.correction_time_budget || !cycle_budget_allows(c, cycle_start, longest)
            push!(attempts, (; knot=k, segment=sid, s0=f.s0, s1=f.s1, status=:not_attempted_budget, objective=NaN, solves=0, seconds=0.0))
            continue
        end
        m = members[findlast(mm -> mm.ids[k] == sid, members)]
        kap = try
            stage_curvatures(tr, m.s)
        catch err
            err isa DomainError || rethrow()
            sched.kappas
        end
        bsched = install_cartesian_ids!(ctrl, (; sched..., Xg=m.X, Ug=m.U, s_nominal=m.s, kappas=kap), m.ids)
        apply_start_values!(ctrl, m.X, m.U, bsched.kappas)
        ta = time_ns()
        tb = ta
        # The solution is read while the bound is still in place: modifying the model first
        # would invalidate the result in JuMP.
        core = with_progress_bounds(ctrl.X[1, k], f.s0, f.s1) do
            optimize!(ctrl.model)
            tb = time_ns()
            extract_core(ctrl, x, bsched.kappas; frames=bsched.asg.frames)
        end
        solves += 1
        topt += (tb - ta) / 1e9
        longest = max(longest, (tb - ta) / 1e9)
        isnan(core.solver_time) || (tip += core.solver_time)
        pc = post_solve_checks(ctrl, x, bsched, core)
        push!(trace, reassignment_entry(ctrl, x, bsched, core, pc.status, -solves, m.ids, sid))
        push!(attempts, (; knot=k, segment=sid, s0=f.s0, s1=f.s1, status=pc.status, objective=core.obj, solves=1, seconds=(tb - ta) / 1e9))
        pc.status === :success && push!(valid, (; core, pc..., sched=bsched, segment=sid))
    end
    best, note = select_branch_candidate(ctrl, x, valid)
    if best === nothing
        sts = join(["segment $(a.segment): $(a.status)" for a in attempts], ", ")
        return (; attempts, trace, solves, t_opt=topt, t_ipopt=tip, chosen=nothing,
            message="branch resolution of knot $k found no valid candidate ($sts)" * note)
    end
    objs = join(["segment $(v.segment): objective $(fmt(v.core.obj))" for v in valid], ", ")
    return (; attempts, trace, solves, t_opt=topt, t_ipopt=tip, chosen=best,
        message="branch resolution of knot $k selected segment $(best.segment) ($objs)" * note)
end

"""
    select_branch_candidate(controller, x, valid) -> (best or nothing, note)

Choose among branch candidates that passed the general post-solve checks
(`valid[i].core` holds `X`, `U` and `obj`). For a controller with an obstacle, candidates
that fail the obstacle acceptance rule (dense collision evaluation; see
[`obstacle_rejection`](@ref)) are discarded first. The lowest objective among the rest
wins. `note` lists the discarded candidates.
"""
function select_branch_candidate(ctrl::NMPCController, x, valid)
    isempty(valid) && return nothing, ""
    keep, notes = valid, String[]
    if ctrl.obstacle !== nothing
        keep = Any[]
        for v in valid
            st, msg = obstacle_rejection(ctrl, collision_check(ctrl, x, v.core.X, v.core.U))
            st === nothing ? push!(keep, v) : push!(notes, "segment $(v.segment) rejected by the obstacle check: $msg")
        end
    end
    note = isempty(notes) ? "" : " [" * join(notes, "; ") * "]"
    isempty(keep) && return nothing, note
    return keep[argmin([v.core.obj for v in keep])], note
end
