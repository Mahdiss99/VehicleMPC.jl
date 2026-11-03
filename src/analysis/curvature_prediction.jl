# Prediction models that differ only in how the piecewise-constant track curvature enters a
# discrete-time prediction. Used to quantify curvature-approximation error (Milestone 2B.1).
# None of these is used by the active NMPC.

"""
Result of a prediction rollout: the predicted Frenet states at the knots (`N+1` columns
when `status === :success`, fewer if the rollout stopped), the curvature used on each
stage, and a status (`:success` or `:domain_error`).
"""
struct PredictionResult
    states::Vector{SVector{4,Float64}}
    kappas::Vector{Float64}
    status::Symbol
    message::String
end

succeeded(r::PredictionResult) = r.status === :success

function rollout(model, x0, U, dt, stage_step)
    xs = [SVector{4,Float64}(x0...)]
    kappas = Float64[]
    for k in axes(U, 2)
        try
            x, kap = stage_step(k, xs[end], SVector{2,Float64}(U[1, k], U[2, k]))
            push!(xs, x)
            push!(kappas, kap)
        catch err
            err isa DomainError || rethrow()
            return PredictionResult(xs, kappas, :domain_error, "stage $k: " * sprint(showerror, err))
        end
    end
    return PredictionResult(xs, kappas, :success, "")
end

"""
    predict_start_curvature(model, track, x0, U, dt; integrator=RK4()) -> PredictionResult

**Method A.** For each stage, the curvature is the value at the start of the stage, along
the prediction's own rollout: `kappa_k = curvature(track, s_k)`. This is what the
open-loop and closed-loop simulators used in Milestones 1–2A (zero-order hold).
"""
predict_start_curvature(model::KinematicBicycle, track::Track, x0, U::AbstractMatrix, dt; integrator=RK4()) =
    rollout(model, x0, U, dt, (k, x, u) -> (kap = curvature(track, x[1]); (discrete_dynamics(model, x, u, kap, dt, integrator), kap)))

"""
    predict_with_schedule(model, x0, U, dt, kappas; integrator=RK4()) -> PredictionResult

Rollout with a given stagewise curvature schedule (constant within each stage).
"""
predict_with_schedule(model::KinematicBicycle, x0, U::AbstractMatrix, dt, kappas::AbstractVector; integrator=RK4()) =
    rollout(model, x0, U, dt, (k, x, u) -> (discrete_dynamics(model, x, u, kappas[k], dt, integrator), Float64(kappas[k])))

"""
    predict_average_curvature(model, track, x0, U, dt, s_nominal; integrator=RK4()) -> PredictionResult

**Method B.** Stagewise average curvature `curvature_schedule(track, s_nominal; method=:average)`
from a nominal progress trajectory `s_nominal[1:N+1]`, held constant within each stage.
This structure is exactly what the JuMP model can represent with one curvature `Parameter`
per stage.
"""
predict_average_curvature(model::KinematicBicycle, track::Track, x0, U::AbstractMatrix, dt, s_nominal::AbstractVector; integrator=RK4()) =
    predict_with_schedule(model, x0, U, dt, curvature_schedule(track, s_nominal; method=:average); integrator)

"""
    predict_split_curvature(model, track, x0, U, dt, s_nominal; integrator=RK4(), crossing_times=nothing) -> PredictionResult

**Method D (event-split with fixed split times).** Every stage is split at the boundaries
that the *nominal* progress crosses in `(s_nominal[k], s_nominal[k+1]]`, and each sub-interval
uses the curvature of its segment. The split times are estimated by linear interpolation of
the nominal progress within the stage, unless `crossing_times[k]` (a vector of times within
stage `k`) is given. With exact crossing times (from the reference), this reproduces the
event-aware solution up to integration error. In a JuMP predictor the split times would be
parameters.
"""
function predict_split_curvature(model::KinematicBicycle, track::Track, x0, U::AbstractMatrix, dt, s_nominal::AbstractVector;
    integrator=RK4(), crossing_times=nothing)
    return rollout(model, x0, U, dt, function (k, x, u)
        sa, sb = s_nominal[k], s_nominal[k+1]
        bs = sb > sa ? segment_boundaries(track, sa, sb) : Float64[]
        taus = crossing_times === nothing ? [dt * (b - sa) / (sb - sa) for b in bs] : collect(Float64, crossing_times[k])
        length(taus) == length(bs) || throw(ArgumentError("stage $k: $(length(taus)) crossing times for $(length(bs)) nominal boundaries"))
        cuts = vcat(0.0, clamp.(taus, 0.0, dt), dt)
        # Curvature of each sub-interval: the segment between consecutive nominal boundaries.
        marks = vcat(sa, bs, sb)
        xx = x
        for j in 1:length(cuts)-1
            tau = cuts[j+1] - cuts[j]
            tau > 0 || continue
            kap = curvature(track, (marks[j] + marks[j+1]) / 2)
            xx = discrete_dynamics(model, xx, u, kap, tau, integrator)
        end
        return xx, average_curvature(track, sa, sb)
    end)
end

"""
    prediction_errors(track, pred, ref) -> NamedTuple

Knot-wise errors of a predicted Frenet trajectory against a reference (both vectors of
states; the shorter length is used). Fields:

- `position`: Cartesian position error [m];
- `heading`: absolute heading error `|psi_pred - psi_ref|` [rad];
- `s`, `ey`, `epsi`, `v`: absolute Frenet differences;
- `invalid`: number of predicted knots that lie outside the Frenet domain of the **actual**
  track, i.e. `1 - kappa(s)·ey ≤ 0`. This can happen when a prediction used a wrong
  curvature. For those knots `position` and `heading` are `NaN`; the pose is undefined
  there.
"""
function prediction_errors(track::Track, pred::AbstractVector, ref::AbstractVector)
    n = min(length(pred), length(ref))
    pos, head = zeros(n), zeros(n)
    invalid = 0
    for k in 1:n
        r = frenet_to_cartesian(track, ref[k][1], ref[k][2], ref[k][3])
        p = try
            frenet_to_cartesian(track, pred[k][1], pred[k][2], pred[k][3])
        catch err
            err isa DomainError || rethrow()
            nothing
        end
        if p === nothing
            invalid += 1
            pos[k] = head[k] = NaN
        else
            pos[k] = hypot(p[1] - r[1], p[2] - r[2])
            head[k] = abs(p[3] - r[3])
        end
    end
    d(i) = [abs(pred[k][i] - ref[k][i]) for k in 1:n]
    return (; position=pos, heading=head, s=d(1), ey=d(2), epsi=d(3), v=d(4), invalid)
end
