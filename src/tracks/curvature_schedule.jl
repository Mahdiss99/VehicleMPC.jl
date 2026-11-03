# Curvature averaging and stagewise curvature schedules for prediction models.
# The physical curvature function of the track is not changed or smoothed.

"""
Default threshold [m] below which `average_curvature` switches from the heading-difference
quotient to a one-sided local limit.
"""
const AVERAGE_CURVATURE_DS_MIN = 1e-6

"""
    segment_boundaries(track, s0, s1) -> Vector{Float64}

Unwrapped progress values of all segment boundaries in the half-open interval
`(min(s0,s1), max(s0,s1)]`, in increasing order. Interior boundaries only: on an open track
the ends `0` and `L` are not included. On a closed track the lap boundary (`k·L`) is a
boundary only if the curvature actually changes there; boundaries where neighbouring
segments have equal curvature are included as well, since they are segment boundaries.
"""
function segment_boundaries(tr::Track, s0::Real, s1::Real)
    lo, hi = minmax(Float64(s0), Float64(s1))
    L, n = track_length(tr), num_segments(tr)
    out = Float64[]
    if tr.closed
        n == 1 && return out                          # single closed segment (circle): no boundary
        for lap in floor(Int, lo / L):floor(Int, hi / L)
            for j in 1:n                              # boundary at the start of segment j
                b = lap * L + tr.s_start[j]
                lo < b <= hi && push!(out, b)
            end
        end
    else
        split_progress(tr, lo); split_progress(tr, hi) # range check (throws outside [0, L])
        for j in 2:n
            b = tr.s_start[j]
            lo < b <= hi && push!(out, b)
        end
    end
    return sort!(out)
end

"""
    curvature_left(track, s) -> Float64

Left limit `lim_{σ↑s} kappa(σ)`. Away from boundaries it equals `curvature(track, s)`; at a
boundary it is the curvature of the segment that ends there. On an open track there is no
left limit at `s = 0`, so a `DomainError` is thrown.
"""
function curvature_left(tr::Track, s::Real)
    lap, sw = split_progress(tr, s)
    j = searchsortedfirst(tr.s_start, sw) - 1        # segment j covers (s_start[j], s_start[j+1]]
    if j < 1
        tr.closed || throw(DomainError(s, "no left limit of the curvature at the start of an open track"))
        j = num_segments(tr)
    end
    return tr.segments[min(j, num_segments(tr))].curvature
end

"""
    average_curvature(track, s0, s1; ds_min=1e-6) -> Float64

Average curvature of the centreline between unwrapped progress values `s0` and `s1`:

    kappa_avg = (psi_c(s1) - psi_c(s0)) / (s1 - s0)

with the continuous, unwrapped centreline heading `psi_c` (`centerline_heading`). Because
`psi_c' = kappa`, this equals `(1/(s1-s0)) ∫ kappa(σ) dσ` exactly, whether the interval
lies inside one segment or crosses one or several boundaries, including closed-track lap
boundaries. The result is symmetric (`s1 < s0` gives the same value).

For `|s1 - s0| < ds_min` the quotient would divide by a nearly zero length. A documented
local limit is used instead, the one-sided limit at `s0` in the direction of travel:

- `s1 > s0`: `curvature(track, s0)` (right limit);
- `s1 < s0`: `curvature_left(track, s0)` (left limit);
- `s1 == s0`: the curvature at `s0`. If `s0` lies exactly on a boundary where the two
  one-sided limits differ, the limit is ambiguous and an `ArgumentError` is thrown.

When `|s1 - s0| < ds_min` straddles a boundary, the one-sided convention changes the
heading predicted over that tiny interval by at most `|Δkappa|·ds_min`.
"""
function average_curvature(tr::Track, s0::Real, s1::Real; ds_min::Real=AVERAGE_CURVATURE_DS_MIN)
    (isfinite(s0) && isfinite(s1)) || throw(DomainError((s0, s1), "progress must be finite"))
    ds_min > 0 || throw(ArgumentError("ds_min must be positive"))
    d = s1 - s0
    abs(d) >= ds_min && return (centerline_heading(tr, s1) - centerline_heading(tr, s0)) / d
    d > 0 && return curvature(tr, s0)
    d < 0 && return curvature_left(tr, s0)
    kr = curvature(tr, s0)
    kl = (!tr.closed && wrap_progress(tr, s0) == 0) ? kr : curvature_left(tr, s0)
    kl == kr || throw(ArgumentError("average curvature over a zero-length interval at the boundary s = $s0 is ambiguous (left $kl, right $kr)"))
    return kr
end

"""
    curvature_schedule(track, s_nominal; method=:average, ds_min=1e-6) -> Vector{Float64}

Stagewise curvature for a prediction with `N = length(s_nominal) - 1` stages, from the
nominal predicted progress `s_nominal[1:N+1]` (unwrapped):

- `method = :average`: `kappa_k = average_curvature(track, s_nominal[k], s_nominal[k+1])`;
- `method = :start`: `kappa_k = curvature(track, s_nominal[k])` (the start-of-step value).

The result is meant for the NMPC's curvature `Parameter`s
(`set_parameter_value(ctrl.kappa[k], kappa_k)`). The active controller does not use it
yet.
"""
function curvature_schedule(tr::Track, s_nominal::AbstractVector; method::Symbol=:average, ds_min::Real=AVERAGE_CURVATURE_DS_MIN)
    length(s_nominal) >= 2 || throw(ArgumentError("need at least two nominal progress values"))
    method in (:average, :start) || throw(ArgumentError("method must be :average or :start, got $method"))
    N = length(s_nominal) - 1
    return method === :start ? [curvature(tr, s_nominal[k]) for k in 1:N] :
           [average_curvature(tr, s_nominal[k], s_nominal[k+1]; ds_min) for k in 1:N]
end
