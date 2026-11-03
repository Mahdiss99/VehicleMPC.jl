"""
    TrackSegment(length, curvature)

Constant-curvature piece of the centreline. `curvature > 0` turns left (counter-clockwise),
`curvature < 0` turns right, `curvature == 0` is a straight line. A single segment may not
span more than one full revolution.
"""
struct TrackSegment{T<:AbstractFloat}
    length::T
    curvature::T

    function TrackSegment{T}(length, curvature) where {T<:AbstractFloat}
        (isfinite(length) && length > 0) || throw(ArgumentError("segment length must be positive and finite, got $length"))
        isfinite(curvature) || throw(ArgumentError("segment curvature must be finite, got $curvature"))
        abs(curvature) * length <= 2π * (1 + 1e-12) ||
            throw(ArgumentError("a single segment may not turn more than 2π (|κ|ℓ = $(abs(curvature) * length))"))
        return new{T}(length, curvature)
    end
end

TrackSegment(length::Real, curvature::Real) = TrackSegment{Float64}(length, curvature)

"""
    Track(segments; start_pose=(0.0, 0.0, 0.0), width=0.0, closed=false, closure_tol=1e-6)

Piecewise constant-curvature centreline built from `segments`, starting at
`start_pose = (x, y, psi)`. When `closed = true` the constructor verifies that the end pose
matches the start pose (position within `closure_tol`, heading modulo 2π) and progress then
wraps periodically. On an open track progress outside `[0, track_length]` is an error.

Segment start headings are stored unwrapped (cumulative turning), so the centreline heading
is a continuous function of progress.
"""
struct Track{T<:AbstractFloat}
    segments::Vector{TrackSegment{T}}
    s_start::Vector{T}            # cumulative progress at each segment start; s_start[end] = length
    poses::Vector{NTuple{3,T}}    # centreline pose (x, y, psi) at each s_start
    width::T
    closed::Bool
end

function Track(segments::AbstractVector{<:TrackSegment}; start_pose=(0.0, 0.0, 0.0), width=0.0,
    closed::Bool=false, closure_tol=1e-6)
    isempty(segments) && throw(ArgumentError("a track needs at least one segment"))
    (isfinite(width) && width >= 0) || throw(ArgumentError("track width must be nonnegative and finite, got $width"))
    all(isfinite, start_pose) || throw(ArgumentError("start pose must be finite, got $start_pose"))
    segs = collect(TrackSegment{Float64}, segments)
    s_start = zeros(length(segs) + 1)
    poses = Vector{NTuple{3,Float64}}(undef, length(segs) + 1)
    poses[1] = Float64.(Tuple(start_pose))
    for (i, seg) in enumerate(segs)
        s_start[i+1] = s_start[i] + seg.length
        poses[i+1] = segment_pose(seg, poses[i], seg.length)
    end
    if closed
        (x0, y0, p0), (x1, y1, p1) = poses[1], poses[end]
        gap = hypot(x1 - x0, y1 - y0)
        gap <= closure_tol || throw(ArgumentError("closed=true but centreline end is $gap m from its start"))
        abs(rem2pi(p1 - p0, RoundNearest)) <= closure_tol ||
            throw(ArgumentError("closed=true but end heading differs from start heading by $(rem2pi(p1 - p0, RoundNearest)) rad"))
    end
    return Track{Float64}(segs, s_start, poses, width, closed)
end

"Pose `(x, y, psi)` at distance `d ∈ [0, length]` along `seg` from start pose `pose0`."
function segment_pose(seg::TrackSegment, pose0, d)
    x0, y0, psi0 = pose0
    kappa = seg.curvature
    half = kappa * d / 2
    chord = kappa == 0 ? d : 2 * sin(half) / kappa   # straight-line distance start → point
    return (x0 + chord * cos(psi0 + half), y0 + chord * sin(psi0 + half), psi0 + 2half)
end

track_length(tr::Track) = tr.s_start[end]
is_closed(tr::Track) = tr.closed
num_segments(tr::Track) = length(tr.segments)

"Net centreline heading change over one traversal of the track [rad] (2π·k for a closed track)."
total_turning(tr::Track) = tr.poses[end][3] - tr.poses[1][3]

"""
    split_progress(track, s) -> (lap, s_wrapped)

Decompose continuous (unwrapped) progress `s` into a lap index and a position
`s_wrapped ∈ [0, track_length)` on a closed track. On an open track `lap == 0` and `s` must
lie in `[0, track_length]` (up to a tolerance of `1e-9·max(1, L)`), otherwise a `DomainError`
is thrown. The state vector always carries unwrapped progress; only geometry look-ups wrap.
"""
function split_progress(tr::Track, s::Real)
    isfinite(s) || throw(DomainError(s, "progress must be finite"))
    L = track_length(tr)
    if tr.closed
        sw = mod(s, L)
        lap = round(Int, (s - sw) / L)
        if sw >= L        # mod can round up to L for tiny negative s
            sw, lap = zero(sw), lap + 1
        end
        return lap, sw
    end
    tol = 1e-9 * max(1, L)
    -tol <= s <= L + tol || throw(DomainError(s, "progress outside the open track [0, $L]"))
    return 0, clamp(s, zero(s), L)
end

wrap_progress(tr::Track, s::Real) = split_progress(tr, s)[2]

"Index of the segment containing wrapped position `s_wrapped`; segments are half-open `[s_i, s_{i+1})`."
segment_index(tr::Track, s_wrapped::Real) = clamp(searchsortedlast(tr.s_start, s_wrapped), 1, num_segments(tr))

"Signed curvature [1/m] at progress `s` (unwrapped progress is accepted on closed tracks)."
curvature(tr::Track, s::Real) = tr.segments[segment_index(tr, wrap_progress(tr, s))].curvature

"""
    centerline_pose(track, s) -> (x, y, psi)

Centreline position and heading at progress `s`. The heading is continuous in `s`: on a
closed track it increases by `total_turning(track)` per lap.
"""
function centerline_pose(tr::Track, s::Real)
    lap, sw = split_progress(tr, s)
    i = segment_index(tr, sw)
    x, y, psi = segment_pose(tr.segments[i], tr.poses[i], sw - tr.s_start[i])
    return (x, y, psi + lap * total_turning(tr))
end

centerline_position(tr::Track, s::Real) = centerline_pose(tr, s)[1:2]
centerline_heading(tr::Track, s::Real) = centerline_pose(tr, s)[3]

"""
    frenet_to_cartesian(track, s, ey, epsi=0) -> (X, Y, psi)

Map Frenet coordinates to a Cartesian pose. `ey` is measured along the left normal of the
centreline and `psi = psi_track(s) + epsi`. Throws a `DomainError` where the Frenet frame is
singular (`1 - kappa*ey ≤ 0`).
"""
function frenet_to_cartesian(tr::Track, s::Real, ey::Real, epsi::Real=0.0)
    (isfinite(ey) && isfinite(epsi)) || throw(DomainError((ey, epsi), "ey and epsi must be finite"))
    kappa = curvature(tr, s)
    1 - kappa * ey > 0 || throw(DomainError(ey, "Frenet frame is singular: 1 - kappa*ey = $(1 - kappa * ey) ≤ 0"))
    x, y, psi = centerline_pose(tr, s)
    return (x - ey * sin(psi), y + ey * cos(psi), psi + epsi)
end

"""
    cartesian_to_frenet(track, X, Y, psi=0; s_hint=nothing, max_lateral=width/2, tol=1e-9) -> (s, ey, epsi)

Project a Cartesian pose onto the centreline. Every segment is searched for an orthogonal
projection with `|ey| ≤ max_lateral`. Without `s_hint` the projection must be unique,
otherwise an `ArgumentError` reports the ambiguity; with `s_hint` the candidate closest in
progress to the hint is returned and, on a closed track, `s` is unwrapped to the lap nearest
the hint. Without a hint `s ∈ [0, track_length)`. `epsi` is wrapped to `(-π, π]`.
Throws an `ArgumentError` when no projection exists — it never returns a placeholder.
"""
function cartesian_to_frenet(tr::Track, X::Real, Y::Real, psi::Real=0.0;
    s_hint::Union{Nothing,Real}=nothing, max_lateral::Real=(tr.width > 0 ? tr.width / 2 : Inf), tol::Real=1e-9)
    all(isfinite, (X, Y, psi)) || throw(DomainError((X, Y, psi), "Cartesian pose must be finite"))
    L = track_length(tr)
    candidates = Tuple{Float64,Float64}[]
    for (i, seg) in enumerate(tr.segments)
        proj = project_onto_segment(seg, tr.poses[i], X, Y, tol)
        proj === nothing && continue
        d, ey = proj
        abs(ey) <= max_lateral + tol || continue
        s = tr.s_start[i] + d
        tr.closed && s >= L - tol && (s = zero(s))
        # Adjacent segments share their boundary point; keep one copy.
        any(c -> abs(c[1] - s) <= 10tol && abs(c[2] - ey) <= 10tol, candidates) || push!(candidates, (s, ey))
    end
    isempty(candidates) && throw(ArgumentError("no centreline projection with |ey| ≤ $max_lateral for point ($X, $Y)"))

    if s_hint === nothing
        length(candidates) == 1 ||
            throw(ArgumentError("ambiguous projection: $(length(candidates)) centreline points within |ey| ≤ $max_lateral; pass s_hint"))
        s, ey = candidates[1]
    else
        sdist = c -> tr.closed ? abs(rem(c[1] - s_hint, L, RoundNearest)) : abs(c[1] - s_hint)
        s, ey = candidates[argmin(sdist.(candidates))]
        tr.closed && (s = s_hint + rem(s - s_hint, L, RoundNearest))
    end
    epsi = rem2pi(psi - centerline_heading(tr, s), RoundNearest)
    return (s, ey, epsi)
end

"Orthogonal projection `(d, ey)` of `(X, Y)` onto one segment, or `nothing` if it falls outside."
function project_onto_segment(seg::TrackSegment, pose0, X, Y, tol)
    x0, y0, psi0 = pose0
    kappa, len = seg.curvature, seg.length
    if kappa == 0
        dx, dy = X - x0, Y - y0
        d = dx * cos(psi0) + dy * sin(psi0)
        ey = -dx * sin(psi0) + dy * cos(psi0)
        -tol <= d <= len + tol || return nothing
        return (clamp(d, zero(d), len), ey)
    end
    # Centre of curvature lies at signed distance 1/kappa along the left normal.
    xc, yc = x0 - sin(psi0) / kappa, y0 + cos(psi0) / kappa
    rho = hypot(X - xc, Y - yc)
    rho > 0 || return nothing
    ey = 1 / kappa - sign(kappa) * rho
    a0 = atan(y0 - yc, x0 - xc)
    swept = mod(sign(kappa) * (atan(Y - yc, X - xc) - a0), 2π)   # angle travelled in the turning direction
    d = swept / abs(kappa)
    if d > len + tol
        full = 2π / abs(kappa)
        full - d <= tol || return nothing
        d = zero(d)                                                # just before the start point
    end
    return (clamp(d, zero(d), len), ey)
end

"Straight open track of the given length starting at `start_pose`."
straight_track(len::Real; width=0.0, start_pose=(0.0, 0.0, 0.0)) =
    Track([TrackSegment(len, 0.0)]; start_pose, width, closed=false)

"""
    circular_track(radius; direction=1, width=0.0, start_pose=(0.0, 0.0, 0.0))

Closed circle of the given radius; `direction = 1` is counter-clockwise (κ = +1/R),
`direction = -1` clockwise (κ = -1/R).
"""
function circular_track(radius::Real; direction::Integer=1, width=0.0, start_pose=(0.0, 0.0, 0.0))
    radius > 0 || throw(ArgumentError("radius must be positive, got $radius"))
    direction in (-1, 1) || throw(ArgumentError("direction must be ±1, got $direction"))
    return Track([TrackSegment(2π * radius, direction / radius)]; start_pose, width, closed=true)
end

"""
Track segment lists copied verbatim from `Environment.jl`, in the legacy
`[length, signed radius]` format (radius 0 ⇒ straight). Widths are the legacy defaults.
"""
const LEGACY_TRACK_SPECS = (
    quarter=(segments=[[2.0, 0.0], [π * 1, 1], [π * 1, -1], [π / 2 * 2, 2], [2.0, 0.0], [π / 2 * 2, 2],
            [4.0, 0.0], [π / 2 * 2, 2], [6.0, 0.0], [π / 2 * 2, 2], [2.0, 0.0]], width=0.7),
    circular=(segments=[[2 * π * 10, 10]], width=0.7),
    rectangular=(segments=[[1.0, 0.0], [π / 2 * 0.5, 0.5], [2.0, 0.0], [π / 2 * 0.5, 0.5], [2.0, 0.0],
            [π / 2 * 0.5, 0.5], [2.0, 0.0], [π / 2 * 0.5, 0.5], [1.0, 0.0]], width=0.7),
    lab=(segments=[[1.6, 0.0], [π / 2 * 0.66, 0.66], [1.95, 0.0], [π / 2 * 0.66, 0.66], [0.6, 0.0],
            [π / 6 * 0.7, 0.7], [0.41, 0.0], [π / 6 * 0.7, -0.7], [1.08, 0.0], [π * 1.440, 1.440], [1.08, 0.0]], width=0.7),
)

"Convert a legacy `[length, signed radius]` entry (radius 0 ⇒ straight) to a `TrackSegment`."
legacy_segment(spec) = TrackSegment(spec[1], spec[2] == 0 ? 0.0 : 1 / spec[2])

"""
    legacy_track(name; width=<legacy default>, closed=<auto>)

Build one of the legacy tracks (`:quarter`, `:circular`, `:rectangular`, `:lab`) from
`LEGACY_TRACK_SPECS` without altering its geometry. By default the track is marked closed
only if its end pose actually matches its start pose within 1e-6.
"""
function legacy_track(name::Symbol; width=getproperty(LEGACY_TRACK_SPECS, name).width, closed=nothing)
    segs = legacy_segment.(getproperty(LEGACY_TRACK_SPECS, name).segments)
    if closed === nothing
        open_track = Track(segs; width)
        (x0, y0, p0), (x1, y1, p1) = open_track.poses[1], open_track.poses[end]
        closed = hypot(x1 - x0, y1 - y0) <= 1e-6 && abs(rem2pi(p1 - p0, RoundNearest)) <= 1e-6
        closed || return open_track
    end
    return Track(segs; width, closed)
end

"The legacy quarter track (`Quarter_trackoption` in `Environment.jl`)."
quarter_track(; width=LEGACY_TRACK_SPECS.quarter.width) = legacy_track(:quarter; width)
