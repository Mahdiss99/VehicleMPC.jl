# Segment-local, smooth centreline formulas (Milestone 2B.4). Within one assigned segment
# the centreline pose is an analytic function of progress, so it can be traced into JuMP
# expressions without the discontinuous segment look-up. See docs/cartesian_endpoint_nmpc.md.

"""
    segment_local_pose(x0, y0, psi0, kappa, sigma, sincf=sinc_un) -> (x, y, psi)

Centreline pose at distance `sigma` from the start pose `(x0, y0, psi0)` of a segment with
constant curvature `kappa`:

    psi = psi0 + kappa·sigma
    (x, y) = (x0, y0) + sigma·sinc_un(kappa·sigma/2)·(cos, sin)(psi0 + kappa·sigma/2)

The same formula covers straight segments (`kappa = 0`) and both curvature signs, without
branching or division. It extends the segment's circle (or line) analytically beyond its
end points, so it equals the track geometry only for `sigma ∈ [0, segment length]`.
"""
function segment_local_pose(x0, y0, psi0, kappa, sigma, sincf=sinc_un)
    theta = kappa * sigma
    c = sigma * sincf(theta / 2)
    return (x0 + c * cos(psi0 + theta / 2), y0 + c * sin(psi0 + theta / 2), psi0 + theta)
end

"""
    frenet_pose_local(frame, s, ey, epsi, sincf=sinc_un) -> (X, Y, psi)

Cartesian pose of the Frenet coordinates `(s, ey, epsi)` using the assigned segment frame
`frame = (s0, x0, y0, psi0, kappa)` (see [`segment_frame`](@ref)). `ey` is measured along the
left normal, and `psi = psi_c(s) + epsi` is unwrapped.
"""
function frenet_pose_local(frame, s, ey, epsi, sincf=sinc_un)
    xc, yc, psic = segment_local_pose(frame.x0, frame.y0, frame.psi0, frame.kappa, s - frame.s0, sincf)
    return (xc - ey * sin(psic), yc + ey * cos(psic), psic + epsi)
end

"""
    segment_frame(track, id) -> (; s0, s1, x0, y0, psi0, kappa)

Constants of the unique segment `id = lap·n + i` (as returned by [`segment_id`](@ref)):
- unwrapped progress interval `[s0, s1]`;
- start position `(x0, y0)` (periodic, so it does not grow with the lap);
- unwrapped start heading `psi0 = psi_i + lap·total_turning`;
- curvature `kappa`.
"""
function segment_frame(track::Track, id::Integer)
    n, L = num_segments(track), track_length(track)
    lap, i = fld(id - 1, n), mod(id - 1, n) + 1
    x0, y0, psi = track.poses[i]
    return (; s0=lap * L + track.s_start[i], s1=lap * L + track.s_start[i+1], x0, y0, psi0=psi + lap * total_turning(track),
        kappa=track.segments[i].curvature)
end
