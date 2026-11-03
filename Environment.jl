# module environment
# export AbstractTrack, Rectangular_trackoption, Lab_trackoption,
#     build_track_keypoint, plot_track, get_curvature, local_to_global, global_to_local
# using PyPlot
using LinearAlgebra
using Parameters
abstract type AbstractTrack end
# Define the track key points in curvilinear coordinates
@with_kw mutable struct Quarter_trackoption <: AbstractTrack
    # Options
    "Start point."
    init_pos::Vector{Float64} = [0.0, 0.0, 0.0]

    "Width of the track."
    track_width::Float64 = 0.7

    "Track directions."
    track_segs::Vector{Vector{Float64}} = [[2.0, 0.0],#[direction, radious]
        [π * 1, 1],
        [π * 1, -1],
        [π / 2 * 2, 2],
        [2.0, 0.0],
        [π / 2 * 2, 2],
        [4.0, 0.0],
        [π / 2 * 2, 2],
        [6.0, 0.0],
        [π / 2 * 2, 2],
        [2.0, 0.0]]

end

@with_kw mutable struct Circular_trackoption <: AbstractTrack
    # Options
    "Start point."
    init_pos::Vector{Float64} = [0.0, 0.0, 0.0]

    "Width of the track."
    track_width::Float64 = 0.7

    "Track directions."
    track_segs::Vector{Vector{Float64}} = [#[direction, radious]
        [2 * π * 10, 10]]
end

@with_kw mutable struct Rectangular_trackoption <: AbstractTrack
    # Options
    "Start point."
    init_pos::Vector{Float64} = [0.0, 0.0, 0.0]

    "Width of the track."
    track_width::Float64 = 0.7

    "Track directions."
    track_segs::Vector{Vector{Float64}} = [[1.0, 0.0],#[direction, radious]
        [π / 2 * 0.5, 0.5],
        [2.0, 0.0],
        [π / 2 * 0.5, 0.5],
        [2.0, 0.0],
        [π / 2 * 0.5, 0.5],
        [2.0, 0.0],
        [π / 2 * 0.5, 0.5],
        [1.0, 0.0]]

end

@with_kw mutable struct Lab_trackoption <: AbstractTrack
    # Options
    "Start point."
    init_pos::Vector{Float64} = [0, 0, 0]

    "Width of the track."
    track_width::Float64 = 0.7

    "Track directions."
    track_segs::Vector{Vector{Float64}} = [[1.6, 0.0],
        [π / 2 * 0.66, 0.66],
        [1.95, 0.0],
        [π / 2 * 0.66, 0.66],
        [0.6, 0.0],
        [π / 6 * 0.7, 0.7],
        [0.41, 0.0],
        [π / 6 * 0.7, -0.7],
        [1.08, 0.0],
        [π * 1.440, 1.440],
        [1.08, 0.0]]

end
# key_pts = [x, y, psi, cumulative length, segment length, signed curvature]
function build_track_keypoint(opts::AbstractTrack)
    num_seg = size(opts.track_segs)[1]
    track_key_pts = [zeros(Float64, 6) for _ in 1:(num_seg+1)]
    track_key_pts[1][1] = opts.init_pos[1]
    track_key_pts[1][2] = opts.init_pos[2]
    track_key_pts[1][3] = opts.init_pos[3]
    for i in 2:(num_seg+1)
        x_prev = track_key_pts[i-1][1]
        y_prev = track_key_pts[i-1][2]
        psi_prev = track_key_pts[i-1][3]
        cum_s_prev = track_key_pts[i-1][4]

        l_seg = opts.track_segs[i-1][1]
        r_seg = opts.track_segs[i-1][2]

        if r_seg == 0.0
            # No curvature (i.e. straight line)
            psi = psi_prev
            x = x_prev + l_seg * cos(psi_prev)
            y = y_prev + l_seg * sin(psi_prev)
            curvature = 0
        else
            dir = sign(r_seg)

            # Find coordinates for center of curved segment
            x_c = x_prev + abs(r_seg) * cos(psi_prev + dir * π / 2)
            y_c = y_prev + abs(r_seg) * sin(psi_prev + dir * π / 2)

            # Angle spanned by curved segment
            span_ang = l_seg / abs(r_seg)

            # Angle of the tangent vector at the end of the segment
            psi = wrap_angle(psi_prev + dir * span_ang)

            ang_norm = wrap_angle(psi_prev + dir * π / 2)
            sign_norm = sign(ang_norm)
            if sign_norm == 0
                sign_norm = 1
            end
            ang = -sign_norm * (π - abs(ang_norm))
            x = x_c + abs(r_seg) * cos(ang + dir * span_ang)
            y = y_c + abs(r_seg) * sin(ang + dir * span_ang)
            curvature = 1 / r_seg
        end
        cum_s = cum_s_prev + l_seg
        track_key_pts[i] = [x, y, psi, cum_s, l_seg, curvature]
    end
    return track_key_pts
end


function plot_track(track_key_pts, track_width, pts_per_unit::Int=20)
    # Create a blank figure and axes
    # fig, ax = subplots()

    # Plot the starting line
    init_x = track_key_pts[1][1]
    init_y = track_key_pts[1][2]
    init_psi = track_key_pts[1][3]
    start_line_x = [init_x + cos(init_psi + π / 2),
        init_x - cos(init_psi + π / 2)]
    start_line_y = [init_y + sin(init_psi + π / 2),
        init_y - sin(init_psi + π / 2)]
    # ax.plot(start_line_x, start_line_y, "r", linewidth=1)

    # Plot the track and boundaries
    x_track = []
    x_bound_in = []
    x_bound_out = []
    y_track = []
    y_bound_in = []
    y_bound_out = []
    for i in 2:size(track_key_pts, 1)
        l_seg = track_key_pts[i][5]
        cum_s = track_key_pts[i-1][4]
        n_pts = round(Int, l_seg * pts_per_unit)
        d = range(0, stop=l_seg, length=n_pts)
        for j in d
            cl_coord = (j + cum_s, 0.0, 0.0)
            xy_coord = local_to_global(track_key_pts, cl_coord)
            push!(x_track, xy_coord[1])
            push!(y_track, xy_coord[2])

            cl_coord = (j + cum_s, track_width / 2, 0.0)
            xy_coord = local_to_global(track_key_pts, cl_coord)
            push!(x_bound_in, xy_coord[1])
            push!(y_bound_in, xy_coord[2])
            cl_coord = (j + cum_s, -track_width / 2, 0.0)
            xy_coord = local_to_global(track_key_pts, cl_coord)
            push!(x_bound_out, xy_coord[1])
            push!(y_bound_out, xy_coord[2])
        end
    end
    # ax.plot(x_track, y_track, "k--", linewidth=1)
    # ax.plot(x_bound_in, y_bound_in, "k")
    # ax.plot(x_bound_out, y_bound_out, "k")


    # return ax
    return x_track, y_track, x_bound_in, y_bound_in, x_bound_out, y_bound_out, start_line_x, start_line_y

end

function global_to_local(track_key_pts, xy_coord, track_width)
    #error if any of inputs are Nonlinea

    #assinging the global cordinate of the point to variables
    x = xy_coord[1]
    y = xy_coord[2]
    psi = xy_coord[3]

    #current position in x,y coordinate
    pos_cur = [x, y]
    cl_coord = (0, 0, 0)
    for i in 1:size(track_key_pts, 1)
        # key_pts = [x, y, psi, cumulative length, segment length, signed curvature]
        x_s = track_key_pts[i][1]
        y_s = track_key_pts[i][2]
        psi_s = track_key_pts[i][3]
        curve_s = track_key_pts[i][6]
        x_f = track_key_pts[i+1][1]
        y_f = track_key_pts[i+1][2]
        psi_f = track_key_pts[i+1][3]
        curve_f = track_key_pts[i+1][6]

        l = track_key_pts[i+1][5]

        pos_s = [x_s, y_s]
        pos_f = [x_f, y_f]

        # Check if at any of the segment start or end points
        if norm(pos_s - pos_cur) == 0
            # At start of segment
            s = track_key_pts[i][4]
            e_y = 0
            e_psi = unwrap!([psi_s, psi])[2] - psi_s
            cl_coord = (s, e_y, e_psi)
            break
        end
        if norm(pos_f - pos_cur) == 0
            # At end of segment
            s = track_key_pts[i+1][4]
            e_y = 0
            e_psi = unwrap!([psi_f, psi])[2] - psi_f
            cl_coord = (s, e_y, e_psi)
            break
        end
        if curve_f == 0
            # Check if on straight segment
            if abs(compute_angle(pos_s, pos_cur, pos_f)) <= π / 2 && abs(compute_angle(pos_f, pos_cur, pos_s)) <= π / 2
                v = pos_cur - pos_s
                ang = compute_angle(pos_s, pos_f, pos_cur)
                e_y = norm(v) * sin(ang)
                # Check if deviation from centerline is within track width 
                if abs(e_y) <= track_width / 2
                    d = norm(v) * cos(ang)
                    s = track_key_pts[i][4] + d
                    e_psi = unwrap!([psi_s, psi])[2] - psi_s
                    cl_coord = (s, e_y, e_psi)
                    break
                else
                    continue
                end
            else
                continue
            end
        else
            # Check if on curved segment
            r = 1 / curve_f
            dir = sign(r)

            # Find coordinates for center of curved segment
            x_c = x_s + abs(r) * cos(psi_s + dir * π / 2)
            y_c = y_s + abs(r) * sin(psi_s + dir * π / 2)
            curve_center = [x_c, y_c]
            span_ang = l / r
            cur_ang = compute_angle(curve_center, pos_s, pos_cur)
            if sign(span_ang) == sign(cur_ang) && abs(span_ang) >= abs(cur_ang)
                v = pos_cur - curve_center
                e_y = -sign(dir) * (norm(v) - abs(r))
                # Check if deviation from centerline is within track width
                if abs(e_y) <= track_width / 2
                    d = abs(cur_ang) * abs(r)
                    s = track_key_pts[i][4] + d
                    e_psi = unwrap!([psi_s + cur_ang, psi])[2] - (psi_s + cur_ang)
                    cl_coord = (s, e_y, e_psi)
                    break
                else
                    continue
                end
            else
                continue
            end

        end
    end

    return cl_coord
end

function local_to_global(track_key_pts, cl_coord)
    s = mod(cl_coord[1], track_key_pts[end][4])
    e_y = cl_coord[2]
    e_psi = cl_coord[3]

    seg_idx = searchsortedlast([i[4] for i in track_key_pts], s)


    x_s = track_key_pts[seg_idx][1]
    y_s = track_key_pts[seg_idx][2]
    psi_s = track_key_pts[seg_idx][3]
    curve_s = track_key_pts[seg_idx][6]
    x_f = track_key_pts[seg_idx+1][1]
    y_f = track_key_pts[seg_idx+1][2]
    psi_f = track_key_pts[seg_idx+1][3]
    curve_f = track_key_pts[seg_idx+1][6]

    l = track_key_pts[seg_idx+1][5]
    d = s - track_key_pts[seg_idx][4] # Distance along current segment

    if curve_f == 0.0
        # Segment is a straight line
        x = x_s + (x_f - x_s) * d / l + e_y * cos(psi_f + π / 2)
        y = y_s + (y_f - y_s) * d / l + e_y * sin(psi_f + π / 2)
        psi = wrap_angle(psi_f + e_psi)
    else
        r = 1.0 / curve_f
        dir = sign(r)

        # Find coordinates for center of curved segment
        x_c = x_s + abs(r) * cos(psi_s + dir * π / 2)
        y_c = y_s + abs(r) * sin(psi_s + dir * π / 2)

        # Angle spanned up to current location along segment
        span_ang = d / abs(r)

        # Angle of the tangent vector at the current location
        psi_d = wrap_angle(psi_s + dir * span_ang)

        ang_norm = wrap_angle(psi_s + dir * π / 2)

        if ang_norm == 0
            ang_norm = ang_norm + 1e-20
        end
        ang = -sign(ang_norm) * (π - abs(ang_norm))

        x = x_c + (abs(r) - dir * e_y) * cos(ang + dir * span_ang)
        y = y_c + (abs(r) - dir * e_y) * sin(ang + dir * span_ang)
        psi = wrap_angle(psi_d + e_psi)
    end

    return [x, y, psi]

end

function wrap_angle(theta)

    if theta <= -π
        wrapped_angle = 2 * π + theta
    elseif theta >= π
        wrapped_angle = theta - 2 * π
    else
        wrapped_angle = theta
    end

    return wrapped_angle
end


function unwrap!(p)
    length(p) < 2 && return p
    for i = 2:length(p)
        d = p[i] - p[i-1]
        if abs(d) > π
            p[i] -= floor((d + π) / (2π)) * 2π
        end
    end
    return p
end

function compute_angle(point_0, point_1, point_2)
    v_1 = point_1 - point_0
    v_2 = point_2 - point_0

    dot_product = dot(v_1, v_2)
    det = v_1[1] * v_2[2] - v_1[2] * v_2[1]
    theta = atan(det, dot_product)

    return theta
end

function customized_mod(x, y)
    r = rem(x, y)
    if r == 0
        copysign(r, y)
    elseif (r > 0) ⊻ (y > 0)
        r + y
    else
        r
    end
end

function get_curvature(track_key_pts, s)
    """Find key point indicies corresponding to current segment
    key_pts = [x, y, psi, cumulative length, segment length, signed curvature]"""
    # print("real s: ", s, "\n")
    s = mod(s, track_key_pts[end][4])
    # print("s: ", s, "\n")
    seg_idx = searchsortedlast([i[4] for i in track_key_pts], s)
    # print("seg_idx: ", seg_idx, "\n")
    # print("trakc: ", track_key_pts, "\n")
    # print("trakc length: ", length(track_key_pts), "\n")
    if seg_idx < length(track_key_pts)
        return track_key_pts[seg_idx+1][6]
    else
        return track_key_pts[1][6]
    end

end

# end