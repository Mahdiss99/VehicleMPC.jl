# Shared configuration, simulation, export and rendering driver of the two racing showcases
# (examples/showcase/race.jl and multiple_obstacles.jl). Every run solves the MPC afresh;
# nothing is loaded from earlier runs. See examples/showcase/README.md.

using VehicleMPC
using Printf
include(joinpath(@__DIR__, "..", "example_common.jl"))           # quantile_of (Milestone 5 helper)

const SHOWCASE_DIR = @__DIR__

# ---------------------------------------------------------------- command-line options
"""
    showcase_options(default_cycles) -> NamedTuple

- `--model=dynamic` (default) or `--model=kinematic`;
- `--video` (default) / `--no-video`;
- `--cycles=N`;
- `--out=DIR` (default `examples/showcase/output`);
- `--python=PATH`: interpreter for the renderer (default: environment variable
  `VEHICLEMPC_PYTHON`, else `python`);
- `--fps=30`;
- `--time-budget=S`: a wall-clock budget [s] for extra attempts and reassignments (default:
  none, see `TIME_BUDGET`); `--no-time-budget` restores the default.
"""
function showcase_options(default_cycles::Integer)
    model, video, cycles, out, fps = :dynamic, true, default_cycles, joinpath(SHOWCASE_DIR, "output"), 30
    python = get(ENV, "VEHICLEMPC_PYTHON", "python")
    for a in ARGS
        if startswith(a, "--model=")
            model = Symbol(split(a, "=")[2])
        elseif a == "--video"
            video = true
        elseif a == "--no-video"
            video = false
        elseif startswith(a, "--cycles=")
            cycles = parse(Int, split(a, "=")[2])
        elseif startswith(a, "--out=")
            out = String(split(a, "="; limit=2)[2])
        elseif startswith(a, "--python=")
            python = String(split(a, "="; limit=2)[2])
        elseif startswith(a, "--fps=")
            fps = parse(Int, split(a, "=")[2])
        elseif a == "--no-time-budget"
            BUDGET[] = Inf
        elseif startswith(a, "--time-budget=")
            BUDGET[] = parse(Float64, split(a, "=")[2])
        else
            error("unknown option $a (use --model=dynamic|kinematic, --video/--no-video, --cycles=N, --out=DIR, --python=PATH, --fps=N, --time-budget=S, --no-time-budget)")
        end
    end
    model in (:dynamic, :kinematic) || error("unsupported --model=$model: the supported vehicle models are dynamic and kinematic")
    return (; model, video, cycles, out, python, fps)
end

# ---------------------------------------------------------------- configuration
const TRACK = quarter_track()                                    # original geometry, 0.70 m wide
const DYN_PARAMS = showcase_dynamic_parameters()                  # synthetic; see docs/dynamic_bicycle_mpc.md
const HORIZON, DT = 10, 0.1
const EGO_RADIUS, BUFFER, OBSTACLE_RADIUS = 0.05, 0.03, 0.10
# Wall-clock budget for extra initialisation attempts and reassignments [s]. The recorded
# showcases use none (Inf): the work per cycle is bounded by count (at most
# max_reassignments = 4 re-solves per attempt and at most five attempts). Retry decisions
# therefore do not use wall-clock speed. Computation time is measured and reported. With a
# 0.3 s budget, a slow state of the development laptop stopped the multi-obstacle run at
# cycle 43 before the alternative initialisations could be tried. `--time-budget=S` sets one.
const TIME_BUDGET = Inf

"""
Obstacle layout of the multi-obstacle showcase: `(name, s, ey)` on `quarter_track()`
(progress [m], lateral offset [m], left positive). The centres are computed from the real
track geometry with `obstacle_on_track`. All radii are $(OBSTACLE_RADIUS) m.
"""
const OBSTACLE_LAYOUT = [
    ("O1 R=+1 m left bend, offset left", 3.60, 0.06),
    ("O2 R=-1 m right bend, offset right", 6.75, -0.06),
    ("O3 R=2 m bend, centred", 14.90, 0.00),
    ("O4 4 m straight, offset right", 18.60, -0.08),
    ("O5 6 m straight, offset left", 25.40, 0.10),
    ("O6 6 m straight, offset right", 27.20, -0.10),
]
showcase_obstacles() = [obstacle_on_track(TRACK, s, ey; radius=OBSTACLE_RADIUS) for (_, s, ey) in OBSTACLE_LAYOUT]

"""
Shared initial physical condition of both showcases:
- the rear axle on the start line on the centreline (`s = 0`, `ey = 0`), heading along the
  track;
- 1.0 m/s longitudinal speed, no lateral speed, no yaw rate.

For the dynamic model this is the CoM state, `lr` ahead of the rear axle. For the
kinematic model it is the rear-axle Frenet state.
"""
function initial_condition()
    Xr, Yr, psi = frenet_to_cartesian(TRACK, 0.0, 0.0, 0.0)
    Xg, Yg, _ = cg_pose_from_rear_axle(DYN_PARAMS, Xr, Yr, psi)
    return (; rear_axle=(X=Xr, Y=Yr, psi), speed=1.0, lateral_speed=0.0, yaw_rate=0.0,
        dynamic_state=[Xg, Yg, psi, 1.0, 0.0, 0.0], dynamic_s0=DYN_PARAMS.lr, kinematic_state=[0.0, 0.0, 0.0, 1.0])
end

const BUDGET = Ref(TIME_BUDGET)                                  # --time-budget=S
dynamic_config() = DynamicNMPCConfig(; horizon=HORIZON, dt=DT, ego_radius=EGO_RADIUS, obstacle_buffer=BUFFER, time_budget=BUDGET[])
kinematic_config() = NMPCConfig(; prediction_model=:cartesian_endpoint_exact, horizon=HORIZON, dt=DT, schedule_corrections=8,
    correction_time_budget=BUDGET[], ego_radius=EGO_RADIUS, obstacle_buffer=BUFFER)

"""
    check_layout(obstacles) -> Vector{String}

Geometric feasibility checks of an obstacle layout on `TRACK`. It returns problems; an
empty vector means none were found.
- **Passage.** At least one side has a lateral passage `ey_o ± D` within the corridor
  `±E`.
- **Neighbouring sections.** No other part of the track (more than `2(D + E)` away in
  progress) passes within `E + D` of the obstacle centre.
- **Spacing.** The obstacles do not overlap each other's required separation.
"""
function check_layout(obstacles)
    E = TRACK.width / 2 - 0.05
    L = track_length(TRACK)
    out = String[]
    for (i, o) in enumerate(obstacles)
        D = EGO_RADIUS + o.radius + BUFFER
        so, eyo, _ = cartesian_to_frenet(TRACK, o.x, o.y)
        (eyo + D <= E || eyo - D >= -E) || push!(out, "obstacle $i blocks both sides (ey = $(eyo), D = $(D), E = $(E))")
        for s in range(0, L; length=3500)
            ds = abs(rem(s - so, L, RoundNearest))
            ds > 2(D + E) || continue
            pc = centerline_position(TRACK, s)
            d = hypot(pc[1] - o.x, pc[2] - o.y)
            d > E + D || (push!(out, "obstacle $i is $(round(d; digits=3)) m from the centreline at s = $(round(s; digits=2)) (another section)"); break)
        end
        for j in i+1:length(obstacles)
            q = obstacles[j]
            hypot(o.x - q.x, o.y - q.y) > o.radius + q.radius + 2 * (EGO_RADIUS + BUFFER) ||
                push!(out, "obstacles $i and $j leave no ego passage between them")
        end
    end
    return out
end

# ---------------------------------------------------------------- simulation
"""
    run_showcase(model, obstacles, cycles) -> NamedTuple

Build the controller for the selected `model`, warm it up with discarded runs (compilation
excluded from the statistics, as in Milestone 5), and run the closed loop for `cycles`
control periods from [`initial_condition`](@ref).

| model | controller | plant |
|---|---|---|
| `:dynamic` | `DynamicNMPCController` (six-state dynamic bicycle) | fine RK4 integration of the same model |
| `:kinematic` | the existing `NMPCController`, Cartesian-endpoint mode | the event-aware kinematic plant |

The kinematic NMPC supports one obstacle at most. Asking for the kinematic model with
several obstacles is therefore an error, never a silent fallback.
"""
function run_showcase(model::Symbol, obstacles, cycles::Integer)
    ic = initial_condition()
    if model === :dynamic
        veh = DynamicBicycle(DYN_PARAMS)
        make = () -> DynamicNMPCController(veh, TRACK, dynamic_config(); obstacles)
        tc = @elapsed (first_ctrl = make())
        tf = @elapsed solve!(first_ctrl, ic.dynamic_state; s_hint=ic.dynamic_s0)
        for _ in 1:5                                              # warm-up runs (discarded)
            succeeded(simulate_dynamic_closed_loop(make(), veh, ic.dynamic_state, min(cycles, 150); s0=ic.dynamic_s0)) && break
        end
        ctrl = make()
        cycle_times = Float64[]
        cl = simulate_dynamic_closed_loop(ctrl, veh, ic.dynamic_state, cycles; s0=ic.dynamic_s0, cycle_times)
        return (; model, ctrl, cl, cycle_times, compile=(construction=tc, first_solve=tf), obstacles)
    elseif model === :kinematic
        length(obstacles) <= 1 || error("unsupported combination: the kinematic NMPCController handles at most one stationary obstacle; " *
                                        "the multi-obstacle showcase is implemented for --model=dynamic only")
        veh = KinematicBicycle(reference_test_vehicle())
        make = () -> isempty(obstacles) ? NMPCController(veh, TRACK, kinematic_config()) :
                     NMPCController(veh, TRACK, kinematic_config(); obstacle=obstacles[1])
        tc = @elapsed (first_ctrl = make())
        tf = @elapsed solve!(first_ctrl, ic.kinematic_state)
        for _ in 1:5
            succeeded(simulate_closed_loop(make(), veh, ic.kinematic_state, min(cycles, 150))) && break
        end
        ctrl = make()
        cycle_times = Float64[]
        cl = simulate_closed_loop(ctrl, veh, ic.kinematic_state, cycles; cycle_times)
        return (; model, ctrl, cl, cycle_times, compile=(construction=tc, first_solve=tf), obstacles)
    end
    error("unsupported --model=$model")
end

# ---------------------------------------------------------------- unified per-cycle records
"Measured full-call times, with a labelled fallback for manually constructed historical runs."
function cycle_seconds(run)
    times = hasproperty(run, :cycle_times) ? run.cycle_times : [r.timing.total for r in run.cl.controller]
    length(times) == length(run.cl.controller) || error("one timing sample per controller call required")
    return times
end
timing_definition(run) = hasproperty(run, :cycle_times) ? "wall time around the complete solve! call; excludes plant integration, export and rendering" :
    "controller-reported internal timer"

"Per-cycle records (control instant, pose, controls, solver data) for either model."
function cycle_records(run)
    cl, rec = run.cl, NamedTuple[]
    times = cycle_seconds(run)
    for (k, r) in enumerate(cl.controller)
        if run.model === :dynamic
            z, f = cl.states[k], cl.frenet[k]
            pose = (X=z[1], Y=z[2], psi=z[3], vx=z[4], vy=z[5], r=z[6], s=f[1], ey=f[2], epsi=f[3])
            plan = r.accepted ? r.path : zeros(2, 0)
            push!(rec, (; k, t=cl.t[k], pose..., delta=r.accepted ? r.u[1] : NaN, a=r.accepted ? r.u[2] : NaN, status=r.status,
                accepted=r.accepted, solver=r.termination_status, iterations=r.iterations, solves=r.nlp_solves,
                total_ms=1e3 * times[k], solve_ms=1e3 * r.timing.solve, plan, message=r.message))
        else
            x = cl.states[k]
            X, Y, psi = cl.poses[k]
            plan = r.accepted ? reduce(hcat, [collect(frenet_to_cartesian(TRACK, r.X[1, j], r.X[2, j], r.X[3, j])[1:2]) for j in axes(r.X, 2)]) : zeros(2, 0)
            delta = r.accepted ? r.u[1] : NaN
            push!(rec, (; k, t=cl.t[k], X, Y, psi, vx=x[4], vy=0.0, r=x[4] * tan(isnan(delta) ? 0.0 : delta) / run.ctrl.vehicle.params.wheelbase,
                s=x[1], ey=x[2], epsi=x[3], delta, a=r.accepted ? r.u[2] : NaN, status=r.status, accepted=r.accepted,
                solver=Symbol(r.termination_status), iterations=r.iterations, solves=r.nlp_solves, total_ms=1e3 * times[k],
                solve_ms=1e3 * r.timing.optimize, plan, message=r.message))
        end
    end
    return rec
end

"Dense plant trajectory `(t, X, Y, psi, vx, vy, r, s, ey, epsi)` of either model."
function plant_samples(run)
    cl = run.cl
    if run.model === :dynamic
        return [(t=cl.fine_t[j], X=z[1], Y=z[2], psi=z[3], vx=z[4], vy=z[5], r=z[6], s=f[1], ey=f[2], epsi=f[3])
                for (j, (z, f)) in enumerate(zip(cl.fine_states, cl.fine_frenet))]
    end
    out = NamedTuple[]
    for (j, x) in enumerate(cl.fine_states)
        X, Y, psi = frenet_to_cartesian(TRACK, x[1], x[2], x[3])
        push!(out, (t=cl.fine_t[j], X, Y, psi, vx=x[4], vy=0.0, r=NaN, s=x[1], ey=x[2], epsi=x[3]))
    end
    return out
end

"Reference point of the collision proxy: the CoM (dynamic) or the rear axle (kinematic)."
proxy_point(run, p) = (p.X, p.Y)

# ---------------------------------------------------------------- statistics
"""
    run_statistics(run) -> NamedTuple

Statistics of the run, all computed from its own data:
- **Outcome:** status, accepted and failed cycles.
- **Progress:** laps and simulated time.
- **Computation:** cycle time (median, p95, p99, max; cycles over 80 and 100 ms; share
  within 100 ms), Ipopt iterations, NLP solves.
- **Track:** minimum physical track clearance and edge violations, both from the dense
  plant trajectory.
- **Obstacles:** minimum clearance to contact and to the required separation, collisions,
  and the number of buffer intrusions.
"""
function run_statistics(run)
    cl = run.cl
    rs = cl.controller
    tt = 1e3 .* cycle_seconds(run)
    its = [r.iterations for r in rs]
    sv = [r.nlp_solves for r in rs]
    ps = plant_samples(run)
    L, W2 = track_length(TRACK), TRACK.width / 2
    s0 = ps[1].s
    maxey = maximum(abs(p.ey) for p in ps)
    obs = run.obstacles
    mincon, minbuf, ncol, nbufsamples = Inf, Inf, 0, 0
    for p in ps, o in obs
        d = hypot(p.X - o.x, p.Y - o.y)
        Dc, D = EGO_RADIUS + o.radius, EGO_RADIUS + o.radius + BUFFER
        mincon, minbuf = min(mincon, d - Dc), min(minbuf, d - D)
        d < Dc && (ncol += 1)
        d < D && (nbufsamples += 1)
    end
    acc = count(r -> r.accepted, rs)
    return (; status=cl.status, message=cl.message, cycles=length(rs), accepted=acc, failed=length(rs) - acc,
        simulated_time=cl.t[end], progress=ps[end].s - s0, laps=(ps[end].s - s0) / L,
        median_ms=quantile_of(tt, 0.5), p95_ms=quantile_of(tt, 0.95), p99_ms=quantile_of(tt, 0.99), max_ms=maximum(tt),
        over_80=count(>(80), tt), over_100=count(>(100), tt), within_100_pct=100 * count(<=(100), tt) / length(tt),
        iterations_median=quantile_of(its, 0.5), iterations_max=maximum(its), solves_mean=sum(sv) / length(sv), solves_max=maximum(sv),
        min_track_clearance=W2 - maxey, track_edge_violations=count(p -> abs(p.ey) > W2, ps),
        obstacles=length(obs), min_contact_clearance=isempty(obs) ? NaN : mincon, min_buffer_clearance=isempty(obs) ? NaN : minbuf,
        collisions=ncol, buffer_intrusion_samples=nbufsamples, plant_violations=length(cl.violations))
end

# ---------------------------------------------------------------- export
jsonval(x::AbstractString) = "\"" * replace(x, "\\" => "\\\\", "\"" => "\\\"", "\n" => " ") * "\""
jsonval(x::Symbol) = jsonval(string(x))
jsonval(x::Bool) = x ? "true" : "false"
jsonval(x::Integer) = string(x)
jsonval(x::Real) = isfinite(x) ? repr(Float64(x)) : "null"
jsonval(::Nothing) = "null"
jsonval(x::Union{AbstractVector,Tuple}) = "[" * join(jsonval.(x), ", ") * "]"
jsonval(x::NamedTuple) = "{" * join(["\"$k\": $(jsonval(v))" for (k, v) in pairs(x)], ", ") * "}"
jsonval(x::AbstractDict) = "{" * join(["\"$k\": $(jsonval(v))" for (k, v) in x], ", ") * "}"
config_fields(c) = NamedTuple{fieldnames(typeof(c))}(Tuple(f isa Union{Real,Symbol,AbstractString,Bool,Nothing} ? f : string(f) for f in (getfield(c, n) for n in fieldnames(typeof(c)))))

"Short commit hash of the code that produced the run; `-dirty` if the package or examples differ from it."
function git_commit()
    try
        h = readchomp(pipeline(`git -C $(SHOWCASE_DIR) rev-parse --short HEAD`; stderr=devnull))
        changes = readchomp(pipeline(`git -C $(SHOWCASE_DIR) status --porcelain -- ../../src ../../examples ../../Project.toml`; stderr=devnull))
        return isempty(changes) ? h : h * "-dirty"
    catch
        return "unknown"
    end
end

"""
    export_run(name, run, stats, opts) -> prefix

Write the data of this run to `<out>/<name>_*`:
- `meta.json`: model, parameters, controller configuration, initial condition, obstacles,
  statistics, compilation times and versions;
- `cycles.csv`: one row per control cycle, including the solver status, acceptance and
  timing;
- `plans.csv`: the accepted predicted path of every cycle;
- `trajectory.csv`: the dense plant trajectory (every 10 ms);
- `track.csv`: the centreline and edges.

The renderer reads only these files.
"""
function export_run(name, run, stats, opts)
    mkpath(opts.out)
    prefix = joinpath(opts.out, name)
    rec = cycle_records(run)
    open(prefix * "_cycles.csv", "w") do io
        println(io, "k,t,X,Y,psi,vx,vy,r,s,ey,epsi,delta,a,status,accepted,solver,iterations,solves,total_ms,solve_ms")
        for r in rec
            println(io, join((r.k, r.t, r.X, r.Y, r.psi, r.vx, r.vy, r.r, r.s, r.ey, r.epsi, r.delta, r.a, r.status, r.accepted, r.solver,
                r.iterations, r.solves, r.total_ms, r.solve_ms), ","))
        end
    end
    open(prefix * "_plans.csv", "w") do io
        println(io, "k,j,x,y")
        for r in rec, j in axes(r.plan, 2)
            println(io, join((r.k, j, r.plan[1, j], r.plan[2, j]), ","))
        end
    end
    open(prefix * "_trajectory.csv", "w") do io
        println(io, "t,X,Y,psi,vx,vy,r,s,ey,epsi")
        for p in plant_samples(run)
            println(io, join((p.t, p.X, p.Y, p.psi, p.vx, p.vy, p.r, p.s, p.ey, p.epsi), ","))
        end
    end
    open(prefix * "_track.csv", "w") do io
        println(io, "s,xc,yc,xl,yl,xr,yr")
        W2 = TRACK.width / 2
        for s in range(0, track_length(TRACK); length=1400)
            c, l, r = frenet_to_cartesian(TRACK, s, 0.0), frenet_to_cartesian(TRACK, s, W2), frenet_to_cartesian(TRACK, s, -W2)
            println(io, join((s, c[1], c[2], l[1], l[2], r[1], r[2]), ","))
        end
    end
    ic = initial_condition()
    obs = [(name=OBSTACLE_LAYOUT[i][1], s=OBSTACLE_LAYOUT[i][2], ey=OBSTACLE_LAYOUT[i][3], x=o.x, y=o.y, radius=o.radius, buffer=BUFFER,
        required_separation=EGO_RADIUS + o.radius + BUFFER) for (i, o) in enumerate(run.obstacles)]
    failed_cycle = findfirst(r -> !r.accepted, run.cl.controller)
    meta = (; name, model=run.model,
        model_description=run.model === :dynamic ? "6-state dynamic bicycle (CoM, linear tyres)" : "4-state kinematic bicycle (rear axle)",
        controller="Nonlinear MPC, JuMP + Ipopt",
        parameters=run.model === :dynamic ? config_fields(DYN_PARAMS) : config_fields(reference_test_vehicle()),
        controller_config=config_fields(run.ctrl.config), horizon=HORIZON, dt=DT, track=(name="quarter_track", length=track_length(TRACK),
            width=TRACK.width, corridor_half_width=TRACK.width / 2 - 0.05),
        collision_proxy=(reference=run.model === :dynamic ? "centre of mass" : "rear axle", ego_radius=EGO_RADIUS),
        initial_condition=(rear_axle=ic.rear_axle, speed=ic.speed, lateral_speed=ic.lateral_speed, yaw_rate=ic.yaw_rate,
            dynamic_state=ic.dynamic_state, kinematic_state=ic.kinematic_state),
        obstacles=obs, statistics=stats, failure=failed_cycle === nothing ? nothing :
                                                 (cycle=failed_cycle, status=run.cl.controller[failed_cycle].status, message=run.cl.message),
        compile_seconds=run.compile, cycle_timing_definition=timing_definition(run), julia=string(VERSION), git_commit=git_commit(),
        body_illustrative=(length=0.34, width=0.15))
    write(prefix * "_meta.json", jsonval(meta))
    return prefix
end

# ---------------------------------------------------------------- report and render
function print_report(title, stats)
    println("== ", title)
    println("status: ", stats.status, isempty(stats.message) ? "" : "  ($(stats.message))")
    @printf("cycles: %d (%d accepted, %d failed); simulated %.1f s; progress %.2f m = %.2f laps\n", stats.cycles, stats.accepted,
        stats.failed, stats.simulated_time, stats.progress, stats.laps)
    @printf("MPC cycle time [ms]: median %.1f, p95 %.1f, p99 %.1f, max %.1f; > 80 ms: %d, > 100 ms: %d (%.1f %% within 100 ms)\n",
        stats.median_ms, stats.p95_ms, stats.p99_ms, stats.max_ms, stats.over_80, stats.over_100, stats.within_100_pct)
    @printf("Ipopt iterations median %.0f, max %d; NLP solves per cycle mean %.2f, max %d\n", stats.iterations_median,
        stats.iterations_max, stats.solves_mean, stats.solves_max)
    @printf("minimum physical track clearance %.4f m; track-edge violations %d; plant violations %d\n", stats.min_track_clearance,
        stats.track_edge_violations, stats.plant_violations)
    stats.obstacles > 0 && @printf("obstacles %d: minimum clearance above contact %.4f m, above the required separation %.4f m; collisions %d; buffer-intrusion samples %d\n",
        stats.obstacles, stats.min_contact_clearance, stats.min_buffer_clearance, stats.collisions, stats.buffer_intrusion_samples)
end

"Render `<prefix>.mp4` and `<prefix>_poster.png` from this run's exported data with render_animation.py."
function render(prefix, opts)
    script = joinpath(SHOWCASE_DIR, "render_animation.py")
    cmd = `$(opts.python) $script --run $prefix --fps $(opts.fps)`
    println("rendering: ", cmd)
    run(cmd)
    return prefix * ".mp4", prefix * "_poster.png"
end

"Run, export, report and (optionally) render one showcase; exit with status 1 if the closed loop failed."
function showcase_main(name, title, obstacles, opts)
    run = run_showcase(opts.model, obstacles, opts.cycles)
    stats = run_statistics(run)
    print_report(title * " ($(opts.model) model)", stats)
    prefix = export_run("$(name)_$(opts.model)", run, stats, opts)
    println("wrote ", prefix, "_{meta.json,cycles.csv,plans.csv,trajectory.csv,track.csv}")
    if opts.video
        mp4, png = render(prefix, opts)
        println("wrote ", mp4, " and ", png)
    end
    stats.status === :success || (println("the closed loop did not complete; the exported data and video show the failure"); exit(1))
    return run, stats
end
