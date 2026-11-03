# Racing showcase pipeline (final milestone): configuration, model selection, export.
# The rendering itself (Python) is checked by `render_animation.py --selftest`.

module ShowcaseTest
using Test
include(joinpath(@__DIR__, "..", "examples", "showcase", "common.jl"))

@testset "shared initial physical condition" begin
    ic = initial_condition()
    zr = rear_axle_pose(DYN_PARAMS, ic.dynamic_state)
    Xr, Yr, psi = frenet_to_cartesian(TRACK, ic.kinematic_state[1], ic.kinematic_state[2], ic.kinematic_state[3])
    @test collect(zr) ≈ [Xr, Yr, psi] atol = 1e-12                          # same physical pose
    @test ic.dynamic_state[4] == ic.kinematic_state[4] == ic.speed           # same longitudinal speed
    @test ic.dynamic_state[5] == 0 && ic.dynamic_state[6] == 0               # no lateral speed, no yaw rate
    @test DYN_PARAMS.lf + DYN_PARAMS.lr ≈ reference_test_vehicle().wheelbase
    @test DYN_PARAMS.delta_max ≈ reference_test_vehicle().delta_max && DYN_PARAMS.a_max == reference_test_vehicle().a_max
    dc, kc = dynamic_config(), kinematic_config()
    @test dc.horizon == kc.horizon == HORIZON && dc.dt == kc.dt == DT
    @test (dc.ws, dc.qy, dc.qpsi, dc.rdelta, dc.ra, dc.rd_delta, dc.rd_a) == (kc.ws, kc.qy, kc.qpsi, kc.rdelta, kc.ra, kc.rd_delta, kc.rd_a)
end

@testset "obstacle layout and feasibility checks" begin
    obs = showcase_obstacles()
    @test length(obs) >= 5 && isempty(check_layout(obs))
    ss = [cartesian_to_frenet(TRACK, o.x, o.y) for o in obs]
    kap = [curvature(TRACK, s[1]) for s in ss]
    @test any(==(0.0), kap) && any(>(0.0), kap) && any(<(0.0), kap)          # straight, positive and negative curvature
    @test any(s -> s[2] > 0.05, ss) && any(s -> s[2] < -0.05, ss)              # offsets to both sides
    @test any(abs(ss[i][1] - ss[j][1]) < 2.0 for i in eachindex(ss), j in eachindex(ss) if i < j)   # two within one horizon
    @test !isempty(check_layout([obstacle_on_track(TRACK, 25.0, 0.0; radius=0.30)]))                # blocked
    @test !isempty(check_layout([obstacle_on_track(TRACK, 25.0, 0.0; radius=0.10), obstacle_on_track(TRACK, 25.15, 0.0; radius=0.10)]))
end

@testset "model selection changes the dynamics; unsupported combination" begin
    rd = run_showcase(:dynamic, CircleObstacle[], 12)
    rk = run_showcase(:kinematic, CircleObstacle[], 12)
    @test rd.ctrl isa DynamicNMPCController && rk.ctrl isa NMPCController
    @test rd.cl isa DynamicClosedLoopResult && rk.cl isa ClosedLoopResult
    @test length(rd.cl.states[1]) == 6 && length(rk.cl.states[1]) == 4
    @test succeeded(rd.cl) && succeeded(rk.cl)
    pd, pk = plant_samples(rd), plant_samples(rk)
    @test hypot(pd[end].X - pk[end].X, pd[end].Y - pk[end].Y) > 1e-3            # different vehicles, different motion
    @test_throws ErrorException run_showcase(:kinematic, showcase_obstacles(), 5)
    @test_throws ErrorException run_showcase(:point_mass, CircleObstacle[], 5)
    global SHOWCASE_RUNS = (rd, rk)
end

@testset "export: data correspond to the run; failures never appear accepted; fresh output" begin
    rd, rk = SHOWCASE_RUNS
    dir = mktempdir()
    opts = (; model=:dynamic, video=false, cycles=12, out=dir, python="python", fps=30)
    for run in (rd, rk)
        st = run_statistics(run)
        prefix = export_run("t_$(run.model)", run, st, opts)
        rows = readlines(prefix * "_cycles.csv")
        @test length(rows) - 1 == length(run.cl.controller) == st.cycles
        hdr = split(rows[1], ",")
        ix, iacc, ist, isol, ims = (findfirst(==(k), hdr) for k in ("X", "accepted", "status", "solver", "total_ms"))
        first = split(rows[2], ",")
        @test parse(Float64, first[ix]) ≈ (run.model === :dynamic ? run.cl.states[1][1] : run.cl.poses[1][1])
        for (j, r) in enumerate(run.cl.controller)
            f = split(rows[j+1], ",")
            @test f[iacc] == string(r.accepted) && f[ist] == string(r.status)
            @test parse(Float64, f[ims]) ≈ 1e3 * run.cycle_times[j]
            @test run.cycle_times[j] >= r.timing.total
        end
        traj = readlines(prefix * "_trajectory.csv")
        @test length(traj) - 1 == length(run.cl.fine_states)
        meta = read(prefix * "_meta.json", String)
        @test occursin("\"model\": \"$(run.model)\"", meta) && occursin("\"initial_condition\"", meta)
        @test occursin("\"cycle_timing_definition\": \"wall time around the complete solve! call", meta)
    end
    # A failing run: the rejected cycle is exported as rejected, without a control.
    veh = DynamicBicycle(DYN_PARAMS)
    ic = initial_condition()
    cf = DynamicNMPCController(veh, TRACK, DynamicNMPCConfig(max_iter=1))
    cycle_times = Float64[]
    clf = simulate_dynamic_closed_loop(cf, veh, ic.dynamic_state, 5; s0=ic.dynamic_s0, cycle_times)
    runf = (; model=:dynamic, ctrl=cf, cl=clf, cycle_times, compile=(construction=0.0, first_solve=0.0), obstacles=CircleObstacle[])
    stf = run_statistics(runf)
    @test stf.status === :controller_failure && stf.failed >= 1
    @test length(cycle_times) == length(clf.controller) == 1
    @test cycle_times[1] >= clf.controller[1].timing.total
    prefix = export_run("t_fail", runf, stf, opts)
    rows = readlines(prefix * "_cycles.csv")
    last = split(rows[end], ",")
    hdr = split(rows[1], ",")
    @test last[findfirst(==("accepted"), hdr)] == "false" && last[findfirst(==("delta"), hdr)] == "NaN"
    @test occursin("\"failure\": {", read(prefix * "_meta.json", String))
    # Fresh output: re-exporting a different run replaces the files.
    t1 = mtime(prefix * "_cycles.csv")
    sleep(1.1)
    export_run("t_fail", rd, run_statistics(rd), opts)
    @test mtime(prefix * "_cycles.csv") > t1
    @test length(readlines(prefix * "_cycles.csv")) - 1 == length(rd.cl.controller)
end
end # module
