# Open-loop simulation: unwrapped progress, failure statuses, and the example script.

@testset "open-loop simulation keeps progress unwrapped" begin
    model = KinematicBicycle(reference_test_vehicle())
    R = 1.0
    tr = circular_track(R; width=0.5)
    L = track_length(tr)
    dt, N, v = 0.05, 200, 2.0
    delta = atan(model.params.wheelbase / R)
    res = simulate_open_loop(model, tr, [0.0, 0.0, 0.0, v], fill([delta, 0.0], N), dt)
    @test succeeded(res)
    @test length(res.states) == N + 1 == length(res.poses) == length(res.t)
    @test length(res.inputs) == N == length(res.curvature)
    s = [x[1] for x in res.states]
    @test s[end] ≈ v * N * dt rtol = 1e-12
    @test s[end] > 3L                                   # several laps, never wrapped
    @test all(diff(s) .> 0)
    @test all(==(1 / R), res.curvature)
    @test maximum(abs(hypot(p[1], p[2] - R) - R) for p in res.poses) <= 1e-9
end

@testset "open-loop failures are reported, not clamped" begin
    model = KinematicBicycle(reference_test_vehicle())
    tr = straight_track(5.0)
    # Actuator limit violated at step 3.
    inputs = [[0.0, 0.0], [0.0, 0.0], [1.0, 0.0]]
    res = simulate_open_loop(model, tr, [0.0, 0.0, 0.0, 1.0], inputs, 0.1)
    @test res.status === :input_limit
    @test length(res.states) == 3
    # Braking below zero speed.
    res = simulate_open_loop(model, tr, [0.0, 0.0, 0.0, 0.3], fill([0.0, -5.0], 5), 0.1)
    @test res.status === :domain_error
    @test all(x -> x[4] >= 0, res.states)
    # Driving off the end of an open track.
    res = simulate_open_loop(model, tr, [4.5, 0.0, 0.0, 2.0], fill([0.0, 0.0], 10), 0.1)
    @test res.status === :domain_error
    @test res.states[end][1] <= track_length(tr)
    # Invalid initial state or time step throws immediately.
    @test_throws DomainError simulate_open_loop(model, tr, [0.0, 0.0, 0.0, -1.0], [[0.0, 0.0]], 0.1)
    @test_throws ArgumentError simulate_open_loop(model, tr, [0.0, 0.0, 0.0, 1.0], [[0.0, 0.0]], 0.0)
end

@testset "open-loop example runs" begin
    example = Module(:OpenLoopExample)
    Base.include(example, joinpath(@__DIR__, "..", "examples", "kinematic_open_loop.jl"))
    res = Base.invokelatest(example.run_open_loop_example)
    @test succeeded(res)
    @test all(p -> all(isfinite, p), res.poses)
end
