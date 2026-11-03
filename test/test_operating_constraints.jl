# Operating-constraint diagnostics: detection, separation from the Frenet/model domain,
# no mutation, and no effect on the default open-loop simulation.

const CMODEL = KinematicBicycle(reference_test_vehicle())
const CP = CMODEL.params
const CTRACK = circular_track(2.0; width=0.7)          # half-width 0.35 m

kinds(vs) = [v.constraint for v in vs]

@testset "feasible state and input" begin
    @test isempty(check_operating_constraints(CMODEL, CTRACK, [1.0, 0.1, 0.2, 2.0], [0.1, 1.0]))
    @test satisfies_operating_constraints(CMODEL, CTRACK, [1.0, 0.1, 0.2, 2.0])
    # Limits are inclusive.
    @test isempty(check_operating_constraints(CMODEL, CTRACK, [0.0, 0.35, 0.0, CP.v_max], [CP.delta_max, CP.a_max]))
    @test isempty(check_operating_constraints(CMODEL, CTRACK, [0.0, -0.35, 0.0, CP.v_min], [-CP.delta_max, CP.a_min]))
end

@testset "track boundary" begin
    vs = check_operating_constraints(CMODEL, CTRACK, [0.0, 0.4, 0.0, 1.0])
    @test kinds(vs) == [:track_boundary]
    @test vs[1].value == 0.4 && vs[1].limit == 0.35
    @test occursin("left", vs[1].message)
    vs = check_operating_constraints(CMODEL, CTRACK, [0.0, -0.5, 0.0, 1.0])
    @test kinds(vs) == [:track_boundary]
    @test vs[1].limit == -0.35
    @test occursin("right", vs[1].message)
    # Width 0 means no boundary is defined.
    @test isempty(check_operating_constraints(CMODEL, circular_track(2.0), [0.0, 1.5, 0.0, 1.0]))
end

@testset "speed limits" begin
    vs = check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, CP.v_max + 0.1])
    @test kinds(vs) == [:speed_max]
    @test vs[1].limit == CP.v_max
    slow = KinematicBicycle(VehicleParameters(wheelbase=0.25, delta_max=0.4, a_min=-5, a_max=5, v_min=0.5, v_max=5))
    vs = check_operating_constraints(slow, CTRACK, [0.0, 0.0, 0.0, 0.2])
    @test kinds(vs) == [:speed_min]
    @test vs[1].value == 0.2 && vs[1].limit == 0.5
end

@testset "input limits" begin
    @test kinds(check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, 1.0], [CP.delta_max + 0.01, 0.0])) == [:steering]
    @test kinds(check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, 1.0], [-CP.delta_max - 0.01, 0.0])) == [:steering]
    @test kinds(check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, 1.0], [0.0, CP.a_min - 0.1])) == [:acceleration_min]
    @test kinds(check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, 1.0], [0.0, CP.a_max + 0.1])) == [:acceleration_max]
    # Consistent with the actuator check used by the simulator.
    for u in ([0.1, 1.0], [CP.delta_max + 0.01, 0.0], [0.0, CP.a_max + 0.1])
        @test within_input_limits(CP, u) == isempty(check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, 1.0], u))
    end
end

@testset "all violations are reported together" begin
    vs = check_operating_constraints(CMODEL, CTRACK, [0.0, 0.6, 0.0, 6.0], [0.6, -7.0])
    @test Set(kinds(vs)) == Set([:track_boundary, :speed_max, :steering, :acceleration_min])
    @test all(v -> !isempty(v.message), vs)
end

@testset "no clamping or mutation" begin
    x, u = [0.0, 0.6, 0.1, 6.0], [0.6, -7.0]
    x0, u0 = copy(x), copy(u)
    check_operating_constraints(CMODEL, CTRACK, x, u)
    @test x == x0 && u == u0
end

@testset "operating constraints are distinct from the Frenet/model domain" begin
    tight = circular_track(1.0; width=0.7)
    # ey = 0.95 on a radius-1 left turn: outside the track AND inside the Frenet exclusion zone.
    x = [0.0, 0.95, 0.0, 1.0]
    @test kinds(check_operating_constraints(CMODEL, tight, x)) == [:track_boundary]
    @test_throws DomainError check_domain(CMODEL, x, [0.0, 0.0], curvature(tight, 0.0))
    # Off the track but still a valid Frenet state: diagnosed, and the model evaluates fine.
    x = [0.0, -0.8, 0.0, 1.0]
    @test kinds(check_operating_constraints(CMODEL, tight, x)) == [:track_boundary]
    @test all(isfinite, continuous_dynamics(CMODEL, x, [0.0, 0.0], curvature(tight, 0.0)))
    # Reverse speed is a model-domain error in check_domain, but only a speed_min violation here.
    @test kinds(check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, -0.1])) == [:speed_min]
    @test_throws DomainError check_domain(CMODEL, [0.0, 0.0, 0.0, -0.1], [0.0, 0.0], 0.0)
    # Non-finite values are invalid, not operating violations.
    @test_throws DomainError check_operating_constraints(CMODEL, CTRACK, [NaN, 0.0, 0.0, 1.0])
    @test_throws DomainError check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, 1.0], [Inf, 0.0])
    @test_throws DimensionMismatch check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 1.0])
    @test_throws DimensionMismatch check_operating_constraints(CMODEL, CTRACK, [0.0, 0.0, 0.0, 1.0], [0.0])
end

@testset "open-loop example leaves the track; the simulation is not stopped" begin
    example = Module(:OpenLoopExampleConstraints)
    Base.include(example, joinpath(@__DIR__, "..", "examples", "kinematic_open_loop.jl"))
    res = Base.invokelatest(example.run_open_loop_example)
    track = Base.invokelatest(example.example_track)
    @test succeeded(res)
    @test length(res.states) == 121                      # all 120 steps were simulated
    viol = trajectory_violations(CMODEL, track, res)
    @test !isempty(viol)
    @test all(p -> p.second.constraint === :track_boundary, viol)
    # Exactly the recorded states with |ey| > width/2 are flagged.
    @test first.(viol) == findall(x -> abs(x[2]) > track.width / 2, res.states)
end
