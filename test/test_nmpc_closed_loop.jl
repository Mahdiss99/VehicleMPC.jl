# Closed-loop NMPC on the straight and circular tracks.

const CLV = KinematicBicycle(reference_test_vehicle())

"Plant lateral acceleration at every fine sample (end point of each substep, its own input)."
function plant_lateral_acceleration(cl, substeps)
    return [lateral_acceleration(CLV, cl.fine_states[j][4], cl.inputs[cld(j - 1, substeps)][1])
            for j in 2:length(cl.fine_states)]
end

@testset "closed loop: straight track" begin
    track = straight_track(120.0; width=0.7)
    ctrl = NMPCController(CLV, track, NMPCConfig())
    cl = simulate_closed_loop(ctrl, CLV, [0.0, 0.2, 0.1, 0.5], 80)
    @test succeeded(cl)
    @test length(cl.states) == 81 && all(r -> r.accepted, cl.controller)
    s = [x[1] for x in cl.states]
    @test all(diff(s) .> 0)                                       # forward progress
    @test cl.states[end][4] ≈ CLV.params.v_max atol = 1e-6        # speed regulated by the v_max bound
    @test maximum(x[4] for x in cl.fine_states) <= CLV.params.v_max + 1e-6
    @test abs(cl.states[end][2]) < 1e-3 && abs(cl.states[end][3]) < 1e-3   # settles on the centreline
    @test maximum(abs(x[2]) for x in cl.states) <= ctrl.ey_max + 1e-6
    @test maximum(abs(x[2]) for x in cl.fine_states) <= track.width / 2
    @test isempty(cl.violations)
    # Smooth inputs: bounded increments, and the controller stops accelerating at v_max.
    @test maximum(abs(cl.inputs[k+1][1] - cl.inputs[k][1]) for k in 1:length(cl.inputs)-1) < 0.1
    @test abs(cl.inputs[end][2]) < 1e-3
    @test all(r -> r.max_dynamics_residual <= 1e-6 && r.max_constraint_violation <= 1e-6, cl.controller)
    @test count(r -> r.warm_started, cl.controller) == 79           # all but the first solve
    @test s[end] < track_length(track)
end

@testset "closed loop: circular track (R = 1 m), full lap" begin
    track = circular_track(1.0; width=0.7)
    ctrl = NMPCController(CLV, track, NMPCConfig())
    cl = simulate_closed_loop(ctrl, CLV, [0.0, 0.0, 0.0, 0.5], 50)
    @test succeeded(cl)
    @test all(r -> r.accepted, cl.controller)
    @test cl.states[end][1] > track_length(track)                 # at least one complete lap (unwrapped s)
    @test all(diff([x[1] for x in cl.states]) .> 0)
    @test maximum(abs(x[2]) for x in cl.states) <= ctrl.ey_max + 1e-6
    @test maximum(abs(x[2]) for x in cl.fine_states) <= track.width / 2
    @test maximum(abs(x[3]) for x in cl.fine_states) <= deg2rad(60)
    @test maximum(abs, plant_lateral_acceleration(cl, 10)) <= ctrl.config.ay_max + 1e-6
    @test isempty(cl.violations)
    @test all(r -> r.max_dynamics_residual <= 1e-6 && r.max_constraint_violation <= 1e-6, cl.controller)
    # Steering near the curvature feed-forward once the speed has settled.
    @test abs(cl.inputs[end][1] - atan(CLV.params.wheelbase)) < 0.05
    # The controller's prediction stays close to the independently integrated plant.
    for k in (5, 25, 45)
        @test maximum(abs, cl.controller[k].X[:, 2] - cl.states[k+1]) <= 1e-4
    end
end

@testset "closed loop: failure stops without applying a control" begin
    ctrl = NMPCController(CLV, straight_track(50.0; width=0.7), NMPCConfig())
    cl = simulate_closed_loop(ctrl, CLV, [0.0, 0.0, 0.0, 6.0], 10)   # above v_max: infeasible
    @test cl.status === :controller_failure
    @test isempty(cl.inputs) && length(cl.states) == 1 && length(cl.controller) == 1
    @test !cl.controller[1].accepted
end
