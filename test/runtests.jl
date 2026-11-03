using VehicleMPC
using Test
using LinearAlgebra
using StaticArrays
using TOML
using JuMP

@testset "VehicleMPC" begin
    @testset "kinematic dynamics" begin
        include("test_kinematic_dynamics.jl")
    end
    @testset "jacobians" begin
        include("test_jacobians.jl")
    end
    @testset "track geometry" begin
        include("test_track_geometry.jl")
    end
    @testset "simulation" begin
        include("test_simulation.jl")
    end
    @testset "operating constraints" begin
        include("test_operating_constraints.jl")
    end
    @testset "piecewise-track integration" begin
        include("test_piecewise_track.jl")
    end
    @testset "curvature schedules and prediction methods" begin
        include("test_curvature_schedule.jl")
    end
    @testset "nonlinear MPC" begin
        include("test_nmpc.jl")
    end
    @testset "nonlinear MPC closed loop" begin
        include("test_nmpc_closed_loop.jl")
    end
    @testset "multi-segment NMPC" begin
        include("test_multisegment_nmpc.jl")
    end
    @testset "progress-linearized curvature (experimental)" begin
        include("test_progress_linearized.jl")
    end
    @testset "segment-pair curvature (experimental)" begin
        include("test_segment_pair.jl")
    end
    @testset "Cartesian-endpoint NMPC (experimental)" begin
        include("test_cartesian_endpoint.jl")
    end
    @testset "between-knot corridor constraints (experimental)" begin
        include("test_intersample_corridor.jl")
    end
    @testset "stationary-obstacle avoidance (experimental)" begin
        include("test_obstacle_avoidance.jl")
    end
    @testset "multi-curvature obstacle avoidance (experimental)" begin
        include("test_multicurvature_obstacle.jl")
    end
    @testset "segment-reassignment reliability (experimental)" begin
        include("test_reassignment_reliability.jl")
    end
    @testset "linearized MPC with OSQP" begin
        include("test_linear_mpc.jl")
    end
    @testset "linearized MPC: curved tracks and obstacle" begin
        include("test_linear_mpc_tracks.jl")
    end
    @testset "dynamic bicycle model" begin
        include("test_dynamic_bicycle.jl")
    end
    @testset "dynamic NMPC with multiple obstacles" begin
        include("test_dynamic_nmpc.jl")
    end
    @testset "racing showcase" begin
        include("test_showcase.jl")
    end
    @testset "reproducibility" begin
        include("test_reproducibility.jl")
    end
end
