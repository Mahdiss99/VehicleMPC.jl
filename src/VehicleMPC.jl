"""
    VehicleMPC

Vehicle dynamics, track geometry and simulation foundation for vehicle MPC in Julia.
Conventions are documented in `docs/model_conventions.md`.
"""
module VehicleMPC

using LinearAlgebra
using StaticArrays
using ForwardDiff
using JuMP
import Ipopt
import OSQP

include("models/vehicle_parameters.jl")
include("models/kinematic_bicycle.jl")
include("models/exact_discretization.jl")
include("models/dynamic_bicycle.jl")
include("tracks/track_geometry.jl")
include("tracks/segment_local_geometry.jl")
include("tracks/curvature_schedule.jl")
include("simulation/integration.jl")
include("simulation/open_loop.jl")
include("simulation/piecewise_track.jl")
include("constraints/operating_constraints.jl")
include("controllers/nmpc_config.jl")
include("controllers/obstacles.jl")
include("controllers/nonlinear_mpc.jl")
include("controllers/curvature_scheduling.jl")
include("controllers/cartesian_endpoint.jl")
include("controllers/intersample_corridor.jl")
include("controllers/linear_mpc.jl")
include("controllers/obstacle_avoidance.jl")
include("controllers/reassignment_reliability.jl")
include("controllers/linear_mpc_tracks.jl")
include("controllers/dynamic_nmpc.jl")
include("simulation/closed_loop.jl")
include("simulation/dynamic_closed_loop.jl")
include("analysis/curvature_prediction.jl")

# Parameters and model
export VehicleParameters, reference_test_vehicle, within_input_limits, LEGACY_PARAMETER_SETS,
    STEERING_SINGULARITY_MARGIN, DEFAULT_EPS_GEOMETRY
export KinematicBicycle, state_dim, input_dim, check_domain, kinematic_rhs, continuous_dynamics, continuous_jacobians

# Discretization
export AbstractIntegrator, ForwardEuler, RK4, discrete_dynamics, discrete_jacobians, discrete_jacobians_ad

# Track geometry
export TrackSegment, Track, track_length, is_closed, num_segments, total_turning,
    split_progress, wrap_progress, segment_index, curvature,
    centerline_pose, centerline_position, centerline_heading,
    frenet_to_cartesian, cartesian_to_frenet,
    straight_track, circular_track, quarter_track, legacy_track, LEGACY_TRACK_SPECS

# Simulation
export SimulationResult, succeeded, simulate_open_loop

# Operating-constraint diagnostics
export ConstraintViolation, check_operating_constraints, satisfies_operating_constraints, trajectory_violations

# Nonlinear MPC (Milestone 2A)
export NMPCConfig, NMPCController, NMPCResult, NMPCTiming, solve!, reset_warm_start!, horizon,
    curvature_values, lateral_acceleration, validate_prediction, between_knot_diagnostics
export ClosedLoopResult, simulate_closed_loop

# Multi-segment NMPC (Milestone 2B.2)
export ScheduleReport, ReferenceReport, stage_curvatures, set_curvature_schedule!, curvature_fixed_point,
    schedule_consistency, reference_check
export curvature_linearization, install_schedule!, segment_id, stage_configuration, curvature_model
export heading_increment, segment_pair_assignment

# Exact Cartesian discretisation and Cartesian-endpoint NMPC (Milestone 2B.4)
export sinc_un, sinc_un_d1, sinc_un_d2, exact_bicycle_increment, exact_cartesian_step, simulate_exact_cartesian
export segment_local_pose, frenet_pose_local, segment_frame
export ExactMapReport, prediction_model, deadline_missed, cartesian_assignment, cartesian_initial_guess, install_cartesian!,
    cartesian_consistency, exact_map_check

# Between-knot corridor constraints (Milestone 2B.5)
export CorridorReport, interior_alphas, interior_point, corridor_parameters, corridor_margins, sample_projection,
    interior_assignment, interior_assignment_check, corridor_check, analytic_corridor_clearance

# Stationary-obstacle collision avoidance (Milestone 3A)
export CircleObstacle, obstacle_on_track, ObstacleReport, required_separation, obstacle_margin, set_obstacle!, obstacle_frenet,
    passage_sides, side_guess, collision_check, solve_once!, obstacle_associations, obstacle_relevant

# Segment-reassignment reliability (Milestone 3B.1)
export ReassignmentReport, detect_assignment_cycle, cycle_budget_allows, install_cartesian_ids!, post_solve_checks

# Branch-selection obstacle acceptance (Milestone 4A correction)
export obstacle_rejection, select_branch_candidate

# Linearized MPC with OSQP (Milestone 4A)
export LinearMPCConfig, LinearMPCController, LinearMPCResult, linearize_exact_map, linearize_lateral_acceleration,
    exact_rollout, validate_plan, nominal_inputs, pursuit_input, shifted_inputs, linear_plan_objective, straight_frame

# Linearized MPC on multi-curvature tracks with a stationary obstacle (Milestone 4BC)
export TrackReferenceError, track_reference, trajectory_references, validate_plan_on_track, plan_objective_on_track,
    obstacle_direction, interior_jacobian, nominal_inputs_local, shifted_inputs_local, pursuit_input_local,
    track_linearization_error

# Multi-segment track dynamics and curvature scheduling (Milestone 2B.1)
export segment_boundaries, curvature_left, average_curvature, curvature_schedule, AVERAGE_CURVATURE_DS_MIN
export SegmentCrossing, TrackIntegrationResult, TrackSimulationResult, locate_crossing, integrate_on_track, simulate_on_track
export cartesian_rhs, integrate_cartesian, simulate_cartesian, frenet_state_to_cartesian
export PredictionResult, predict_start_curvature, predict_with_schedule, predict_average_curvature,
    predict_split_curvature, prediction_errors

# Six-state dynamic bicycle and its NMPC with multiple stationary obstacles (final milestone)
export DynamicBicycleParameters, DynamicBicycle, legacy_dynamic_parameters, showcase_dynamic_parameters, slip_angles, tyre_forces,
    dynamic_rhs, dynamic_lateral_acceleration, check_dynamic_domain, dynamic_rk4_step, dynamic_step, integrate_dynamic,
    dynamic_operating_violations, lateral_mode_rates, steady_state_cornering, max_cornering_speed, cg_pose_from_rear_axle, rear_axle_pose
export DynamicNMPCConfig, DynamicNMPCController, DynamicNMPCResult, dynamic_rollout, validate_dynamic_plan, pursuit_guess,
    relevant_obstacles, side_sequence_guesses, chain_frenet, DynamicClosedLoopResult, simulate_dynamic_closed_loop

end # module
