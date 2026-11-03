# Development history

VehicleMPC was built in milestones. This page summarises each one, with its technical
report and benchmark. The reports and recorded benchmark results are kept unchanged as
historical records. A report describes the state of the code when it was written; later
milestones may have added options, but did not change the defaults unless stated.

| milestone | content | report | benchmark / results |
|---|---|---|---|
| 1 | Kinematic bicycle model in Frenet coordinates (rear axle), domain checks, Euler/RK4 discretisation, analytical and ForwardDiff Jacobians, piecewise constant-curvature tracks, Frenet ↔ Cartesian conversion, open-loop simulation, operating-constraint diagnostics; audit of the original scripts | `docs/model_conventions.md`, `docs/legacy_audit.md` | – |
| 2A | Progress-maximising nonlinear MPC (JuMP + Ipopt, direct multiple shooting), persistent model with parameter updates, warm start, independent validation, closed-loop simulation | `docs/nmpc_formulation.md` | `benchmarks/nmpc_runtime.jl` → `results/nmpc_runtime.md` |
| 2B.1 | Event-aware integration across segment boundaries, independent Cartesian reference model, stagewise average curvature, curvature-prediction error analysis | `docs/multisegment_prediction.md` | `multisegment_prediction` |
| 2B.2 | Multi-segment NMPC: curvature schedule, consistency test, event-aware re-simulation, bounded re-solves, optional qualified acceptance of stale schedules | `docs/multisegment_nmpc.md` | `multisegment_nmpc` |
| 2B.3A | Experimental progress-linearized curvature model | `docs/progress_aware_curvature.md` | `progress_aware_curvature` |
| 2B.3B | Experimental exact segment-pair curvature model | `docs/segment_pair_curvature.md` | `segment_pair_curvature` |
| 2B.4 | Exact zero-order-hold Cartesian map; Cartesian-endpoint NMPC with segment-local formulas and bounded segment reassignment | `docs/cartesian_endpoint_nmpc.md` | `cartesian_endpoint_nmpc` |
| 2B.5 | Experimental between-knot corridor constraints (not continued) | `docs/intersample_corridor_constraints.md` | `intersample_corridor` |
| 3A | Stationary circular-obstacle avoidance (Cartesian-endpoint NMPC) on a straight track; disk ego proxy, knot and interior collision constraints, dense collision validation | `docs/stationary_collision_avoidance.md` | `stationary_obstacle` |
| 3B | Obstacle avoidance on the multi-curvature quarter track; geometric relevance across segments and laps; curvature-aware passing guesses | `docs/multicurvature_collision_avoidance.md` | `multicurvature_obstacle` |
| 3B.1 | Reassignment cycle detection, local branch resolution, sequential passing-side encounters, optional cycle budget | `docs/segment_reassignment_reliability.md` | `reassignment_reliability` |
| 4A | Linearized MPC (JuMP + OSQP) on a straight track: exact-map Jacobians, persistent QP with in-place updates, trust region, successive linearisation, nonlinear validation. Separate correction of the 3B.1 branch selection (obstacle acceptance before selection) | `docs/linearized_mpc.md` | `linear_mpc` |
| 4BC | Linearized MPC on multi-curvature tracks with one stationary obstacle: local affine track coordinates, supporting-halfspace collision constraints at knots and interior samples, softened inequalities, trust-region memory, shared passing-side guesses, validation on the actual track | `docs/linearized_multicurvature_collision_avoidance.md` | `linear_multicurvature_obstacle` |
| 5 | Release preparation: standalone examples, API and benchmark-comparison documentation, README, CI, reliability tests (no controller changes) | `docs/api.md`, `docs/benchmark_comparison.md`, `docs/release_readiness.md` | – |
| final | Six-state dynamic bicycle (CoM, linear axle tyres, operating domain instead of a regulariser); dynamic NMPC (JuMP + Ipopt, RK4 multiple shooting, segment-local geometry, dynamic lateral-acceleration and slip limits); simultaneous multi-obstacle constraints with persistent slots and bounded side-sequence initialisation; racing showcases with model selection and rendered videos; removal of the superseded legacy scripts | `docs/dynamic_bicycle_mpc.md`, `examples/showcase/README.md`, `docs/legacy_audit.md` §7 | `examples/showcase/` (recorded runs in the README) |

Benchmark scripts are in `benchmarks/`, with results in `benchmarks/results/<name>.md`.

## Status of the nonlinear MPC options

**Default `NMPCConfig()`** (Frenet model with the stagewise average-curvature schedule):
- works on single-segment tracks (straight, circle);
- **does not** work on multi-segment tracks such as `quarter_track()`: the first cycle is
  rejected as schedule-inconsistent (`examples/nmpc_quarter.jl` shows this).

**Multi-segment tracks.** Complete quarter-track laps were obtained with:
- bounded schedule corrections plus qualified stale-schedule acceptance (2B.2);
- the experimental curvature models of 2B.3A/2B.3B, in some settings;
- the Cartesian-endpoint model with bounded segment reassignments (2B.4), which was then
  used for the obstacle work.

**Obstacles on the quarter track** (3B/3B.1).
- Scenarios at segment boundaries are the difficult ones.
- One (E3) still fails within the 80 ms correction budget.
- The 3B.1 options (cycle detection, branch resolution, sequential encounters) are opt-in.

## Status of the linearized MPC

**Straight track model (4A).**
- It is the default on a single straight segment without an obstacle.
- Its closed-loop results are reproduced bit for bit after 4BC.

**Local track model (4BC)**, for curved tracks and/or one stationary obstacle. On the
quarter track (N = 10, three interior halfspace samples):
- it completed all ten 3B obstacle scenarios in all three runs;
- it was faster than the nonlinear MPC, and made 7–10 % less progress.

See `docs/benchmark_comparison.md`.
