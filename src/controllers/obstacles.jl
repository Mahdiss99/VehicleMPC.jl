# Stationary circular obstacle and its diagnostics (Milestone 3A). The constraints and the
# solve logic are in obstacle_avoidance.jl. See docs/stationary_collision_avoidance.md.

"""
    CircleObstacle(; x, y, radius)

A stationary circular obstacle with centre `(x, y)` and `radius` [m], in the track's
Cartesian frame.
"""
struct CircleObstacle
    x::Float64
    y::Float64
    radius::Float64
    function CircleObstacle(x, y, radius)
        all(isfinite, (x, y, radius)) && radius >= 0 ||
            throw(ArgumentError("obstacle centre must be finite and radius nonnegative, got ($x, $y, $radius)"))
        return new(x, y, radius)
    end
end

CircleObstacle(; x, y, radius) = CircleObstacle(x, y, radius)

"""
    obstacle_on_track(track, s, ey; radius) -> CircleObstacle

Experiment utility: a stationary obstacle centred at the Cartesian position of the Frenet
coordinates `(s, ey)` on `track` (`frenet_to_cartesian`), i.e. placed with the track's
actual geometry. The obstacle is fixed in Cartesian space from then on.
"""
obstacle_on_track(track::Track, s::Real, ey::Real; radius::Real) = CircleObstacle(frenet_to_cartesian(track, s, ey)[1:2]..., radius)

"""
    required_separation(obstacle, ego_radius, buffer) -> Float64

Required centre separation `D = ego_radius + obstacle.radius + buffer`.

The ego vehicle is represented as a disk of radius `ego_radius`, centred at the model's
reference point (the rear axle). This is a collision proxy, not a model of the vehicle
body.
"""
function required_separation(ob::CircleObstacle, ego_radius::Real, buffer::Real)
    (ego_radius >= 0 && buffer >= 0) || throw(ArgumentError("ego_radius and buffer must be nonnegative"))
    D = ego_radius + ob.radius + buffer
    D > 0 || throw(ArgumentError("the required separation must be positive"))
    return Float64(D)
end

"""
    obstacle_margin(px, py, ox, oy, D) -> value

Scaled collision margin `((px - ox)² + (py - oy)² - D²)/(2D)`. It is `≥ 0` exactly
outside the disk of radius `D`, and close to the metric clearance `|p - o| - D` near the
boundary. Generic: Float64, duals, JuMP expressions (with `D` and `1/(2D)` as Parameters).
"""
obstacle_margin(px, py, ox, oy, D) = ((px - ox)^2 + (py - oy)^2 - D^2) / (2D)

"""
Obstacle diagnostics of one control cycle (Milestone 3A).

**Geometry:** `present`; the obstacle `x`, `y`, `radius`; `ego_radius`, `buffer`;
`required = D`; and `collision_distance = ego_radius + radius` (disks touching).

**At the constrained points** (knots `k ≥ 2` plus the interior obstacle samples):
- `knot_min_distance`: the smallest centre distance;
- `knot_min_clearance`: that distance minus `D`.

**Dense evaluation** (`check_samples` per interval): the plan's inputs propagated from
the measured state with the exact Cartesian map.
- `min_distance`, `min_clearance` (`- D`) and `physical_clearance`
  (`- collision_distance`);
- `separation_violations`: samples with distance `< D - acceptance_tol`;
- `collisions`: samples with distance `< collision_distance - acceptance_tol`;
- `first_violation_time`: the first separation violation [s];
- `violation_between_knots`: whether any separation violation lies strictly between knots.

**Decision:**
- `side`: the passing side used, `:left`, `:right` or `:none` (no candidate selection);
- `attempts`: every solve attempt of the cycle as `(initialization, status, nlp_solves,
  seconds)`;
- `passage_left`, `passage_right`: whether the planning corridor leaves a lateral gap of
  at least `D` beside the obstacle (from its track projection);
- `failure_cause`: `:none`, `:initial_collision`, `:infeasible_geometry` (geometrically
  blocked by the lateral-gap heuristic), `:dynamically_infeasible` (suspected: no side
  guess stayed free of contact on the track), `:solver_convergence`, `:initialization`,
  `:segment_assignment`, `:constraint_violation`, `:collision`, `:separation` or `:other`;
- `message`: details.

**Association** (Milestone 3B):
- `association_s`, `association_ey`: the obstacle association used for initialisation
  (unwrapped progress and signed offset);
- `associations`: the number of separate associations within the horizon window;
- `guesses`: `(side, clearance above D at the guess's knots, minimum track clearance of the
  guess)` for every side guess built in this cycle (`-Inf` track clearance if its rollout
  failed).

**Timing** (Milestone 3B.1): `phase_times = (relevance, guesses, attempts, other)` [s]
within the cycle. `attempts` covers the NLP solves, reassignments and validation of all
attempts; `other` is the remaining bookkeeping (result assembly).
"""
struct ObstacleReport
    present::Bool
    x::Float64
    y::Float64
    radius::Float64
    ego_radius::Float64
    buffer::Float64
    required::Float64
    collision_distance::Float64
    knot_min_distance::Float64
    knot_min_clearance::Float64
    check_samples::Int
    min_distance::Float64
    min_clearance::Float64
    physical_clearance::Float64
    separation_violations::Int
    collisions::Int
    first_violation_time::Float64
    violation_between_knots::Bool
    side::Symbol
    attempts::Vector{Tuple{Symbol,Symbol,Int,Float64}}
    passage_left::Bool
    passage_right::Bool
    failure_cause::Symbol
    message::String
    association_s::Float64
    association_ey::Float64
    associations::Int
    guesses::Vector{Tuple{Symbol,Float64,Float64}}
    phase_times::NTuple{4,Float64}
end

const OBSTACLE_NONE = ObstacleReport(false, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, 0, NaN, NaN, NaN, 0, 0, NaN, false,
    :none, Tuple{Symbol,Symbol,Int,Float64}[], false, false, :none, "", NaN, NaN, 0, Tuple{Symbol,Float64,Float64}[],
    (NaN, NaN, NaN, NaN))
