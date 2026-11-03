# Stationary-obstacle collision avoidance (Milestone 3A, experimental)

**Status: experimental, demonstrated on one straight-track scenario family; not a general
collision-avoidance capability.** It is an engineering application of standard
obstacle-avoidance MPC (nonconvex distance constraints in a nonlinear MPC). No novelty is
claimed. Results are in `benchmarks/results/stationary_obstacle.md`, and the figures are
`docs/figures/stationary_obstacle_trajectory.png` and `stationary_obstacle_signals.png`.

**Summary** (straight track; obstacle of radius 0.10 m on the centreline; required
separation 0.18 m):

- **N = 10 (1 s horizon).** With collision constraints at the knots, with or without
  interior samples, the vehicle approaches, swerves, passes and continues. In all three
  runs:
  - the closest plant approach was 0.174 m, i.e. 2.4 cm away from touching the obstacle
    and 0.6 cm inside the 3 cm safety buffer;
  - the plant never left the physical track.
- **N = 20 (2 s horizon), knot-only and 1 interior sample: rejected at cycle 2.** The
  plans satisfied the constraints at the knots (and the single interior sample), but the
  independent dense check found them cutting through the obstacle between those points
  (minimum distance 6e-8 m and 0.137 m respectively, against 0.15 m for contact). No
  control was applied.
- **N = 20, 3 interior samples: completes all three runs**, closest approach 2.0 cm from
  touching, 1.0 cm inside the buffer. It is not real-time capable here: 20 of 300 cycles
  exceeded 100 ms, mostly during the approach, with up to 115 Ipopt iterations per solve.
- **Offset obstacle (8 cm off-centre):** every configuration completes at both horizons.
- **Corridor blocked on both sides:** every tested start state fails with
  `:solver_failure`, cause `:infeasible_geometry`, even where braking to a stop would
  have been feasible. No braking candidate is implemented.

## 1. Problem formulation

The controller is the exact Cartesian-endpoint NMPC (Milestone 2B.4), with
`interior_samples = 0`, `strict_corridor = false`, a 5 cm planning margin, ≤ 8 segment
reassignments within the 80 ms budget, and the independent physical-track check. The
objective, weights, limits, vehicle and Ipopt options are unchanged.

**Added constraints.** For the obstacle `o = (ox, oy)` with radius `r_o`, the ego collision
radius `r_e` and the buffer `b`, the required centre separation is

    D = r_e + r_o + b,

and the constraint added at every predicted knot `k = 2 … N+1` is

    m(p_k) = (|p_k − o|² − D²) / (2D) ≥ 0,

where `p_k` is the exact segment-local Cartesian position of the knot. The latter is a
function of the decision variables `X[:, k]`; there is no nominal trajectory.

**Scaling.** Dividing by `2D` makes `m ≈ |p − o| − D` near the boundary, i.e. metres, like
the other margins.

**Persistence.** `(ox, oy, D)` are JuMP Parameters. `set_obstacle!` moves or resizes the
obstacle without rebuilding the model.

**Optional interior samples** (`obstacle_samples = M`). The same constraint is added at
`tau = j·dt/(M+1)`, `j = 1 … M`, of every interval. Each position is the exact intermediate
position `interior_point` (2B.5) of the knot pose and the interval's inputs.

**Measured state.** It is not a decision variable. If its disk already violates `D`
(beyond `acceptance_tol`), the cycle returns `:initial_collision` without calling the
solver.

There are no binary variables and no mixed-integer solver.

## 2. Collision geometry assumptions

- **The ego vehicle** is a disk of radius `ego_radius` centred at the model's reference
  point (the rear axle). This is a **proxy**, not the vehicle body: the reference point is
  not the body centre, and the body is not round.
- **The obstacle** is a stationary circle, fully known, in the track's Cartesian frame.
- **Two thresholds are distinguished:**
  - *required separation* `D`, which includes the buffer and is the constraint;
  - *collision*: the disks touch, `|p − o| < r_e + r_o`.

## 3. Nonconvexity and initialisation

The feasible set (outside a disk) is nonconvex. A centreline guess runs straight into the
obstacle, and the solution depends on the side the guess takes.

**Candidate guesses** (`side_guess`). These are dynamically consistent trajectories for
passing on the left or the right.
- **Target offset:** the middle of the passage gap on that side,
  `(ey_o ± D ± ey_max)/2`.
- **Smooth window:** the offset is held while within `D` of the obstacle in progress, with
  1.5 m cosine ramps on either side.
- **Steering:** pure pursuit (look-ahead `clamp(0.5·v, 0.5, 2) m`), clipped to the steering
  limit and to `|v² tan δ/L| ≤ ay_max`.
- **Acceleration:** from the shifted previous plan, or the quasi-steady profile.
- **Rollout:** the event-aware vehicle model on the actual track. A test checks that each
  guess is reproduced by re-simulating its own inputs.

**Selection** (`solve_with_obstacle!`):
1. If the obstacle is out of the horizon's reach: one ordinary solve.
2. At the encounter (the obstacle becomes relevant and no side has been chosen): one solve
   from each open side's guess. The accepted result with the lower objective is kept, and
   its side is remembered. This is the only case with two solves by design.
3. Afterwards: one warm-started solve, which keeps the side. If it is rejected, one retry
   from the remembered side's guess.

Every attempt is recorded in `result.obstacle.attempts` (initialisation, status, NLP
solves, seconds). `timing.total` and `nlp_solves` cover all attempts. Iteration limits and
correction budgets are unchanged.

## 4. Acceptance and failure handling

A cycle is accepted only if all of the following hold:
- the existing acceptance holds: solver status, dynamics residual, all bounds and
  inequalities, valid segment assignments, and the event-aware physical-track
  re-simulation;
- every imposed collision constraint (knots and interior samples, recomputed with the
  true geometry) holds to `acceptance_tol`;
- the **dense collision evaluation** (`collision_check`: exact Cartesian propagation of the
  plan, 50 samples per interval) finds no collision.

Intrusions into the buffer between constrained points are reported (`separation_violations`,
`violation_between_knots`, `first_violation_time`), but are not rejected by default.

- `obstacle_acceptance = :separation` rejects them as well. With that setting, every
  configuration tested here is rejected (§6).

**Statuses:**
- `:initial_collision`;
- `:obstacle_constraint_violation`;
- `:collision_detected`;
- the existing ones (`:solver_failure`, ...).

`result.obstacle.failure_cause` gives `:initial_collision`, `:infeasible_geometry` (no
lateral passage and the solver failed), `:solver_convergence` (several initialisations
failed), `:initialization` (one did), `:constraint_violation`, `:collision`, `:separation`
or `:other`.

A rejected cycle returns no control. There is no emergency fallback.

## 5. Interface

```julia
cfg = NMPCConfig(prediction_model=:cartesian_endpoint_exact, schedule_corrections=8,
    ego_radius=0.05, obstacle_buffer=0.03, obstacle_samples=3)
ctrl = NMPCController(vehicle, track, cfg; obstacle=CircleObstacle(x=8.0, y=0.0, radius=0.10))
r = solve!(ctrl, x)                       # r.obstacle :: ObstacleReport
set_obstacle!(ctrl, CircleObstacle(x=9.0, y=0.05, radius=0.10))   # Parameters only
```

Without `obstacle`, the controller is unchanged; a closed loop was checked to be
identical. The new configuration fields (`ego_radius`, `obstacle_buffer`,
`obstacle_samples`, `obstacle_check_samples`, `obstacle_acceptance`) have no effect then.

## 6. Test scenario and results

**Setup.**
- **Track:** straight, 80 m, width 0.7 m. With the 5 cm margin the corridor is
  `|ey| ≤ 0.30 m`.
- **Start:** `[0, 0, 0, 0.5]`; 100 control cycles; 3 runs per configuration.
- **Dimensions (synthetic, not taken from the test vehicle):**
  - `ego_radius = 0.05 m`, equal to the margin, so the ego disk lies inside the physical
    track whenever its reference point is in the corridor;
  - obstacle radius 0.10 m, buffer 0.03 m, so `D = 0.18 m` and collision at 0.15 m.
- **Geometric feasibility.** With the obstacle on the centreline, the reference point may
  use `0.18 ≤ |ey| ≤ 0.30`: a 12 cm passage on each side. With the 8 cm offset, the gaps are
  4 cm (left) and 20 cm (right).

**Closed loop**, centred obstacle (all values over 3 runs):

| N | configuration | outcome | closest plant distance [m] | margin to D / to contact [m] | plant track clearance [m] | accepted plans entering the buffer |
|--:|:--|:--|:-:|:-:|:-:|:-:|
| 10 | no obstacle (baseline) | 47.96 m ×3 | **0.012** (drives through it) | −0.168 / −0.138 | 0.350 | – |
| 10 | knots only | 47.95 m ×3 | 0.174 | −0.006 / **+0.024** | 0.176 | 30 (all between knots) |
| 10 | knots + 1 interior | 47.95 m ×3 | 0.174 | −0.006 / +0.024 | 0.176 | 30 |
| 10 | knots + 3 interior | 47.95 m ×3 | 0.174 | −0.006 / +0.024 | 0.176 | 30 |
| 20 | knots only | **rejected at cycle 2** ×3 (`:collision_detected`; dense min 6e-8 m) | – | – | – | – |
| 20 | knots + 1 interior | **rejected at cycle 2** ×3 (dense min 0.137 m < 0.15 m) | – | – | – | – |
| 20 | knots + 3 interior | 47.93 m ×3 | 0.170 | −0.010 / **+0.020** | 0.181 | 60 (all between knots) |

- **Offset obstacle:** all configurations complete at both horizons, passing on the right.
  Closest approach 0.173–0.176 m (2.3–2.6 cm from contact). No plant violations.
- **Plant:** there were no physical-track violations in any run (`check_operating_constraints`
  over every plant substep).
- **Progress cost of avoiding:** 0.01–0.03 m over 10 s, against 47.96 m without the
  obstacle.
- **Passing side.** With the centred obstacle both sides are equivalent. The chosen side
  differs between configurations (left/right), and once between runs of the same
  configuration. After the encounter it never switched.

**Between-knot limitation (measured).**
- **At N = 20:** the vehicle is at 5 m/s, so knots are 0.5 m apart, which is more than the
  obstacle's 0.36 m `D`-diameter. The optimiser placed knots on both sides of the disk.
- **Why the dense check catches it:** the imposed constraints held at the solution
  (`knot_min_clearance ≥ 0`). The collision was found only by the independent evaluation,
  which rejected the plan.
- **With 3 interior samples:** the spacing is 0.125 m. The remaining between-sample dips are
  covered by the 3 cm buffer in this scenario (closest dense distance 0.169 m > 0.15 m).
  That is an empirical margin, not a guarantee.

**Blocked corridor** (obstacle radius 0.30 m, `D = 0.38 m` > `ey_max`):
- Both passages are blocked.
- From `[6.6, 0, 0, 5]` stopping is impossible, and both candidates fail
  (`:infeasible_geometry`).
- From `[0, 0, 0, 1]` with N = 20 the same happens, although braking would be feasible.
- From the same start with N = 10 the obstacle is still out of reach, and that single cycle
  succeeds.

## 7. Computation time

All timings come from one Windows laptop, Julia 1.11.5, one thread. The whole cycle is
included: candidates, solves, reassignments and all validation. Warmed up:

| scenario, N | configuration | mean / median / p95 / p99 / max [ms] | > 80 / > 100 ms | encounter cycles (2 solves): median / max [ms] |
|:--|:--|:-:|:-:|:-:|
| centre, 10 | no obstacle | 6.0 / 5.5 / 7.7 / 9.5 / 17.6 | 0 / 0 | – |
| centre, 10 | knots + 3 interior | 13.8 / 12.8 / 19.1 / 44.0 / 85.6 | 1 / 0 | 71.2 / 85.6 |
| centre, 20 | no obstacle | 12.8 / 12.5 / 17.3 / 19.9 / 30.2 | 0 / 0 | – |
| centre, 20 | knots + 3 interior | 39.9 / 26.8 / 141.9 / 300.8 / 345.5 | **22 / 20** | 300.8 / 323.1 |
| offset, 20 | knots + 3 interior | 28.0 / 24.4 / 44.5 / 102.3 / 116.6 | 8 / 4 | 104.7 / 116.6 |

- **Centred obstacle, N = 20, 3 interior samples.** The overruns are the first 7 cycles of
  every run: the encounter, then the approach with the obstacle near the horizon end.
  These are single solves of 80–345 ms with up to 115 Ipopt iterations. They are not GC
  pauses: GC was zero in all but one of those cycles.
- **GC-dominated cycles.** Two other cycles were dominated by garbage collection: 183 ms
  GC (centre, N = 20) and 197 ms GC (offset, N = 20, knots only).
- **Mean Ipopt iterations:** 7–9 without the obstacle; 9–15 with it in the configurations
  that completed. The two N = 20 configurations rejected at cycle 2 averaged 36 and 54.
- **First construction and solve in a process**, compilation included: 5.7 s / 13.8 s.
  Priming solves of new models took 12–325 ms.

There is no real-time guarantee. At N = 20 with 3 interior samples, the 10 Hz target was
missed in about 7 % of the cycles of this scenario.

## 8. Limitations

- **One scenario family:** one stationary circular obstacle on a straight track. Nothing
  was tested with curved tracks, several obstacles, moving obstacles or other agents.
- **Proxy geometry:** a disk proxy at the rear axle. The body geometry is not modelled.
- **Between constrained points** collisions are not excluded by construction. They are
  detected by sampled evaluation (50 samples per interval), and avoided here only with
  interior samples plus the buffer. There is no continuous-time certificate.
- **Local solutions.** Ipopt finds local solutions; the side candidates mitigate this but
  do not guarantee a global one.
- **No braking candidate.** When no lateral passage exists the controller fails rather
  than stopping, even when stopping is feasible.
- **Runtime.** The 2 s horizon with 3 interior samples exceeds 100 ms in about 7 % of
  cycles (on approach), and two candidate solves per encounter cost 70–320 ms. N = 10 met
  100 ms in every cycle of these runs.
- **Side choice in symmetric cases** depends on small objective differences, and differed
  between runs.
