# Dynamic bicycle MPC with multiple stationary obstacles (final milestone)

This report covers:
- the six-state dynamic bicycle model, `DynamicBicycle`;
- its nonlinear MPC, `DynamicNMPCController`;
- the simultaneous multi-obstacle formulation;
- the two racing showcases in `examples/showcase/`.

The kinematic and linearized controllers of the earlier milestones are unchanged; the
kinematic model is still the default everywhere outside the showcase.

## 1. Model

### 1.1 State, input and conventions

    z = [X, Y, ψ, vx, vy, r]        u = [δ, a]

| symbol | meaning | unit |
|:--|:--|:--|
| `X, Y` | position of the **centre of mass (CoM)** in the global frame | m |
| `ψ` | heading of the body x-axis, unwrapped, counter-clockwise positive | rad |
| `vx, vy` | CoM velocity in the body frame (x forward, **y left**) | m/s |
| `r` | yaw rate `ψ̇`, counter-clockwise positive | rad/s |
| `δ` | front steering angle, **positive to the left** | rad |
| `a` | longitudinal acceleration command (net longitudinal force / m) | m/s² |

The kinematic model of the earlier milestones is referenced at the **rear axle**. The two
reference points are related by `p_CoM = p_rear + lr (cos ψ, sin ψ)`
(`cg_pose_from_rear_axle`, `rear_axle_pose`). The showcases use this transform so that both
models start from the same physical pose (section 6.2).

### 1.2 Equations

    Ẋ  = vx cos ψ − vy sin ψ
    Ẏ  = vx sin ψ + vy cos ψ
    ψ̇  = r
    v̇x = a − F_yf sin δ / m + r vy
    v̇y = (F_yf cos δ + F_yr) / m − r vx
    ṙ  = (lf F_yf cos δ − lr F_yr) / Iz

Tyres are linear, **per axle** (both tyres of an axle together):

    α_f = δ − atan((vy + lf r) / vx)        F_yf = Cf α_f
    α_r =   − atan((vy − lr r) / vx)        F_yr = Cr α_r

The longitudinal tyre forces are not modelled separately: `a` is the commanded net
longitudinal acceleration. The legacy code wrote `2 c_f` with `c_f` per tyre, so
`Cf = 2 c_f`. The body-frame lateral acceleration of the CoM is the model's own quantity

    a_y = (F_yf cos δ + F_yr) / m = v̇y + r vx,

which is what the controller limits. The kinematic expression `v² tan δ / L` is not used for
this model.

The tests check:
- the sign conventions (left steering gives positive slip, lateral force, `a_y` and yaw
  acceleration);
- the projection of `vy` on the global frame;
- the `a_y` identity and the mass scaling;
- straight-line motion at zero steering, and the steady circle at `δ = L/R`;
- ForwardDiff Jacobians against central differences;
- RK4 fourth-order convergence.

### 1.3 Low-speed treatment

The legacy model divided by `max(|vx|, 1e-10)` in the dynamics and by `vx + 1e-10` in its
Jacobian. That keeps the code from dividing by zero, but it does not make the model valid:
- the slip angles and lateral forces are meaningless as `vx → 0`;
- the two regularisations disagree for `vx < 0`;
- the Jacobian entries reach 3·10⁹ at `vx = 0` (`docs/legacy_audit.md`).

The package instead **restricts the model to its operating domain**:
- **Definition.** The model is defined for `vx > 0` only. `check_dynamic_domain` raises a
  `DomainError` otherwise, and the integrators raise one if `vx` reaches 0 inside a step.
  There is no regulariser.
- **Operating domain.** The controller imposes `vx ∈ [vx_min, vx_max]` with `vx_min > 0`
  as a hard constraint, at every knot and RK4 sub-step.
- **Measurements.** A measurement outside the model domain is rejected
  (`:invalid_measurement`). No control is computed for it.
- **Choice of `vx_min`.** The lateral and yaw modes have rates of about
  `(Cf + Cr)/(m vx)` and `(lf² Cf + lr² Cr)/(Iz vx)` (`lateral_mode_rates`). `vx_min` is
  chosen so that `rate · h` stays inside RK4's real-axis stability bound (about 2.78) for
  the NLP sub-step `h = dt/4 = 0.025 s`.
  - For the showcase parameters at `vx_min = 0.8 m/s`, `rate · h ≈ 1.6`.
  - The test "low-speed domain" checks this, and checks that one MPC step agrees with a
    0.1 ms integration to within the acceptance tolerance there.

Starting from standstill is therefore outside the scope of this model. Such a start would
need a kinematic or blended low-speed model, which is not implemented.

### 1.4 Integration

- **NLP.** RK4, 4 sub-steps per 0.1 s interval (`dynamic_step`, `substeps = 4`).
- **Plant and validation.** An independent RK4 with a 1 ms step (`integrate_dynamic`,
  `plant_step = 1e-3`).

Both are the same equations. The plant is not a different "true" vehicle: model mismatch
is out of scope. The difference between the two integrations is measured in every cycle
and bounded by `consistency_tol = 5e-3`.

### 1.5 Parameters and feasibility on the R = 1 m bends

| set | m [kg] | Iz [kg m²] | lf = lr [m] | Cf = Cr [N/rad per axle] | vx [m/s] | \|α\| ≤ | δ | a [m/s²] |
|:--|--:|--:|--:|--:|:--|--:|--:|:--|
| `legacy_dynamic_parameters()` | 1.98 | 0.024 | 0.125 | 2 × 1.25 = 2.5 | [0.3, 5] | 0.12 | ±25° | ±5 |
| `showcase_dynamic_parameters()` | 1.98 | 0.024 | 0.125 | 40 | [0.8, 5] | 0.12 | ±25° | ±5 |

**The legacy tyres cannot race on `quarter_track()`.** Steady-state cornering
(`steady_state_cornering`) on the R = 1 m bends gives:
- at 2 m/s (`a_y = 4 m/s²`), a front slip angle of about **1.58 rad**, far outside any
  linear tyre range;
- within `|α| ≤ 0.12 rad`, a maximum speed of about **0.55 m/s** (`max_cornering_speed`).

This is consistent with the legacy dynamic MPC prototype's 0.65 m/s speed cap. The test
"low-speed domain" checks both numbers.

**The showcase therefore uses documented synthetic tyres.**
- `Cf = Cr = 40 N/rad` per axle, about 4 /rad per unit axle load: plausible for a 2 kg car.
- At `a_y = 4 m/s²` the slip angle is about 0.1 rad, inside `|α| ≤ 0.12`.
- The maximum speed on R = 1 m at 4 m/s² is 2.0 m/s.

The mass, inertia and axle distances are the legacy values. Neither set is a calibrated
vehicle.

## 2. Nonlinear MPC

### 2.1 Objective

Identical to the kinematic `NMPCController` (`docs/nmpc_formulation.md`), with the same
weights `ws, qy, qpsi, rdelta, ra, rd_delta, rd_a`:
- terminal progress reward;
- lateral and heading penalties;
- input and input-rate penalties;
- the speed scale is `vx_max`.

### 2.2 Variables and dynamics (direct multiple shooting)

**Knots.** Each knot `k = 1…N` holds:
- the CoM's track coordinates `(s, ey, eψ)` with respect to an **assigned segment** (as in
  the Cartesian-endpoint NMPC);
- the body velocities `(vx, vy, r)`.

**Knot poses.** The Cartesian pose is given by the segment-local formulas
`frenet_pose_local`. They use the segment's start pose and curvature (JuMP Parameters) and
a registered `sinc`, which is smooth at κ = 0. There is no discontinuous segment look-up
inside the NLP and no curvature averaging.

**Dynamics.** RK4 equalities in Cartesian coordinates. Each sub-step has its own state
variables, and each RK4 stage has its own variables `G`. This keeps the expression graph
linear in N; nesting four RK4 stages symbolically made a solve about 12× slower
(628 ms → 53 ms median, measured).

**Progress.** Unwrapped: the assigned segment carries the lap offset, so crossing the
start/finish line needs no special treatment.

**Segment assignment.**
1. Knots are assigned from a dynamically consistent rollout of the initial guess.
2. After a solve, every knot is checked against its segment (`frame_valid`).
3. Knots that left their segment are reassigned from the Cartesian solution
   (`cartesian_assignment`), and the NLP is re-solved.

At most `max_reassignments = 4` rounds run. A knot still outside its segment after that
gives `:segment_assignment_invalid`, and the plan is not applied.

**Assignment cycles.** A knot within millimetres of a segment boundary can fall into the
other segment in either segment's frame: the progress reward differs between a straight
and an arc frame on the outside of a bend. Plain reassignment then alternates without end.
This was found in the final 250-cycle race at 2.26 laps (cycle 225), on the boundary from
the straight to the R = 2 m arc at s ≈ 13.42 m of the lap, 4–7 mm from the boundary.

When a reassignment would repeat an assignment already tried in this attempt, the current
assignment is kept. The alternating knots' progress is bounded to their assigned segments
(`s0 ≤ s ≤ s1`) for the remaining re-solves. The segment-local formulas are exact on the
closed interval, so the result is a valid plan of a restricted problem, and every
acceptance check still applies. The same state now solves with one bounded re-solve, and
the dynamic race runs 400 cycles (4.2 laps) without failure (with no time budget).

### 2.3 Constraints

| constraint | where |
|:--|:--|
| `|ey| ≤ w/2 − track_margin`, `|eψ| ≤ 60°` | knots |
| `vx_min ≤ vx ≤ vx_max` | knots and RK4 sub-steps |
| `|δ| ≤ δ_max`, `a_min ≤ a ≤ a_max` | inputs |
| `|α_f|, |α_r| ≤ alpha_max` (model validity) | knots and sub-steps |
| `|a_y| ≤ ay_max = 4 m/s²`, dynamic `a_y` of section 1.2 | knots and sub-steps |
| `‖p − o_i‖² ≥ D_i²` per obstacle slot | knots and every RK4 sub-step position |

### 2.4 Acceptance

A cycle's plan is applied only if all of the following hold.
1. **Solver.** Ipopt reports a locally optimal point (`LOCALLY_SOLVED`).
2. **Assignment.** Every knot lies inside its assigned segment.
3. **Independent rollout.** The inputs are re-simulated with the 1 ms integrator from the
   measured state (`validate_dynamic_plan`).
   - **Consistency.** Position, heading and velocity agree with the plan to `5e-3`.
   - **Track.** The true track projection of every sample stays inside the track. This
     uses `cartesian_to_frenet` on the real geometry, not the NLP's local frames.
   - **Operating domain.** `vx` and the inputs to `1e-4`; slip angles and `a_y` to
     `1e-2` (they are imposed exactly on the NLP's RK4 prediction).
   - **Obstacles.** A dense check (50 samples per interval) finds no contact (distance
     below `r_ego + r_i`) for any obstacle. Buffer intrusions are measured and reported.
     Distances at the constrained points stay at least `D_i − 5e-3`.

**On rejection.**
- The result has `accepted = false` and `u = nothing`, an explicit status, and the list of
  failed checks.
- The warm start is cleared.
- The closed-loop simulator stops with `:controller_failure`. It never applies a rejected
  or extrapolated control.

**Initialisation attempts in a cycle.**

| situation | attempts, in order |
|:--|:--|
| no obstacle within reach | warm start (the shifted previous plan), then a cold pure-pursuit guess |
| a newly relevant obstacle | the ranked side-sequence guesses (section 3.3), then the warm start |
| otherwise | warm start, then the committed sides' guess |

Further attempts and reassignments start only if the elapsed time plus the longest solve
so far fits in `time_budget`.

## 3. Multiple obstacles

### 3.1 Separation

Each obstacle `i` is a circle `(o_i, r_i)` with its own buffer `d_b,i`. The ego vehicle is
a disk of radius `r_ego = 0.05 m` centred at the CoM. The constraint, with

    D_i = r_ego + r_i + d_b,i,

is imposed in squared form, `‖p − o_i‖² ≥ D_i²`. This is smooth, and needs no
side-dependent linearisation.

All obstacles are imposed **simultaneously**. Two obstacles inside one horizon are both
constrained in the same NLP.

### 3.2 Slots and filtering

**Slots.** The NLP has one constraint slot per obstacle: Parameters `(ox, oy, D²)`.
- A slot is **active** (`D_i²`) when the obstacle centre is within
  `1.5 · vx_max · N · dt + D_i` of the CoM. This is a conservative Euclidean reach: a
  horizon cannot travel further.
- Otherwise the slot is **inactive** (`D² = 0`, trivially satisfied).

**Consequences.**
- No constraint can be left over from an earlier obstacle.
- Nothing is rebuilt when obstacles come and go.
- Single-obstacle behaviour is the special case K = 1. The kinematic and linearized
  single-obstacle controllers are untouched.

### 3.3 Passing-side initialisation

Ipopt is a local solver, so the initial guess decides on which side an obstacle is passed.
`side_sequence_guesses` builds pursuit guesses:
- the lateral target follows a smooth ramp to `ey_o ± (D_i + margin)` around each obstacle;
- the guesses are rolled out with the dynamic model.

Sides are chosen as follows.
- **Committed.** Obstacles already being passed keep their side.
- **Enumerated.** The first `max_side_obstacles = 2` newly encountered obstacles have both
  sides enumerated, so there are at most 4 sequences.
- **Greedy.** Any further obstacle takes the side with the wider gap.

The guesses are ranked by:
1. whether they clear all obstacles and stay on the track;
2. their minimum clearance;
3. their track clearance.

The work per cycle is bounded regardless of the number of obstacles.

**Reported side.** The side is that of the accepted plan's own path at its closest
approach. Ipopt may converge to the other side than the guess, so the guess's label is used
only while the plan does not yet reach the obstacle. This was corrected during the test
work of this milestone: previously the guess's label was reported even when the plan
passed on the other side.

### 3.4 What is and is not guaranteed

- **Guaranteed.** The constraints hold at the knots and at the RK4 sub-step positions of
  the NLP's prediction.
- **Not constrained.** Between sub-steps the path is a curve, and a chord can cut the
  buffer by up to its sagitta: about 1 cm at the showcase speeds. The dense validation
  measures this and reports it as a *buffer intrusion*.
- **Rejected.** Contact with an obstacle's physical radius in the validation rollout
  rejects the plan.

There is no continuous-time guarantee, and no guarantee that a feasible plan exists. A
blocked track is reported as an explicit controller failure, as tested.

## 4. Model selection and supported combinations

The showcases take `--model=dynamic|kinematic`, which selects **both** the controller and
the plant.

| `--model` | controller | plant | `race.jl` (no obstacle) | `multiple_obstacles.jl` |
|:--|:--|:--|:--|:--|
| `dynamic` (default) | `DynamicNMPCController` | 1 ms RK4 of the dynamic model | supported | supported |
| `kinematic` | `NMPCController` (Cartesian-endpoint, unchanged) | event-aware kinematic plant | supported | **error**: the kinematic NMPC handles at most one obstacle |

Any other value raises an error. The unsupported combination never falls back silently to
another model.

## 5. Obstacle layout of the showcase

Six obstacles on the original `quarter_track()` (width 0.70 m), each of radius 0.10 m, with
`d_b = 0.03 m` and `D = 0.18 m`. They are placed with `obstacle_on_track(TRACK, s, ey)`
from the real geometry.

| | s [m] | ey [m] | section |
|:--|--:|--:|:--|
| O1 | 3.60 | +0.06 | R = +1 m (left) bend |
| O2 | 6.75 | −0.06 | R = −1 m (right) bend |
| O3 | 14.90 | 0.00 | R = 2 m bend |
| O4 | 18.60 | −0.08 | 4 m straight |
| O5 | 25.40 | +0.10 | 6 m straight |
| O6 | 27.20 | −0.10 | 6 m straight, 1.8 m after O5, so both are in one horizon |

`check_layout` verifies the layout before every run, and the tests check it.
- **Passage.** Every obstacle leaves a passage of at least `D` on one side within the
  corridor.
- **Neighbouring sections.** No other part of the track comes within `E + D` of an
  obstacle.
- **Spacing.** Obstacles do not overlap each other's separation.

The positions are fixed for the whole run.

## 6. Showcases

### 6.1 Pipeline

`examples/showcase/race.jl` and `multiple_obstacles.jl` each:
1. build the controller;
2. time compilation separately (construction and first solve);
3. warm up with discarded runs;
4. run the closed loop afresh. Nothing is loaded from earlier runs.

They then export the run (`*_meta.json`, `*_cycles.csv`, `*_plans.csv`,
`*_trajectory.csv`, `*_track.csv`) and call `render_animation.py`.

The renderer:
- draws only what the export contains: the plant trajectory, the accepted predicted path,
  the solver status and acceptance of every cycle, and this run's statistics;
- refuses inconsistent data (`--selftest`);
- never interpolates over a failed cycle or edits frames;
- ends the video at the failure if a run fails, and reports it.

### 6.2 Common initial condition

Both models start with the rear axle on the start line on the centreline, heading along the
track, at 1.0 m/s, with no lateral velocity or yaw rate. For the dynamic model this is the
CoM state `lr` ahead of the rear axle. The test "shared initial physical condition" checks
the transform.

## 7. Results

### 7.1 Recorded showcase runs

Recorded with commit `b982f1b` on Julia 1.11.5. The timing values below use that revision's
internal controller timer, before the complete-call timing correction. Fresh showcases
measure around `solve!`; the historical values are preserved. Each run has 250 cycles (25 s simulated),
with no wall-clock budget. The videos, posters and full statistics are in `docs/media/`.

| run | outcome | laps | MPC time median / p95 / max [ms] | ≤ 100 ms | Ipopt it. median / max | NLP solves / cycle (mean / max) | min. track clearance |
|:--|:--|--:|--:|--:|--:|--:|--:|
| race, dynamic | 250/250 accepted | 2.56 | 87.5 / 175.9 / 532.5 | 78.0 % | 15 / 48 | 1.12 / 3 | 4.77 cm |
| six obstacles, dynamic | 250/250 accepted | 2.29 | 131.6 / 268.7 / 559.0 | 7.2 % | 17 / 77 | 1.12 / 3 | 4.77 cm |
| race, kinematic | 250/250 accepted | 2.59 | 13.2 / 28.5 / 65.9 | 100 % | 15 / 23 | 1.14 / 2 | 4.65 cm |

**Obstacle run** (six obstacles, 10 ms plant trajectory):
- no contact; minimum clearance above contact 1.94 cm;
- the required separation `D` was entered in 14 samples, by at most 1.06 cm, between
  constrained sub-step points (section 3.4);
- no track-edge or plant violations.

O5 and O6 (1.8 m apart, offset to opposite sides) are constrained in the same horizons.

**Observations.**
- **Progress.** The dynamic and kinematic races make similar progress (2.56 vs 2.59 laps).
  They are different vehicles: the dynamic one is limited by tyre slip and the dynamic
  `a_y`, the kinematic one by `v² tan δ / L`.
- **Cost of the dynamic NLP.** Per cycle it costs about 6.6× the kinematic one in median
  time, and about 10× with six obstacles. It has six states, RK4 stage variables and slip/`a_y` constraints at
  every sub-step.
- **Timing.** Many of its cycles exceed the 0.1 s control period on this laptop (in its
  slow state during these runs). The simulation does not model computation delay.
- **Reproducibility.** The outcome is reproducible: two independent runs of the obstacle
  configuration gave the same trajectory statistics. Only the timing differed.

### 7.2 Repeated runs

`benchmarks/dynamic_showcase.jl` runs each configuration three times:
- compilation is timed separately;
- each run has a warm-up;
- all cycles are kept.

The results are in `benchmarks/results/dynamic_showcase.md`.

### 7.3 Defects found and corrected during this milestone

- **Assignment cycles.** Reassignment could cycle at a segment boundary (section 2.2),
  which stopped the first 250-cycle race at cycle 225. It is now resolved by bounding the
  alternating knots; a regression test covers it.
- **Reported side.** The reported passing side was the guess's label, even when Ipopt
  converged to the other side (section 3.3). It is now taken from the accepted plan.
- **Time budget.** With the 0.3 s wall-clock budget first used for the showcase, the
  obstacle run failed at cycle 43 in the laptop's slow state: alternatives were "not
  attempted (budget)". The showcases now bound the work by count.

## 8. Limitations

- **Single model.** The NLP and the plant are the same model with different integrators.
  There is no model mismatch, no noise and no estimation.
- **Parameters.** Synthetic, not calibrated. The legacy tyres cannot race on this track
  (section 1.5).
- **Low speed.** The model is restricted to `vx ≥ 0.8 m/s`, so there are no standstill
  starts.
- **Local solutions.** Ipopt is local, so neither the plans nor the passing sides are
  globally optimal.
- **No continuous-time collision guarantee.** Buffer intrusions of about 1 cm between RK4
  sub-steps are possible, and are measured and reported.
- **Timing.** Solve times (tens of ms, with p95 above 100 ms for six obstacles) exceed the
  0.1 s control period in some cycles. The showcases are offline simulations; nothing here
  is real-time capable or certified. The time budget only limits extra attempts; it is
  not a deadline. The recorded showcases use none (the work is bounded by count, so the
  outcome is machine-independent). The simulation does not model computation delay: the
  control is applied at the start of its period whatever the solve time.
- **Obstacles.** Stationary only, and the kinematic NMPC still supports a single obstacle.
