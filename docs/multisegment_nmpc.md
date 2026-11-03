# Multi-segment nonlinear MPC (Milestone 2B.2)

**Status: functional prototype with unresolved timing, and with an important qualification.**

- **Default configuration fails.** With the default configuration (one NLP solve per
  cycle, stale schedules rejected) the controller cannot start on `quarter_track()`: the
  first cycle is always rejected as schedule-inconsistent.
- **Laps need opt-in settings.** Complete laps (2.37 laps in each of three 250-cycle runs,
  with no plant constraint violation) were obtained only with two non-default options:
  bounded schedule corrections (≤ 8 extra NLP solves within an 80 ms budget), and the
  opt-in policy that accepts a still-stale schedule if the independent event-aware
  re-simulation on the actual track satisfies every hard constraint.
- **Deadline.** In that configuration one cycle in 250 exceeded 100 ms in the final run, so
  the 10 Hz requirement is not met in every cycle.

There is no collision avoidance. The reference test vehicle and the legacy track are not
calibrated against a physical system.

Code:
- `src/controllers/curvature_scheduling.jl` (new);
- `src/controllers/nonlinear_mpc.jl` and `nmpc_config.jl` (extended);
- `src/simulation/closed_loop.jl` (event-aware plant);
- examples: `examples/nmpc_quarter.jl`;
- experiments: `benchmarks/multisegment_nmpc.jl`, with results in
  `benchmarks/results/multisegment_nmpc.md`.

## 1. Prediction model

The kinematic bicycle, the objective, the constraints and the persistent JuMP model of
Milestone 2A are unchanged (`docs/nmpc_formulation.md`). The only modelling change is the
curvature used on stage `k`:

```text
x_{k+1} = F_RK4(x_k, u_k; kappa_k, dt),     kappa_k = (psi_c(s̄_{k+1}) − psi_c(s̄_k)) / (s̄_{k+1} − s̄_k)
```

Here `psi_c` is the unwrapped centreline heading and `s̄` a nominal progress trajectory.

- `kappa_k` is computed outside Ipopt (`curvature_schedule`/`stage_curvatures`) and written
  into the existing `kappa[k]` JuMP Parameters before every solve.
- The NLP sees constants, so it stays as smooth as in 2A: there is no segment look-up, no
  `mod`, and no derivative through the piecewise curvature.
- Ipopt's derivatives of the dynamics constraints use the updated values. A test evaluates
  every dynamics constraint at a rollout consistent with the current schedule (residual 0)
  and with another schedule (residual > 1e-3).
- The JuMP model is never rebuilt when the predicted trajectory crosses a boundary.

Single-segment tracks are unchanged: they use the exact segment curvature on every stage,
with no averaging and no rounding. Their results are bitwise identical to Milestone 2A
(§7).

## 2. Frenet safety for every legal curvature

Every stagewise average is a convex combination of segment curvatures, so it lies in
`[kappa_min, kappa_max]` over all segments. The condition `1 − kappa·ey ≥ 2·eps_geometry`
is affine in `kappa` (and in `ey`), so it holds on that whole range iff it holds at the two
extremes. The constructor therefore:

- checks the knot corridor `|ey| ≤ width/2 − track_margin` at both extremes, and throws
  `ArgumentError` when it fails (e.g. a κ = 4 arc on a 0.7 m track);
- bounds the RK4 stage points by the intersection of both half-spaces. For the quarter
  track (κ ∈ [−1, 1]) this is `−0.8 ≤ ey ≤ 0.8`, so the denominator is ≥ 0.2 at every
  evaluation for every legal parameter value.

`set_curvature_schedule!` range-checks each value and throws for anything outside the
range, so no curvature update can break this guarantee. The measured state is checked with
the **actual** curvature at its position, and also with the first stage's scheduled
curvature, since `X[:,1]` is a Parameter at which the dynamics are evaluated.

## 3. Schedule construction (before the solve)

Fixed point (`curvature_fixed_point`) for an input sequence `U` from the measured state:

```text
repeat (≤ schedule_max_iter = 20):
    kappa = stage_curvatures(track, s);   X = rollout(x, U, kappa)
    if max_k |X[1,k] − s_k| ≤ schedule_tol (1e-6 m): converged
    s = X[1, :]
```

**Initialisation.**
- **Warm start:**
  1. take the inputs shifted from the last accepted solution;
  2. take the nominal progress from that solution, shifted and re-anchored at the measured
     progress;
  3. run the fixed point.

  If it does not converge, exactly one cold initialisation is tried (`:cold_fallback`).
- **Cold start:** a quasi-steady centreline speed profile (`quasi_steady_guess`):
  - speed limited by `v ≤ min(v_max, sqrt(ay_max/|kappa(s)|))`;
  - a forward pass at `a_max` and a backward pass at `|a_min|` on a 1 cm progress grid;
  - curvature feed-forward steering.

  This replaced a first version that used constant speed and zero acceleration. That
  version produced a trivially "converged" all-straight schedule whose optimised solution
  failed both the consistency test (0.5 rad) and the reference check.

**Failure handling.** A schedule that has not converged is never used. If no converged
schedule exists, `solve!` returns `:scheduling_failure` without calling Ipopt. The same
happens when the nominal horizon leaves an open track.

**Stationary vehicle on a boundary.** A stage whose nominal progress changes by less than
1e-6 m uses the right-limit curvature, i.e. the segment containing `s_k`. Under the
forward-driving model (`v ≥ 0`, `|epsi| ≤ 60°`, so `sdot ≥ 0`) that is the segment the
vehicle can enter. `average_curvature(track, b, b)` at a jump still throws; the controller
does not call it there.

## 4. Post-solve validation

**Knot validation** (as in 2A): solver status, finiteness, dynamics residual with the
scheduled curvature, bounds, actuator limits.

**Schedule consistency** along the optimised progress `s*`:

```text
r_k = psi_c(s*_{k+1}) − psi_c(s*_k) − kappa_k·(s*_{k+1} − s*_k)          [rad]
```

`r_k` is exactly the absolute-heading error that the averaged curvature contributes on
stage `k` of the optimised trajectory (`docs/multisegment_prediction.md`, §2). Its partial
sums are the predicted heading errors at the knots. Defaults:

- `max |r_k| ≤ 5e-3 rad` and `max |Σ r| ≤ 1e-2 rad`. For scale, 0.01 rad over the
  remaining horizon (≤ 2 s at ≤ 2 m/s, i.e. 4 m) is a lateral prediction error of at most
  about 4 cm.
- The progress mismatch `max |s* − s̄|` and the number of crossing stages are always
  reported. The mismatch is not used for acceptance by default.

**Independent reference check on the actual track** (mandatory by default):
1. re-simulate the optimised inputs from the measured state with `simulate_on_track` (actual
   piecewise curvature, located crossings, 5 samples per interval);
2. check at every sample: the actual track edge (no margin), speed, inputs,
   `|epsi| ≤ epsi_max`, and `|ay| ≤ ay_max` at both ends of every sub-interval;
3. report the knot errors of the prediction against this reference (position, heading,
   progress, `ey`).

Leaving the model domain or an open track fails the check.

**Statuses.**

| status | meaning | control applied? |
|:--|:--|:-:|
| `:success` | knots valid, schedule consistent, reference passed | yes |
| `:schedule_inconsistent` | valid for its schedule, but the schedule is stale along the optimised progress | no |
| `:reference_violation` | the actual-track re-simulation violates a hard constraint or the domain | no |
| `:scheduling_failure` | no converged schedule; solver not called | no |
| `:solver_failure` / `:invalid_solution` | as in 2A | no |
| `:success_stale_schedule` | **opt-in** (`stale_schedule_policy = :accept_if_reference_valid`): stale after all corrections, but the reference check passed | yes (qualified) |

A hard-constraint failure takes precedence over staleness. There is no fallback input.

## 5. Schedule corrections and the computation budget

The default is **one NLP solve per cycle**. Optionally (`schedule_corrections > 0`), a
stale result triggers a correction:
1. rebuild the schedule by the fixed point from the solution's inputs and progress;
2. warm-start from the solution;
3. re-solve.

A correction starts only if the elapsed time plus the duration of the previous
`optimize!` stays within `correction_time_budget` (80 ms). This is an estimate: a
correction that runs longer than the previous one can still overshoot the deadline, and
did once (110.9 ms). Every cycle's `timing.total` includes:
- schedule construction;
- parameter and start-value updates;
- all NLP solves;
- knot validation;
- the consistency test;
- the reference check.

`nlp_solves` is recorded per cycle.

## 6. Fixed-point and correction convergence (observed)

**Pre-solve fixed point.**
- It converged in every cycle of every Experiment D run, with at most 39 iterations summed
  over a cycle's corrections.
- A **correction's** fixed point did fail once: in D5 at cycle 9, its rollout left the
  model domain. The failure was reported, and the cycle was rejected.
- Scale-error regression (2B.1): a nominal 10–20 % too slow or too fast converges to the
  same schedule (to 1e-6).
- Forced non-convergence (`schedule_max_iter = 1`) is reported as `:scheduling_failure`.

**Why one solve is rarely enough.** The schedule is consistent with the *guess's* progress.
The optimiser then accelerates or brakes differently, which moves the progress at the
crossing stage by `δs`, and `r ≈ Δkappa·δs`. To meet 5e-3 rad at `Δkappa ≈ 1`, `δs` must
be below 5 mm.

**Experiment C** (straight → R = 2 m arc, speeds 1–2.8 m/s, crossing at 10/50/90 % of the
first interval):
- cold start with one solve: always stale, 0.06–0.15 rad;
- warm start with one solve: 3 of 9 cases consistent, the others 6.6e-3–1.2e-2 rad;
- with corrections: all consistent after 4 solves (cold) or 1–2 solves (warm);
- the reference check passed in all 36 cases.

**Correction iteration on the quarter track.** It is a contraction, but its rate varies:
- **Cold start:** 0.179 → 0.069 → 0.036 → 0.016 → 0.008 → 0.0038 rad, accepted after 5
  corrections (factor about 0.5).
- **One closed-loop cycle at s ≈ 8.5 m** (entering the κ = 0.5 arc from the κ = −1 arc at
  about 3.2 m/s): 0.0445 → 0.040 → 0.029 → 0.014 → 0.0067 → 0.0046 rad, needing 15 solves
  (factor about 0.85).

These were observed in exploratory runs during development. The rate is set by how
strongly the optimised braking point depends on the averaged curvature of the crossing
stage. No cycling was observed.

## 7. Results

### Single-segment regression (Experiments A and B)

With the default configuration, 100 straight-track cycles and 70 circle cycles were all
`:success` with one solve each. The examples' CSV outputs (state, input and solver-status
columns) are **bitwise identical** to Milestone 2A (commit `fd8c357`). The added reference
check costs 0.06–0.08 ms per cycle.

### Quarter track (Experiment D, 250 cycles, three runs per configuration)

| configuration | outcome (3 runs) | statuses (run 1) | plant violations | max plant \|ey\| | median / p95 / p99 / max [ms] | > 80 / > 100 ms |
|:--|:--|:--|--:|--:|:-:|:-:|
| D1 default: 1 solve, reject stale, margin 0.05 | stopped at cycle 1 (×3) | `schedule_inconsistent` 1 | 0 | 0 | 20 | 0 / 0 |
| D2 reject stale, ≤ 8 corrections in 80 ms | stopped at cycle 37 (×3; in an earlier run once at cycle 1) | success 36, stale 1 | 0 | 0.147 | 34 / 69 / 71 / 72 | 0 / 0 |
| **D3 qualified, ≤ 8 corrections in 80 ms, margin 0.05** | **2.37 laps (×3)** | success 237, stale-but-verified 13 | **0** | 0.152 | 30 / 70 / 77 / 111 | 1 / **1** |
| D4 = D3 with margin 0.10 | 2.33 laps (×3) | success 237, stale-but-verified 13 | 0 | 0.124 | 27 / 70 / 78 / 79 | 0 / 0 |
| D5 = D4, cold start every cycle | stopped at cycle 9 (×3) | `reference_violation` at cycle 9 | 0 | 0.044 | 70 / 80 / 81 / 81 | 1 / 0 |

D3 run 1 in detail:
- 26 boundary crossings by the plant;
- solves per cycle: 1 (70 cycles), 2 (68), 3 (62), 4 (24), 5 (19), 6 (5), 7 (1), 8 (1);
- no solver failures and no scheduling failures.

Accepted predictions against the actual-track reference:
- worst position error 0.11 m, heading 0.054 rad, progress 0.07 m, `ey` 0.075 m;
- these maxima occur in the stale-but-verified cycles;
- consistent cycles are within about 3 cm (D2: 3.1 cm);
- one-step prediction against the plant: ≤ 1.7 cm.

D1 is deterministic. D2's stopping cycle depends on wall-clock time (budget).

Figure: [quarter-track trajectory, `ey`, speed and per-cycle computation time (D3)](figures/multiseg_nmpc_quarter.png).
Reproduce it with `julia --project=. examples/nmpc_quarter.jl`, or
`julia --project=examples examples/plot_nmpc.jl` for the plots.

### Is the 5 cm margin sufficient?

**Evidence for.** In D3 (≤ 8 corrections) the plant never came closer than 0.20 m to the
track edge (max `|ey|` 0.152 m on ±0.35 m), in three runs. The margin was not tested hard
by the trajectory itself.

**Evidence against.** In an exploratory run with ≤ 3 corrections, at the same s ≈ 8.3 m
transition, a plan was rejected: its re-simulated horizon tail crossed the actual edge by
3.1 cm (`ey = −0.381 m` at t = 2.0 s). The reference check stopped that run. Stale-schedule
prediction errors of 4–16 cm were observed, which exceeds 5 cm.

**Conclusion.** The 5 cm margin is **not established as sufficient**. Feasibility of
applied controls rests on the independent reference check, not on the margin.

### Computation time and bottleneck (D3, all 250 cycles of run 1)

| component | mean [ms] |
|:--|--:|
| schedule construction (warm) | 0.05 |
| parameter update + start values | 0.15 |
| NLP solves (`optimize!` wall time, summed per cycle) | 32.4 |
| knot validation | 0.13 |
| reference check | 0.54 |

- **Bottleneck.** It is the **number of NLP solves** needed to make the schedule
  consistent: each solve takes about 10–20 ms here, and up to 8 corrections are allowed.
  Scheduling, validation and the reference check together take < 1 ms.
- **Cold vs warm.** Cold starts are much worse. With a cold start every cycle (D5) the
  median is 70 ms, and the run fails at cycle 9: the corrections' rollout left the model
  domain and the plan violated the actual edge. Warm starts (D4) have a median of 27 ms.
- **Wall-clock variability.** Absolute times again varied between sessions on this laptop
  (per-solve times of 10–50 ms on different runs).

## 8. Limitations and open issues (for review)

1. **The default configuration fails on multi-segment tracks.** With fixed per-stage
   curvature parameters and one solve, the schedule is almost never consistent after the
   optimiser moves the progress. Laps require bounded corrections plus the opt-in
   qualified acceptance.
2. **Correction convergence is slow and variable** (factor 0.5–0.85 per solve), so the
   number of solves per cycle is unpredictable. The deadline was missed once in 250 cycles
   (111 ms), and the strict policy ran out of budget at s ≈ 8.3 m.
3. **Qualified acceptances use knowingly inaccurate predictions** (up to 0.11 m and
   0.054 rad against the actual track). Their safety rests entirely on the reference check
   over the horizon. There is no guarantee beyond the horizon.
4. **No recursive feasibility, no terminal ingredients, no real-time guarantee.** The
   timings are wall-clock measurements on one Windows laptop with large run-to-run
   variability.
5. **Margin.** The 5 cm margin is not proven sufficient (§7).

*Update (2B.3A):* the first direction below, a first-order schedule sensitivity inside the
NLP, was implemented as an experimental option and evaluated. See
`docs/progress_aware_curvature.md`. It is not the default.

**Candidate directions for Milestone 2B.3** (not implemented; they need approval):
- **First-order schedule sensitivity inside the NLP.** Replace `psi_c(s_k)` in the stage
  curvature by its first-order expansion about the nominal knots. The coefficients are
  computed outside Ipopt, so the NLP stays smooth. The schedule would then follow small
  progress changes within one solve, and the slow correction iteration should become
  unnecessary in most cycles.
- **Split only the crossing stage** at a parameterised time (2B.1 method D, first stage or
  crossing stage only).
- **Ipopt dual warm start**, or a real-time-iteration / SQP scheme, to reduce the cost per
  correction.
- **Terminal speed or curvature constraints**, so the horizon tail does not end in
  near-limit states, which is where the reference check rejected plans.
