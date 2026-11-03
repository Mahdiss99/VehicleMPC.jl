# Progress-aware curvature prediction (Milestone 2B.3A, experimental)

**Status: experimental. Not the default** (`curvature_model = :stagewise_average` remains
the default and is unchanged).

**Summary of the evidence:**

- **Single crossings.** For an isolated straight → arc transition from a cold start, the
  progress-linearized model with a trust region makes a single NLP solve consistent in
  9 of 9 cases. The stagewise model is stale in 9 of 9.
- **Quarter track, N = 20.** No progress-linearized variant completed a lap.
- **Quarter track, N = 10.** With the 10 cm margin and bounded corrections, it completed
  2.48 laps in each of 3 runs, every cycle strictly consistent:
  - mean 1.14 NLP solves per cycle;
  - median 8.9 ms, p99 23.0 ms;
  - **one cycle at 129 ms** in 750.

  The stagewise controller under the same conditions also completed every run (mean 2.12
  solves, median 15.8 ms, p99 52.4 ms, no cycle above 100 ms).
- **A one-solve default is not justified.** The pure one-solve linearized controller fails
  in the first 1–5 cycles at both horizons.

Code:
- `src/controllers/curvature_scheduling.jl`: `curvature_linearization`, `install_schedule!`
  and the diagnostics;
- `src/controllers/nonlinear_mpc.jl`: the augmented NLP;
- benchmark: `benchmarks/progress_aware_curvature.jl`, with results in
  `benchmarks/results/progress_aware_curvature.md`.

## 1. The original trajectory-based curvature update (`MPC with JuMP/main.jl`, unchanged)

- **Parameters and use.** `@NLparameter(model, κ[i=1:N] == 0.0)`. The dynamics constraint
  `Dynamics*[k in 2:N]` (interval `k−1 → k`) uses `κ[k]`.
- **Update after each solve:**
  - `κ[j] = get_curvature(track, predicted_states[1, j+1])` for `j = 2:N−1`, i.e. the
    **point** curvature at knot `j+1` of the previous prediction. After the one-step shift
    this is approximately the *end* knot `j` of the new interval `j−1 → j`;
  - `κ[N] = κ[N−1]`;
  - `κ[1]` is set to the current point curvature, but no constraint reads it.
- **First solve.** It uses `κ ≡ 0`.
- **Limitations:**
  1. **Point sampling.** A crossing anywhere inside an interval applies the full jump to
     the whole interval (end-point zero-order hold, cf. method A in
     `multisegment_prediction.md`).
  2. **No consistency.** Nothing checks the curvature against the new optimised progress.
  3. **First cycle.** The first cycle ignores the track curvature.
  4. **Legacy model and API.** It is coupled to the legacy kinematic model, which has a
     lateral-velocity error, a heading-sign error and ±90° steering, and it uses the legacy
     `@NLparameter` API.

  The idea retained here is to take the curvature of each stage from the previous
  prediction. The stagewise model (2B.2) replaced point sampling by the exact interval
  average. The model below adds first-order dependence on the optimised progress.

## 2. First-order sensitivity of the stagewise average curvature

For `a < b` (or `b < a`), with the continuous unwrapped centreline heading `psi_c`
(`psi_c' = kappa` almost everywhere):

```text
K(a, b) = (psi_c(b) − psi_c(a)) / (b − a)
∂K/∂a = (−kappa(a)(b − a) + psi_c(b) − psi_c(a)) / (b − a)² = (K − kappa(a)) / (b − a)
∂K/∂b = ( kappa(b)(b − a) − psi_c(b) + psi_c(a)) / (b − a)² = (kappa(b) − K) / (b − a)
```

These hold wherever `kappa` is continuous at the respective end point. The tests compare
them with central finite differences (to 1e-6) for:
- straight → arc;
- κ +1 → −1;
- two transitions in one stage;
- the lab track's short segments;
- a closed-track lap boundary.

## 3. Augmented NLP (`curvature_model = :progress_linearized`)

Per stage `k`:

```text
kappa_eff[k] = kbar_k + A_k (s_k − sbar_k) + B_k (s_{k+1} − sbar_{k+1}),
kappa_min ≤ kappa_eff[k] ≤ kappa_max                        (hard variable bounds)
x_{k+1} = F_RK4(x_k, u_k; kappa_eff[k], dt)
```

- **Inputs from outside Ipopt.** `kbar`, `A`, `B` (N each) and `sbar` (N+1) are JuMP
  Parameters, updated before every solve from the self-consistent schedule (2B.2 fixed
  point) along the warm-start or cold-start trajectory. The model is never rebuilt.
- **Frenet safety.** `kappa_eff` is a variable bounded to the track's curvature range. The
  2B.2 Frenet-safe bounds on the RK4 stage points cover that whole range (affine argument),
  so no evaluation can approach `1 − kappa·ey = 0`, at trial points too.
- **Measured state.** `X[:,1]` is evaluated with `kappa_eff[1]`, which may take any value
  in the range, so it is checked against both curvature extremes before scheduling.
- **Derivatives.** The `kappa_eff` constraints are Parameter × variable (quadratic in JuMP,
  affine once the Parameters are fixed). JuMP's AD differentiates them, together with the
  RK4 constraints that use `kappa_eff`. A test evaluates the constraint functions at
  constructed points (residual ≤ 1e-12), and solutions satisfy the affine model to 1e-6.
- **Objective and limits.** Unchanged.
- **Single-segment tracks.** These use the 2A/2B.2 fixed-curvature formulation, since
  `A = B = 0` there. The results are identical (tested bitwise).

**Optional trust region** (`linearization_trust_region = ρ`; `Inf`, i.e. off, by default).
This adds `|s_k − sbar_k| ≤ ρ·|Δsbar|` on the knots of every stage with non-zero
sensitivities. It is an additional constraint, introduced because of the failure described
in §6.

**Post-solve.**
- The 2B.2 consistency residuals are evaluated with the curvature actually used
  (`kappa_eff*`): `r_k = psi_c(s*_{k+1}) − psi_c(s*_k) − kappa_eff*_k·Δs*_k`.
- Also reported:
  - the model error `max_k |kappa_eff*_k − K(s*_k, s*_{k+1})|`;
  - `configuration_changes` (stages whose optimised end points lie in different segments
    than the nominal ones);
  - `saturated_stages` (`kappa_eff` on a bound while `kbar` is not);
  - active trust-region knots.
- The independent event-aware reference check, the knot validation with `kappa_eff*`, and
  all acceptance rules are those of 2B.2.

## 4. Non-differentiable situations

| situation | handling |
|:--|:--|
| interval inside one segment (including end points) | `K` locally constant; `A = B = 0` set **exactly**. Computing them from heading differences would add rounding noise of order `eps·|psi|/Δs²` |
| one or several transitions inside the interval | analytic `A`, `B` (§2) |
| `|Δsbar| < linearization_min_interval` (1 mm), stationary | constant local model `A = B = 0`; `kbar` from `stage_curvatures` (right limit below 1e-6 m); counted in `short_intervals` |
| nominal knot within `boundary_knot_tol` (1e-6 m) of a boundary | no ordinary derivative. The **mean of the left and right one-sided derivatives** is used (`kappa` at the knot = mean of both limits) and counted in `boundary_knots`. The model is then first-order accurate for neither direction. This is reported, not hidden |
| optimised end point moves into another segment | the local model's assumed configuration no longer holds. Counted in `configuration_changes`; the consistency residual and the reference check decide acceptance |
| closed-track lap boundary | unwrapped progress; segment ids include the lap |

No curvature is smoothed, and no solver exception is introduced: domain problems remain
explicit statuses.

## 5. Complexity

- **NLP size.** +N variables (`kappa_eff`), +N equality constraints (Parameter × variable),
  +4N+1 Parameters. With a trust region, another +2N linear constraints and N+1
  Parameters. For N = 20, RK4: 326 → 346 variables, 260 → 280 equalities.
- **Hessian.** It gains the `kappa_eff`–state couplings.
- **Iterations.** Measured mean Ipopt iterations per solve are 15.6–20.3 versus 13.9–15.0
  for the stagewise model.
- **Coefficients.** Computed in O(N), with segment searches (`curvature_linearization`).
  Their cost is part of the schedule time and is small next to a solve.
- **Main effect.** Fewer NLP solves per cycle: 1.14 instead of 2.12 on average for
  N = 10, as in the results below.

## 6. Experiments (`benchmarks/progress_aware_curvature.jl`)

**Setup.** Reference test vehicle, unchanged `quarter_track()`, start `[0, 0, 0, 0.5]`,
250 control cycles of 0.1 s, 3 runs per configuration interleaved in one process, and the
event-aware plant. Every cycle counts, including failed ones. Weights, limits, `ay_max`,
`epsi_max` and tolerances are unchanged; margins and horizons are stated per row.

### Horizon N = 20 (2 s; the 2B.2 setting)

| configuration | outcome (3 runs) | NLP solves mean / max | median / p99 / max [ms] | > 100 ms |
|:--|:--|:-:|:-:|--:|
| A stagewise, 1 solve, reject | stop at cycle 1 (×3) | 1 / 1 | 21.6 / 21.8 / 21.9 | 0 |
| B stagewise, ≤ 8 corrections, reject | stop at cycle 37 (×3) | 2.95 / 6 | 37.0 / 75.9 / 78.4 | 0 |
| C3 stagewise, corrections, qualified, margin 0.05 | 2.37 laps (×3); 36 qualified cycles | 2.49 / 7 | 30.2 / 76.5 / 110.6 | 1 |
| C4 stagewise, corrections, qualified, margin 0.10 | 2.33 laps (×3); 41 qualified cycles | 2.43 / 6 | 27.6 / 76.1 / 78.5 | 0 |
| D0 linearized, no trust region, 1 solve | stop at cycle 4 (×3) | 1 / 1 | 16.6 / 30.1 / 31.0 | 0 |
| D linearized, TR 0.1, 1 solve, margin 0.05 | stop at cycle 5 (×3), residual 0.0052 rad | 1 / 1 | 18.0 / 26.2 / 26.3 | 0 |
| D′ linearized, TR 0.1, 1 solve, margin 0.10 | stop at cycle 26 (×3), `LOCALLY_INFEASIBLE` | 1 / 1 | 18.1 / 167 / 173 | 3 |
| E linearized, TR 0.1, corrections, margin 0.05 | stop at cycle 103, 1.02 laps (×3): actual-edge violation 6 mm | 1.06 / 2 | 18.3 / 37.6 / 40.2 | 0 |
| E′ linearized, TR 0.1, corrections, margin 0.10 | stop at cycle 26 (×3), `LOCALLY_INFEASIBLE` | 1 / 1 | 18.8 / 170 / 172 | 3 |

**Failure mechanisms of the linearized model at N = 20 (diagnosed):**

1. **Terminal-stage exploitation (D0).** The final knot appears only in the progress reward.
   Through `B < 0` at a crossing stage, the optimiser moved it 0.25 m past the nominal (a
   whole stage length). That drove `kappa_eff` to its bound (−1.0) while the exact average
   was −0.32, giving a residual of 0.32 rad. The exact `K(b)` is strongly nonlinear: at
   δb = +0.025/+0.055/+0.095 m the linear model is off by 0.013/0.055/0.147 1/m.
2. **Trust-region trade-off (D, D′, E′).** A hard trust region keeps the model accurate,
   but can make the NLP infeasible. At cycle 26: with ρ = 0.1 the solve is infeasible
   after 84 iterations; with ρ = 0.3 it is feasible but stale (0.055 rad); without a trust
   region it is stale (0.121 rad). The infeasible solves also take up to 173 ms.
3. **Intra-stage averaging error (E).** With a consistent schedule, the actual-track
   re-simulation still crossed the edge by 6 mm at t = 1.84 s on the R = 1 m S-bend
   (Δκ = 2). This is the 2B.1 intra-stage error `Δκ·(v·dt)²/8` (about 2 cm per crossing
   at 3 m/s), which no schedule consistency removes.

### Horizon N = 10 (1 s) versus N = 20 (dt = 0.1 s = control period in both)

| configuration | N = 10: outcome · solves mean · median / p99 / max [ms] · > 100 ms | N = 20: outcome |
|:--|:--|:--|
| C4 stagewise, corrections, qualified, margin 0.10 | 2.46 laps ×3, **0 qualified** · 2.12 · 15.2 / 50.2 / 68.0 · 0 | 2.33 laps ×3, 41 qualified |
| B′ stagewise, corrections, reject, margin 0.10 | **2.46 laps ×3**, strictly consistent · 2.12 · 15.8 / 52.4 / 74.5 · 0 | stop at cycle 1 or 37 |
| L1 linearized, no TR, 1 solve, margin 0.10 | stop at cycle 1 ×3 | stop at cycle 5 ×3 |
| L8 linearized, no TR, corrections, reject, margin 0.10 | **2.48 laps ×3**, strictly consistent · **1.14** · **8.9 / 23.0 / 129.4** · **1** | stop at cycle 19 ×3 |
| L8s = L8 with margin 0.05 | stop at cycle 13 ×3 | stop at cycle 6 ×3 |
| T1 linearized, TR 0.1, 1 solve, margin 0.10 | stop at cycle 13 ×3 | stop at cycle 26 ×3 |

**L8 at N = 10 in detail** (750 cycles):
- all cycles strictly consistent, no plant violations, minimum plant distance to the edge
  0.098 m;
- maximum stage residual 4.7e-3 rad;
- 72 configuration changes and 39 saturated stages occurred in accepted cycles, so the
  local model's assumptions did not always hold; the consistency test and the reference
  check were the gate;
- prediction errors against the actual track: first-step ≤ 1.7 cm, horizon ≤ 3.5 cm,
  heading ≤ 4.7e-3 rad.

**Horizon findings:**
- The shorter horizon removes the terminal-stage problems and improves consistency for
  *both* formulations: with N = 10 the stagewise controller needs no qualified
  acceptances.
- Its cost is planning reach: 1 s look-ahead instead of 2 s. Lap progress was nevertheless
  slightly higher here (2.46–2.48 vs 2.33 laps in 25 s).
- The optional prediction steps `dt = 0.05` were not evaluated. Keeping the 10 Hz control
  period with a 50 ms prediction step needs a shift of two prediction steps per cycle and a
  first input held over two steps, which `solve!`/`simulate_closed_loop` do not implement
  yet.

### Straight → arc, cold start, one solve (9 cases: v = 1, 2, 2.8 m/s; crossing at 10/50/90 %)

| formulation | consistent after one solve | stage residual | position error vs actual track |
|:--|--:|:-:|:-:|
| stagewise | 0 / 9 | 0.06–0.15 rad | 15–21 cm |
| linearized, no trust region | 0 / 9 | 6.4e-3–7.9e-2 rad | 0.6–15 cm |
| linearized, TR 0.1 | **9 / 9** | ≤ 2.8e-3 rad | ≤ 1.0 cm |

### Single-segment regression

The straight (100 cycles) and circle (70 cycles) closed loops are identical
(`states` and `inputs` equal) with both curvature models, and identical to Milestone 2A.

Figure: [per-cycle time and NLP solves, run 1 of C4 (N = 20), B′ (N = 10) and L8 (N = 10)](figures/progress_aware_quarter.png).
L8's 129 ms cycle occurred in a later run and is not in this figure.

## 7. Comparison

| | legacy (`main.jl`) | stagewise average (2B.2, default) | progress-linearized (this milestone) |
|:--|:--|:--|:--|
| curvature per stage | point value at the predicted end knot | exact average over the nominal interval | average + first-order dependence on the optimised progress |
| consistency checked | no | yes (residuals + reference) | yes (same, with `kappa_eff*`) |
| exact in heading when | never | `s* = sbar` | `s*` within the nominal segment configuration, to second order |
| typical NLP solves on the quarter track | 1 (no check) | 2.1–2.5 | 1.1 (N = 10) |
| known failure modes | wrong model | slow correction convergence (N = 20) | terminal-stage exploitation; trust-region infeasibility; configuration changes |

## 8. Is a one-solve default justified?

**No.**
- **No one-solve run completed a lap.** Every one-solve configuration of the
  progress-linearized model (D0, D, D′, L1, T1) stopped within the first 26 cycles at both
  horizons.
- **The completed laps needed corrections.** L8 used them in 14 % of cycles, with up to 3
  solves.
- **L8 depends on non-default settings:**
  - the 1 s horizon (N = 10);
  - the 10 cm margin. With the calibrated 5 cm margin, L8s stopped at cycle 13;
  - and it had one deadline miss in 750 cycles.

  The stagewise controller under the same settings (B′, N = 10) completed the same laps
  with no deadline miss, at about 1.8× the median cost.

## 9. Remaining limitations

1. **The first-order model is only locally valid.** Its error grows quadratically with
   `δs/Δs` at crossing stages and is first-order wrong after a configuration change. The
   terminal stage is the most exposed.
2. **No reliable trust-region choice.** A hard trust region causes infeasibility; no
   setting was both feasible and accurate on every cycle. A softened (penalised) trust
   region would change the objective and was not tried.
3. **Margins and horizons.** The 5 cm margin was insufficient for every linearized
   configuration that got past the first few cycles. All completed laps used 10 cm (or the
   qualified stagewise policy). The `dt = 0.05` horizons were not evaluated.
4. **Deadline.** One cycle above 100 ms in L8, and one in C3. There is no real-time
   guarantee, and timings vary between sessions on this laptop.
5. **Safety meaning of the checks.** Reference-model feasibility over the horizon is a
   simulated property of the predicted plan. It is not a guarantee of future safety, and
   there are no recursive-feasibility results.

*Update (2B.3B):* the candidate below was implemented as the experimental
`:segment_pair_exact` mode; see `docs/segment_pair_curvature.md`.

**A candidate for review (not implemented at 2B.3A).** Within a fixed segment configuration
(start in segment `i`, end in segment `j`), the heading increment is *exactly affine*:
`psi_c(b) − psi_c(a) = c_ij + kappa_j·b − kappa_i·a`. The constraint
`kappa_eff[k]·(s_{k+1} − s_k) = c_ij + kappa_j·s_{k+1} − kappa_i·s_k` would then make the
consistency residual identically zero inside the configuration, with no division by
`Δs`, so it is well defined when stationary. That would remove failure mechanism 1 and the
need for a trust region. Only configuration changes would remain to be detected. This
departs from the first-order formulation specified for this milestone, so it needs
approval.
