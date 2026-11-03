# Exact segment-pair curvature (Milestone 2B.3B, experimental)

*Update (2B.4):* the intra-stage error identified in §6 is removed by the experimental
Cartesian-endpoint prediction model; see `docs/cartesian_endpoint_nmpc.md`.

**Status: experimental, not the default.** `curvature_model = :stagewise_average` remains
the default and is unchanged.

**Summary of the evidence** (`benchmarks/results/segment_pair_curvature.md`;
`quarter_track()`, 3 × 250 cycles per configuration):

- **N = 20 (2 s) with a 10 cm planning margin.** The segment-pair controller with bounded
  corrections completed 2.35 laps in each run with **all 750 cycles fully consistent**:
  mean 1.08 NLP solves, median 16.4 ms, p99 37.7 ms, max 46.9 ms, no cycle above 80 ms.
  In the same setting the stagewise and progress-linearized controllers stopped within
  37 cycles.
- **With the 5 cm margin every formulation failed**, the segment-pair ones included.
  - The cause is now isolated. The segment-pair curvature residual was 1.5e-11 rad, but the
    event-aware re-simulation of the plan crossed the track edge by 1.5–5.4 mm.
  - That is the intra-stage error of holding one curvature per RK4 stage across a jump
    (multisegment_prediction.md, §3). An exact curvature *integral* does not remove it.
- **One solve per cycle is not reliable in general.**
  - Without membership bounds, one solve failed as soon as an optimised knot left its
    assigned segment (cycles 5–19).
  - With membership bounds, one solve completed N = 20 / 10 cm in all runs, but became
    infeasible at N = 10 (cycles 32–34).

Code:
- `src/controllers/curvature_scheduling.jl`: `heading_increment`,
  `segment_pair_assignment`, `install_segment_pair!` and the validity checks;
- `src/controllers/nonlinear_mpc.jl`: the persistent equality;
- `src/controllers/nmpc_config.jl`: `curvature_model`, `segment_membership`,
  `segment_change_policy`;
- benchmark: `benchmarks/segment_pair_curvature.jl`;
- tests: `test/test_segment_pair.jl`.

## 1. Derivation and exactness condition

The centreline heading `psi_c` is continuous and piecewise affine in progress, with slope
`kappa_j` on segment `j`. Assign knot `k` to the segment `σ(k)` containing its nominal
progress `sbar_k`, using the half-open convention: a knot exactly on a boundary belongs to
the segment it enters. On the closed interval `I_k = [lo_σ(k), hi_σ(k)]` of that segment,
`psi_c(s) = psi_c(sbar_k) + kappa_σ(k)·(s − sbar_k)`.

For stage `k`, with `i = σ(k)` and `j = σ(k+1)`, it follows that **for all `s_k ∈ I_k` and
`s_{k+1} ∈ I_{k+1}`**:

```text
psi_c(s_{k+1}) − psi_c(s_k) = Δpsibar_k + kappa_j (s_{k+1} − sbar_{k+1}) − kappa_i (s_k − sbar_k),
Δpsibar_k = psi_c(sbar_{k+1}) − psi_c(sbar_k).
```

This identity is exact, including any number of complete segments between the end points
and closed-track lap boundaries in unwrapped progress. The NLP imposes

```text
kappa_eff[k] · (s_{k+1} − s_k) = Δpsibar_k + kappa_j (s_{k+1} − sbar_{k+1}) − kappa_i (s_k − sbar_k)        (crossing stages)
```

so `kappa_eff[k]` equals the exact average curvature `K(s_k, s_{k+1})` **only while both
optimised knots stay in their assigned intervals**. Outside them, the identity uses the
wrong slope for the part of the move beyond the boundary. The result is then neither
exact nor a first-order approximation, so it is detected and reported (§4).

**Precision.** `Δpsibar_k` is computed as a local curvature integral (`heading_increment`:
the sum of `kappa_j` times the overlap of `[a, b]` with segment `j`), not as a difference of
global headings. Checked against `centerline_heading` differences over three tracks,
several laps and `Δs` from 1e-4 to 1.3 m, the maximum discrepancy is **1.1e-14 rad**.

**What this does not make exact.** The vehicle model still uses one curvature per RK4
stage. A trajectory that crosses a jump mid-stage is therefore integrated as if the average
curvature applied throughout the stage. Its heading at the knots is exact, but the lateral
position within and after the stage carries the intra-stage error
`≈ Δkappa·(v·dt)²·θ(1−θ)/2` (2B.1). The full stagewise vehicle integration is not exact.

## 2. The NLP form (persistent, one equality per stage)

```text
w_k · [ kappa_eff[k](s_{k+1} − s_k) − Δpsibar_k − kj_k (s_{k+1} − sbar_{k+1}) + ki_k (s_k − sbar_k) ]
  + (1 − w_k) · (kappa_eff[k] − kc_k) = 0,
kappa_min ≤ kappa_eff[k] ≤ kappa_max
```

- **Parameters.** `w`, `Δpsibar`, `ki`, `kj`, `kc` (N each) and `sbar` (N+1) are JuMP
  Parameters, updated each cycle. `w_k ∈ {0, 1}` selects the form without rebuilding the
  model.
- **The equality is smooth, but not convex.** For fixed Parameters it is bilinear in
  `(kappa_eff, s)` on crossing stages and linear otherwise. The constraint set is
  non-convex because of the bilinear term, and the RK4 dynamics are non-convex already.
  Ipopt therefore finds local solutions only.
- **Frenet safety.** `kappa_eff` is hard-bounded to the track's curvature range. The
  Frenet-safe stage-point bounds (2B.2) cover that whole range, and the measured state is
  checked against both extremes. No evaluation can approach `1 − kappa·ey = 0`.
- **Derivatives.** JuMP's AD differentiates the product Parameter × variable × variable.
  The tests check the constraint function at constructed points (residual ≤ 1e-12), and
  solutions satisfy the equality to ≤ 1e-7.

## 3. Degenerate stages

| stage (nominal) | form | rationale |
|:--|:--|:--|
| both knots in the same segment | `w = 0`, `kappa_eff = kappa_segment` | exact while both stay; avoids the bilinear form where `κ_eff` is constant anyway |
| `|Δsbar| < linearization_min_interval` (1 mm), including stationary | `w = 0`, `kappa_eff = kappa` of the **entering** segment (`σ(k)`) | at `Δs = 0` the bilinear equality is `0 = 0`, leaving `kappa_eff` free in its range. This rule pins it; no division by `Δs` |
| stationary exactly on a boundary | entering segment (half-open) | the segment the forward-driving vehicle enters |
| short interval straddling a boundary | as above; counted in `short_intervals` | heading error ≤ `|Δkappa|·1 mm` |
| several segments inside one stage | `w = 1`; `Δpsibar` contains the interior segments | the identity stays exact (tested) |

- **Pinning does not block motion.** `kappa_eff` is fixed, but `s_{k+1}` stays free, so the
  vehicle can accelerate from rest.
- **A "stationary" stage that starts moving across a boundary** puts its end knot outside
  the assigned interval. This is reported as a configuration change (§4).
- **Conditioning.** For crossing stages with small but non-zero `Δs*`, `∂/∂kappa_eff` of
  the equality is `Δs*`, so `kappa_eff` becomes weakly determined. It stays within its hard
  bounds, and the post-solve residual measures the actual error. Crossing stages shorter
  than 1 mm (nominal) use the constant form instead.

## 4. Segment-configuration validity: two approaches

**Approach A (validation; default for this mode).** The knots are free.

After the solve, every optimised knot is tested against its assigned interval (tolerance
1e-7 m). Stages with a knot outside are reported as `configuration_changes`; the
segment-pair identity is then not claimed to hold. Two policies decide what happens next:

- `segment_change_policy = :residual` (default): the consistency residuals, evaluated with
  the exact average curvature along the optimised progress, decide acceptance. A knot may
  leave its segment while the residual stays within the 5e-3 rad limit. Such cycles are
  counted.
- `segment_change_policy = :reject`: any configuration change makes the schedule stale.
  With `schedule_corrections > 0` the existing correction mechanism then re-solves with
  an assignment rebuilt from the optimised trajectory, within the 80 ms budget. Accepted
  cycles are then exact to the solver tolerance (residuals ≤ 1e-9 rad in the benchmark).

**Approach B (membership bounds, `segment_membership = true`).** Every progress knot
`s_k` (k ≥ 2) gets variable bounds `I_k`, updated each cycle with `set_lower_bound` and
`set_upper_bound`. The identity is then exact by construction, and Ipopt respects variable
bounds (to 1e-8). The costs:

- the terminal and intermediate knots cannot move past their segment's end within one
  solve;
- near transitions this can make the NLP infeasible. At N = 10 it did, at cycles 32–34,
  with `LOCALLY_INFEASIBLE` after up to 96 iterations. Corrections cannot help, because
  infeasibility is a solver failure, not staleness.

At N = 20 / 10 cm, approach B with one solve completed every run. So neither approach
dominates. Approach A with corrections was the only one that completed both horizons at
10 cm.

## 5. Results (quarter track, 3 × 250 cycles, 10 Hz control period)

**Laps per run, and fully consistent cycles:**

| configuration | N = 10, margin 0.10 | N = 20, margin 0.10 | margin 0.05 (N = 10 / N = 20) |
|:--|:--|:--|:--|
| A stagewise, ≤ 8 corrections, reject | 2.46 ×3, 100 % | stop at cycle 1/37/37 | stop at cycle 42 / 37 |
| B progress-linearized (2B.3A best) | 2.48 ×3, 100 % | stop at cycle 19 | stop at cycle 13/13/1 / 6 |
| C segment-pair, 1 solve | stop at cycle 5 (segment change) | stop at cycle 19 (segment change) | stop at cycle 5 / 6 |
| **D segment-pair, ≤ 8 corrections** | **2.48 ×3, 100 %** | **2.35 ×3, 100 %** | stop at cycle 97 / 6 (edge) |
| **D2 = D, reject segment changes** | **2.48 ×3, 100 %** | **2.35 ×3, 100 %** | stop at cycle 97 / 6 (edge) |
| E segment-pair + membership, 1 solve | stop at cycle 34 (infeasible) | **2.35 ×3, 100 %** | stop at cycle 32 / 6 |
| E8 = E with corrections | stop at cycle 34 (infeasible) | **2.35 ×3, 100 %** | stop at cycle 32 / 6 |

**Timing of the configurations that completed** (all 750 cycles, NLP solves per cycle,
milliseconds):

| configuration | N | solves mean / max | median / p95 / p99 / max | > 80 / > 100 ms |
|:--|--:|:-:|:-:|:-:|
| A stagewise | 10 | 2.12 / 8 | 16.1 / 39.3 / 53.2 / 72.5 | 0 / 0 |
| B progress-linearized | 10 | 1.14 / 3 | 9.3 / 19.6 / 23.3 / 28.6 | 0 / 0 |
| D segment-pair + corrections | 10 | 1.06 / 2 | 9.1 / 18.9 / 20.7 / 28.2 | 0 / 0 |
| D2 (reject segment changes) | 10 | 1.10 / 2 | 9.2 / 19.4 / 21.7 / 41.9 | 0 / 0 |
| **D segment-pair + corrections** | **20** | **1.08 / 2** | **16.4 / 32.5 / 37.7 / 46.9** | **0 / 0** |
| D2 (reject segment changes) | 20 | 1.14 / 4 | 16.6 / 34.2 / 43.6 / 70.4 | 0 / 0 |
| E membership, 1 solve | 20 | 1.00 / 1 | 16.4 / 19.9 / 23.3 / **139.9** | 1 / **1** |
| E8 membership + corrections | 20 | 1.00 / 1 | 16.4 / 19.9 / 23.2 / 90.9 | 1 / 0 |

- **Timing outliers are not algorithmic.** E and E8 performed identical computations,
  since E8's corrections never triggered and the trajectories are the same. Yet their
  maxima were 139.9 and 90.9 ms. The rare outliers are therefore operating-system or
  runtime variability on this laptop, not solver work.
- **Mean Ipopt iterations per solve** were 15.1–17.9 for the segment-pair runs and
  13.9–14.6 for stagewise.

**Accuracy and feasibility** (accepted cycles; N = 20, margin 0.10; D / D2 / E):

- **Stage residual.** D 4.5e-3 rad (21 accepted cycles with a segment change under the
  residual policy). D2 9.9e-10 rad (no segment changes accepted). E 1.2e-6 rad.
- **Predicted vs actual-track reference.** First-step error 2.0e-2 m. Horizon position
  error 3.8 / 4.1 / 4.0 cm. Heading error ≤ 4.5e-3 rad.
- **Minimum physical edge clearance:**
  - **predicted** (reference re-simulation): 4.9 cm;
  - **plant**: 15.2–15.7 cm. No plant constraint violation in any run.
- **Minimum clearance from the tightened planning corridor:**
  - **predicted**: −5.1 cm. The re-simulated plans leave the 10 cm-tightened corridor by up
    to about 5 cm between knots and near the horizon end, but not the track.
  - **plant**: +5.2 to 5.7 cm.
- **Progress.** Minimum optimised stage progress 0.070 m (N = 20), so no stalled or
  backward predicted progress occurred.

**Single transition** (straight → arc, cold start, one solve, 9 cases):
- segment-pair: `:success` in 9 of 9, residual ≤ 3.9e-9 rad, horizon prediction error
  0.3–4.9 mm;
- stagewise and progress-linearized (without trust region): stale in 9 of 9.

**Single-segment regression.** The straight and circle closed loops are identical
(`states` and `inputs`) under the segment-pair option and the default.

Figure: [per-cycle time and solves at N = 20 / 10 cm: A (stopped at cycle 1 in run 1) and D](figures/segment_pair_quarter.png).

## 6. Why the 5 cm margin fails

With the 5 cm margin, all segment-pair variants at N = 20 stopped at the same cycle (6),
with the curvature residual at 1.5e-11 rad. The reference check found the plan crossing
the actual edge by 5.4 mm at predicted time t = 1.84 s (the quarter track's largest curvature jump is Δκ = 2, at the R = ±1 m S-bend).

At N = 10 they reached cycle 97 and stopped with a 1.5 mm crossing. Since the curvature
integral was exact, this is the intra-stage averaging error: about 2 cm per crossing at
3 m/s, plus the excursion between knots of a plan that rides the corridor edge (predicted
corridor clearance down to −4.7 cm). These exceed what a 5 cm margin can absorb. The
reference check rejects such plans correctly.

## 7. Comparison

| | stagewise (default) | progress-linearized | segment-pair |
|:--|:--|:--|:--|
| curvature model | fixed average over the nominal interval | first-order in the optimised knots | exact within the assigned segments |
| error after the knots move inside their segments | first order in `δs` | second order in `δs/Δs` | **zero** |
| error after a knot changes segment | first order | first order | wrong slope beyond the boundary; detected |
| extra NLP size | none | N vars + N equalities | N vars + N equalities (bilinear) |
| quarter track, N = 20 / 10 cm, strict | not completed | not completed | **completed (D, D2, E)** |

## 8. Real-time behaviour

All timings are wall-clock measurements on one Windows laptop (Julia 1.11.5, one thread).

- **D at N = 20 / 10 cm:** 0 of 750 cycles above 80 ms.
- **Other runs:** isolated cycles of 90–140 ms appeared that are not explained by the
  computation itself (§5).

There is no real-time guarantee, no deadline-aware fallback and no recursive feasibility.

## 9. Recommendation (for review)

1. **Keep `:stagewise_average` as the default for now**, as instructed.
2. **Candidate for promotion after review:**
   - `:segment_pair_exact` with approach A, `segment_change_policy = :reject` (D2) and
     bounded corrections;
   - or the residual policy (D), which has slightly better timing.

   Both were the only configurations that completed both horizons with strict consistency
   at a 10 cm margin. D2 accepts only cycles whose identity holds to the solver tolerance.
3. **N = 20 is practical with segment-pair + corrections at a 10 cm margin** in these
   tests. It is not practical at 5 cm with any formulation.
4. **The next accuracy limit is intra-stage, not the curvature integral.** Options (not
   implemented; need approval):
   - an event-split crossing stage (2B.1 method D) with the segment-pair assignment, so
     the split time is consistent with the same segment configuration;
   - or a smaller prediction step around transitions.

   Either would address the remaining 5 cm-margin failures.
