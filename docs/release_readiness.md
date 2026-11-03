# Release readiness

This page gives the current release checklist and preserves the historical Milestone 5
integration audit below. Historical observations are identified by their milestone; they
are not a statement that a previously resolved item is still missing.

**Not done in Milestone 5:** no version tag, no GitHub release, no Julia General Registry
submission, and no archive or DOI record.

## 1. Current release checklist

1. **License present.** The root `LICENSE` contains the MIT License, copyright 2026
   Mahdis Rabbani. The README links to it.
2. **Citation metadata (optional).** A draft `CITATION.cff` is given in section 4 for
   review. It is not committed, because it would describe a release that does not exist
   yet.
3. **Version and tag.** `Project.toml` has `version = "0.1.0"`, appropriate for a first
   public research release. Tagging `v0.1.0` and creating a GitHub release are left to
   the author.
4. **Registry (optional).** Registration in the General Registry would need:
   - the license;
   - a `[compat]` entry for every non-standard-library dependency (present);
   - a decision on the package name (`VehicleMPC` must not already be registered, which
     needs checking at submission time).

The release corrections also cover single-interval dynamic-plan validation, complete-call
timing in fresh showcases, the renderer usage example and the dynamic-controller API.
Run `julia --project=. -e 'using Pkg; Pkg.test()'` and the CI smoke commands before tagging.
The README clone URL targets `Mahdiss99/VehicleMPC.jl`; verify that this is the final
published repository URL. A source ZIP does not contain Git history: review commit
messages and earlier tracked contents in the original checkout before changing visibility.

## 2. Historical Milestone 5 integration audit

### Fixed in Milestone 5 (low risk, no change to numerical results)

- **Misleading QP-feasibility statements.** The 4BC document and the `build_local_qp!`
  docstring said that the slacks make every QP feasible. They now state which constraints
  are softened and which remain hard, that feasibility holds only when the nominal
  trajectory satisfies the hard bounds, and that validation is mandatory. The docstring
  also still described the earlier L1 penalty as the default; the default is quadratic.
- **Linear controller, invalid measurement with an obstacle.** The result reported
  `obstacle.present = false`. It now reports the obstacle, with
  `failure_cause = :invalid_measurement`. Only this diagnostic field changed; the control
  outcome is unchanged.
- **README.** Rewritten for first-time readers; the milestone chronology moved to
  `docs/development_history.md`.
- **API documentation.** Added `docs/api.md`: both controllers, state conventions,
  solver status vs acceptance, result fields, timing and violation reporting.
- **Reliability coverage.** New tests check that:
  - `set_obstacle!` reaches the next QP's halfspace rows;
  - a rejected cycle of the linear controller leaves no warm start or trust-region memory
    and returns no control;
  - the invalid-measurement report above is correct.

  The equivalent nonlinear-MPC properties were already tested.

### Reported, not changed (larger or behaviour-changing)

- **Duplicated validation code.** `validate_plan` (4A, straight track only) and
  `validate_plan_on_track` (4BC, any track) overlap. Likewise the straight-model helpers
  `nominal_inputs`, `shifted_inputs` and `pursuit_input` overlap their `_local` versions.
  The straight versions are kept so that the 4A results reproduce bit for bit.
  Consolidation is possible, but would need a new regression baseline.
- **Development options in `LinearMPCConfig`.** These remain available, but were not
  benchmarked as defaults:
  - `slack_penalty = :l1/:none`;
  - `step_policy = :rollout` on the local model;
  - `update_mode = :rebuild`;
  - `warm_tail = :repeat`;
  - `primal_start = :none`.
- **Different defaults between the controllers:**
  - `horizon` is 20 (NMPC) and 10 (LMPC);
  - `encounter_strategy` is `:both` and `:sequential`;
  - `obstacle_samples` is 0 and 3.

  Unifying them would change existing examples and results.
- **Different numerical checks:**
  - *Tolerances.* The NMPC checks its knots with `acceptance_tol = 1e-6`. The LMPC
    validates its exact rollout with `validation_tol = 1e-4`, and speed with 1e-6.
  - *Track geometry.* The NMPC re-simulates its plan with the event-aware integrator
    (Frenet mode), or checks the exact map and segment assignment (Cartesian-endpoint
    mode). The LMPC projects its exact Cartesian rollout at 10 samples per interval.
  - *Obstacle.* Both use the same dense collision evaluation (50 samples per interval)
    and contact-based acceptance.
  - `validate_plan_on_track` can apply the LMPC validation to NMPC plans; this was used
    for the comparison.
- **NMPC passing-side bookkeeping.** At an encounter, the NMPC records the side of its
  plan at the knot nearest the obstacle association. If the plan does not reach the
  obstacle yet, this can record the side opposite to the guess that produced the plan:
  Example D reports `:left` after the `:right` guess was accepted. The recorded side is
  used for later retries. Changing it would change NMPC closed-loop behaviour, so it was
  not touched (the linear controller records the guess's side, since 4BC).
- **Dependency coupling.** Ipopt and OSQP are both hard dependencies, so a user of one
  controller installs both. Moving them to package extensions is possible on Julia
  ≥ 1.9, but is a structural change.
- **Compilation latency with time budgets.** The first call of each code path compiles
  (seconds). With the default 80 ms budgets, a fresh Julia session can fail its first
  cycles. The examples warm up first, and the API page documents this. A precompilation
  workload (for example PrecompileTools) would reduce the latency, but adds a dependency.
- **Garbage-collection pauses** of 100–128 ms occasionally exceeded 100 ms per cycle in
  the benchmarks. Reducing allocations is possible future work.
- **Bit-for-bit reproducibility of the linear MPC under CPU load.** Even with the time
  budget removed, one closed loop of the local model (quarter track, obstacle E3, M = 3)
  differed from cycle 2 onwards, with the same outcome. That run was made while another
  Julia process ran the test suite. Rerun on an idle machine, the Milestone 5 code
  reproduced the Milestone 4BC baseline bit for bit, and the baseline itself repeated
  exactly.
  - *Likely cause* (not proven): OSQP's default automatic adaptive-ρ interval
    (`adaptive_rho_interval = 0`) schedules ρ updates from the measured setup time, which
    makes the iterates depend on timing. This is the only timing-dependent element left
    when the budgets are infinite.
  - *Workaround:* a fixed interval can be set through
    `LinearMPCConfig(osqp_settings=["adaptive_rho_interval" => 25])`. This was not
    benchmarked and is not the default (the solver settings were not changed in
    Milestone 5).
- **Terminology.** The documents use "linearized MPC", "linear MPC" and "LMPC" for the same
  controller, and mix British and American spelling. Not changed in the historical
  reports.

## 3. Dependency and reproducibility checks

**`Project.toml`.**

| item | value |
|---|---|
| `name` | `VehicleMPC` |
| `uuid` | `320a1753-2a45-4350-bea1-9ba85918dfca` |
| `authors` | Mahdis Rabbani |
| `version` | `0.1.0` |
| `[compat]` | `julia = "1.10"`, ForwardDiff `0.10, 1`, Ipopt `1.16`, JuMP `1.32`, OSQP `0.8`, StaticArrays `1`, Parameters `0.12, 0.13` |
| test-only extras | `Parameters` (the track regression test loads the legacy `Environment.jl`, which uses it), `Printf` (the showcase test loads `examples/showcase/common.jl`), `TOML`, `Test` |

**Package code.**
- No plotting dependency (a test checks this). Plots lives in `examples/Project.toml`.
- No absolute paths in package, test or example code (a test checks this).

**Clean installation.** `Manifest.toml` is not committed (`.gitignore`). A clean copy of
the tracked files instantiates and passes `Pkg.test()` on Julia 1.10.9 and 1.11.5 (run for
Milestone 5).

**CI.** `.github/workflows/CI.yml` runs `Pkg.test()` on Julia 1.10 and 1.11
(ubuntu-latest, with dependency caching), followed by headless smoke runs of the four
standalone examples with `--no-time-budget`. Since the final milestone it also runs the three
supported showcase combinations without video (`--no-video`; the showcases set no time budget); Python is not
installed in CI, so the renderer is not exercised there. The full benchmarks are not run in CI.

**CI timing risk.** A few existing tests run closed loops with the default 80 ms time
budget. On a slow CI runner such a test could fail because of time rather than
correctness. The latest CI outcome is in the repository's Actions tab.

## 4. Draft citation metadata (for review, not committed)

```yaml
# CITATION.cff — DRAFT, not part of the repository. Fill in after a license and a release exist.
cff-version: 1.2.0
message: "If you use this software, please cite it as below."
title: "VehicleMPC: nonlinear and linearized model predictive control for a kinematic bicycle on piecewise constant-curvature tracks (Julia)"
authors:
  - family-names: Rabbani
    given-names: Mahdis
    email: mrabbani@ucdavis.edu
version: 0.1.0          # only once tagged
# date-released: YYYY-MM-DD   # only once released
# license: MIT                # only once a license is chosen
repository-code: "https://github.com/Mahdiss99/VehicleMPC.jl"
type: software
```

No DOI, archive record or publication exists. None should be added until one does.
