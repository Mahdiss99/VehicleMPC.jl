# Racing showcases

Two closed-loop racing demonstrations on the original `quarter_track()` (34.85 m, 0.70 m wide):

| script | scenario |
|:--|:--|
| `race.jl` | full-track racing, no obstacles |
| `multiple_obstacles.jl` | full-track racing past six stationary obstacles, several of them in one horizon |

**Every run solves the MPC afresh.** No trajectory, plan or statistic is loaded from an
earlier run. The video and poster are rendered only from the data the run has just
exported.

## Requirements

- **Julia.** Either environment works, because the showcases need only `VehicleMPC`
  and the standard library:
  - the `examples` environment (Julia ≥ 1.11, as for the other examples):

        julia --project=examples -e "using Pkg; Pkg.instantiate()"

  - or the package environment itself (Julia ≥ 1.10), e.g.
    `julia --project=. examples/showcase/race.jl`. CI uses this form.

- **Python** (only for `--video`). Python ≥ 3.10 with the packages in
  `requirements.txt`: matplotlib, numpy, and imageio-ffmpeg, which bundles ffmpeg with
  libx264.

      python -m venv .venv-showcase
      .venv-showcase/bin/pip install -r examples/showcase/requirements.txt   # Windows: .venv-showcase\Scripts\pip

  Pass the interpreter with `--python=PATH` or the environment variable
  `VEHICLEMPC_PYTHON`. The default is `python`.

## Commands

    julia --project=examples examples/showcase/race.jl --model=dynamic --video
    julia --project=examples examples/showcase/multiple_obstacles.jl --model=dynamic --video
    julia --project=examples examples/showcase/race.jl --model=kinematic --video

| option | meaning |
|:--|:--|
| `--model=dynamic` (default) \| `kinematic` | vehicle model of **both** the MPC and the simulated plant |
| `--video` (default) / `--no-video` | render `<name>.mp4` and `<name>_poster.png` after the run |
| `--cycles=N` | control cycles (default 250, i.e. 25 s of simulated time) |
| `--out=DIR` | output directory (default `examples/showcase/output/`, git-ignored) |
| `--python=PATH` | Python interpreter for the renderer |
| `--fps=N` | video frame rate (default 30) |
| `--time-budget=S` | a wall-clock budget [s] for extra initialisation attempts and reassignments (default: none; see below) |

The renderer can also be called on its own, on a run's exported files:

    python examples/showcase/render_animation.py --run examples/showcase/output/race_dynamic [--fps 30] [--speed 0]
    python examples/showcase/render_animation.py --run examples/showcase/output/race_dynamic --selftest

## Supported combinations

| `--model` | controller | plant | `race.jl` | `multiple_obstacles.jl` |
|:--|:--|:--|:--|:--|
| `dynamic` | `DynamicNMPCController`: six-state dynamic bicycle, RK4 in the NLP | 1 ms RK4 of the dynamic model | ✓ | ✓ |
| `kinematic` | `NMPCController`: the existing kinematic NMPC, Cartesian-endpoint mode | event-aware kinematic plant | ✓ | ✗ explicit error |

The kinematic NMPC supports at most one obstacle. `multiple_obstacles.jl --model=kinematic`
therefore stops with an error; it never falls back to the dynamic model. Any other
`--model` value is also an error.

**Common to both models:**
- the same track, horizon (N = 10), period (0.1 s) and objective weights;
- the same initial physical condition: the rear axle on the start line, on the centreline,
  at 1.0 m/s, with no lateral speed and no yaw rate.

The dynamic model's state is the CoM, `lr = 0.125 m` ahead of the rear axle; the transform
is tested. Model details, parameters and limits are in `docs/dynamic_bicycle_mpc.md`.

## Work per cycle: bounded by count, not by time

The showcases set no wall-clock budget. The work per cycle is bounded by count instead:
- at most 5 initialisation attempts per cycle;
- at most `max_reassignments = 4` re-solves per attempt.

Removing the time budget avoids retry decisions based on machine speed. Exact numerical
outcomes can still vary with floating-point arithmetic and solver versions. The computation
time of every cycle is measured, reported, and shown in the video.

**Why.** With a 0.3 s budget, a slow state of the development laptop stopped the
multi-obstacle run at cycle 43. The budget expired before the alternative initialisations
could be tried. The same configuration without a budget completes.

This is a showcase setting. The controller default (`DynamicNMPCConfig().time_budget =
0.1`) is unchanged. Cycles that take longer than the 0.1 s control period are counted in
the statistics. The simulation does not model computation delay, and nothing here is a
real-time claim.

## What a run does

1. **Feasibility check.** `multiple_obstacles.jl` checks the obstacle layout first
   (`check_layout`): a passage on at least one side of each obstacle, no other track
   section nearby, and no overlapping separations.
2. **Controller construction**, timing compilation separately: model construction and the
   first solve.
3. **Warm-up** with discarded closed-loop runs, so that compilation is not in the
   statistics.
4. **Recorded closed loop**, with every cycle kept. A rejected plan is never applied; the
   run then stops with `controller_failure`.
5. **Export** to `<out>/<name>_<model>_*`:

   | file | contents |
   |:--|:--|
| `meta.json` | model, parameters, configuration, initial condition, obstacles, statistics, timing definition, compilation times, Julia version, commit (`-dirty` if the code differs from it) |
   | `cycles.csv` | per cycle: state, control (`NaN` if rejected), solver status, acceptance, iterations, timing |
   | `plans.csv` | the accepted predicted path of every cycle |
   | `trajectory.csv` | the plant trajectory every 10 ms |
   | `track.csv` | centreline and edges |

6. **Rendering** (`--video`): an H.264 MP4 (yuv420p, `+faststart`) and a PNG poster.

The script exits with status 1 if the closed loop did not complete. The exported data and
the video then show the failure; nothing is interpolated or edited.

## The video

**Track view.** A rotated vehicle body with an arrowhead along the heading ψ. Obstacles
are drawn to scale, with their required separation as a dashed circle. Also shown: the
accepted predicted path of the current cycle, and the driven trail.

**Close-up inset.** Drawn to scale. The body outline is illustrative: the collision proxy
is the 5 cm disk.

**Telemetry.**
- Model and solver.
- Speed `v_x`, steering δ and lap progress.
- Ipopt status versus acceptance of the current cycle, kept separate: a locally optimal
  solution can still be rejected by the validation.
- MPC computation time.
- For the obstacle run: obstacle count, current clearance and minimum clearance so far.

**Timing.** Playback is accelerated to about 20–25 s, and the factor is shown in the
footer.

**End card.** The statistics of this run.

## Cycle timing

Fresh runs record wall-clock time immediately before and after each complete `solve!`
call. The reported `total_ms` and timing statistics include final controller bookkeeping
and a rejected call, when one occurs. Plant integration, data export and video rendering
are outside this measurement. Metadata records `cycle_timing_definition` explicitly.

The committed videos and their metadata predate this measurement change and use the
controller's internal timer. Their numerical values are retained as historical results.

## Recorded results

See the README at the repository root ("Racing showcases") for the recorded videos and the
results table.
