# Example C/D: multi-segment NMPC on the legacy quarter track (Milestone 2B.2).
#
# Run from the repository root:
#
#     julia --project=. examples/nmpc_quarter.jl
#
# Writes examples/output/nmpc_quarter_*.csv. Plots: examples/plot_nmpc.jl.

include("nmpc_common.jl")

"""
Configurations shown by the example. Both use the baseline weights, limits and the 5 cm
track margin. They differ only in how a stale curvature schedule is handled:

- `baseline`: the default `NMPCConfig()`. One NLP solve per cycle, stale schedules
  rejected. On this track it stops at the first cycle; that outcome is documented, not
  hidden.
- `verified_qualified`: up to 8 schedule corrections within an 80 ms budget. After that,
  a still-stale schedule is accepted only if the independent event-aware re-simulation on
  the actual track satisfies every hard constraint over the whole horizon
  (`:success_stale_schedule`, counted separately).
"""
const QUARTER_CONFIGS = (
    baseline=NMPCConfig(),
    verified_qualified=NMPCConfig(schedule_corrections=8, stale_schedule_policy=:accept_if_reference_valid),
)

"""
    run_nmpc_quarter(; config, n_steps=250)

Reference test vehicle on `quarter_track()` (unchanged legacy geometry, 0.7 m wide),
starting at `[0, 0, 0, 0.5]`. The closed loop uses the event-aware plant.
"""
function run_nmpc_quarter(; config=QUARTER_CONFIGS.verified_qualified, n_steps=250)
    vehicle = KinematicBicycle(reference_test_vehicle())
    ctrl = NMPCController(vehicle, quarter_track(), config)
    cl = simulate_closed_loop(ctrl, vehicle, [0.0, 0.0, 0.0, 0.5], n_steps)
    return (; ctrl, cl)
end

if abspath(PROGRAM_FILE) == @__FILE__
    for (name, cfg) in pairs(QUARTER_CONFIGS)
        (; ctrl, cl) = run_nmpc_quarter(; config=cfg)
        summarize("quarter track, $name", ctrl, cl)
        st = [r.status for r in cl.controller]
        println("controller statuses: ", join(["$s: $(count(==(s), st))" for s in unique(st)], ", "))
        println("NLP solves per cycle: ", join(["$n: $(count(r -> r.nlp_solves == n, cl.controller))" for n in sort(unique(r.nlp_solves for r in cl.controller))], ", "))
        println("boundary crossings by the plant: ", length(cl.crossings))
        println("wrote ", write_closed_loop_csv(joinpath(@__DIR__, "output", "nmpc_quarter_$name.csv"), cl))
        println()
    end
end
