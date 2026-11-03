# Showcase B: nonlinear MPC racing with six stationary obstacles on the quarter track, all
# imposed simultaneously, from the same initial physical condition as showcase A.
#
#     julia --project=. examples/showcase/multiple_obstacles.jl --model=dynamic --video
#
# The obstacle layout (common.jl, OBSTACLE_LAYOUT) is checked for geometric feasibility
# before the run. --model=kinematic is rejected with an explicit error: the kinematic
# NMPCController supports one obstacle. Exports examples/showcase/output/
# multiple_obstacles_<model>_* and renders the video from that data.
# Options: --no-video, --cycles=N, --out=DIR, --python=PATH, --fps=N.

include(joinpath(@__DIR__, "common.jl"))

if abspath(PROGRAM_FILE) == @__FILE__
    opts = showcase_options(250)
    obstacles = showcase_obstacles()
    problems = check_layout(obstacles)
    isempty(problems) || error("obstacle layout is not feasible:\n" * join(problems, "\n"))
    println("obstacle layout: ", length(obstacles), " obstacles, geometric checks passed")
    showcase_main("multiple_obstacles", "Racing with multiple obstacles", obstacles, opts)
end
