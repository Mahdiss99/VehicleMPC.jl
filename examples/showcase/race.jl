# Showcase A: nonlinear MPC racing on the full multi-curvature quarter track, no obstacles.
#
#     julia --project=. examples/showcase/race.jl --model=dynamic --video
#     julia --project=. examples/showcase/race.jl --model=kinematic --video
#
# Solves the MPC at every control update, simulates the selected vehicle model, exports the
# data of this run to examples/showcase/output/race_<model>_*, and renders
# race_<model>.mp4 and race_<model>_poster.png from it (renderer: render_animation.py,
# Python with matplotlib, numpy and imageio-ffmpeg; see examples/showcase/README.md).
# Options: --no-video, --cycles=N, --out=DIR, --python=PATH, --fps=N.

include(joinpath(@__DIR__, "common.jl"))

if abspath(PROGRAM_FILE) == @__FILE__
    opts = showcase_options(250)
    showcase_main("race", "Full-track racing", CircleObstacle[], opts)
end
