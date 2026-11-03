# Reproducibility guards for the new package code.

const REPO_ROOT = normpath(joinpath(@__DIR__, ".."))
const NEW_CODE_DIRS = ("src", "test", "examples")

@testset "no absolute paths in package code" begin
    abs_path = r"(?<![A-Za-z])[A-Za-z]:[\\/]|/(Users|home)/"
    offenders = String[]
    for dir in NEW_CODE_DIRS, (root, _, files) in walkdir(joinpath(REPO_ROOT, dir)), f in files
        endswith(f, ".jl") || continue
        path = joinpath(root, f)
        for (n, line) in enumerate(eachline(path))
            startswith(lstrip(line), "abs_path =") && continue    # this check itself
            occursin(abs_path, line) && push!(offenders, "$path:$n")
        end
    end
    @test isempty(offenders)
end

@testset "package has no plotting dependency" begin
    deps = keys(TOML.parsefile(joinpath(REPO_ROOT, "Project.toml"))["deps"])
    @test isempty(intersect(deps, ("Plots", "PyPlot", "PyCall", "GR", "Makie")))
end
