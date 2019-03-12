using Test
using Logging
using LightGraphs, MetaGraphs
using Parameters
# Package Under Test
using CRCBS

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))

# Check equality of two arrays
@inline function array_isapprox(x::AbstractArray{F},
                  y::AbstractArray{F};
                  rtol::F=sqrt(eps(F)),
                  atol::F=zero(F)) where {F<:AbstractFloat}

    # Easy check on matching size
    if length(x) != length(y)
        return false
    end

    for (a,b) in zip(x,y)
        @test isapprox(a,b, rtol=rtol, atol=atol)
    end
end

# Check if array equals a single value
@inline function array_isapprox(x::AbstractArray{F},
                  y::F;
                  rtol::F=sqrt(eps(F)),
                  atol::F=zero(F)) where {F<:AbstractFloat}

    for a in x
        @test isapprox(a, y, rtol=rtol, atol=atol)
    end
end

# Define package tests
@time @testset "CRCBS Package Tests" begin
    testdir = joinpath(dirname(@__DIR__), "test")
    # @time @testset "CRCBS.MainTests" begin
    #     include(joinpath(testdir, "test_main.jl"))
    # end
    @time @testset "CRCBS.OverHaulTests" begin
        include(joinpath(testdir, "test_overhaul.jl"))
    end
    @time @testset "CRCBS.HeuristicTests" begin
        include(joinpath(testdir, "test_heuristics.jl"))
    end
    # @time @testset "CRCBS.ImplicitGraphsTests" begin
    #     include(joinpath(testdir, "implicit_graph_tests.jl"))
    # end
    # @time @testset "CRCBS.DemoTests" begin
    #     include(joinpath(testdir, "test_demo.jl"))
    # end
end
