module CRCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs, GraphUtils
using LinearAlgebra
using NearestNeighbors

include("problem_definitions.jl")
include("general_interface.jl")
include("common.jl")
include("utils.jl")
include("low_level_search/implicit_graphs.jl")

include("solvers.jl")

include("environments/CBS.jl")
include("environments/multi_stage_CBS.jl")

end # module
