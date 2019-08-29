module CRCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs, GraphUtils
using LinearAlgebra
using NearestNeighbors
using JuMP

include("problem_definitions.jl")
include("general_interface.jl")
include("cost_models.jl")
include("common.jl")

include("low_level_search/heuristics.jl")
include("low_level_search/implicit_graphs.jl")

include("solvers.jl")

include("environments/CBS.jl")
include("environments/multi_stage_CBS.jl")
include("environments/meta_agent_cbs.jl")

include("flow_problems.jl")

end # module
