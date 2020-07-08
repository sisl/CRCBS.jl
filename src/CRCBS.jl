module CRCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs, GraphUtils
using LinearAlgebra, SparseArrays
using NearestNeighbors
using JuMP


include("problem_definitions.jl")
include("core.jl")
include("general_interface.jl")
include("cost_models.jl")
include("heuristics.jl")
include("core_utils.jl")
include("common.jl")

include("low_level_search/a_star.jl")

include("solver_utils.jl")
include("solvers.jl")
include("solvers/meta_agent_cbs.jl")

include("environments/cbs.jl")
include("environments/multi_stage_cbs.jl")
include("environments/meta_agent_cbs.jl")

include("flow_problems.jl")

end # module
