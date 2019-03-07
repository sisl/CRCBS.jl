module ContinuousTimeCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

include("utils.jl")
include("CT_graph.jl") #Contains functions to generate and use CT graphs

""" ------------------------------------------------------------------------
                Everything common in between CT and CBS
    ------------------------------------------------------------------------ """

"""
    type alias for a path through the graph
"""
const GraphPath = Vector{Edge}

"""
    returns the time for traversal of a GraphPath. Defaults to the length of the
    path, but it may be useful in case we need to override later
"""
traversal_time(path::GraphPath) = length(path)

"""
    type alias for a list of agent paths
"""
const LowLevelSolution = Vector{GraphPath}

"""
    A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
    a graph `G` whose edges have unit length, as well as a set of start and goal
    vertices on that graph. Note that this is the _labeled_ case, where each
    agent has a specific assigned destination.
"""
struct MAPF{G <: AbstractGraph} # Multi Agent Path Finding Problem
    graph::G
    starts::Vector{Int}
    goals::Vector{Int}
end

""" ------------------------------------------------------------------------
    ------------------------------------------------------------------------ """

# --------------------- Node Conflicts -------------------------------------- #
"""
    Encodes a conflict between two agents at a particular node. (No time)
"""
struct NodeConflict
    agent1_id::Int
    agent2_id::Int
    node_id::Int
end

"""
    Returns an invalid NodeConflict
"""
function invalid_node_conflict()
    NodeConflict(-1,-1,-1)
end

"""
    checks if a node conflict is valid
"""
function is_valid(conflict::NodeConflict)
    return (conflict.agent1_id != -1)
end
# --------------------------------------------------------------------------- #


# --------------------- Edge Conflicts -------------------------------------- #
"""
    Encodes a conflict between two agents at a particular edge.
    This means that the agents are trying to swap places at time t.
    This struct is able to consider conflicts on edge paths (if the
    robots' trajectories overlap for more than one edge), in which case we
    consider the extrema of the common path here.
"""
struct EdgeConflict
    agent1_id::Int
    agent2_id::Int
    node1_id::Int
    node2_id::Int
end

"""
    returns an invalid EdgeConflict
"""
function invalid_edge_conflict()
    return EdgeConflict(-1,-1,-1,-1)
end

"""
    checks if an edge node is invalid
"""
function is_valid(conflict::EdgeConflict)
    return (conflict.agent1_id != -1)
end
# --------------------------------------------------------------------------- #

# ------------------------ Constraints -------------------------------------- #
"""
    Encodes a constraint that agent `a` may not occupy vertex `v` until time `t`
"""
struct VertexConstraint
    a::Int # agent ID
    v::Int # vertex ID
    t::Float# time ID
end

"""
    Encodes a constraint that agent `a` may not occupy edge [node1_id,node2_id]
    until time `t.
    This does not restrict occupancy of node1_id at all (nodeconflicts take
    care of this), so t designates the nominal departure time.
"""
struct EdgeConstraint
    a::Int # agent ID
    node1_id::Int
    node2_id::Int
    t::Float # time ID
end





end
