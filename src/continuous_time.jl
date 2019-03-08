module ContinuousTimeCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

include("utils.jl")
include("CT_graph.jl") #Contains functions to generate and use CT graphs

""" ------------------------------------------------------------------------
                       Common between CT and CBS
    ------------------------------------------------------------------------ """

"""
    type alias for a path through the graph
"""
const GraphPath = Vector{Edge}
get_start_node(path::GraphPath) = get(path,1,Edge(-1,-1)).src
get_final_node(path::GraphPath) = get(path,length(path),Edge(-1,-1)).dst
traversal_time(path::GraphPath) = sum([get_prop(path,e,:weight) for e in path])

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
abstract type CBSConstraint end
get_agent_id(c::CBSConstraint) = c.a

"""
Encodes a constraint that agent `a` may not occupy vertex `v` until time `t`
"""
struct NodeConstraint <: CBSConstraint
    a::Int # agent ID
    v::Int # vertex ID
    t::Float# time ID
end

"""
    Encodes a constraint that agent `a` may not occupy edge [node1_id,node2_id]
    until time `t`.
    This does not restrict occupancy of node1_id at all (nodeconflicts take
    care of this), so t designates the nominal departure time.
"""
struct EdgeConstraint <: CBSConstraint
    a::Int # agent ID
    node1_id::Int
    node2_id::Int
    t::Float # time ID
end

"""Helper function for reversing an `EdgeConstraint`"""
flip(c::EdgeConstraint) = EdgeConstraint(c.a,c.node2_id,c.node1_id,c.t)


"""constraint dictionary for fast constraint lookup within a_star"""
@with_kw struct ConstraintDict
    node_constraints::Dict{NodeConstraint,Bool} = Dict{NodeConstraint,Bool}()
    edge_constraints::Dict{EdgeConstraint,Bool} = Dict{EdgeConstraint,Bool}()
    a::Int = -1 # agent_id
end

""" Helper function to merge two instances of `ConstraintDict` """
function Base.merge(d1::ConstraintDict,d2::ConstraintDict)
    @assert(d1.a==d2.a)
    ConstraintDict(
        merge(d1.node_constraints,d2.node_constraints),
        merge(d1.edge_constraints,d2.edge_constraints),
        d1.a
    )
end

"""
     Combines two `Dict`s of `ConstraintDict`s into a single `Dict` of
     `ConstraintDict`s where the value associated with each key in the
     resulting dictionary is the union of the values for the input dictionaries
     at that key
"""
function Base.merge(dict1::Dict{K,ConstraintDict},dict2::Dict{K,ConstraintDict}) where K
    new_dict = typeof(dict1)()
    for k in union(collect(keys(dict1)),collect(keys(dict2)))
        new_dict[k] = merge(get(dict1,k,ConstraintDict()), get(dict1,k,ConstraintDict()))
    end
    return new_dict
end
# --------------------------------------------------------------------------- #



# ------------------------ Constraint Tree ---------------------------------- #

""" ------------------------------------------------------------------------
                       Common between CT and CBS
    ------------------------------------------------------------------------ """

""" A node of a constraint tree. Each node has a set of constraints, a candidate
    solution (set of robot paths), and a cost"""
@with_kw mutable struct ConstraintTreeNode
    # maps agent_id to the set of constraints involving that agent
    constraints::Dict{Int,ConstraintDict} = Dict{Int,ConstraintDict}()
    # set of paths (one per agent) through graph
    solution::LowLevelSolution = LowLevelSolution()
    # cost = sum([length(path) for path in solution])
    cost::Int = -1
    # index of parent node
    parent::Int = -1
    # indices of two child nodes
    children::Tuple{Int,Int} = (-1,-1)
    # unique id
    id::Int = -1
end

const NULL_NODE = -1

"""Construct an empty `ConstraintTreeNode` from a `MAPF` instance"""
function initialize_root_node(mapf::MAPF)
    ConstraintTreeNode(
        solution = LowLevelSolution([GraphPath() for a in 1:num_agents(mapf)]),
        constraints = Dict{Int,ConstraintDict}(
            i=>ConstraintDict(a=i) for i in 1:length(mapf.starts)
            ))
end

"""Initialize a new `ConstraintTreeNode` with the same `solution` and
    `constraints` as the parent node"""
function initialize_child_node(parent_node::ConstraintTreeNode)
    ConstraintTreeNode(
        solution = copy(parent_node.solution),
        constraints = copy(parent_node.constraints)
    )
end

"""retrieve constraints corresponding to this node and this path"""
function get_constraints(node::ConstraintTreeNode, path_id::Int)
    return get(node.constraints, path_id, ConstraintDict())
end

""" Check if a set of constraints would be violated by adding an Edge from
    the final vertex of `path` to `v`"""
function violates_constraints(constraints::ConstraintDict,v,path)
    t = length(path) + 1
    if get(constraints.node_constraints,NodeConstraint(constraints.a,v,t),false)
        return true
    else
        v1 = get_final_node(path)
        v2 = v
        if get(constraints.edge_constraints,EdgeConstraint(constraints.a,v1,v2,t),false)
            return true
        end
    end
    return false
end

"""adds a `NodeConstraint` to a ConstraintTreeNode"""
function add_constraint!(node::ConstraintTreeNode,constraint::NodeConstraint,mapf::MAPF)
    if (constraint.v != mapf.goals[constraint.a])
        node.constraints[constraint.a].node_constraints[constraint] = true
        return true
    end
    return false
end

"""adds an `EdgeConstraint` to a ConstraintTreeNode"""
function add_constraint!(node::ConstraintTreeNode,constraint::EdgeConstraint,mapf::MAPF)
    if (constraint.v1 != mapf.goals[constraint.a]) && (constraint.v1 != mapf.goals[constraint.a])
        node.constraints[constraint.a].edge_constraints[constraint] = true
        node.constraints[constraint.a].edge_constraints[flip(constraint)] = true
        return true
    end
    return false
end

""" ------------------------------------------------------------------------
    ------------------------------------------------------------------------ """

# --------------------------------------------------------------------------- #

"""Helper function to get the cost of a particular solution"""
function get_cost(paths::LowLevelSolution,graph::MAPF)
    return sum([sum([get_prop(graph,e,:weight) for e in p]) for p in paths])
end

"""Helper function to get the cost of a particular node"""
function get_cost(node::ConstraintTreeNode)
    return get_cost(node.solution)
end

"""Returns an empty `ConstraintTreeNode`"""
function empty_constraint_node()
    ConstraintTreeNode()
end


# ------------------------------ CTCBS -------------------------------------- #

"""Adds occupancy to a MetaGraph property with nominal times of arrival
at each node and each edge"""




# --------------------------------------------------------------------------- #
























end
