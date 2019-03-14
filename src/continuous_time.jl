module ContinuousTimeCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

using Distributions
using HCurbature

include("utils.jl")
include("CT_graph.jl") #Contains functions to generate and use CT graphs
include("low_level_search/a_star.jl") #Modified version of astar

""" ------------------------------------------------------------------------
                       Common between CT and CBS
    ------------------------------------------------------------------------ """

"""
    type alias for a path through the graph
"""
const GraphPath = Vector{Edge}
get_start_node(path::GraphPath) = get(path,1,Edge(-1,-1)).src
get_final_node(path::GraphPath) = get(path,length(path),Edge(-1,-1)).dst
traversal_time(path::GraphPath,mapf::MAPF) = sum([get_prop(mapf.graph,e,:weight) for e in path])

"""type alias for a list of agent paths"""
const LowLevelSolution = Vector{GraphPath}

function is_valid(solution::LowLevelSolution,mapf::MAPF)
    """checks if a solution is valid"""
    for (i,path) in enumerate(solution)
        if get_start_node(path) != mapf.starts[i]
            return false
        end
        if get_final_node(path) != mapf.goals[i]
            return false
        end
    end
    return true
end

"""A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
    a graph `G` whose edges have unit length, as well as a set of start and goal
    vertices on that graph. Note that this is the _labeled_ case, where each
    agent has a specific assigned destination."""
struct MAPF{G <: AbstractGraph} # Multi Agent Path Finding Problem
    graph::G
    starts::Vector{Int}
    goals::Vector{Int}
    lambda::Float64
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
    conflict_probability::Float
end

function invalid_node_conflict()
    """Returns an invalid NodeConflict"""
    NodeConflict(-1,-1,-1,-1)
end

function is_valid(conflict::NodeConflict)
    """checks if a node conflict is valid"""
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
    conflict_probability::Float
end

"""
    returns an invalid EdgeConflict
"""
function invalid_edge_conflict()
    return EdgeConflict(-1,-1,-1,-1,-1)
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

"""Encodes a constraint that agent `a` may not occupy vertex `v` until time `t`"""
struct NodeConstraint <: CBSConstraint
    a::Int # agent ID
    v::Int # vertex ID
    t::Float# time ID
end

"""Encodes a constraint that agent `a` may not occupy edge [node1_id,node2_id]
    until time `t`.
    This does not restrict occupancy of node1_id at all (nodeconflicts take
    care of this), so t designates the nominal departure time."""
struct EdgeConstraint <: CBSConstraint
    a::Int # agent ID
    node1_id::Int
    node2_id::Int
    t::Float # time ID
end

function verifies_NodeConstraint(Constraint_to_test::NodeConstraint,
    Constraint_to_verify_against::NodeConstraint)
    """Checks if constraint 1 verifies constraint 2"""
    t_max = Constraint_to_verify_against.t
    return Constraint_to_test.t >= t_max #This means that the constraint is not violated
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



"""adds a `NodeConstraint` to a ConstraintTreeNode"""
function add_constraint!(node::ConstraintTreeNode,constraint::NodeConstraint,mapf::MAPF)
    if (constraint.v != mapf.goals[constraint.a])
        node.constraints[constraint.a].node_constraints[constraint] = true
        return true
    end
    return false
end

"""adds an `EdgeConstraint` t o a ConstraintTreeNode"""
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

""" Check if a set of constraints would be violated by adding an Edge from
    the final vertex of `path` to `v`"""
function violates_constraints(constraints::ConstraintDict,v,path,mapf::MAPF)
    t = traversal_time(path,mapf)

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
# --------------------------------------------------------------------------- #

"""Helper function to get the cost of a particular solution"""
function get_cost(paths::LowLevelSolution,mapf::MAPF)
    return sum([sum([get_prop(maph.graph,e,:weight) for e in p]) for p in paths])
end

"""Helper function to get the cost of a particular node"""
function get_cost(node::ConstraintTreeNode)
    return get_cost(node.solution)
end

"""Returns an empty `ConstraintTreeNode`"""
function empty_constraint_node()
    ConstraintTreeNode()
end


# --------------- Finding and sorting likely collisions --------------------- #

function get_collision_probability(n1,t1,n2,t2,nn,t_delay,lambda)

    function f(x)
        y = x[1]
        t = x[2]
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * lambda^(n1+n2) *  (t)^(n1-1) * (y+t)^(n2-1) * exp(-lambda*(y+2*t)) / (factorial(n1-1)*factorial(n2-1))
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * (lambda^(n1) * (t)^(n1-1)  * exp(-lambda*(y+2*t)) / factorial(n1-1))*(lambda^(n2) * (y+t)^(n2-1)/factorial(n2-1))
        density = (1-cdf(Gamma(nn,lambda),abs(t2-t1+y))) * pdf(Gamma(n1,lambda), t) * pdf(Gamma(n2,lambda),t-y)
        return density
    end

    function g(x)
        y = x[1]
        t = x[2]
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * lambda^(n1+n2) *  (t)^(n1-1) * (y+t)^(n2-1) * exp(-lambda*(y+2*t)) / (factorial(n1-1)*factorial(n2-1))
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * (lambda^(n1) * (t)^(n1-1)  * exp(-lambda*(y+2*t)) / factorial(n1-1))*(lambda^(n2) * (y+t)^(n2-1)/factorial(n2-1))
        density = (1-cdf(Gamma(nn,lambda),abs(t2-t1+y))) * pdf(Gamma(n1,lambda), t+y) * pdf(Gamma(n2,lambda),t)
        return density
    end

    a = [-1000.0;0.0]
    b = [0.0,1000.0]
    m = [0.0;0.0]
    n = [1000.0,1000.0]
    C1,err1 = hcubature(f,a,b)
    C2,err2 = hcubature(g,m,n)
    C = C1 + C2
    err = err1 + err2

    return C, err
end

function fill_graph_with_path(robot_id::Int, robotpath::GraphPath, mapf::MAPF)
    """This function adds the path traversed by the robot to the nodes and
    edges of the metagraph"""

    # Initialize at 1 instead of 0 to avoid bugs
    sum_ns_traversed = get_prop(mapf.Graph,robotpath[1][1], :n_delay)
    time_traversed = 0
    set_prop!(mapf.Graph,robotpath[1][1],:occupancy,(robot_id, sum_ns_traversed, 0))

    # Loop through all edges
    for e in robotpath:

        # Get properties of the edge and last vertex
        t_edge = get_prop(mapf.Graph,e, :weight)
        n_next_node = get_prop(mapf.Graph, e[2], :n_delay)

        # Add occupancy for the edge
        occupancy = get_prop(mapf.Graph,e,:occupancy)
        setindex!(occupancy,(sum_ns_traversed, time_traversed),robot_id) #robot_id is the key
        set_prop!(mapf.Graph,e,:occupancy,occupancy)

        # Update nominal time
        time_traversed += t_edge

        # Add occupancy for second vertex of the edge
        occupancy = get_prop(mapf.Graph,e[2],:occupancy)
        setindex!(occupancy,(sum_ns_traversed, time_traversed),robot_id) #robot_id is the key
        set_prop!(mapf.Graph,e,:occupancy,occupancy)

        # Update sum of ns traversed
        sum_ns_traversed += n_next_node

    return mapf
end

function clear_graph_occupancy(mapf::MAPF)
    """Removes all occupancies from the graph. We should only care about removing
    occupancy information concerning one robot at a time and not use this function."""
    for v in vertices(mapf.graph)
        rem_prop!(mapf.graph, v, :occupancy)
    end
    for e in edges(mapf.graph)
        rem_prop!(mapf.graph, e, :occupancy)
    end
    return mapf.graph
end

function get_next_conflict(mapf::MAPF)
    """At this point mapf.graph should be filled with occupancy information"""
    num_robots = length(mapf.starts)

# --------------------------------------------------------------------------- #
























end
