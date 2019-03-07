module CRCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

include("utils.jl")
include("low_level_search/a_star.jl")

export
    MAPF,
    GraphPath,
    get_start_node,
    get_final_node,
    traversal_time,
    LowLevelSolution,
    is_valid,
    NodeConflict,
    detect_node_conflict,
    invalid_node_conflict,
    EdgeConflict,
    detect_edge_conflict,
    invalid_edge_conflict,
    NodeConstraint,
    EdgeConstraint,
    ConstraintTreeNode,
    ConstraintDict,
    get_constraints,
    # get_constraint_dict,
    add_constraint!,
    # compare_constraint_nodes,
    # combine_constraints,
    violates_constraints,
    get_cost,
    empty_constraint_node,
    get_next_conflicts,
    get_conflicts,
    generate_constraints_from_conflict,
    low_level_search!,
    CBS


"""
    A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
    a graph `G` whose edges have unit length, as well as a set of start and goal
    vertices on that graph. Note that this is the _labeled_ case, where each
    agent has a specific assigned destination.
"""
struct MAPF{G <: AbstractGraph} # Multi Agent Path Finding Problem
    graph::G
    # dist_matrix::Matrix{Float64}
    starts::Vector{Int}
    goals::Vector{Int}
end

# """ helper to retrieve the distance matrix for a `MAPF` instance """
# get_dist_matrix(mapf::MAPF) = mapf.dist_matrix

# """
#     Construct `MAPF` from graph, starts, goals
# """
# function MAPF(graph::G where {G<:AbstractGraph},starts::Vector{Int},goals::Vector{Int})
#     MAPF(graph,get_dist_matrix(graph),starts,goals)
# end

""" Type alias for a path through the graph """
const GraphPath = Vector{Edge{Int}}

"""
    Returns the edge at time t or a "self-loop" edge on the final node of the
    path
"""
function get_edge(path::GraphPath, t::Int)
    if length(path) < t
        return Edge(path[end].dst,path[end].dst)
    else
        return path[t]
    end
end
get_start_node(path::GraphPath) = get(path,1,Edge(-1,-1)).src
get_final_node(path::GraphPath) = get(path,length(path),Edge(-1,-1)).dst

"""
    returns the time for traversal of a GraphPath. Defaults to the length of the
    path, but it may be useful in case we need to override later
"""
traversal_time(path::GraphPath) = length(path)

""" Type alias for a list of agent paths """
const LowLevelSolution = Vector{GraphPath}

"""
    checks if a solution is valid
"""
function is_valid(solution::LowLevelSolution,mapf::MAPF)
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


# @enum ConflictType begin
#     NULL_CONFLICT = 0
#     NODE_CONFLICT = 1
#     EDGE_CONFLICT = 2     # waiting to be picked up (perhaps redundant?)
# end
# struct Conflict
#     conflict_type::ConflictType
#     agent1_id::Int
#     agent2_id::Int
#     node1_id::Int
#     node2_id::Int
#     t::Int
# end
# function detect_conflict(edge1::Edge,edge2::Edge)
#     if detect_node_conflict(edge1,edge2)
#         return NODE_CONFLICT # node conflict
#     elseif detect_edge_conflict(edge1,edge2)
#         return EDGE_CONFLICT
#     end
#     return NULL_CONFLICT
# end

abstract type CBSConflict end

"""
    Encodes a conflict wherein two agents occupy a particular node at a
    particular time
"""
struct NodeConflict <: CBSConflict
    agent1_id::Int
    agent2_id::Int
    node_id::Int
    t::Int
end

""" Checks for a `NodeConflict` between two `Edge`s """
function detect_node_conflict(edge1::Edge,edge2::Edge)
    if edge1.dst == edge2.dst
        return true
    end
    return false
end

""" Checks for a `NodeConflict` between two `GraphPath`s at time t """
function detect_node_conflict(path1::GraphPath,path2::GraphPath,t::Int)
    e1 = get_edge(path1,t)
    e2 = get_edge(path2,t)
    if detect_node_conflict(e1,e2)
        return true
    end
    return false
end

""" Returns an invalid NodeConflict """
invalid_node_conflict() = NodeConflict(-1,-1,-1,-1)

""" Checks if a node conflict is valid """
is_valid(conflict::NodeConflict) = (conflict.agent1_id != -1)

"""
    Encodes a conflict between two agents at a particular edge at a particular
    time. This means that the agents are trying to swap places at time t.
"""
struct EdgeConflict <: CBSConflict
    agent1_id::Int
    agent2_id::Int
    node1_id::Int
    node2_id::Int
    t::Int
end

""" Checks for an `EdgeConflict` between two `Edge`s """
function detect_edge_conflict(edge1::Edge,edge2::Edge)
    if (edge1.src == edge2.dst) && (edge1.dst == edge2.src)
        return true
    end
    return false
end

""" Checks for an `EdgeConflict` between two `GraphPath`s at time t """
function detect_edge_conflict(path1::GraphPath,path2::GraphPath,t::Int)
    e1 = get_edge(path1,t)
    e2 = get_edge(path2,t)
    if detect_edge_conflict(e1,e2)
        return true
    end
    return false
end

""" Returns an invalid EdgeConflict """
invalid_edge_conflict() = EdgeConflict(-1,-1,-1,-1,-1)

""" checks if an edge node is invalid """
is_valid(conflict::EdgeConflict) = (conflict.agent1_id != -1)

abstract type CBSConstraint end
"""
    Encodes a constraint that agent `a` may not occupy vertex `v` at time `t`
"""
struct NodeConstraint <: CBSConstraint
    a::Int # agent ID
    v::Int # vertex ID
    t::Int # time ID
end

"""
    Encodes a constraint that agent `a` may not traverse `Edge(v1,v2)` at time
    step `t`
"""
struct EdgeConstraint <: CBSConstraint
    a::Int # agent ID
    v1::Int # ID of first vertex in edge
    v2::Int # ID of second vertex in edge
    t::Int # time ID
end

"""
    Helper function for reversing an `EdgeConstraint`
"""
flip(c::EdgeConstraint) = EdgeConstraint(c.a,c.v2,c.v1,c.t)

"""
    constraint dictionary for fast constraint lookup within a_star
"""
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

"""
    A node of a constraint tree. Each node has a set of constraints, a candidate
    solution (set of robot paths), and a cost
"""
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

"""
    retrieve constraints corresponding to this node and this path
"""
function get_constraints(node::ConstraintTreeNode, path_id::Int)
    return get(node.constraints, path_id, ConstraintDict())
end

"""
    Check if a set of constraints would be violated by adding an Edge from
    the final vertex of `path` to `v`
"""
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

"""
    Construct and empty `ConstraintTreeNode` from a `MAPF` instance
"""
function ConstraintTreeNode(mapf::MAPF)
    ConstraintTreeNode(
        constraints = Dict{Int,ConstraintDict}(
            i=>ConstraintDict(a=i) for i in 1:length(mapf.starts)
            ))
end

"""
    adds a `NodeConstraint` to a ConstraintTreeNode
"""
function add_constraint!(node::ConstraintTreeNode,constraint::NodeConstraint,mapf::MAPF)
    if (constraint.v != mapf.goals[constraint.a])
        node.constraints[constraint.a].node_constraints[constraint] = true
        return true
    end
    return false
end

"""
    adds an `EdgeConstraint` to a ConstraintTreeNode
"""
function add_constraint!(node::ConstraintTreeNode,constraint::EdgeConstraint,mapf::MAPF)
    if (constraint.v1 != mapf.goals[constraint.a]) && (constraint.v1 != mapf.goals[constraint.a])
        node.constraints[constraint.a].edge_constraints[constraint] = true
        node.constraints[constraint.a].edge_constraints[flip(constraint)] = true
        return true
    end
    return false
end

# """
#     checks to see if two `ConstraintTreeNode`s are identical (in terms of their
#     constraints)
# """
# function compare_constraint_nodes(node1::ConstraintTreeNode,node2::ConstraintTreeNode)
#     constraints1 = union([collect(keys(v)) for (k,v) in node1.constraints])
#     constraints2 = union([collect(keys(v)) for (k,v) in node2.constraints])
#     return length(setdiff(constraints1,constraints2)) == 0
# end

"""
    Helper function to get the cost of a particular solution
"""
function get_cost(paths::LowLevelSolution)
    return sum([length(p) for p in paths])
end
"""
    Helper function to get the cost of a particular node
"""
function get_cost(node::ConstraintTreeNode)
    return get_cost(node.solution)
end

"""
    Returns an empty `ConstraintTreeNode`
"""
function empty_constraint_node()
    ConstraintTreeNode()
end

"""
    Returns a `NodeConflict` and an `EdgeConflict` next conflicts.
    The function returns after finding the FIRST conflice (NodeConflict or
        EdgeConflict), which means that at least one of the returned conflicts
        will always be invalid. The rational for returning both anyway is to
        preserve stability of the function's return type.
    If node_conflict and edge_conflict are both invalid, the search has reached
        the end of the paths.

    args:
    - t_:       time index at which to begin the search
    - i_:       index of path 1 at which to begin the search
    - j_:       index of path 2 at which to begin the search
    - tmax:     maximum lookahead time (defaults to the length of the longest
        path)
    Search begins at time `t_`, `paths[i_]`, `paths[j_]`, then returns after
        finding the first conflict.
"""
function get_next_conflicts(paths::LowLevelSolution,
        i_::Int=1,
        j_::Int=2,
        t_::Int=1,
        tmax::Int=maximum([traversal_time(p) for p in paths])
        )
    node_conflict = invalid_node_conflict()
    edge_conflict = invalid_edge_conflict()
    # begin search from time t, paths[i_], paths[j_]
    t = t_; i = i_; j_ = max(j_,i+1)

    path1 = get(paths,i,GraphPath()) # in case i is beyond the length of paths
    e1 = get_edge(path1,t)
    for j in j_:length(paths)
        path2 = paths[j]
        e2 = get_edge(path2,t)
        if detect_node_conflict(e1,e2)
            node_conflict = NodeConflict(i,j,e1.dst,t)
            return node_conflict, edge_conflict
        elseif detect_edge_conflict(e1,e2)
            edge_conflict = EdgeConflict(i,j,e1.src,e1.dst,t)
            return node_conflict, edge_conflict
        end
    end
    # Continue search from next time step
    for t in t_+1:tmax
        for (i,path1) in enumerate(paths)
            e1 = get_edge(path1,t)
            for j in i+1:length(paths)
                path2 = paths[j]
                e2 = get_edge(path2,t)
                if detect_node_conflict(e1,e2)
                    node_conflict = NodeConflict(i,j,e1.dst,t)
                    return node_conflict, edge_conflict
                elseif detect_edge_conflict(e1,e2)
                    edge_conflict = EdgeConflict(i,j,e1.src,e1.dst,t)
                    return node_conflict, edge_conflict
                end
            end
        end
    end
    return node_conflict, edge_conflict
end

# """
#     Returns the next conflict that is not already in the current ConstraintTreeNode
# """

"""
    Returns a list of all conflicts that occur in a given solution

    args:
    - paths:        a list of graph edges to be traversed by the agents
"""
function get_conflicts(paths::LowLevelSolution)
    # TODO Make this way faster
    node_conflicts = Vector{NodeConflict}()
    edge_conflicts = Vector{EdgeConflict}()
    t_max = maximum([length(path) for path in paths])
    nc, ec = get_next_conflicts(paths)
    while true
        if is_valid(nc)
            push!(node_conflicts, nc)
            conflict = nc
        elseif is_valid(ec)
            push!(edge_conflicts, ec)
            conflict = ec
        else
            break
        end
        nc, ec = get_next_conflicts(
            paths,
            conflict.agent1_id,
            conflict.agent2_id+1,
            conflict.t,
            t_max
            )
    end
    return node_conflicts, edge_conflicts
end

"""
    generates a set of constraints from a NodeConflict
"""
function generate_constraints_from_conflict(conflict::NodeConflict)
    return [
        # Agent 1 may not occupy node at time t + 1
        NodeConstraint(
            conflict.agent1_id,
            conflict.node_id,
            conflict.t
        ),
        # Agent 2 may not occupy node at time t + 1
        NodeConstraint(
            conflict.agent2_id,
            conflict.node_id,
            conflict.t
        )
        ]
end

"""
    generates a set of constraints from an EdgeConflict
"""
function generate_constraints_from_conflict(conflict::EdgeConflict)
    return [
        # Agent 1 may not traverse Edge(node1,node2) at time t
        EdgeConstraint(
            conflict.agent1_id,
            conflict.node1_id,
            conflict.node2_id,
            conflict.t # + 1
        ),
        # Agent 2 may not traverse Edge(node2,node1) at time t
        EdgeConstraint(
            conflict.agent2_id,
            conflict.node2_id,
            conflict.node1_id,
            conflict.t# + 1
        )
        ]
end

"""
    Returns a low level solution for a MAPF with constraints
"""
function low_level_search!(mapf::MAPF,node::ConstraintTreeNode,path_finder=LightGraphs.a_star)
    # compute an initial solution
    solution = LowLevelSolution()
    for i in 1:length(mapf.starts)
        # TODO allow passing custom heuristic
        path = path_finder(mapf.graph,mapf.starts[i],mapf.goals[i],get_constraints(node,i))
        push!(solution,path)
    end
    # sum of individual costs (SIC)
    cost = get_cost(solution)
    node.solution = solution
    node.cost = get_cost(solution)
    # TODO check if solution is valid
    return solution, cost
    return true
end

"""
    The Conflict-Based search algorithm for multi-agent path finding
"""
function CBS(mapf::MAPF,path_finder=LightGraphs.a_star)
    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    root_node = ConstraintTreeNode(mapf)
    low_level_search!(mapf,root_node)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        node_conflict, edge_conflict = get_next_conflicts(node.solution)
        if is_valid(node_conflict)
            constraints = generate_constraints_from_conflict(node_conflict)
        elseif is_valid(edge_conflict)
            constraints = generate_constraints_from_conflict(edge_conflict)
        else
            print("Optimal Solution Found!\n")
            return node.solution, node.cost
        end

        # generate new nodes from constraints
        for constraint in constraints
            new_node = ConstraintTreeNode(constraints=copy(node.constraints))
            if add_constraint!(new_node,constraint,mapf)
                low_level_search!(mapf,new_node)
                if is_valid(new_node.solution, mapf)
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
    end
    print("No Solution Found. Returning default solution")
    return LowLevelSolution(), typemax(Int)
end

include("low_level_search/heuristics.jl")

end # module
