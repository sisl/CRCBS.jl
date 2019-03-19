module CRCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

using Distributions
using HCubature
using Combinatorics

include("utils.jl")
include("CT_graph.jl") #Contains functions to generate and use CT graphs
include("low_level_search/a_star.jl") #Modified version of astar

export
    MAPF,
    GraphPath,
    CRCBS


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


struct MAPF{G <: AbstractGraph} # Multi Agent Path Finding Problem
    """A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
        a graph `G`, as well as a set of start and goal
        vertices on that graph. Note that this is the _labeled_ case, where each
        agent has a specific assigned destination."""
    graph::G
    starts::Vector{Int}
    goals::Vector{Int}
    lambda::Float64
    epsilon::Float64
    t_delay::Float64
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
    t1::Float
    t2::Float
end

function invalid_node_conflict()
    """Returns an invalid NodeConflict"""
    NodeConflict(-1,-1,-1,-1,-1)
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
    t1::Float #Nominal time robot 1 leaves node 2
    t2::Float #Nominal time robot 2 leaves node 1
end

"""
    returns an invalid EdgeConflict
"""
function invalid_edge_conflict()
    return EdgeConflict(-1,-1,-1,-1,-1,-1)
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
    This does restrict occupancy of node1_id (nodeconflicts take
    care of collision probability, but presence of robot at node1 would enable
    robots to swap positions), so t designates the nominal arrival time."""
struct EdgeConstraint <: CBSConstraint
    a::Int # agent ID
    node1_id::Int
    node2_id::Int
    t::Float # time ID
end

function verifies_NodeConstraint(Constraint_to_test::NodeConstraint,
    Constraint_to_verify_against::NodeConstraint)
    """Checks if constraint 1 verifies constraint 2 supposing that the concerned
    node is the same in both cases"""
    t_max = Constraint_to_verify_against.t
    return Constraint_to_test.t >= t_max #This means that the constraint is not violated
end

function verifies_EdgeConstraint(Constraint_to_test::EdgeConstraint,
    Constraint_to_verify_against::EdgeConstraint,mapf::MAPF)
    """Checks if constraint 1 verifies constraint 2 supposing that the concerned
    edge is the same in both cases"""
    n1 = Constraint_to_test.node1_id
    n2 = Constraint_to_test.node2_id
    t_edge = get_prop(mapf.graph,n1,n2,:weight)
    t_max = Constraint_to_verify_against.t + t_edge
    return Constraint_to_test.t >= t_max #This means that the constraint is not violated
end

"""Helper function for reversing an `EdgeConstraint`"""
flip(c::EdgeConstraint) = EdgeConstraint(c.a,c.node2_id,c.node1_id,c.t)


"""constraint dictionary for fast constraint lookup within a_star"""
@with_kw struct ConstraintDict
    node_constraints::Dict{Int,Float64} = Dict{Int,Float64}()
    edge_constraints::Dict{Tuple{Int,Int},Float64} = Dict{Tuple{Int,Int},Float64}()
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
#The conversion from NodeConstraint to ConstraintDict is done here
function add_constraint!(node::ConstraintTreeNode,constraint::NodeConstraint,mapf::MAPF)
    if (constraint.v != mapf.goals[constraint.a])
        node.constraints[constraint.a].node_constraints[constraint.v] = constraint.t
        return true
    end
    return false
end

"""adds an `EdgeConstraint` t o a ConstraintTreeNode"""
function add_constraint!(node::ConstraintTreeNode,constraint::EdgeConstraint,mapf::MAPF)
    if (constraint.v1 != mapf.goals[constraint.a]) && (constraint.v1 != mapf.goals[constraint.a])
        node.constraints[constraint.a].edge_constraints[(constraint.node1_id,constraint.node2_id)] = constraint.t
        # For the moment we only want edge constraints to be unidirectional
        #node.constraints[constraint.a].edge_constraints[(constraint.node2_id,constraint.node1_id)] = constraint.t
        return true
    end
    return false
end

""" ------------------------------------------------------------------------
    ------------------------------------------------------------------------ """


function violates_constraints(constraints::ConstraintDict,v,path,mapf::MAPF)
    """ Check if a set of constraints would be violated by adding an Edge from
        the final vertex of `path` to `v`
        returning true means the constraints are violated"""

    # Time it takes to go through path
    t1 = traversal_time(path,mapf)

    # We add the time it takes to go to v from the end of path
    v1 = get_final_node(path)
    v2 = v
    e = Edge(v1,v2)
    t = t1 + get_prop(mapf.graph,e,:weight)

    # If such constraint exists, agent is only authorized to visit v after t_min
    t_min = get(constraints.node_constraints,v,false)

    # -- Node Check -- #
    # If there is a time constraint for this agent and vertex
    # and if agent wants to visit v before authorized time t_min
    if t_min != false
        if t_min < t
            return true
        end

    # -- Edge Check -- #
    else
        t_min_edge = get(constraints.edge_constraints,(v1,v2),false)

        # If there is a time constraint for this agent and edge
        # and if agent wants to visit e before authorized time t_min_edge
        if t_min_edge != false
            if t1 < t_min_edge #Arrives at the first node of the edge before min time
                return true
            end
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
function get_cost(node::ConstraintTreeNode,mapf::MAPF)
    return get_cost(node.solution,mapf)
end

"""Returns an empty `ConstraintTreeNode`"""
function empty_constraint_node()
    ConstraintTreeNode()
end


# --------------- Finding and sorting likely collisions --------------------- #

function get_collision_probability_node(n1,t1,n2,t2,nn,lambda)

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

function get_collision_probability_edge(n1,t1,n2,t2,t_edge,lambda)

    function f(x)
        z = x[1]
        t = x[2]
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * lambda^(n1+n2) *  (t)^(n1-1) * (y+t)^(n2-1) * exp(-lambda*(y+2*t)) / (factorial(n1-1)*factorial(n2-1))
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * (lambda^(n1) * (t)^(n1-1)  * exp(-lambda*(y+2*t)) / factorial(n1-1))*(lambda^(n2) * (y+t)^(n2-1)/factorial(n2-1))
        density = pdf(Gamma(n1,lambda), t) * pdf(Gamma(n2,lambda),t-z)
        return density
    end

    function g(x)
        z = x[1]
        t = x[2]
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * lambda^(n1+n2) *  (t)^(n1-1) * (y+t)^(n2-1) * exp(-lambda*(y+2*t)) / (factorial(n1-1)*factorial(n2-1))
        #density = hcubature(d1,[t2-t1+y],[1000],rtol = 0.5)[1] * (lambda^(n1) * (t)^(n1-1)  * exp(-lambda*(y+2*t)) / factorial(n1-1))*(lambda^(n2) * (y+t)^(n2-1)/factorial(n2-1))
        density = pdf(Gamma(n1,lambda), t+z) * pdf(Gamma(n2,lambda),t)
        return density
    end

    z1 = minimum([t1-t2-t_edge,0])
    z2 = minimum([t1-t2+t_edge,0])

    z3 = maximum([t1-t2-t_edge,0])
    z4 = maximum([t1-t2+t_edge,0])

    a = [z1;0.0]
    b = [z2,1000.0]
    m = [z3;0.0]
    n = [z4,1000.0]
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
    for e in robotpath

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

function get_most_likely_conflicts(mapf::MAPF,paths::LowLevelSolution)
    """Returns the most likely conflict that occurs in a given solution:
    - paths:        a list of graph edges to be traversed by the agents"""

    epsilon = mapf.epsilon
    lambda = mapf.lambda

    # The first step is to fill the graph with occupancy information
    mapf_occupied = mapf
    for (robot_id,robotpath) in enumerate(paths)
        mapf_occupied = fill_graph_with_path(robot_id,robotpath,mapf_occupied)
    end

    node_conflict = invalid_node_conflict()
    edge_conflict = invalid_edge_conflict()

    # Initialize max conflict probabilities at 0
    node_conflict_p = 0
    edge_conflict_p = 0

    # ----- Node conflicts ----- #
    for v in vertices(mapf_occupied.graph)

        #Get node information
        nn = get_prop(mapf_occupied.graph, v, :n_delay)
        occupancy = get_prop(mapf_occupied.graph,v,:occupancy)

        # There is interaction at this node
        if length(occupancy) >= 2
            list_of_occupants = keys(occupancy)
            pairs_of_occupants = collect(combinations(list_of_occupants),2)
            for (r1,r2) in pairs_of_occupants
                (n1,t1) = occupancy[r1]
                (n2,t2) = occupancy[r2]
                cp = get_collision_probability_node(n1,t1,n2,t2,nn,lambda)

                # Conflict is likely and higher than all previously found
                if cp > maximum(epsilon,node_conflict_p)
                    node_conflict_p = cp
                    node_conflict = NodeConflict(r1,r2,v,t1,t2)
                end
            end
        end
    end

    # ----- Edge conflicts ----- #
    for e in edges(mapf_occupied.graph)

        #Get edge information. We consider confrontations as robots arriving in
        #opposite directions only
        t_edge = get_prop(mapf.graph,e,:weight)
        occupancy = get_prop(mapf_occupied.graph,e,:occupancy)
        reverse_edge = Edge(e[2],e[1])
        reverse_occupancy = get_prop(mapf_occupied.graph,reverse_edge,:occupancy)

        # There is interaction at this edge
        if length(occupancy)*length(reverse_occupancy) >= 1
            occupants_1 = keys(occupancy)
            occupants_2 = keys(reverse_occupancy)
            for robot1_id in occupants_1
                for robot2_id in occupants_2
                    (n1,t1) = occupancy[robot1_id]
                    (n2,t2) = reverse_occupancy[robot2_id]
                    cp = get_collision_probability_edge(n1,t1,n2,t2,t_edge,lambda)

                    if cp > maximum(epsilon,edge_conflict_p)
                        edge_conflict_p = cp
                        #time at which robot 1/2 leaves from node 2/1 (the last)
                        edge_conflict = EdgeConflict(r1,r2,e[1],e[2],t1+t_edge,t2+t_edge)
                    end
                end
            end
        end
    end

    mapf = clear_graph_occupancy(mapf)
    if node_conflict_p >= edge_conflict_p
        return node_conflict, invalid_edge_conflict()
    else
        return invalid_node_conflict(), edge_conflict
    end
end

function get_next_conflicts(mapf::MAPF,paths::LowLevelSolution,
        i_::Int=1,
        j_::Int=2,
        )
    """Returns a NodeConflict and an EdgeConflict next conflicts.
    The function returns after finding the most likely conflict (NodeConflict or
        EdgeConflict), which means that at least one of the returned conflicts
        will always be invalid. The rational for returning both anyway is to
        preserve stability of the function's return type.
    If node_conflict and edge_conflict are both invalid, the search has reached
        the end of the paths."""
    node_conflict = invalid_node_conflict()
    edge_conflict = invalid_edge_conflict()
    # begin search from paths[i_], paths[j_]
    i = i_; j_ = max(j_,i+1)

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
# --------------------------------------------------------------------------- #

# ----------------- Create constraints from conflicts ----------------------- #

"""generates a set of constraints from a NodeConflict"""
function generate_constraints_from_conflict(node::ConstraintTreeNode,conflict::NodeConflict,t_delay::Float)
    # If there was already a constraint and this was not enough to prevent collision,
    # we want to add t_delay to the already present delay
    t_yield1 = maximum(conflict.t2,conflict.t1)
    t_yield2 = maximum(conflict.t2,conflict.t1)

    robot1_id = conflict.agent1_id
    pre_existing_constraintDict = get(node.constraints,robot1_id,false)
    if pre_existing_constraintDict != false
        t_yield1 = get(pre_existing_constraintDict.node_constraints,conflict.node_id,t_yield1)
    end

    robot2_id = conflict.agent2_id
    pre_existing_constraintDict = get(node.constraints,robot2_id,false)
    if pre_existing_constraintDict != false
        t_yield2 = get(pre_existing_constraintDict.node_constraints,conflict.node_id,t_yield2)
    end

    return [
        # Agent 1 may not occupy node until agent 2 leaves the node, with t_delay
        NodeConstraint(
            conflict.agent1_id,
            conflict.node_id,
            t_yield1 + t_delay
        ),
        # Agent 2 may not occupy node until agent 1 leaves the node, with t_delay
        NodeConstraint(
            conflict.agent2_id,
            conflict.node_id,
            t_yield2 + t_delay
        )
        ]
end


"""generates a set of constraints from an EdgeConflict"""
function generate_constraints_from_conflict(node::ConstraintTreeNode,conflict::EdgeConflict)
    t_yield1 = maximum(conflict.t2,conflict.t1)
    t_yield2 = maximum(conflict.t2,conflict.t1)

    robot1_id = conflict.agent1_id
    pre_existing_constraintDict = get(node.constraints,robot1_id,false)
    if pre_existing_constraintDict != false
        t_yield1 = get(pre_existing_constraintDict.edge_constraints,(conflict.node1_id,conflict.node2_id),t_yield1)
    end

    robot2_id = conflict.agent2_id
    pre_existing_constraintDict = get(node.constraints,robot2_id,false)
    if pre_existing_constraintDict != false
        t_yield2 = get(pre_existing_constraintDict.edge_constraints,(conflict.node1_id,conflict.node2_id),t_yield2)
    end

    return [
        # Agent 1 may not enter node 1 of Edge(node1,node2) until robot 2 is
        # finished traversing node 1.
        EdgeConstraint(
            conflict.agent1_id,
            conflict.node1_id,
            conflict.node2_id,
            t_yield1 + t_delay # + 1
        ),
        # Agent 2 may not enter node 2 of Edge(node1,node2) until robot 1 is
        # finished traversing node 2.
        EdgeConstraint(
            conflict.agent2_id,
            conflict.node2_id,
            conflict.node1_id,
            t_yield2 + t_delay# + 1
        )
        ]
end


"""Returns a low level solution for a MAPF with constraints"""
function low_level_search!(mapf::MAPF,
    node::ConstraintTreeNode,
    idxs=collect(1:num_agents(mapf)),
    path_finder=LightGraphs.a_star)
    # compute an initial solution
    # solution = LowLevelSolution()
    for i in idxs
        # TODO allow passing custom heuristic
        path = path_finder(mapf.graph,mapf.starts[i],mapf.goals[i],get_constraints(node,i))
        node.solution[i] = path
        # push!(solution,path)
    end
    # sum of individual costs (SIC)
    # cost = get_cost(node.solution)
    # node.solution = solution
    node.cost = get_cost(node.solution,mapf)
    # TODO check if solution is valid
    return node.solution, node.cost
    # return true
end


"""
    Continuous Time CBS algorithm
"""
function CRCBS(mapf::MAPF,path_finder=LightGraphs.a_star)
    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    root_node = initialize_root_node(mapf)
    low_level_search!(mapf,root_node)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        node_conflict, edge_conflict = get_most_likely_conflicts(mapf,node.solution)
        if is_valid(node_conflict)
            constraints = generate_constraints_from_conflict(node,node_conflict,mapf.t_delay)
        elseif is_valid(edge_conflict)
            constraints = generate_constraints_from_conflict(node,edge_conflict,mapf.t_delay)
        else
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            return node.solution, node.cost
        end

        # generate new nodes from constraints
        for constraint in constraints
            new_node = initialize_child_node(node)
            if add_constraint!(new_node,constraint,mapf)
                low_level_search!(mapf,new_node,[get_agent_id(constraint)])
                if is_valid(new_node.solution, mapf)
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
    end
    print("No Solution Found. Returning default solution")
    return LowLevelSolution(), typemax(Int)
end


end # module
end
