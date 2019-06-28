module CRCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

using Distributions
using HCubature
using Combinatorics

using JLD
using Random
using Plots
using CSV
using DataFrames

include("utils.jl")
include("CT_graph.jl") #Contains functions to generate and use CT graphs
include("Whittaker.jl")


export
    MAPF,
    GraphPath,
    get_collision_probability_edge,
    get_collision_probability_node,
    clear_graph_occupancy,
    invalid_node_conflict,
    count_most_likely_conflicts,
    violates_constraints,
    fill_graph_with_path,
    generate_constraints_from_conflict,
    count_node_conflicts,
    count_edge_conflicts,
    CTCBS,
    STTCBS

struct MAPF{G <: AbstractGraph} # Multi Agent Path Finding Problem
    """A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
        a graph `G`, as well as a set of start and goal
        vertices on that graph. Note that this is the _labeled_ case, where each
        agent has a specific assigned destination."""
    graph::G
    starts::Vector{Int}
    # start times = time at which the robots appear on the graph and suddenly need to accomplish task k
    start_times::Vector{Float64}
    # goals now contains lists, thanks to which we can set intermediate goals (pick up object, etc)
    goals::Vector{Array{Int64,1}}
    # Each intermediary goal takes some additional time for task completion, which is stored here
    goal_completion_times::Vector{Array{Float64,1}}
    lambda::Float64
    epsilon::Float64
    t_delay::Float64
end

num_agents(mapf::MAPF) = length(mapf.starts)

"""
    type alias for a path through the graph
"""
const GraphPath = Vector{Edge}
get_start_node(path::Vector{E} where {E <: Edge}) = get(path,1,Edge(-1,-1)).src
get_final_node(path::Vector{E} where {E <: Edge}) = get(path,length(path),Edge(-1,-1)).dst
traversal_time(path::Vector{E} where {E <: Edge},mapf::MAPF) = sum([get_prop(mapf.graph,e,:weight) for e in path])

"""type alias for a list of agent paths"""
const LowLevelSolution = Vector{GraphPath}

function is_valid(solution::LowLevelSolution,mapf::MAPF)
    """checks if a solution is valid"""
    for (i,path) in enumerate(solution)
        if length(path) == 0
            return false
        end
        if get_start_node(path) != mapf.starts[i]
            return false
        end
        if get_final_node(path) != mapf.goals[i][end]
            return false
        end
    end
    return true
end

# --------------------- Node Conflicts -------------------------------------- #
"""
    Encodes a conflict between two agents at a particular node.
"""
struct NodeConflict
    agent1_id::Int
    agent2_id::Int
    node_id::Int
    t1::Float64 # Nominal time of arrival of agent 1
    t2::Float64 # Nominal time of arrival of agent 2
    conflict_probability::Float64
end

function invalid_node_conflict()
    """Returns an invalid NodeConflict"""
    NodeConflict(-1,-1,-1,-1,-1,-1)
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
    t1::Float64 #Nominal time robot 1 leaves node 2
    t2::Float64 #Nominal time robot 2 leaves node 1
    collision_probability::Float64
end

function invalid_edge_conflict()
    """returns an invalid EdgeConflict"""
    return EdgeConflict(-1,-1,-1,-1,-1,-1,-1)
end

function is_valid(conflict::EdgeConflict)
    """checks if an edge node is invalid"""
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
    t::Float64# time ID
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
    t::Float64 # time ID
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
    # We add the constraint whatever happens (wait before reaching your goal to let the other agent pass)
    node.constraints[constraint.a].node_constraints[constraint.v] = constraint.t
    return true
    # if constraint.v != mapf.goals[constraint.a][end] #is the constraint not the final goal of the agent?
    #     node.constraints[constraint.a].node_constraints[constraint.v] = constraint.t
    #     return true
    # end
    # return false
end

"""adds an `EdgeConstraint` t o a ConstraintTreeNode"""
function add_constraint!(node::ConstraintTreeNode,constraint::EdgeConstraint,mapf::MAPF)
    node.constraints[constraint.a].edge_constraints[(constraint.node1_id,constraint.node2_id)] = constraint.t
    return true
    # if (constraint.node2_id != mapf.goals[constraint.a][end])
    #     println("Adding constraint")
    #     println(constraint)
    #     node.constraints[constraint.a].edge_constraints[(constraint.node1_id,constraint.node2_id)] = constraint.t
    #     # For the moment we only want edge constraints to be unidirectional
    #     #node.constraints[constraint.a].edge_constraints[(constraint.node2_id,constraint.node1_id)] = constraint.t
    #     return true
    # end
    # return false
end


function violates_constraints(constraints::ConstraintDict,v,path,mapf::MAPF,start_time::Float64)
    """ Check if a set of constraints would be violated by adding an Edge from
        the final vertex of `path` to `v`
        returning true means the constraints are violated"""

    # Time it takes to go through path
    t1 = start_time
    if length(path) >= 1
        t1 += traversal_time(path,mapf)
        v1 = get_final_node(path)
    else
        v1 = mapf.starts[constraints.a] #Get the start point of the concerned agent
        t1 += 0.0
    end

    # We add the time it takes to go to v from the end of path
    v2 = v
    e = Edge(v1,v2)
    if has_edge(mapf.graph,e)
        t = t1 + get_prop(mapf.graph,e,:weight)
    else
        @assert(length(path) == 0)
        return false
    end

    # If such constraint exists, agent is only authorized to visit v after t_min
    t_min = get(constraints.node_constraints,v,false)


    # -- Node Check -- #
    # If there is a time constraint for this agent and vertex
    # and if agent wants to visit v before authorized time t_min
    if t_min != false
        if t_min > t
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
    #println("Getting cost of a solution: ", paths)
    return sum([sum([get_prop(mapf.graph,e,:weight) for e in p]) for p in paths])
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
    #Count node conflicts quickly to elimininate cases with zeros
    res = @timed(count_node_conflicts(n1,t1,n2,t2,nn,lambda;num_particles=1000))
    preliminary_count = res[1]
    dtcount = res[2]
    if preliminary_count < 0.0001
        return 0.0,0.0,dt_count
    else
        return integrate_node_with_box(n1,t1,n2,t2,nn,lambda,dtcount)
    end
end

function integrate_node_with_box(n1,t1,n2,t2,nn,lambda,dtcount)
    function h(x)
        y = x[1]
        t = x[2]
        density = (1-cdf(Gamma(nn,lambda),abs(t2-t1-y))) * pdf(Gamma(n1,lambda), t) * pdf(Gamma(n2,lambda),t-y)
        return density
    end

    tmax = (-n1*n2 + n1 + n2 - 1)*lambda/(1-n1)
    ymax = tmax - (n1-1)*lambda

    yleft = min(ymax,t1-t2)
    yright = max(ymax,t1-t2)

    tleft = max(0,min(tmax,t1-t2+(n1-1)/lambda))
    tright=max(0,max(tmax,t1-t2+(n1-1)/lambda))

    left = [yleft-bound_radius,max(tleft-bound_radius,0)]
    right = [yright + bound_radius, tright+bound_radius]

    res1 = @timed(hcubature(h,left,right,maxevals=10^8))
    C,err = res1[1]
    dtint = res1[2] #time spent performing integration

    return C, err, dtint + dtcount
end

function get_collision_probability_node_old(n1,t1,n2,t2,nn,lambda)

    function h(x)
        y = x[1]
        t = x[2]
        density = (1-cdf(Gamma(nn,lambda),abs(t1-t2-y))) * pdf(Gamma(n2,lambda), t) * pdf(Gamma(n1,lambda),t-y)
        return density
    end

    a = [-100.0;-100.0]
    b = [100.0,100.0]
    res1 = @timed(hcubature(h,a,b,maxevals=10^8))
    C,err = res1[1]
    dt = res1[2] #time spent performing integration

    return C, err, dt
end

function count_node_conflicts(n1,t1,n2,t2,nn,lambda;num_particles=10000,task_completion=[0,0])
    """Monte Carlo simulation"""
    EA1 = rand(Gamma(n1,lambda),num_particles)
    EA2 = rand(Gamma(n2,lambda),num_particles)
    ed1 = rand(Gamma(nn,lambda),num_particles)
    ed2 = rand(Gamma(nn,lambda),num_particles)

    r1_arrivals = t1 .+ EA1
    r1_departures = r1_arrivals + ed1 .+ task_completion[1]
    r2_arrivals = t2 .+ EA2
    r2_departures = r2_arrivals + ed2 .+ task_completion[2]

    num_conflicts =length(findall(((r2_departures-r1_arrivals).>0) .& ((r1_departures-r2_arrivals).>0)   ))

    return num_conflicts/num_particles
end

function get_collision_probability_edge(n1,t1,n2,t2,t_edge,lambda)
    #Count edge conflicts quickly to elimininate cases with zeros
    res = @timed(count_edge_conflicts(n1,t1,n2,t2,t_edge,lambda;num_particles=1000))
    preliminary_count = res[1]
    dtcount = res[2]
    if preliminary_count < 0.0001
        return 0.0,0.0,dt_count
    else
        return integrate_edge_with_box(n1,t1,n2,t2,t_edge,lambda,dtcount)
    end
end

function integrate_edge_with_box(n1,t1,n2,t2,t_edge,lambda,dtcount)
    function h(x)
        e2 = x[1]
        density = (cdf(Gamma(n2,lambda),max(0,(e2+t2-t1+t_edge))) -  cdf(Gamma(n2,lambda),max(0,(e2+t2-t1-t_edge))) ) * pdf(Gamma(n1,lambda), e2)
        return density
    end

    a = [0.0]
    b = [100.0]

    res1 = @timed(hcubature(h,a,b,maxevals=10^8))
    C,err = res1[1]
    dtint = res1[2] #time spent performing integration

    return C, err, dtint + dtcount
end

function get_collision_probability_edge_old(n1,t1,n2,t2,t_edge,lambda)

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
    b = [z2,100.0]
    m = [z3;0.0]
    n = [z4,100.0]
    res1 = @timed(hcubature(f,a,b))
    res2 = @timed(hcubature(g,m,n))
    C1,err1 = res1[1]
    C2,err2 = res2[1]
    C = C1 + C2
    err = err1 + err2
    dt = res1[2] + res2[2] #time spent performing integration

    return C, err, dt
end

function count_edge_conflicts(n1,t1,n2,t2,t_edge,lambda;num_particles=10000)
    """Monte Carlo simulation"""
    EA1 = rand(Gamma(n1,lambda),num_particles)
    EA2 = rand(Gamma(n2,lambda),num_particles)

    r1_arrivals = t1 .+ EA1
    r1_departures = r1_arrivals .+ t_edge
    r2_arrivals = t2 .+ EA2
    r2_departures = r2_arrivals .+ t_edge

    num_conflicts =length(findall(((r2_departures-r1_arrivals).>0) .& ((r1_departures-r2_arrivals).>0)   ))

    return num_conflicts/num_particles
end

function fill_graph_with_path!(robot_id::Int, robotpath::GraphPath, mapf::MAPF)
    """This function adds the path traversed by the robot to the nodes and
    edges of the metagraph
    :occupancy is a property of each vertex and edge, and is a dictionary
    mapping a robot id to the tuple (n,t,t_task) for a node and (n,t) for
    an edge."""

    # This is a counter for the advancement of our robot along its destination.
    # Thanks to this counter, we are able to track when it will spend time
    # performing tasks at stations, and add this to our nominal times. In
    # addition, we wish to track the status of our robot to decide which
    # conflicts to expand.
    advancement = 1

    # Get start vertex
    start_vert = robotpath[1].src

    # Get uncertainty information from the starting point
    sum_ns_traversed = get_prop(mapf.graph,start_vert, :n_delay)

    # Get starting time
    time_traversed = mapf.start_times[robot_id]

    # Check whether this start node is a node at which the robot needs to perform a task
    if start_vert == mapf.goals[robot_id][advancement]
        t_task = mapf.goal_completion_times[robot_id][advancement]
        advancement += 1
    else
        t_task = 0.0
    end

    # Get occupancy object for the start node
    occupancy = get_prop(mapf.graph,start_vert,:occupancy)

    # Add element to the occupancy dictionary of the start vertex:
    # Map robot id to (n,t,t_task)
    setindex!(occupancy,(sum_ns_traversed, time_traversed,t_task),robot_id) #robot_id is the key

    # Update the occupancy property of the vertex with the new dictionary
    set_prop!(mapf.graph,robotpath[1].src,:occupancy,occupancy)

    # Now, after adding occupancy info, add t_task to time_traversed
    time_traversed += t_task

    # Loop through all edges
    for e in robotpath

        # Get properties of the edge and last vertex
        t_edge = get_prop(mapf.graph,e, :weight)
        n_next_node = get_prop(mapf.graph, e.dst, :n_delay)

        # Add occupancy for the edge
        occupancy = get_prop(mapf.graph,e,:occupancy)
        setindex!(occupancy,(sum_ns_traversed, time_traversed),robot_id) #robot_id is the key
        set_prop!(mapf.graph,e,:occupancy,occupancy)

        # Update nominal time
        time_traversed += t_edge


        # Check whether the end node is a node at which the robot needs to perform a task
        if e.dst == mapf.goals[robot_id][advancement]
            t_task = mapf.goal_completion_times[robot_id][advancement]
            advancement += 1
        else
            t_task = 0.0
        end

        # Add occupancy for second vertex of the edge
        occupancy = get_prop(mapf.graph,e.dst,:occupancy)
        setindex!(occupancy,(sum_ns_traversed, time_traversed,t_task),robot_id) #robot_id is the key
        set_prop!(mapf.graph,e.dst,:occupancy,occupancy)

        # Update traversal time with t_task
        time_traversed += t_task

        # Update sum of ns traversed
        sum_ns_traversed += n_next_node
    end

    return
end

function clear_graph_occupancy!(mapf::MAPF)
    """Removes all occupancies from the graph. Ideally, we should only care about removing
        occupancy information concerning one robot at a time and not use this function.
        However, we are expanding a multitude of nodes with different solutions
        and will be lacking memory to store all intermediate occupancy graphs,
        so we can't that much do this and need this function."""
    for v in vertices(mapf.graph)
        set_prop!(mapf.graph, v, :occupancy, Dict{Int64, Tuple{Int64,Float64,Float64}}())
    end
    for e in edges(mapf.graph)
        set_prop!(mapf.graph, e, :occupancy, Dict{Int64, Tuple{Int64,Float64}}())
    end
    return
end

function count_most_likely_conflicts!(mapf::MAPF,paths::LowLevelSolution,num_interactions)
    """Returns the most likely conflict that occurs in a given solution:
    - paths:        a list of graph edges to be traversed by the agents"""
    # If num_interactions[2] == 0, it means we are the first time we want to
    # count interactions, so we change both num_interactions[1] and [2]

    # print("Trying to get the most likely conflict from solution: \n")
    # print(paths, "\n")
    interaction_count = 0

    countingtime = 0

    epsilon = mapf.epsilon
    lambda = mapf.lambda
    conflict_params_node = []
    conflict_params_edge = []

    clear_graph_occupancy!(mapf::MAPF)

    # The first step is to fill the graph with occupancy information
    for (robot_id,robotpath) in enumerate(paths)
        fill_graph_with_path!(robot_id,robotpath,mapf)
    end

    node_conflict = invalid_node_conflict()
    edge_conflict = invalid_edge_conflict()

    # Initialize max conflict probabilities at 0
    node_conflict_p = 0.0
    edge_conflict_p = 0.0

    # ----- Node conflicts ----- #
    for v in vertices(mapf.graph)

        #Get node information
        nn = get_prop(mapf.graph, v, :n_delay)
        occupancy = get_prop(mapf.graph,v,:occupancy)

        # There is interaction at this node
        if length(occupancy) >= 2
            list_of_occupants = collect(keys(occupancy))
            pairs_of_occupants = collect(combinations(list_of_occupants,2))
            if num_interactions[2] == 0
                interaction_count += length(pairs_of_occupants)
            end
            for (r1,r2) in pairs_of_occupants
                (n1,t1,t_task1) = occupancy[r1]
                (n2,t2,t_task2) = occupancy[r2]
                #(cp,err,dt) = get_collision_probability_node(n1,t1,n2,t2,nn,lambda)
                res = @timed(count_node_conflicts(n1,t1,n2,t2,nn,lambda;task_completion=[t_task1,t_task2]))
                cp = res[1]
                dt = res[2]

                countingtime += dt

                # Conflict is likely and higher than all previously found
                if cp > maximum([epsilon,node_conflict_p])
                    node_conflict_p = cp
                    node_conflict = NodeConflict(r1,r2,v,t1,t2,cp)
                    conflict_params_node = [n1,n2,lambda,nn]
                end
            end
        end
    end

    # ----- Edge conflicts ----- #
    for e in edges(mapf.graph)

        if e.src < e.dst

            #Get edge information. We consider confrontations as robots arriving in
            #opposite directions only
            t_edge = get_prop(mapf.graph,e,:weight)
            occupancy = get_prop(mapf.graph,e,:occupancy)
            reverse_edge = Edge(e.dst,e.src)
            reverse_occupancy = get_prop(mapf.graph,reverse_edge,:occupancy)

            # There is interaction at this edge
            if length(occupancy)*length(reverse_occupancy) >= 1 && e.src < e.dst
                occupants_1 = keys(occupancy)
                occupants_2 = keys(reverse_occupancy)
                for robot1_id in occupants_1
                    for robot2_id in occupants_2
                        if robot1_id != robot2_id
                            if num_interactions[2] == 0
                                interaction_count += 1
                            end
                            (n1,t1) = occupancy[robot1_id]
                            (n2,t2) = reverse_occupancy[robot2_id]
                            #(cp,err,dt) = get_collision_probability_edge(n1,t1,n2,t2,t_edge,lambda)

                            res = @timed(count_edge_conflicts(n1,t1,n2,t2,t_edge,lambda))
                            cp = res[1]
                            dt = res[2]
                            countingtime += dt

                            if cp > maximum([epsilon,edge_conflict_p])
                                edge_conflict_p = cp
                                #time at which robot 1/2 leaves from node 2/1 (the last)
                                edge_conflict = EdgeConflict(robot1_id,robot2_id,e.src,e.dst,t1+t_edge,t2+t_edge,cp)
                                conflict_params_edge = [n1,n2,lambda,t_edge]
                            end
                        end
                    end
                end
            end
        end
    end

    # Set the number of interactions
    if num_interactions[2] == 0
        num_interactions[2] = 1
        num_interactions[1] = interaction_count
    end

    clear_graph_occupancy!(mapf)
    if node_conflict_p >= edge_conflict_p
        return node_conflict, invalid_edge_conflict(), countingtime,conflict_params_node
    else
        return invalid_node_conflict(), edge_conflict, countingtime,conflict_params_edge
    end
end

# --------------------------------------------------------------------------- #

# ----------------- Create constraints from conflicts ----------------------- #

"""generates a set of constraints from a NodeConflict"""
function generate_constraints_from_conflict(node::ConstraintTreeNode,conflict::NodeConflict,t_delay::Float64,conflict_params,epsilon)
    # If there was already a constraint and this was not enough to prevent collision,
    # we want to add t_delay to the already present delay

    n1 = conflict_params[1]
    n2 = conflict_params[2]
    lambda = conflict_params[3]
    nn = conflict_params[4]

    t_yield1 = conflict.t1
    t_yield2 = conflict.t2

    robot1_id = conflict.agent1_id
    robot2_id = conflict.agent2_id

    #Agent 1 yields:
    tf1 = t_yield1
    Pc=1
    while Pc > epsilon
        tf1 += t_delay
        Pc = count_node_conflicts(n1,tf1,n2,t_yield2,nn,lambda;num_particles=5000)
    end

    #Agent 2 yields:
    tf2 = t_yield2
    Pc=1
    while Pc > epsilon
        tf2 += t_delay
        Pc = count_node_conflicts(n1,t_yield1,n2,tf2,nn,lambda;num_particles=5000)
    end


    return [
        # Agent 1 may not occupy node until agent 2 leaves the node, with t_delay
        NodeConstraint(
            conflict.agent1_id,
            conflict.node_id,
            tf1
        ),
        # Agent 2 may not occupy node until agent 1 leaves the node, with t_delay
        NodeConstraint(
            conflict.agent2_id,
            conflict.node_id,
            tf2
        )
        ]
end


"""generates a set of constraints from an EdgeConflict"""
function generate_constraints_from_conflict(node::ConstraintTreeNode,conflict::EdgeConflict,t_delay::Float64,conflict_params,epsilon)
    t_yield1 = conflict.t1
    t_yield2 = conflict.t2

    n1 = conflict_params[1]
    n2 = conflict_params[2]
    lambda = conflict_params[3]
    t_edge = conflict_params[4]

    robot1_id = conflict.agent1_id
    #pre_existing_constraintDict = get(node.constraints,robot1_id,false)
    #if pre_existing_constraintDict != false
    #    t_yield1 = get(pre_existing_constraintDict.edge_constraints,(conflict.node1_id,conflict.node2_id),t_yield1)
    #end

    robot2_id = conflict.agent2_id
    #pre_existing_constraintDict = get(node.constraints,robot2_id,false)
    #if pre_existing_constraintDict != false
    #    t_yield2 = get(pre_existing_constraintDict.edge_constraints,(conflict.node1_id,conflict.node2_id),t_yield2)
    #end

    #Agent 1 yields:
    tf1 = t_yield1
    Pc=1
    while Pc > epsilon
        tf1 += t_delay
        Pc = count_edge_conflicts(n1,tf1,n2,t_yield2,t_edge,lambda;num_particles=5000)
    end

    #Agent 2 yields:
    tf2 = t_yield2
    Pc=1
    while Pc > epsilon
        tf2 += t_delay
        Pc = count_edge_conflicts(n1,t_yield1,n2,tf2,t_edge,lambda;num_particles=5000)
    end


    return [
        # Agent 1 may not enter node 1 of Edge(node1,node2) until robot 2 is
        # finished traversing node 1.
        EdgeConstraint(
            conflict.agent1_id,
            conflict.node1_id,
            conflict.node2_id,
            tf1 # + 1
        ),
        # Agent 2 may not enter node 2 of Edge(node1,node2) until robot 1 is
        # finished traversing node 2.
        EdgeConstraint(
            conflict.agent2_id,
            conflict.node2_id,
            conflict.node1_id,
            tf2# + 1
        )
        ]
end

function low_level_search!(mapf::MAPF,
    node::ConstraintTreeNode,
    distmx,
    distmx_DP,
    idxs=collect(1:num_agents(mapf)),
    path_finder=LightGraphs.a_star)
    """Returns a low level solution for a MAPF with constraints"""
    # compute an initial solution
    # solution = LowLevelSolution()
    println("ASTAR: ", length(idxs))
    sleep(0.1)

    # Counter for how long it takes to perform astar
    astar_time = 0.0

    for i in idxs #Agent we perform it on

        # Agent i starts at time start_time[i]
        start_time = mapf.start_times[i]
        # Its path is originally empty
        path = Vector{}()

        # We go through all intermediate goals one by one
        for (k,goal) in enumerate(mapf.goals[i])

            #If it is our first iteration, we start at starts[i]
            if k == 1
                start = mapf.starts[i]
            else
                # We start at the vertex of the intermediary goal we just accomplished.
                # Start time was prepared at the end of the last iteration
                start = mapf.goals[i][k-1]
            end

            # Astar
            pathwtime = @timed(path_finder(mapf.graph,start,goal,start_time,get_constraints(node,i),mapf,distmx_DP,distmx))

            # Extract the time it took to find the path
            astar_time += pathwtime[2]

            # Merge the intermediary paths with the new path
            path = vcat(path, pathwtime[1][1])

            # Expected completion time for the path portion we just found
            path_time = pathwtime[1][2]

            # Prepare next start time by adding expected traversal time
            start_time += path_time

            # Add the time it takes to complete the intermediary task, if applicable
            start_time += try
                mapf.goal_completion_times[i][k]
            catch
                0.0
            end

        end

        # Add completed path to solution
        node.solution[i] = path
    end

    # Get the nominal cost of the entire solution to add to the new STTCBS node
    node.cost = try
        get_cost(node.solution,mapf) #We want the cost of the entire solution, not just the paths we found.
    catch
        typemax(Int32)
    end

    return node.solution, node.cost, astar_time
end


include("low_level_search/a_star.jl") #Modified version of astar

function prepare_edge_weight_mx!(mapf::MAPF)
    """ Returns the matrix of expected travel times from vertex to vertex """
    # For astar, add delay information to the graph that you didn't have while constructing it
    distmx = 1000000 .* ones(length(vertices(mapf.graph)),length(vertices(mapf.graph)))
    for e in edges(mapf.graph)
        n_delay = get_prop(mapf.graph,e.dst, :n_delay)
        lambda = mapf.lambda
        t_edge = get_prop(mapf.graph,e, :weight)
        distmx[e.src,e.dst] = t_edge + n_delay*lambda
        set_prop!(mapf.graph,e,:expTravelTime,t_edge + n_delay*lambda)
    end
    for v in vertices(mapf.graph)
        # You can pause at a node for one second or multiples.
        # This seems restrictive, but we can lower it as needed, which may make
        # some paths a little longer.
        distmx[v,v] = 1.0
    end
    return distmx
end

"""
    Continuous Time CBS algorithm
"""
function CTCBS(mapf::MAPF,path_finder=LightGraphs.a_star)

    # Get the matrix of edge weights
    distmx = prepare_edge_weight_mx!(mapf)

    #Now that the weight matrix is computed, let's find the distmx for the heuristic
    distmx_DP = compute_distance_matrix(mapf.graph,distmx)

    # Max number of iterations before we consider STTCBS has "failed" for time reasons
    max_iterations = 800

    # Counter for the number of iterations
    iteration_count = 0

    # The priority queue stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    # Time we spent calculating conflict probability (by counting or by integrating)
    countingtime = 0.0

    # Time spent finding paths
    time_spent_on_astar = 0.0

    # We collect the number of interactions we find
    num_interactions = [0,0]

    # Root node R containing 0 constraints
    root_node = initialize_root_node(mapf)

    # Apply A* without conflicts (naive paths)
    _,_,astartime = low_level_search!(mapf,root_node,distmx,distmx_DP)
    sleep(0.01) # for printing purposes

    # Add the time we spent on astar to astartime
    time_spent_on_astar += astartime

    # If the solution enables all agents to reach their goals, we add it to the queue.
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    # Loop through the Priority Queue by best-first search
    while length(priority_queue) > 0 && iteration_count < max_iterations

        print("\n \n")

        # Get lowest cost node from the queue
        node = dequeue!(priority_queue)

        # Print current solution
        #println("Solution being examined:")
        #println(node.solution)

        # Run our model to find the most likely conflict
        node_conflict, edge_conflict, counting_deltat,conflict_params = count_most_likely_conflicts!(mapf,node.solution,num_interactions)

        # Add the time we spent finding probabilities to countingtime
        countingtime += counting_deltat

        # One of node or edge conflicts should be invalid (one conflict is more likely than the other).
        # Therefore we create constraints for the one of them that is the most likely, thus valid
        if is_valid(node_conflict)

            # Print
            #println("Agents ", node_conflict.agent1_id, " and ", node_conflict.agent2_id, " conflict at node ", node_conflict.node_id)

            # Create two node constraints
            constraints = generate_constraints_from_conflict(node,node_conflict,mapf.t_delay,conflict_params,mapf.epsilon)

        elseif is_valid(edge_conflict)

            # Print
            #println("Agents ", edge_conflict.agent1_id, " and ", edge_conflict.agent2_id, " conflict at edge ", edge_conflict.node1_id, " - ", edge_conflict.node2_id)

            # Create two edge conflicts
            constraints = generate_constraints_from_conflict(node,edge_conflict,mapf.t_delay,conflict_params,mapf.epsilon)
            println("Constraints")
            println(constraints)

        else

            # If we have found no conflict with probability greater than ϵ, we found the optimal solution!
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            print("Time spent on probability count: ", countingtime, " \n")
            print("Time spent on path finding: ", time_spent_on_astar, " \n")
            return (node.solution, node.cost,countingtime,time_spent_on_astar,num_interactions,iteration_count)
        end

        # generate new nodes from constraints
        for constraint in constraints

            # create node with all of the previous constraints
            new_node = initialize_child_node(node)

            # Try to add the new constraint: this will fail if you try to steer
            # away an agent from its goal (final, not intermediary).
            # The algorithm can therefore fail if two agents are assigned the
            # same final goal and are found to conflict while arriving there.
            if add_constraint!(new_node,constraint,mapf)
                _,_,astartime = low_level_search!(mapf,new_node,distmx,distmx_DP,[get_agent_id(constraint)])
                sleep(0.01)

                # Add time spent on astar
                time_spent_on_astar += astartime

                #If new solution is valid, add it to the queue
                if is_valid(new_node.solution, mapf)
                    #print("Consequently we found the solutions: \n")
                    #print(new_node.solution, "\n")
                    #print("Adding new node to priority queue","\n")
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
        iteration_count += 1
    end

    # If the max number of iterations has passed or if the priority queue has emptied, failure
    print("No Solution Found. Returning default solution")
    return (LowLevelSolution(), typemax(Int),countingtime,time_spent_on_astar,num_interactions[1],iteration_count)
end


include("simulations.jl")
include("plotting.jl")

end # module
