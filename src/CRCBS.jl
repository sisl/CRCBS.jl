module CRCBS

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

include("problem_definitions.jl")
include("utils.jl")
include("low_level_search/a_star.jl")
include("low_level_search/implicit_graphs.jl")

export
    CBSPath,
    CBS_State,
    CBS_Action,
    # get_start_node,
    # get_final_node,
    traversal_time,
    LowLevelSolution,
    is_valid,
    StateConflict,
    detect_state_conflict,
    invalid_state_conflict,
    ActionConflict,
    detect_action_conflict,
    invalid_action_conflict,
    ConflictTable,
    detect_conflicts,
    detect_conflicts!,
    NodeConstraint,
    EdgeConstraint,
    ConstraintTreeNode,
    initialize_root_node,
    initialize_child_node,
    ConstraintDict,
    get_constraints,
    add_constraint!,
    violates_constraints,
    get_cost,
    empty_constraint_node,
    get_next_conflicts,
    get_conflicts,
    generate_constraints_from_conflict,
    low_level_search!,
    AbstractMAPFSolver,
    CBS


# const CBS_State = Int
@with_kw struct CBS_State
    vtx::Int = -1
end

"""
    Check if `s1` and `s2` match. This does not necessarily reflect equality. It
    is a helper to check if the start state matches the goal state
"""
states_match(s1::CBS_State,s2::CBS_State) = (s1.vtx == s2.vtx)

const CBS_Action = Edge{Int}
CBS_Action() = Edge(-1,-1)

# return an invalid_state
CRCBS.invalid_state(t) = CBS_State()
CRCBS.invalid_action(t) = CBS_Action()
"""
    return a special `wait` state, where the agent remains in the current state
"""
wait(s::CBS_State) = CBS_Action(s.vtx,s.vtx)

struct CBSLowLevelEnv{G <: AbstractGraph,C} <: AbstractLowLevelEnv{CBS_State,CBS_Action}
    graph::G
    constraints::C
end
# get_possible_actions
struct ActionIter
    env::CBSLowLevelEnv
    s::Int # source state
    neighbor_list::Vector{Int} # length of target edge list
end
struct ActionIterState
    idx::Int # idx of target node
end
function Base.iterate(it::ActionIter)
    iter_state = ActionIterState(0)
    return iterate(it,iter_state)
end
function Base.iterate(it::ActionIter, iter_state::ActionIterState)
    iter_state = ActionIterState(iter_state.idx+1)
    if iter_state.idx > length(it.neighbor_list)
        return nothing
    end
    Edge(it.s,it.neighbor_list[iter_state.idx]), iter_state
end
CRCBS.get_possible_actions(env::CBSLowLevelEnv,s::CBS_State) = ActionIter(env,s.vtx,outneighbors(env.graph,s.vtx))
# get_next_state
CRCBS.get_next_state(env::CBSLowLevelEnv,s::CBS_State,a::CBS_Action) = CBS_State(a.dst)
CRCBS.get_next_state(s::CBS_State,a::CBS_Action) = CBS_State(a.dst)
# get_transition_cost
CRCBS.get_transition_cost(env::CBSLowLevelEnv,s::CBS_State,a::CBS_Action,sp::CBS_State) = 1
# check for constraint violation
function CRCBS.violates_constraints(env::CBSLowLevelEnv, path::Path{CBS_State,CBS_Action}, s::CBS_State, a::CBS_Action, sp::CBS_State)
    t = length(path) + 1
    if get(env.constraints.node_constraints,NodeConstraint(get_agent_id(env.constraints),sp.vtx,t),false)
        return true
    else
        v1 = s.vtx
        v2 = sp.vtx
        if get(env.constraints.edge_constraints,EdgeConstraint(get_agent_id(env.constraints),v1,v2,t),false)
            return true
        end
    end
    return false
end
# check criteria for premature termination
CRCBS.check_termination_criteria(env::CBSLowLevelEnv,cost,path,s) = false

"""
    returns the tuple (s,a,s') corresponding to step t of `path`, or the tuple
    (s,wait(s),s) corresponding to waiting at that node of the path
"""
function get_path_node(path::Path,t)
    if t <= length(path)
        return path[t]
    else
        t₀ = length(path)
        sp = get_final_state(path)
        for τ in length(path)+1:t
            s = sp
            a = wait(s)
            sp = get_next_state(s,a)
        end
        return PathNode(s,a,sp)
    end
end

"""
    Extends `path` to match a given length by adding tuples corresponding to
    waiting at the final state

    args:
    - path      the path to be extended
    - the desired length of the new path
"""
function extend_path!(path::Path,T::Int)
    while length(path) < T
        s = get_final_state(path)
        a = wait(s)
        push!(path,PathNode(s,wait(s),get_next_state(s,a)))
    end
    return path
end

"""
    Extends a copy of `path` to match a given length by adding tuples
    corresponding to waiting at the final state

    args:
    - path      the path to be extended
    - the desired length of the new path
"""
function extend_path(path::Path,T::Int)
    new_path = copy(path)
    while length(new_path) < T
        s = get_final_state(new_path)
        a = wait(s)
        push!(new_path,PathNode(s,wait(s),get_next_state(s,a)))
    end
    return new_path
end


""" Type alias for a path through the graph """
const CBSPath = Path{CBS_State,CBS_Action}
# get_start_node(path::CBSPath) = get_initial_state(path)
# get_final_node(path::CBSPath) = get_final_state(path)
traversal_time(path::CBSPath) = length(path)

"""
    Returns the edge at time t or a "self-loop" edge on the final node of the
    path
"""
function get_action(path::Path{S,A}, t::Int) where {S,A}
    if length(path) < t
        return wait(get_final_state(path))
    elseif length(path) > 0
        return path[t].a
    else
        a = invalid_action(t) # must be overridden
        @assert(typeof(a) == A)
        return a
    end
end

""" Type alias for a list of agent paths """
const LowLevelSolution = Vector{CBSPath}

"""
    Checks if an individual path satisfies start and end constraints
"""
function is_valid(path::CBSPath,start,goal)
    return (states_match(get_initial_state(path), start)
        && states_match(get_final_state(path), goal))
end

"""
    checks if a solution is valid
"""
function is_valid(solution::LowLevelSolution,mapf::MAPF)
    for (i,path) in enumerate(solution)
        if !is_valid(path,mapf.starts[i],mapf.goals[i])
            return false
        end
    end
    return true
end

abstract type CBSConflict end

"""
    Encodes a conflict wherein two agents occupy a particular node at a
    particular time
"""
struct StateConflict{S} <: CBSConflict
    agent1_id::Int
    agent2_id::Int
    node1_id::S
    node2_id::S
    t::Int
end
agent1_id(conflict::StateConflict) = conflict.agent1_id
agent2_id(conflict::StateConflict) = conflict.agent2_id
node1_id(conflict::StateConflict) = conflict.node1_id
node2_id(conflict::StateConflict) = conflict.node2_id
time_of(conflict::StateConflict) = conflict.t

""" Returns an invalid StateConflict """
invalid_state_conflict() = StateConflict(-1,-1,invalid_state(0),invalid_state(0),-1)

""" Checks if a node conflict is valid """
is_valid(conflict::StateConflict) = (agent1_id(conflict) != -1)

"""
    Detect a `StateConflict` between two path nodes. Must be overridden for each
    specific path class
"""
function detect_state_conflict(n1::PathNode{CBS_State,CBS_Action},n2::PathNode{CBS_State,CBS_Action})
    if n1.sp.vtx == n2.sp.vtx
        return true
    end
    return false
end

""" Checks for a `StateConflict` between two `CBSPath`s at time t """
function detect_state_conflict(path1::CBSPath,path2::CBSPath,t::Int)
    if detect_state_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

"""
    Encodes a conflict between two agents at a particular edge at a particular
    time. This means that the agents are trying to swap places at time t.
"""
struct ActionConflict{S} <: CBSConflict
    agent1_id::Int
    agent2_id::Int
    node1_id::S
    node2_id::S
    t::Int
end
agent1_id(conflict::ActionConflict) = conflict.agent1_id
agent2_id(conflict::ActionConflict) = conflict.agent2_id
node1_id(conflict::ActionConflict) = conflict.node1_id
node2_id(conflict::ActionConflict) = conflict.node2_id
time_of(conflict::ActionConflict) = conflict.t

""" Returns an invalid ActionConflict """
invalid_action_conflict() = ActionConflict(-1,-1,invalid_state(0),invalid_state(0),-1)

""" checks if an edge node is invalid """
is_valid(conflict::ActionConflict) = (agent1_id(conflict) != -1)

"""
    Detect an `ActionConflict` between two path nodes. Must be overridden for
    each specific path class
"""
function detect_action_conflict(n1::PathNode{CBS_State,CBS_Action},n2::PathNode{CBS_State,CBS_Action})
    if (n1.a.src == n2.a.dst) && (n1.a.dst == n2.a.src)
        return true
    end
    return false
end

""" Checks for an `ActionConflict` between two `CBSPath`s at time t """
function detect_action_conflict(path1::CBSPath,path2::CBSPath,t::Int)
    # e1 = get_action(path1,t)
    # e2 = get_action(path2,t)
    if detect_action_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

"""
    A lookup table to store all conflicts that have been detected
"""
@with_kw struct ConflictTable
    state_conflicts::Dict{Tuple{Int,Int},Vector{StateConflict}} = Dict{Tuple{Int,Int},Vector{StateConflict}}()
    action_conflicts::Dict{Tuple{Int,Int},Vector{ActionConflict}} = Dict{Tuple{Int,Int},Vector{ActionConflict}}()
end
""" helper for retrieving conflicts associated with agents i and j """
function get_conflicts(conflict_table::ConflictTable,i::Int,j::Int)
    if i < j
        state_conflicts = get(conflict_table.state_conflicts, (i,j), Vector{StateConflict}())
        action_conflicts = get(conflict_table.action_conflicts, (i,j), Vector{ActionConflict}())
    else
        state_conflicts = get(conflict_table.state_conflicts, (j,i), Vector{StateConflict}())
        action_conflicts = get(conflict_table.action_conflicts, (j,i), Vector{ActionConflict}())
    end
    return state_conflicts, action_conflicts
end

function Base.copy(c::ConflictTable)
   c_new = ConflictTable()
   for (k,v) in c.state_conflicts
       c_new.state_conflicts[k] = copy(v)
   end
   for (k,v) in c.action_conflicts
       c_new.action_conflicts[k] = copy(v)
   end
   return c_new
end


"""
    detect conflicts between paths
"""
function detect_conflicts(path1::CBSPath,path2::CBSPath,i::Int,j::Int)
    state_conflicts = Vector{StateConflict}()
    action_conflicts = Vector{ActionConflict}()
    if length(path1) > length(path2)
        path2 = extend_path(path2,length(path1))
    elseif length(path1) < length(path2)
        path1 = extend_path(path1,length(path2))
    end
    @assert(length(path1) == length(path2))
    for t in 1:length(path1)
        path_node1 = path1[t]
        path_node2 = path2[t]
        # check for state conflicts
        if detect_state_conflict(path_node1,path_node2)
            push!(state_conflicts, StateConflict(i,j,path_node1.sp,path_node2.sp,t))
        end
        # check for action conflicts
        if detect_action_conflict(path_node1,path_node2)
            push!(action_conflicts, ActionConflict(i,j,path_node1.s,path_node2.s,t))
        end
    end
    return state_conflicts, action_conflicts
end

"""
    Populates a `ConflictTable` with all conflicts that occur in a given solution

    args:
    - conflict_table        a `ConflictTable` to store the detected conflicts
    - paths:                a list of `Path`s, one for each individual agent
    - idxs                  (optional) a list of agent ids for which to check
                            collisions against all other agents
"""
function detect_conflicts!(conflict_table::ConflictTable, paths::LowLevelSolution, idxs=collect(1:length(paths)))
    for (i,path1) in enumerate(paths)
        for (j,path2) in enumerate(paths)
            if (j ∈ idxs) && (j <= i) # save time by working only on the upper triangle
                continue
            end
            state_conflicts, action_conflicts = detect_conflicts(path1,path2,i,j)
            conflict_table.state_conflicts[(i,j)] = state_conflicts
            conflict_table.action_conflicts[(i,j)] = action_conflicts
        end
    end
    return conflict_table
end

"""
    Returns a `ConflictTable` of all conflicts that occur in a given solution

    args:
    - conflict_table        a `ConflictTable` to store the detected conflicts
    - paths:                a list of `Path`s, one for each individual agent
    - idxs                  (optional) a list of agent ids for which to check
                            collisions against all other agents
"""
function detect_conflicts(paths::LowLevelSolution, idxs=collect(1:length(paths)))
    conflict_table = ConflictTable()
    detect_conflicts!(conflict_table,paths,idxs)
end

"""
    returns the next conflict (temporally) that occurs in a conflict table
"""
function get_next_conflicts(conflict_table::ConflictTable)
    state_conflicts = Vector{StateConflict}()
    for (k,v) in conflict_table.state_conflicts
        if length(v) > 0
            push!(state_conflicts, v[1])
        end
    end
    sort!(state_conflicts, by=c->time_of(c))
    action_conflicts = Vector{ActionConflict}()
    for (k,v) in conflict_table.action_conflicts
        if length(v) > 0
            push!(action_conflicts, v[1])
        end
    end
    sort!(action_conflicts, by=c->time_of(c))

    state_conflict = get(state_conflicts, 1, invalid_state_conflict())
    action_conflict = get(action_conflicts, 1, invalid_action_conflict())
    if time_of(state_conflict) <= time_of(action_conflict)
        return state_conflict, invalid_action_conflict()
    else
        return invalid_state_conflict(), action_conflict
    end
end

"""
    Returns a `StateConflict` and an `ActionConflict` next conflicts.
    The function returns after finding the FIRST conflice (StateConflict or
        ActionConflict), which means that at least one of the returned conflicts
        will always be invalid. The rational for returning both anyway is to
        preserve stability of the function's return type.
    If state_conflict and action_conflict are both invalid, the search has reached
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
    state_conflict = invalid_state_conflict()
    action_conflict = invalid_action_conflict()

    # extend paths so that they all match in length
    extended_paths = [extend_path(path,tmax) for path in paths]

    # begin search from time t, paths[i_], paths[j_]
    t = t_; i = i_; j_ = max(j_,i+1)

    path1 = get(extended_paths,i,CBSPath()) # in case i is beyond the length of paths
    n1 = get_path_node(path1,t)
    for j in j_:length(extended_paths)
        path2 = extended_paths[j]
        n2 = get_path_node(path2,t)
        if detect_state_conflict(n1,n2)
            state_conflict = StateConflict(i,j,get_sp(n1),get_sp(n2),t)
            return state_conflict, action_conflict
        elseif detect_action_conflict(n1,n2)
            action_conflict = ActionConflict(i,j,get_s(n1),get_s(n2),t)
            return state_conflict, action_conflict
        end
    end
    # Continue search from next time step
    for t in t_+1:tmax
        for (i,path1) in enumerate(extended_paths)
            n1 = get_path_node(path1,t)
            for j in i+1:length(extended_paths)
                path2 = extended_paths[j]
                n2 = get_path_node(path2,t)
                if detect_state_conflict(n1,n2)
                    state_conflict = StateConflict(i,j,get_sp(n1),get_sp(n2),t)
                    return state_conflict, action_conflict
                elseif detect_action_conflict(n1,n2)
                    action_conflict = ActionConflict(i,j,get_s(n1),get_s(n2),t)
                    return state_conflict, action_conflict
                end
            end
        end
    end
    return state_conflict, action_conflict
end

"""
    Returns a list of all conflicts that occur in a given solution

    args:
    - paths:        a list of graph edges to be traversed by the agents
"""
function get_all_conflicts(paths::LowLevelSolution)
    state_conflicts = Vector{StateConflict}()
    action_conflicts = Vector{ActionConflict}()
    for (i,path1) in paths
        for (j,path2) in paths
            if j <= i
                continue
            end
            s_conflicts, a_conflicts = detect_conflicts(path1,path2,i,j)
            union!(state_conflicts, s_conflicts)
            union!(action_conflicts, a_conflicts)
        end
    end
    sort!(state_conflicts,by=c->time_of(c))
    sort!(action_conflicts,by=c->time_of(c))
    return state_conflicts, action_conflicts
end

"""
    Returns a list of all conflicts that occur in a given solution

    args:
    - paths:        a list of graph edges to be traversed by the agents
"""
function get_conflicts(paths::LowLevelSolution)
    # TODO Make this way faster
    state_conflicts = Vector{StateConflict}()
    action_conflicts = Vector{ActionConflict}()
    t_max = maximum([length(path) for path in paths])
    nc, ec = get_next_conflicts(paths)
    while true
        if is_valid(nc)
            push!(state_conflicts, nc)
            conflict = nc
        elseif is_valid(ec)
            push!(action_conflicts, ec)
            conflict = ec
        else
            break
        end
        nc, ec = get_next_conflicts(
            paths,
            agent1_id(conflict),
            agent2_id(conflict)+1,
            time_of(conflict),
            t_max
            )
    end
    return state_conflicts, action_conflicts
end

abstract type CBSConstraint end
get_agent_id(c::CBSConstraint) = c.a

"""
    Encodes a constraint that agent `a` may not occupy vertex `v` at time `t`
"""
struct NodeConstraint{S} <: CBSConstraint
    a::Int # agent ID
    # v::Int # vertex ID
    v::S
    t::Int # time ID
end

"""
    Encodes a constraint that agent `a` may not traverse `Edge(v1,v2)` at time
    step `t`
"""
struct EdgeConstraint{S} <: CBSConstraint
    a::Int # agent ID
    v1::S # ID of first vertex in edge
    v2::S # ID of second vertex in edge
    t::Int # time ID
end

"""
    Helper function for reversing an `EdgeConstraint`
"""
flip(c::EdgeConstraint) = EdgeConstraint(get_agent_id(c),c.v2,c.v1,c.t)

"""
    constraint dictionary for fast constraint lookup within a_star
"""
@with_kw struct ConstraintDict
    node_constraints::Dict{NodeConstraint,Bool} = Dict{NodeConstraint,Bool}()
    edge_constraints::Dict{EdgeConstraint,Bool} = Dict{EdgeConstraint,Bool}()
    a::Int = -1 # agent_id
end
get_agent_id(c::ConstraintDict) = c.a

""" Helper function to merge two instances of `ConstraintDict` """
function Base.merge(d1::ConstraintDict,d2::ConstraintDict)
    @assert(get_agent_id(d1)==get_agent_id(d2))
    ConstraintDict(
        merge(d1.node_constraints,d2.node_constraints),
        merge(d1.edge_constraints,d2.edge_constraints),
        get_agent_id(d1)
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
    # maintains a list of all conflicts
    conflict_table::ConflictTable = ConflictTable()
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

"""
    Returns an empty `ConstraintTreeNode`
"""
function empty_constraint_node()
    ConstraintTreeNode()
end

"""
    Construct an empty `ConstraintTreeNode` from a `MAPF` instance
"""
function initialize_root_node(mapf::MAPF)
    ConstraintTreeNode(
        solution = LowLevelSolution([CBSPath() for a in 1:num_agents(mapf)]),
        constraints = Dict{Int,ConstraintDict}(
            i=>ConstraintDict(a=i) for i in 1:length(mapf.starts)
            ))
end

"""
    Initialize a new `ConstraintTreeNode` with the same `solution` and
    `constraints` as the parent node
"""
function initialize_child_node(parent_node::ConstraintTreeNode)
    ConstraintTreeNode(
        solution = copy(parent_node.solution),
        constraints = copy(parent_node.constraints),
        conflict_table = copy(parent_node.conflict_table)
    )
end

"""
    retrieve constraints corresponding to this node and this path
"""
function get_constraints(node::ConstraintTreeNode, path_id::Int)
    return get(node.constraints, path_id, ConstraintDict())
end

# """
#     Check if a set of constraints would be violated by adding an Edge from
#     the final vertex of `path` to `v`
# """
# function violates_constraints(constraints::ConstraintDict,v,path)
#     t = length(path) + 1
#     if get(constraints.node_constraints,NodeConstraint(constraints.a,v,t),false)
#         return true
#     else
#         v1 = get_final_node(path)
#         v2 = v
#         if get(constraints.edge_constraints,EdgeConstraint(constraints.a,v1,v2,t),false)
#             return true
#         end
#     end
#     return false
# end

"""
    adds a `NodeConstraint` to a ConstraintTreeNode
"""
function add_constraint!(node::ConstraintTreeNode,constraint::NodeConstraint,mapf::MAPF)
    if (constraint.v != mapf.goals[get_agent_id(constraint)])
        node.constraints[get_agent_id(constraint)].node_constraints[constraint] = true
        return true
    end
    return false
end

"""
    adds an `EdgeConstraint` to a ConstraintTreeNode
"""
function add_constraint!(node::ConstraintTreeNode,constraint::EdgeConstraint,mapf::MAPF)
    if (constraint.v1 != mapf.goals[get_agent_id(constraint)]) && (constraint.v1 != mapf.goals[get_agent_id(constraint)])
        node.constraints[get_agent_id(constraint)].edge_constraints[constraint] = true
        node.constraints[get_agent_id(constraint)].edge_constraints[flip(constraint)] = true
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
    generates a set of constraints from a StateConflict
"""
function generate_constraints_from_conflict(conflict::StateConflict)
    return [
        # Agent 1 may not occupy node at time t + 1
        NodeConstraint(
            agent1_id(conflict),
            node1_id(conflict),
            time_of(conflict)
        ),
        # Agent 2 may not occupy node at time t + 1
        NodeConstraint(
            agent2_id(conflict),
            node2_id(conflict),
            time_of(conflict)
        )
        ]
end

"""
    generates a set of constraints from an ActionConflict
"""
function generate_constraints_from_conflict(conflict::ActionConflict)
    return [
        # Agent 1 may not traverse Edge(node1,node2) at time t
        EdgeConstraint(
            agent1_id(conflict),
            node1_id(conflict),
            node2_id(conflict),
            time_of(conflict) # + 1
        ),
        # Agent 2 may not traverse Edge(node2,node1) at time t
        EdgeConstraint(
            agent2_id(conflict),
            node2_id(conflict),
            node1_id(conflict),
            time_of(conflict) # + 1
        )
        ]
end

"""
    Returns a low level solution for a MAPF with constraints
"""
function low_level_search!(mapf::MAPF,
    node::ConstraintTreeNode,
    idxs=collect(1:num_agents(mapf)),
    path_finder=A_star)
    # Only compute a path for the indices specified by idxs
    for i in idxs
        # TODO allow passing custom heuristic
        path = path_finder(
            CBSLowLevelEnv(mapf.graph,get_constraints(node,i)),
            mapf.starts[i],
            s -> states_match(s,mapf.goals[i])
            )
        node.solution[i] = path
    end
    node.cost = get_cost(node.solution)
    # TODO check if solution is valid
    return true
end

""" Abstract type for algorithms that solve MAPF instances """
abstract type AbstractMAPFSolver end

"""
    The Conflict-Based Search algorithm for multi-agent path finding - Sharon et
    al 2012

    https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
"""
struct CBS <: AbstractMAPFSolver end

"""
    Run Conflict-Based Search on an instance of MAPF
"""
function (solver::CBS)(mapf::MAPF,path_finder=A_star)
    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    root_node = initialize_root_node(mapf)
    low_level_search!(mapf,root_node)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        state_conflict, action_conflict = get_next_conflicts(node.conflict_table)
        if is_valid(state_conflict)
            @show constraints = generate_constraints_from_conflict(state_conflict)
        elseif is_valid(action_conflict)
            @show constraints = generate_constraints_from_conflict(action_conflict)
        else
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            return node.solution, node.cost
        end

        # generate new nodes from constraints
        for constraint in constraints
            new_node = initialize_child_node(node)
            if add_constraint!(new_node,constraint,mapf)
                low_level_search!(mapf,new_node,[get_agent_id(constraint)])
                detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)]) # update conflicts related to this agent
                if is_valid(new_node.solution, mapf)
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
    end
    print("No Solution Found. Returning default solution")
    return LowLevelSolution(), typemax(Int)
end

"""
    The Improved Conflict-Based Search Algorithm - Boyarski et al 2015

    https://www.ijcai.org/Proceedings/15/Papers/110.pdf
"""
struct ICBS <: AbstractMAPFSolver end

"""
    Run Improved Conflict-Based Search on an instance of MAPF
"""
function (solver::ICBS)(mapf::MAPF,path_finder=LightGraphs.a_star)    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    root_node = initialize_root_node(mapf)
    low_level_search!(mapf,root_node)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        state_conflicts, action_conflicts = get_conflicts(node.solution)
        # state_conflict, action_conflict = get_next_conflicts(node.solution)
        if is_valid(state_conflict)
            constraints = generate_constraints_from_conflict(state_conflict)
        elseif is_valid(action_conflict)
            constraints = generate_constraints_from_conflict(action_conflict)
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

include("low_level_search/heuristics.jl")

end # module
