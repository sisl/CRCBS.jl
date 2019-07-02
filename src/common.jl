export
    is_valid,

    ConflictType,
        NULL_CONFLICT,
        STATE_CONFLICT,
        ACTION_CONFLICT,
    Conflict,
    conflict_type,
    agent1_id,
    agent2_id,
    node1,
    node2,

    detect_conflicts,
    detect_conflicts!,

    ConflictTable,
    get_conflicts,
    get_next_conflict,
    add_conflict!,

    StateConflict,
    detect_state_conflict,

    ActionConflict,
    detect_action_conflict,

    CBSConstraint,
    get_agent_id,
    StateConstraint,
    ActionConstraint,

    ConstraintDict,
    add_constraint!,

    ConstraintTreeNode,
    initialize_root_node,
    initialize_child_node,
    get_constraints,
    violates_constraints,
    get_cost,
    generate_constraints_from_conflict


"""
    Checks if an individual path satisfies start and end constraints
"""
function is_valid(path::Path{S,A},start::S,goal::S) where {S,A}
    return (states_match(get_initial_state(path), start)
        && states_match(get_final_state(path), goal))
end

"""
    checks if a solution is valid
"""
function is_valid(solution::LowLevelSolution{S,A},starts::Vector{S},goals::Vector{S}) where {S,A}
    for (i,path) in enumerate(solution)
        if !is_valid(path,starts[i],goals[i])
            return false
        end
    end
    return true
end

"""
    checks if a solution is valid
"""
function is_valid(solution::LowLevelSolution,mapf::MAPF)
    is_valid(solution,mapf.starts,mapf.goals)
end

################################################################################
################################## Conflicts ###################################
################################################################################

abstract type CBSConflict end

@enum ConflictType begin
    NULL_CONFLICT   = 0
    STATE_CONFLICT  = 1
    ACTION_CONFLICT = 2
end
@with_kw struct Conflict{P1 <: PathNode,P2 <: PathNode}
    conflict_type::ConflictType = NULL_CONFLICT
    agent1_id::Int = -1
    agent2_id::Int = -1
    node1::P1 = P1()
    node2::P2 = P2()
    t::Int = -1
end
conflict_type(conflict::Conflict) = conflict.conflict_type
agent1_id(conflict::Conflict) = conflict.agent1_id
agent2_id(conflict::Conflict) = conflict.agent2_id
node1(conflict::Conflict) = conflict.node1
node2(conflict::Conflict) = conflict.node2
state1(conflict::Conflict) = get_s(node1(conflict))
state2(conflict::Conflict) = get_s(node2(conflict))
action1(conflict::Conflict) = get_a(node1(conflict))
action2(conflict::Conflict) = get_a(node2(conflict))
next_state1(conflict::Conflict) = get_sp(node1(conflict))
next_state2(conflict::Conflict) = get_sp(node2(conflict))
time_of(conflict::Conflict) = conflict.t

Base.isless(c1::Conflict,c2::Conflict) = (
    [c1.t,c1.agent1_id,c1.agent2_id,c1.conflict_type] < [c2.t,c2.agent1_id,c2.agent2_id,c2.conflict_type]
)

""" Checks if a conflict is valid """
is_valid(conflict::Conflict) = ((conflict_type(conflict) != NULL_CONFLICT)
                                && (agent1_id(conflict) != agent2_id(conflict))
                                && (agent1_id(conflict) != -1)
                                && (agent2_id(conflict) != -1))

""" Default (invalid) conflict """
default_conflict() = Conflict{PathNode{DefaultState,DefaultState},PathNode{DefaultState,DefaultState}}()

""" add detected conflicts to conflict table """
function detect_conflicts!(conflict_table,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)
    error("detect_conflicts!(conflict_table,n1,n2,i,j,t) not yet implemented \n
        for conflict_table::",typeof(conflict_table),". \n
        Aborting Conflict Checking.")
    return conflict_table
end

"""
    detect conflicts between paths
"""
function detect_conflicts!(conflict_table,path1::Path,path2::Path,i::Int,j::Int)
    if length(path1) > length(path2)
        path2 = extend_path(path2,length(path1))
    elseif length(path1) < length(path2)
        path1 = extend_path(path1,length(path2))
    end
    @assert(length(path1) == length(path2))
    for t in 1:length(path1)
        path_node1 = path1[t]
        path_node2 = path2[t]
        detect_conflicts!(conflict_table,path_node1,path_node2,i,j,t)
    end
    return conflict_table
end

"""
    Populates a `ConflictTable` with all conflicts that occur in a given
    solution. Conflict checking is performed in a pairwise fashion between
    all paths.

    args:
    - conflict_table        a `ConflictTable` to store the detected conflicts
    - paths:                a list of `Path`s, one for each individual agent
    - idxs                  (optional) a list of agent ids for which to check
                            collisions against all other agents
"""
function detect_conflicts!(conflict_table, paths::LowLevelSolution, idxs=collect(1:length(paths)))
    for (i,path1) in enumerate(paths)
        for (j,path2) in enumerate(paths)
            if !((j ∈ idxs) && (j > i)) # save time by working only on the upper triangle
                continue
            end
            detect_conflicts!(conflict_table,path1,path2,i,j)
        end
    end
    return conflict_table
end

"""
    Encodes a conflict wherein two agents occupy a particular node at a
    particular time.

    Default constructor returns an INVALID instance
"""
@with_kw struct StateConflict{S1,S2}
    agent1_id::Int = -1
    agent2_id::Int = -1
    state1::S1 = S1()
    state2::S2 = S2()
    t::Int = -1
end
conflict_type(s::StateConflict) = StateConflict
agent1_id(conflict::StateConflict) = conflict.agent1_id
agent2_id(conflict::StateConflict) = conflict.agent2_id
state1(conflict::StateConflict) = conflict.state1
state2(conflict::StateConflict) = conflict.state2
time_of(conflict::StateConflict) = conflict.t

""" Checks if a node conflict is valid """
is_valid(conflict::StateConflict) = (agent1_id(conflict) != -1)

"""
    Detect a `StateConflict` between two path nodes. Must be overridden for each
    specific path class
"""
function detect_state_conflict(n1::PathNode,n2::PathNode)
    error("detect_state_conflict(n1,n2) Not Implemented for \nn1::",
        typeof(n1),",\nn2::",typeof(n2))
    return false
end

""" Checks for a `StateConflict` between two `Path`s at time t """
function detect_state_conflict(path1::Path, path2::Path, t::Int)
    if detect_state_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

"""
    Encodes a conflict between two agents at a particular edge at a particular
    time. This means that the agents are trying to swap places at time t.

    Default constructor returns an INVALID instance
"""
@with_kw struct ActionConflict{S1,S2} <: CBSConflict
    agent1_id::Int = -1
    agent2_id::Int = -1
    state1::S1 = S1()
    state2::S2 = S2()
    t::Int = -1
end
conflict_type(s::ActionConflict) = ActionConflict
agent1_id(conflict::ActionConflict) = conflict.agent1_id
agent2_id(conflict::ActionConflict) = conflict.agent2_id
state1(conflict::ActionConflict) = conflict.state1
state2(conflict::ActionConflict) = conflict.state2
time_of(conflict::ActionConflict) = conflict.t

""" checks if an edge node is invalid """
is_valid(conflict::ActionConflict) = (agent1_id(conflict) != -1)

"""
    Detect an `ActionConflict` between two path nodes. Must be overridden for
    each specific path class
"""
function detect_action_conflict(n1::PathNode,n2::PathNode)
    error("detect_action_conflict(n1,n2) Not Implemented for \nn1::",
        typeof(n1),",\nn2::",typeof(n2))
    return false
end

""" Checks for an `ActionConflict` between two `Path`s at time t """
function detect_action_conflict(path1::Path, path2::Path,t::Int)
    if detect_action_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

"""
    A lookup table to store all conflicts that have been detected
"""
@with_kw struct ConflictTable
    state_conflicts::Dict{Tuple{Int,Int},Vector{Conflict}} = Dict{Tuple{Int,Int},Vector{Conflict}}()
    action_conflicts::Dict{Tuple{Int,Int},Vector{Conflict}} = Dict{Tuple{Int,Int},Vector{Conflict}}()
end

""" helper for retrieving conflicts associated with agents i and j """
function get_conflicts(conflict_table::ConflictTable,i::Int,j::Int)
    if i < j
        state_conflicts = get(conflict_table.state_conflicts, (i,j), Vector{Conflict}())
        action_conflicts = get(conflict_table.action_conflicts, (i,j), Vector{Conflict}())
    else
        return get_conflicts(conflict_table,j,i)
    end
    return state_conflicts, action_conflicts
end

"""
    helper to insert conflicts into ConflictTable
"""
function add_conflict!(conflict_table::ConflictTable,conflict)
    if !is_valid(conflict)
        return
    end
    i = agent1_id(conflict)
    j = agent2_id(conflict)
    if conflict_type(conflict) == STATE_CONFLICT
        vec = get!(conflict_table.state_conflicts, (i,j), valtype(conflict_table.state_conflicts)())
        push!(vec, conflict)
    elseif conflict_type(conflict) == ACTION_CONFLICT
        vec = get!(conflict_table.action_conflicts, (i,j), valtype(conflict_table.action_conflicts)())
        push!(vec, conflict)
    end
end

"""
    returns the next conflict (temporally) that occurs in a conflict table
"""
function get_next_conflict(conflict_table::ConflictTable)
    conflicts = sort(union(conflict_table.state_conflicts,conflict_table.action_conflicts))
    return get(conflicts,1,nothing)
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

################################################################################
################################# Constraints ##################################
################################################################################

abstract type CBSConstraint end
get_agent_id(c::CBSConstraint) = c.a

"""
    Encodes a constraint that agent `a` may not occupy vertex `v` at time `t`
"""
struct StateConstraint{S} <: CBSConstraint
    a::Int # agent ID
    v::S   # PathNode?
    t::Int # time ID
end

"""
    Encodes a constraint that agent `a` may not traverse `Edge(v1,v2)` at time
    step `t`
"""
struct ActionConstraint{A} <: CBSConstraint
    a::Int  # agent ID
    v::A    # PathNode?
    t::Int  # time ID
end

"""
    constraint dictionary for fast constraint lookup within a_star
"""
@with_kw struct ConstraintDict
    state_constraints::Dict{StateConstraint,Bool} = Dict{StateConstraint,Bool}()
    action_constraints::Dict{ActionConstraint,Bool} = Dict{ActionConstraint,Bool}()
    a::Int = -1 # agent_id
end
get_agent_id(c::ConstraintDict) = c.a

""" Helper function to merge two instances of `ConstraintDict` """
function Base.merge(d1::ConstraintDict,d2::ConstraintDict)
    @assert(get_agent_id(d1)==get_agent_id(d2))
    ConstraintDict(
        merge(d1.state_constraints,d2.state_constraints),
        merge(d1.action_constraints,d2.action_constraints),
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
    Adds a `StateConstraint` to a ConstraintDict
"""
function add_constraint!(constraint_dict::ConstraintDict,constraint::StateConstraint)
    if get_agent_id(constraint_dict) == get_agent_id(constraint)
        constraint_dict.state_constraints[constraint] = true
    end
end

"""
    Adds an `ActionConstraint` to a ConstraintDict
"""
function add_constraint!(constraint_dict::ConstraintDict,constraint::ActionConstraint)
    if get_agent_id(constraint_dict) == get_agent_id(constraint)
        constraint_dict.action_constraints[constraint] = true
    end
end


"""
    A node of a constraint tree. Each node has a set of constraints, a candidate
    solution (set of robot paths), and a cost
"""
@with_kw mutable struct ConstraintTreeNode{S,A} #,E<:AbstractLowLevelEnv{S,A}} # CBS High Level Node
    # # Environment
    # env::E
    # maps agent_id to the set of constraints involving that agent
    constraints::Dict{Int,ConstraintDict} = Dict{Int,ConstraintDict}()
    # maintains a list of all conflicts
    conflict_table::ConflictTable = ConflictTable()
    # set of paths (one per agent) through graph
    solution::LowLevelSolution = LowLevelSolution{S,A}()
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
    retrieve constraints corresponding to this node and this path
"""
function get_constraints(node::ConstraintTreeNode, path_id::Int)
    return get(node.constraints, path_id, ConstraintDict())
end

"""
    adds a `StateConstraint` to a ConstraintTreeNode
"""
function add_constraint!(node::ConstraintTreeNode,constraint::StateConstraint,mapf::MAPF)
    if !(states_match(constraint.v, mapf.goals[get_agent_id(constraint)]))
        node.constraints[get_agent_id(constraint)].state_constraints[constraint] = true
        return true
    end
    return false
end

"""
    adds an `ActionConstraint` to a ConstraintTreeNode
"""
function add_constraint!(node::ConstraintTreeNode,constraint::ActionConstraint,mapf::MAPF)
    if (constraint.v1 != mapf.goals[get_agent_id(constraint)]) && (constraint.v1 != mapf.goals[get_agent_id(constraint)])
        node.constraints[get_agent_id(constraint)].action_constraints[constraint] = true
        node.constraints[get_agent_id(constraint)].action_constraints[flip(constraint)] = true
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
        StateConstraint(
            agent1_id(conflict),
            state1(conflict),
            time_of(conflict)
        ),
        # Agent 2 may not occupy node at time t + 1
        StateConstraint(
            agent2_id(conflict),
            state2(conflict),
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
        ActionConstraint(
            agent1_id(conflict),
            state1(conflict),
            state2(conflict),
            time_of(conflict) # + 1
        ),
        # Agent 2 may not traverse Edge(node2,node1) at time t
        ActionConstraint(
            agent2_id(conflict),
            state2(conflict),
            state1(conflict),
            time_of(conflict) # + 1
        )
        ]
end
