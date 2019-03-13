export
    is_valid,

    ConflictType,
        NULL_CONFLICT,
        STATE_CONFLICT,
        ACTION_CONFLICT,
    Conflict,
    conflict_type,

    detect_conflicts,
    detect_conflicts!,

    StateConflict,
    detect_state_conflict,

    ActionConflict,
    detect_action_conflict,

    CBSConstraint,
    get_agent_id,
    StateConstraint,
    ActionConstraint,

    ConstraintDict


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

""" Checks if a conflict is valid """
is_valid(conflict::Conflict) = (conflict_type(conflict) != NULL_CONFLICT)

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
# function detect_conflicts(path1::Path,path2::Path,i::Int,j::Int)
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
    # return state_conflicts, action_conflicts
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
            # state_conflicts, action_conflicts = detect_conflicts(path1,path2,i,j)
            detect_conflicts!(conflict_table,path1,path2,i,j)
            # conflict_table.state_conflicts[(i,j)] = state_conflicts
            # conflict_table.action_conflicts[(i,j)] = action_conflicts
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
    adds a `StateConstraint` to a ConstraintDict
"""
function add_constraint!(constraint_dict::ConstraintDict,constraint::StateConstraint)
    if get_agent_id(constraint_dict) == get_agent_id(constraint)
        constraint_dict.state_constraints[constraint] = true
    end
end

"""
    adds a `ActionConstraint` to a ConstraintDict
"""
function add_constraint!(constraint_dict::ConstraintDict,constraint::ActionConstraint)
    if get_agent_id(constraint_dict) == get_agent_id(constraint)
        constraint_dict.action_constraints[constraint] = true
    end
end
