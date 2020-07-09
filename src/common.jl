###############################################################################
################################## Conflicts ###################################
################################################################################
export
    ConflictType,
        NULL_CONFLICT,
        STATE_CONFLICT,
        ACTION_CONFLICT

@enum ConflictType begin
    NULL_CONFLICT   = 0
    STATE_CONFLICT  = 1
    ACTION_CONFLICT = 2
end

export
    Conflict,
    conflict_type,
    agent1_id,
    agent2_id,
    node1,
    node2,
    DefaultConflict

@with_kw struct Conflict{P1 <: PathNode,P2 <: PathNode}
    conflict_type   ::ConflictType  = NULL_CONFLICT
    agent1_id       ::Int           = -1
    agent2_id       ::Int           = -1
    node1           ::P1            = P1()
    node2           ::P2            = P2()
    t               ::Int           = typemax(Int)
end
const SymmetricConflict{N} = Conflict{N,N}
conflict_type(conflict::Conflict)   = conflict.conflict_type
agent1_id(conflict::Conflict)       = conflict.agent1_id
agent2_id(conflict::Conflict)       = conflict.agent2_id
node1(conflict::Conflict)           = conflict.node1
node2(conflict::Conflict)           = conflict.node2
state1(conflict::Conflict)          = get_s(node1(conflict))
state2(conflict::Conflict)          = get_s(node2(conflict))
action1(conflict::Conflict)         = get_a(node1(conflict))
action2(conflict::Conflict)         = get_a(node2(conflict))
next_state1(conflict::Conflict)     = get_sp(node1(conflict))
next_state2(conflict::Conflict)     = get_sp(node2(conflict))
time_of(conflict::Conflict)         = conflict.t
Base.string(conflict::Conflict) = string(
    "conflict type: ",conflict_type(conflict),
    ": agent1=",agent1_id(conflict),
    ", agent2=", agent2_id(conflict),
    ", v1=(",get_s(node1(conflict)).vtx,",",get_sp(node1(conflict)).vtx,")",
    ", v2=(",get_s(node2(conflict)).vtx,",",get_sp(node2(conflict)).vtx,")",
    ", t=",get_s(node1(conflict)).t)

""" Default (invalid) conflict """
const DefaultConflict = Conflict{DefaultPathNode,DefaultPathNode}

Base.isless(c1::C,c2::C) where {C<:Conflict} = (
    [c1.t,c1.agent1_id,c1.agent2_id,c1.conflict_type] < [c2.t,c2.agent1_id,c2.agent2_id,c2.conflict_type]
)

""" Checks if a conflict is valid """
is_valid(conflict::C) where {C<:Conflict} = ((conflict_type(conflict) != NULL_CONFLICT)
                                && (agent1_id(conflict) != agent2_id(conflict))
                                && (agent1_id(conflict) != -1)
                                && (agent2_id(conflict) != -1))

export
    detect_conflicts!,
    reset_conflict_table!,
    detect_conflicts

""" add detected conflicts to conflict table """
function detect_conflicts!(conflict_table,n1::P1,n2::P2,i::Int,j::Int,t::Int) where {P1<:PathNode,P2<:PathNode}
    error("detect_conflicts!(conflict_table,n1,n2,i,j,t) not yet implemented \n
        for conflict_table::",typeof(conflict_table),". \n
        Aborting Conflict Checking.")
    return conflict_table
end

"""
    detect conflicts between paths
"""
function detect_conflicts!(conflict_table,path1::P1,path2::P2,i::Int,j::Int,t0::Int=1) where {P1<:Path,P2<:Path}
    # print("detect_conflicts!(conflict_table,path1::Path,path2::Path,i::Int,j::Int)\n")
    if length(path1) > length(path2)
        path2 = extend_path(path2,length(path1))
    elseif length(path1) < length(path2)
        path1 = extend_path(path1,length(path2))
    end
    @assert(length(path1) == length(path2))
    reset_conflict_table!(conflict_table,i,j)
    for t in t0:length(path1)
        path_node1 = path1[t]
        path_node2 = path2[t]
        detect_conflicts!(conflict_table,path_node1,path_node2,i,j,t)
    end
    return conflict_table
end

"""
    Populates a `ConflictTable` with all conflicts that occur in a given vector
    of paths. Conflict checking is performed in a pairwise fashion between
    all paths.

    args:
    - conflict_table        a `ConflictTable` to store the detected conflicts
    - paths:                a list of `Path`s, one for each individual agent
    - idxs                  (optional) a list of agent ids for which to check
                            collisions against all other agents
"""
function detect_conflicts!(conflict_table, paths::Vector{P}, idxs=collect(1:length(paths)),args...) where {P<:Path}
    # print("detect_conflicts!(conflict_table, paths::LowLevelSolution, idxs=collect(1:length(paths)))\n")
    for (i,path1) in enumerate(paths)
        for (j,path2) in enumerate(paths)
            if j <= i
                continue
            end
            if (i in idxs) || (j in idxs) # save time by working only on the upper triangle
                detect_conflicts!(conflict_table,path1,path2,i,j,args...)
            end
        end
    end
    return conflict_table
end

function detect_conflicts!(conflict_table, solution::L, args...) where {L <: LowLevelSolution}
    detect_conflicts!(conflict_table,get_paths(solution),args...)
end

export
    detect_state_conflict

"""
    Detect a `StateConflict` between two path nodes. Must be overridden for each
    specific path class
"""
function detect_state_conflict(n1::P1,n2::P2) where {P1<:PathNode,P2<:PathNode}
    error("detect_state_conflict(n1,n2) Not Implemented for \nn1::",
        typeof(n1),",\nn2::",typeof(n2))
    return false
end

""" Checks for a `StateConflict` between two `Path`s at time t """
function detect_state_conflict(path1::P1, path2::P2, t::Int) where {P1<:Path,P2<:Path}
    if detect_state_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

export
    detect_action_conflict

"""
    Detect an `ActionConflict` between two path nodes. Must be overridden for
    each specific path class
"""
function detect_action_conflict(n1::P1,n2::P2) where {P1<:PathNode,P2<:PathNode}
    error("detect_action_conflict(n1,n2) Not Implemented for \nn1::",
        typeof(n1),",\nn2::",typeof(n2))
    return false
end

""" Checks for an `ActionConflict` between two `Path`s at time t """
function detect_action_conflict(path1::P1, path2::P2,t::Int) where {P1<:Path,P2<:Path}
    if detect_action_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

export
    ConflictTable,
    get_conflicts,
    count_conflicts,
    add_conflict!,
    get_next_conflict

"""
    A lookup table to store all conflicts that have been detected
"""
@with_kw struct ConflictTable{C<:Conflict}
    state_conflicts::Dict{Tuple{Int,Int},Vector{C}} = Dict{Tuple{Int,Int},Vector{C}}()
    action_conflicts::Dict{Tuple{Int,Int},Vector{C}} = Dict{Tuple{Int,Int},Vector{C}}()
    # TODO sorted_conflict_list
end

""" helper for retrieving conflicts associated with agents i and j """
function get_conflicts(conflict_table::ConflictTable,i::Int,j::Int)
    if i <= j
        state_conflicts = get(conflict_table.state_conflicts, (i,j), Vector{Conflict}())
        action_conflicts = get(conflict_table.action_conflicts, (i,j), Vector{Conflict}())
    else
        @assert !haskey(conflict_table.state_conflicts, (i,j))
        @assert !haskey(conflict_table.action_conflicts, (i,j))
        return get_conflicts(conflict_table,j,i)
    end
    return state_conflicts, action_conflicts
end

"""
    `count_conflicts(conflict_table::ConflictTable,i::Int,j::Int)`

    helper for counting the number of conflicts between agent i and agent j
"""
function count_conflicts(conflict_table::ConflictTable,i::Int,j::Int)
    state_conflicts, action_conflicts = get_conflicts(conflict_table,i,j)
    return length(state_conflicts) + length(action_conflicts)
end
function count_conflicts(conflict_table::ConflictTable,idxs1::Vector{Int},idxs2::Vector{Int})
    N = 0
    n = max(maximum(idxs1),maximum(idxs2))
    mat = sparse(zeros(Int,n,n))
    for i in idxs1
        for j in idxs2
            if i != j
                n = count_conflicts(conflict_table,i,j)
                if i < j
                    mat[i,j] = n
                else
                    mat[j,i] = n
                end
            end
        end
    end
    return sum(mat)
end
function count_conflicts(conflict_table::ConflictTable)
    N = 0
    for (k,v) in conflict_table.state_conflicts
        N += length(v)
    end
    for (k,v) in conflict_table.action_conflicts
        N += length(v)
    end
    return N
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
    @assert i < j
    if conflict_type(conflict) == STATE_CONFLICT
        tab = conflict_table.state_conflicts
    elseif conflict_type(conflict) == ACTION_CONFLICT
        tab = conflict_table.action_conflicts
    end
    vec = get!(tab, (i,j), valtype(conflict_table.state_conflicts)())
    push!(vec, conflict)
    # tab[(i,j)] = vec
    # elseif conflict_type(conflict) == ACTION_CONFLICT
    #     vec = get!(conflict_table.action_conflicts, (i,j), valtype(conflict_table.action_conflicts)())
    #     push!(vec, conflict)
    #     conflict_table.action_conflicts[(i,j)] = vec
    # end
    # insert_to_sorted_array!(conflict_table.conflict_list, conflict)
end

"""
    `get_next_conflict(conflict_table::ConflictTable)`

    Returns the next conflict (temporally) that occurs in a conflict table
"""
function get_next_conflict(conflict_table::ConflictTable)
    conflict = DefaultConflict()
    for (k,v) in conflict_table.state_conflicts
        sort!(v)
        candidate = get(v,1,DefaultConflict())
        if time_of(candidate) < time_of(conflict)
            conflict = candidate
        end
    end
    for (k,v) in conflict_table.action_conflicts
        sort!(v)
        candidate = get(v,1,DefaultConflict())
        if time_of(candidate) < time_of(conflict)
            conflict = candidate
        end
    end
    return conflict
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

function reset_conflict_table!(conflict_table::ConflictTable,i::Int,j::Int)
    if i <= j
        conflict_table.state_conflicts[(i,j)]   =  Vector{Conflict}()
        conflict_table.action_conflicts[(i,j)]  =  Vector{Conflict}()
    else
        return reset_conflict_table!(conflict_table,j,i)
    end
    return conflict_table
end

# """
#     Returns a `ConflictTable` of all conflicts that occur in a given solution
#
#     args:
#     - conflict_table        a `ConflictTable` to store the detected conflicts
#     - paths:                a list of `Path`s, one for each individual agent
#     - idxs                  (optional) a list of agent ids for which to check
#                             collisions against all other agents
# """
function detect_conflicts(paths::Vector{P}, idxs=collect(1:length(paths)),args...) where {P <: Path}
    # print("detect_conflicts(paths::LowLevelSolution, idxs=collect(1:length(paths)))\n")
    conflict_table = ConflictTable{SymmetricConflict{node_type(P())}}()
    detect_conflicts!(conflict_table,paths,idxs,args...)
    conflict_table
end
function detect_conflicts(solution::L, idxs=collect(1:length(get_paths(solution))),args...) where {L<:LowLevelSolution}
    # print("detect_conflicts(paths::LowLevelSolution, idxs=collect(1:length(paths)))\n")
    conflict_table = ConflictTable{SymmetricConflict{node_type(P())}}()
    detect_conflicts!(conflict_table,solution,idxs,args...)
    conflict_table
end

""" add detected conflicts to conflict table """
function detect_conflicts!(conflicts::ConflictTable,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)
    # print("detect_conflicts!(conflicts::ConflictTable,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)\n")
    # state conflict
    if detect_state_conflict(n1,n2)
        add_conflict!(conflicts,Conflict(
            conflict_type = STATE_CONFLICT,
            agent1_id = i,
            agent2_id = j,
            node1 = n1,
            node2 = n2,
            t = t
        ))
    end
    if detect_action_conflict(n1,n2)
        add_conflict!(conflicts,Conflict(
            conflict_type = ACTION_CONFLICT,
            agent1_id = i,
            agent2_id = j,
            node1 = n1,
            node2 = n2,
            t = t
        ))
    end
end

################################################################################
################################# Constraints ##################################
################################################################################

export
    CBSConstraint,
    get_agent_id,
    get_time_of,
    StateConstraint,
    ActionConstraint


abstract type CBSConstraint end
# struct CBSConstraint{N}
#     a::Int # agent ID
#     v::N   # PathNode
#     t::Int # time ID
#     type::Symbol
# end
get_agent_id(c::CBSConstraint) = c.a
get_time_of(c::CBSConstraint) = c.t
get_path_node(c::CBSConstraint) = c.v
get_constraint_type(c::CBSConstraint) = c.type
for op in [:get_s,:get_a,:get_sp]
    @eval $op(c::CBSConstraint) = $op(get_path_node(c))
end
Base.isless(c1::CBSConstraint,c2::CBSConstraint) = ([c1.t] < [c2.t])
function Base.string(c::CBSConstraint)
    p = get_path_node(c)
    string(
        "$(get_constraint_type(c)):",
        " agent_id=$(get_agent_id(c)), t=$(get_time_of(c)), ",
        "$(string(get_s(p))) -- $(string(get_a(p))) -- X$(string(get_sp(p)))X"
        )
end
# StateConstraint(args...) = CBSConstraint(args...,:StateConstraint)
# ActionConstraint(args...) = CBSConstraint(args...,:ActionConstraint)


"""
    Encodes a constraint that agent `a` may not occupy vertex `v` at time `t`
"""
struct StateConstraint{N} <: CBSConstraint
    a::Int # agent ID
    v::N   # PathNode
    t::Int # time ID
end
function Base.string(c::StateConstraint)
    p = get_path_node(c)
    string(
    "StateConstraint: agent_id=$(get_agent_id(c)), t=$(get_time_of(c)), ",
    "$(string(get_s(p))) -- $(string(get_a(p))) -- X$(string(get_sp(p)))X")
end

"""
    Encodes a constraint that agent `a` may not traverse `Edge(v1,v2)` at time
    step `t`
"""
struct ActionConstraint{N} <: CBSConstraint
    a::Int  # agent ID
    v::N    # PathNode
    t::Int  # time ID
end
function Base.string(c::ActionConstraint)
    p = get_path_node(c)
    string(
    "StateConstraint: agent_id=$(get_agent_id(c)), t=$(get_time_of(c)), ",
    "$(string(get_s(p))) -- X$(string(get_a(p)))X -- $(string(get_sp(p)))")
end

################################################################################
############################## Constraint Tables ###############################
################################################################################
export
    serialize,
    deserialize,
    num_states,
    num_actions

"""
    serialize(env,state,t)

Encodes a state as an integer
"""
function serialize end

"""
    deserialize(env,idx,t)

Decodes an integer encoding of a state of type `state_type(env)`
"""
function deserialize end

"""
    num_states(env)

Returns the cardinality of the single agent state space for an environment.
    If the state and action spaces are finite and discrete, a discrete
    constraint table may be used for fast lookup.
"""
function num_states end

"""
    num_actions(env)

Returns the cardinality of the single agent state space for an environment.
    If the state and action spaces are finite and discrete, a discrete
    constraint table may be used for fast lookup.
"""
function num_actions end

export
    SpaceTrait,
    DiscreteSpace,
    ContinuousSpace,
    state_space_trait,
    action_space_trait

abstract type SpaceTrait end
struct DiscreteSpace <: SpaceTrait end
struct ContinuousSpace <: SpaceTrait end

"""
    state_space_trait(env)

Defaults to `DiscreteSpace`
"""
state_space_trait(env) = DiscreteSpace()

"""
    action_space_trait(env)

Defaults to `DiscreteSpace`
"""
action_space_trait(env) = DiscreteSpace()

state_space_trait(mapf::MAPF) = state_space_trait(mapf.env)
action_space_trait(mapf::MAPF) = action_space_trait(mapf.env)
num_states(mapf::MAPF) = num_states(mapf.env)
num_actions(mapf::MAPF) = num_actions(mapf.env)

export DiscreteConstraintTable

"""
    DiscreteStateTable

Stores constraints for a discrete state space
"""
struct DiscreteConstraintTable
    state_constraints::SparseMatrixCSC{Bool,Int}
    action_constraints::SparseMatrixCSC{Bool,Int}
    agent_id::Int
end
function DiscreteConstraintTable(env,agent_id=-1,tf=num_states(env))
    @assert isa(state_space_trait(env),DiscreteSpace)
    DiscreteConstraintTable(
        sparse(zeros(Bool,num_states(env),tf)),
        sparse(zeros(Bool,num_actions(env),tf)),
        agent_id
        )
end

export
    state_constraints,
    action_constraints,
    add_constraint!,
    has_constraint,
    search_constraints

get_agent_id(c::DiscreteConstraintTable)                = c.agent_id
state_constraints(c::DiscreteConstraintTable)           = c.state_constraints
action_constraints(c::DiscreteConstraintTable)          = c.action_constraints
function sorted_state_constraints(env,table::DiscreteConstraintTable)
    row_idxs, col_idxs, _ = findnz(table.state_constraints)
    s_constraints = Vector{StateConstraint{node_type(env)}}()
    for (idx,t) in zip(row_idxs,col_idxs)
        sp,_ = deserialize(env,state_type(env)(),idx,t)
        n = node_type(env)(sp=sp)
        push!(s_constraints,StateConstraint(get_agent_id(table),n,t))
    end
    return s_constraints
end
function sorted_action_constraints(env,table::DiscreteConstraintTable)
    row_idxs, col_idxs, _ = findnz(table.action_constraints)
    constraints = Vector{ActionConstraint{node_type(env)}}()
    for (idx,t) in zip(row_idxs,col_idxs)
        a,_ = deserialize(env,action_type(env)(),idx,t)
        n = node_type(env)(a=a)
        push!(constraints,ActionConstraint(get_agent_id(table),n,t))
    end
    return constraints
end
"""
    search_constraints(env,table,n::PathNode)

    Returns all `StateConstraint`s and `ActionConstraint`s that match `n`,
    regardless of time.
"""
function search_constraints(env,table::DiscreteConstraintTable,n::N) where {N<:PathNode}
    idx,_ = serialize(env,get_sp(n),-1)
    row_idxs, col_idxs, _ = findnz(table.state_constraints)
    s_constraints = Vector{StateConstraint{node_type(env)}}()
    for (i,(r,t)) in enumerate(zip(row_idxs,col_idxs))
        if r == idx
            push!(s_constraints,StateConstraint(get_agent_id(table),n,t))
        end
    end
    idx,_ = serialize(env,get_a(n),-1)
    row_idxs, col_idxs, _ = findnz(table.action_constraints)
    a_constraints = Vector{ActionConstraint{node_type(env)}}()
    for (i,(r,t)) in enumerate(zip(row_idxs,col_idxs))
        if r == idx
            push!(a_constraints,ActionConstraint(get_agent_id(table),n,t))
        end
    end
    return s_constraints, a_constraints
end

"""
    Adds a `StateConstraint` to a DiscreteConstraintTable
"""
function add_constraint!(env,table::DiscreteConstraintTable,c::StateConstraint)
    @assert get_agent_id(table) == get_agent_id(c)
    idx,t = serialize(env,get_sp(get_path_node(c)),get_time_of(c))
    table.state_constraints[idx,t] = true
    table
end

"""
    Adds an `ActionConstraint` to a DiscreteConstraintTable
"""
function add_constraint!(env,table::DiscreteConstraintTable,c::ActionConstraint)
    @assert get_agent_id(table) == get_agent_id(c)
    idx,t = serialize(env,get_a(get_path_node(c)),get_time_of(c))
    table.action_constraints[idx,t] = true
    table
end

"""
    has_constraint(env,table,c::StateConstraint)
"""
function has_constraint(env,table::DiscreteConstraintTable,c::StateConstraint)
    @assert get_agent_id(table) == get_agent_id(c)
    idx,t = serialize(env,get_sp(get_path_node(c)),get_time_of(c))
    return table.state_constraints[idx,t]
end

"""
    has_constraint(env,table,c::ActionConstraint)
"""
function has_constraint(env,table::DiscreteConstraintTable,c::ActionConstraint)
    @assert get_agent_id(table) == get_agent_id(c)
    idx,t = serialize(env,get_sp(get_path_node(c)),get_time_of(c))
    return table.action_constraints[idx,t]
end

export
    ConstraintTable,
    get_agent_id,
    state_constraints,
    action_constraints,
    sorted_state_constraints,
    sorted_action_constraints,
    add_constraint!

"""
    constraint dictionary for fast constraint lookup within a_star
"""
@with_kw struct ConstraintTable{N}
    # Sets
    state_constraints::Set{StateConstraint{N}} = Set{StateConstraint{N}}()
    action_constraints::Set{ActionConstraint{N}} = Set{ActionConstraint{N}}()
    # Vectors
    sorted_state_constraints::Vector{StateConstraint{N}} = Vector{StateConstraint{N}}()
    sorted_action_constraints::Vector{ActionConstraint{N}} = Vector{ActionConstraint{N}}()
    a::Int = -1 # agent_id
end
get_agent_id(c::ConstraintTable) = c.a
state_constraints(c::ConstraintTable) = c.state_constraints
action_constraints(c::ConstraintTable) = c.action_constraints
sorted_state_constraints(env,c::ConstraintTable) = c.sorted_state_constraints
sorted_action_constraints(env,c::ConstraintTable) = c.sorted_action_constraints
function search_constraints(env,table::ConstraintTable,n::N) where {N<:PathNode}
    idx,_ = serialize(env,get_sp(n),-1)
    s_constraints = Vector{StateConstraint{node_type(env)}}()
    for constraint in sorted_state_constraints(env,table)
        sp = get_sp(get_path_node(constraint))
        idxp, _ = serialize(env,sp,-1)
        if idxp == idx
            push!(s_constraints,constraint)
        end
    end
    idx,_ = serialize(env,get_a(n),-1)
    a_constraints = Vector{ActionConstraint{node_type(env)}}()
    for constraint in sorted_action_constraints(env,table)
        a = get_a(get_path_node(constraint))
        idxp, _ = serialize(env,a,-1)
        if idxp == idx
            push!(a_constraints,constraint)
        end
    end
    return s_constraints, a_constraints
end
# """ Helper function to merge two instances of `ConstraintTable` """
# function Base.merge(d1::ConstraintTable,d2::ConstraintTable)
#     @assert(get_agent_id(d1)==get_agent_id(d2))
#     ConstraintTable(
#         merge(d1.state_constraints,d2.state_constraints),
#         merge(d1.action_constraints,d2.action_constraints),
#         get_agent_id(d1)
#     )
# end
#
# """
#      Combines two `Dict`s of `ConstraintTable`s into a single `Dict` of
#      `ConstraintTable`s where the value associated with each key in the
#      resulting dictionary is the union of the values for the input dictionaries
#      at that key
# """
# function Base.merge(dict1::Dict{K,ConstraintTable},dict2::Dict{K,ConstraintTable}) where K
#     new_dict = typeof(dict1)()
#     for k in union(collect(keys(dict1)),collect(keys(dict2)))
#         new_dict[k] = merge(get(dict1,k,ConstraintTable()), get(dict1,k,ConstraintTable()))
#     end
#     return new_dict
# end

"""
    Adds a `StateConstraint` to a ConstraintTable
"""
function add_constraint!(env,constraint_dict::ConstraintTable,constraint::StateConstraint)
    @assert get_agent_id(constraint_dict) == get_agent_id(constraint)
    push!(constraint_dict.state_constraints, constraint)
    insert_to_sorted_array!(constraint_dict.sorted_state_constraints, constraint)
end

"""
    Adds an `ActionConstraint` to a ConstraintTable
"""
function add_constraint!(env,constraint_dict::ConstraintTable,constraint::ActionConstraint)
    @assert get_agent_id(constraint_dict) == get_agent_id(constraint)
    push!(constraint_dict.action_constraints, constraint)
    insert_to_sorted_array!(constraint_dict.sorted_action_constraints, constraint)
end

function has_constraint(env,table::ConstraintTable,c::StateConstraint)
    @assert get_agent_id(table) == get_agent_id(c)
    return (c in table.state_constraints)
end
function has_constraint(env,table::ConstraintTable,c::ActionConstraint)
    @assert get_agent_id(table) == get_agent_id(c)
    return (c in table.action_constraints)
end

export
    ConstraintTreeNode,
    solution_type,
    initialize_root_node,
    initialize_child_search_node,
    get_constraints,
    violates_constraints,
    generate_constraints_from_conflict

const DefaultSolution = LowLevelSolution{DefaultState,DefaultAction,Float64,TravelTime}
"""
    A node of a constraint tree. Each node has a set of constraints, a candidate
    solution (set of robot paths), and a cost
"""
# @with_kw mutable struct ConstraintTreeNode{S,T,C,D} #,E<:AbstractLowLevelEnv{S,A}} # CBS High Level Node
@with_kw mutable struct ConstraintTreeNode{S,C,D} #,E<:AbstractLowLevelEnv{S,A}} # CBS High Level Node
    # set of paths (one per agent) through graph
    solution        ::S = DefaultSolution()
    # maps agent_id to the set of constraints involving that agent
    constraints     ::C = Dict{Int,ConstraintTable{node_type(solution)}}()
    # meta-agent groups
    # groups          ::Vector{Vector{Int}}  = Vector{Vector{Int}}()
    # maintains a list of all conflicts
    conflict_table  ::D = ConflictTable{SymmetricConflict{node_type(solution)}}()
    # cost = sum([length(path) for path in solution])
    # cost            ::T                    = get_cost(solution)
    # index of parent node
    parent          ::Int = -1
    # indices of two child nodes
    children        ::Tuple{Int,Int} = (-1,-1)
    # unique id
    id              ::Int = -1
end
for op in [
    :cost_type,:state_type,:action_type,:path_type,:get_cost,:get_paths,
    :get_path_costs,:set_cost!,:set_solution_path!,:set_path_cost!,
    ]
    @eval $op(node::ConstraintTreeNode,args...) = $op(node.solution,args...)
end
solution_type(node::ConstraintTreeNode{S,C,D}) where {S,C,D} = S

"""
    `initialize_root_node`

    Construct an empty `ConstraintTreeNode` from a `AbstractMAPF` instance
"""
function initialize_root_node(mapf::AbstractMAPF,solution=get_initial_solution(mapf))
    ConstraintTreeNode(
        solution    = solution,
        constraints = Dict(
            i=>ConstraintTable{node_type(solution)}(a=i) for i in 1:num_agents(mapf)
            ),
        id = 1)
end
function initialize_root_node(solver,mapf::AbstractMAPF,solution=get_initial_solution(mapf))
    initialize_root_node(mapf,solution)
end

"""
    `initialize_child_search_node(parent_node::ConstraintTreeNode)`

    Initialize a new `ConstraintTreeNode` with the same `solution` and
    `constraints` as the parent node
"""
function initialize_child_search_node(parent_node::N, solution=copy(parent_node.solution)) where {N<:ConstraintTreeNode}
    N(
        solution        = solution,
        constraints     = deepcopy(parent_node.constraints),
        conflict_table  = deepcopy(parent_node.conflict_table),
        parent          = parent_node.id
    )
end
function initialize_child_search_node(mapf::AbstractMAPF,parent_node::N, solution=copy(parent_node.solution)) where {N<:ConstraintTreeNode}
    initialize_child_search_node(parent_node, solution)
end
function initialize_child_search_node(solver, mapf::AbstractMAPF,parent_node::N, solution=copy(parent_node.solution)) where {N<:ConstraintTreeNode}
    initialize_child_search_node(mapf, parent_node, solution)
end

"""
    retrieve constraints corresponding to this node and this path
"""
function get_constraints(node::N, path_id::Int) where {N<:ConstraintTreeNode}
    return get(node.constraints, path_id,
        ConstraintTable{node_type(node)}(a = path_id))
end

"""
    adds a `StateConstraint` to a ConstraintTreeNode
"""
function add_constraint!(env,node::N,constraint::CBSConstraint) where {N<:ConstraintTreeNode}
    add_constraint!(env,get_constraints(node, get_agent_id(constraint)), constraint)
    return true
end

function detect_conflicts!(node::ConstraintTreeNode, args...)
    detect_conflicts!(node.conflict_table,node.solution,args...)
end
for op in [:count_conflicts,:get_next_conflict,:reset_conflict_table!,:get_conflicts]
    @eval $op(node::ConstraintTreeNode,args...) = $op(node.conflict_table,args...)
end

for op in [:state_constraints, :action_constraints, :sorted_state_constraints, :sorted_action_constraints,]
    @eval $op(env,node::ConstraintTreeNode,idx) = $op(get_constraints(node,idx))
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
    generates a Vector of (State or Action) Constraints from a conflict
"""
function generate_constraints_from_conflict(conflict::Conflict)
    constraints = Vector{CBSConstraint}()
    if conflict_type(conflict) == STATE_CONFLICT
        push!(constraints, StateConstraint(
            agent1_id(conflict),
            node1(conflict),
            time_of(conflict)
        ))
        push!(constraints, StateConstraint(
            agent2_id(conflict),
            node2(conflict),
            time_of(conflict)
        ))
    elseif conflict_type(conflict) == ACTION_CONFLICT
        push!(constraints, ActionConstraint(
            agent1_id(conflict),
            node1(conflict),
            time_of(conflict)
        ))
        push!(constraints, ActionConstraint(
            agent2_id(conflict),
            node2(conflict),
            time_of(conflict)
        ))
    end
    return constraints
end
