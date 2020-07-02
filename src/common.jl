export
    # is_valid,

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
    DefaultConflict,

    reset_conflict_table!,
    detect_conflicts,
    detect_conflicts!,

    ConflictTable,
    get_conflicts,
    count_conflicts,
    get_next_conflict,
    add_conflict!,

    # StateConflict,
    detect_state_conflict,

    # ActionConflict,
    detect_action_conflict,

    CBSConstraint,
    get_agent_id,
    get_time_of,
    StateConstraint,
    ActionConstraint,

    ConstraintTable,
    add_constraint!,

    ConstraintTreeNode,
    initialize_root_node,
    initialize_child_search_node,
    get_constraints,
    violates_constraints,
    generate_constraints_from_conflict


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
# """
#     Reset conflict table (clear all conflicts between agent i and agent j)
# """
# function reset_conflict_table!(conflict_table,i::Int,j::Int)
#     error("reset_conflict_table!(conflict_table, i,j) not yet implemented \n
#         for conflict_table::",typeof(conflict_table), ". \n Aborting.")
#     return conflict_table
# end

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
function detect_conflicts!(conflict_table,path1::P1,path2::P2,i::Int,j::Int;t0::Int=1) where {P1<:Path,P2<:Path}
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
function detect_conflicts!(conflict_table, paths::Vector{P}, idxs=collect(1:length(paths));kwargs...) where {P<:Path}
    # print("detect_conflicts!(conflict_table, paths::LowLevelSolution, idxs=collect(1:length(paths)))\n")
    for (i,path1) in enumerate(paths)
        for (j,path2) in enumerate(paths)
            if j <= i
                continue
            end
            if !((i ∈ idxs) || (j ∈ idxs)) # save time by working only on the upper triangle
                continue
            end
            detect_conflicts!(conflict_table,path1,path2,i,j;kwargs...)
        end
    end
    return conflict_table
end

function detect_conflicts!(conflict_table, solution::L, idxs=collect(1:length(get_paths(solution)));kwargs...) where {L <: LowLevelSolution}
    detect_conflicts!(conflict_table,get_paths(solution),idxs;kwargs...)
end

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
    mat = sparse(zeros(Int,maximum(idxs1),maximum(idxs2)))
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

"""
    Returns a `ConflictTable` of all conflicts that occur in a given solution

    args:
    - conflict_table        a `ConflictTable` to store the detected conflicts
    - paths:                a list of `Path`s, one for each individual agent
    - idxs                  (optional) a list of agent ids for which to check
                            collisions against all other agents
"""
function detect_conflicts(paths::Vector{P}, idxs=collect(1:length(paths))) where {P <: Path}
    # print("detect_conflicts(paths::LowLevelSolution, idxs=collect(1:length(paths)))\n")
    conflict_table = ConflictTable{SymmetricConflict{node_type(P())}}()
    detect_conflicts!(conflict_table,paths,idxs)
    conflict_table
end
function detect_conflicts(solution::L, idxs=collect(1:length(get_paths(solution)))) where {L<:LowLevelSolution}
    # print("detect_conflicts(paths::LowLevelSolution, idxs=collect(1:length(paths)))\n")
    detect_conflicts(get_paths(solution),idxs)
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

abstract type CBSConstraint end
get_agent_id(c::CBSConstraint) = c.a
get_time_of(c::CBSConstraint) = c.t
get_path_node(c::CBSConstraint) = c.v

"""
    Encodes a constraint that agent `a` may not occupy vertex `v` at time `t`
"""
struct StateConstraint{N} <: CBSConstraint
    a::Int # agent ID
    v::N   # PathNode
    t::Int # time ID
end
Base.isless(c1::CBSConstraint,c2::CBSConstraint) = ([c1.t] < [c2.t])
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

export
    ConstraintTable,
    get_agent_id,
    state_constraints,
    action_constraints,
    sorted_state_constraints,
    sorted_action_constraints

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
sorted_state_constraints(c::ConstraintTable) = c.sorted_state_constraints
sorted_action_constraints(c::ConstraintTable) = c.sorted_action_constraints

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
function add_constraint!(constraint_dict::ConstraintTable,constraint::StateConstraint)
    @assert get_agent_id(constraint_dict) == get_agent_id(constraint)
    push!(constraint_dict.state_constraints, constraint)
    insert_to_sorted_array!(constraint_dict.sorted_state_constraints, constraint)
end

"""
    Adds an `ActionConstraint` to a ConstraintTable
"""
function add_constraint!(constraint_dict::ConstraintTable,constraint::ActionConstraint)
    if get_agent_id(constraint_dict) == get_agent_id(constraint)
        push!(constraint_dict.action_constraints, constraint)
        # insert constraint so that sorted order is maintained
        insert_to_sorted_array!(constraint_dict.sorted_action_constraints, constraint)
    else
        error("get_agent_id(constraint_dict) != get_agent_id(constraint)")
    end
end

const DefaultSolution = LowLevelSolution{DefaultState,DefaultAction,Float64,TravelTime}
"""
    A node of a constraint tree. Each node has a set of constraints, a candidate
    solution (set of robot paths), and a cost
"""
@with_kw mutable struct ConstraintTreeNode{S,T,C,D} #,E<:AbstractLowLevelEnv{S,A}} # CBS High Level Node
    # set of paths (one per agent) through graph
    solution        ::S                     = DefaultSolution()
    # maps agent_id to the set of constraints involving that agent
    constraints     ::C = Dict{Int,ConstraintTable{node_type(solution)}}()
    # meta-agent groups
    groups          ::Vector{Vector{Int}}  = Vector{Vector{Int}}()
    # maintains a list of all conflicts
    conflict_table  ::D = ConflictTable{SymmetricConflict{node_type(solution)}}()
    # cost = sum([length(path) for path in solution])
    cost            ::T                    = get_cost(solution)
    # index of parent node
    parent          ::Int                  = -1
    # indices of two child nodes
    children        ::Tuple{Int,Int}       = (-1,-1)
    # unique id
    id              ::Int                  = -1
end
action_type(node::N) where {N <: ConstraintTreeNode} = action_type(node.solution)
state_type(node::N) where {N <: ConstraintTreeNode} = state_type(node.solution)
cost_type(node::N) where {N <: ConstraintTreeNode} = cost_type(node.solution)
get_cost_type(node::N) where {N <: ConstraintTreeNode} = get_cost_type(node.solution)
get_cost(node::N) where {N <: ConstraintTreeNode} = get_cost(node.solution)
function set_cost!(node::N,cost) where {N <: ConstraintTreeNode}
    set_cost!(node.solution,cost)
    node.cost = cost
end

"""
    `initialize_root_node`

    Construct an empty `ConstraintTreeNode` from a `AbstractMAPF` instance
"""
function initialize_root_node(mapf::MAPF,solution=get_initial_solution(mapf))
    ConstraintTreeNode(
        solution    = solution,
        cost        = get_cost(solution),
        constraints = Dict(
            i=>ConstraintTable{node_type(solution)}(a=i) for i in 1:num_agents(mapf)
            ),
        id = 1)
end

"""
    `initialize_child_search_node(parent_node::ConstraintTreeNode)`

    Initialize a new `ConstraintTreeNode` with the same `solution` and
    `constraints` as the parent node
"""
function initialize_child_search_node(parent_node::N, solution=copy(parent_node.solution)) where {N<:ConstraintTreeNode}
    N(
        solution        = solution,
        cost            = deepcopy(get_cost(solution)),
        constraints     = deepcopy(parent_node.constraints),
        groups          = deepcopy(parent_node.groups),
        conflict_table  = deepcopy(parent_node.conflict_table),
        parent          = parent_node.id
    )
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
function add_constraint!(node::N,constraint::CBSConstraint) where {N<:ConstraintTreeNode}
    # Constraint is not legal if it prevents an agent from reaching its goal
    # if !(states_match(get_s(constraint.v), mapf.goals[get_agent_id(constraint)])
    #     || states_match(get_sp(constraint.v), mapf.goals[get_agent_id(constraint)]))
    add_constraint!(get_constraints(node, get_agent_id(constraint)), constraint)
    return true
end

function detect_conflicts!(node::ConstraintTreeNode, args...;kwargs...)
    detect_conflicts!(node.conflict_table,node.solution,args...;kwargs...)
end
for op in [:count_conflicts,:get_next_conflict,:reset_conflict_table!,:get_conflicts]
    @eval $op(node::ConstraintTreeNode,args...) = $op(node.conflict_table,args...)
end

for op in [:state_constraints, :action_constraints, :sorted_state_constraints, :sorted_action_constraints,]
    @eval $op(node::ConstraintTreeNode,idx) = $op(get_constraints(node,idx))
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
    Helper function to get the cost of a particular node
"""
function get_cost(node::N) where {N <: ConstraintTreeNode}
    return get_cost(node.solution)
end

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
