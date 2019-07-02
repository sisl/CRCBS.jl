export
    CBS

# CBS submodule
module CBS

using ..CRCBS
using Parameters, LightGraphs, DataStructures

@with_kw struct State
    vtx::Int        = -1 # vertex of graph
    t::Int          = -1
end
@with_kw struct Action
    e::Edge{Int}    = Edge(-1,-1)
    Δt::Int         = 1
end
@with_kw struct CBSLowLevelEnv{G <: AbstractGraph} <: AbstractLowLevelEnv{State,Action}
    graph::G                    = Graph()
    constraints::ConstraintDict = ConstraintDict()
    goal::State                 = State()
    agent_idx::Int              = -1
end

CRCBS.states_match(s1::State,s2::State) = (s1.vtx == s2.vtx)
CRCBS.is_goal(env::CBSLowLevelEnv,s::State) = states_match(s, env.goal)
CRCBS.wait(s::State) = Action(e=Edge(s.vtx,s.vtx))

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
    Action(e=Edge(it.s,it.neighbor_list[iter_state.idx])), iter_state
end
CRCBS.get_possible_actions(env::CBSLowLevelEnv,s::State) = ActionIter(env,s.vtx,outneighbors(env.graph,s.vtx))
CRCBS.get_next_state(s::State,a::Action) = State(a.e.dst,s.t+a.Δt)
CRCBS.get_next_state(env::CBSLowLevelEnv,s::State,a::Action) = get_next_state(s,a)
CRCBS.get_transition_cost(env::CBSLowLevelEnv,s::State,a::Action,sp::State) = 1
function CRCBS.violates_constraints(env::CBSLowLevelEnv, path::Path{State,Action}, s::State, a::Action, sp::State)
    t = length(path) + 1
    if get(env.constraints.state_constraints,StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t),false)
        # @show s,a,sp
        return true
    elseif get(env.constraints.action_constraints,ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t),false)
        # @show s,a,sp
        return true
    end
    return false
end
CRCBS.check_termination_criteria(env::CBSLowLevelEnv,cost,path,s) = false

""" Type alias for a path through the graph """
const CBSPath = Path{State,Action}

""" Returns an invalid StateConflict """
# invalid_state_conflict() = StateConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
invalid_state_conflict() = Conflict{PathNode{State,State},PathNode{State,State}}(conflict_type=STATE_CONFLICT)

"""
    Detect a `StateConflict` between two CBS path nodes.
"""
function CRCBS.detect_state_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if n1.sp.vtx == n2.sp.vtx
        return true
    end
    return false
end

""" Returns an invalid ActionConflict """
# invalid_action_conflict() = ActionConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
invalid_action_conflict() = ActionConflict{State,State}()

"""
    Detect an `ActionConflict` between two CBS path nodes.
"""
function CRCBS.detect_action_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if (n1.a.e.src == n2.a.e.dst) && (n1.a.e.dst == n2.a.e.src)
        return true
    end
    return false
end

"""
    Construct an empty `ConstraintTreeNode` from a `MAPF` instance
"""
function initialize_root_node(mapf::MAPF)
    ConstraintTreeNode(
        solution = LowLevelSolution{State,Action}([CBSPath() for a in 1:num_agents(mapf)]),
        constraints = Dict{Int,ConstraintDict}(
            i=>ConstraintDict(a=i) for i in 1:length(mapf.starts)
            ),
        id = 1)
end

"""
    Initialize a new `ConstraintTreeNode` with the same `solution` and
    `constraints` as the parent node
"""
function initialize_child_node(parent_node::ConstraintTreeNode)
    ConstraintTreeNode(
        solution = copy(parent_node.solution),
        constraints = copy(parent_node.constraints),
        conflict_table = copy(parent_node.conflict_table),
        parent = parent_node.id
    )
end

"""
    The Conflict-Based Search algorithm for multi-agent path finding - Sharon et
    al 2012

    https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
"""
struct CBSsolver <: AbstractMAPFSolver end

"""
    Returns a low level solution for a MAPF with constraints
"""
function low_level_search!(
    solver::CBSsolver,
    mapf::MAPF,
    node::ConstraintTreeNode,
    idxs=collect(1:num_agents(mapf)),
    path_finder=A_star)
    # Only compute a path for the indices specified by idxs
    for i in idxs
        # TODO allow passing custom heuristic
        path = path_finder(
            CBSLowLevelEnv(
                graph = mapf.graph,
                constraints = get_constraints(node,i),
                goal = mapf.goals[i],
                agent_idx = i
                ),
                mapf.starts[i],
                s -> states_match(s,mapf.goals[i])
            )
        node.solution[i] = path
    end
    node.cost = get_cost(node.solution)
    # TODO check if solution is valid
    return true
end

"""
    Run Conflict-Based Search on an instance of MAPF
"""
function (solver::CBSsolver)(mapf::MAPF,path_finder=A_star)
    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()
    # node_list = Vector{ConstraintTreeNode}()

    root_node = initialize_root_node(mapf)
    low_level_search!(solver,mapf,root_node)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if is_valid(root_node.solution,mapf)
        # @show root_node
        enqueue!(priority_queue, root_node => root_node.cost)
        # push!(node_list,root_node)
    end

    k = 0
    while length(priority_queue) > 0
        @show k += 1
        node = dequeue!(priority_queue)
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        @show conflict.node1.sp, conflict.agent1_id
        if is_valid(conflict)
            constraints = generate_constraints_from_conflict(conflict)
        else
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            return node.solution, node.cost
        end

        # generate new nodes from constraints
        for constraint in constraints
            new_node = initialize_child_node(node)
            # new_node.id = length(node_list) + 1
            if add_constraint!(new_node,constraint) && length(node.solution[get_agent_id(constraint)].path_nodes) > constraint.t
                low_level_search!(solver,mapf,new_node,[get_agent_id(constraint)])
                for p in new_node.solution
                    @show [n.sp.vtx for n in p.path_nodes]
                end
                detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)]) # update conflicts related to this agent
                if is_valid(new_node.solution, mapf)
                    # @show new_node.constraints
                    enqueue!(priority_queue, new_node => new_node.cost)
                    # push!(node_list, new_node)
                end
            end
        end
        # if k > 5
        #     break
        # end
    end
    # @show length(node_list)
    print("No Solution Found. Returning default solution")
    return LowLevelSolution{State,Action}(), typemax(Int)
end

end

# const CBS_State = Int
# @with_kw struct CBS_State
#     vtx::Int = -1
# end
# states_match(s1::CBS_State,s2::CBS_State) = (s1.vtx == s2.vtx)
# const CBS_Action = Edge{Int}
# CBS_Action() = Edge(-1,-1)
# wait(s::CBS_State) = CBS_Action(s.vtx,s.vtx)
#
# @with_kw struct CBSLowLevelEnv{G <: AbstractGraph,C} <: AbstractLowLevelEnv{CBS_State,CBS_Action}
#     graph       ::G         = Graph()
#     constraints ::C         = Vector()
#     goal        ::CBS_State = CBS_State()
#     agent_idx   ::Int       = -1
# end
#
# # get_possible_actions
# struct ActionIter
#     env::CBSLowLevelEnv
#     s::Int # source state
#     neighbor_list::Vector{Int} # length of target edge list
# end
# struct ActionIterState
#     idx::Int # idx of target node
# end
# function Base.iterate(it::ActionIter)
#     iter_state = ActionIterState(0)
#     return iterate(it,iter_state)
# end
# function Base.iterate(it::ActionIter, iter_state::ActionIterState)
#     iter_state = ActionIterState(iter_state.idx+1)
#     if iter_state.idx > length(it.neighbor_list)
#         return nothing
#     end
#     Edge(it.s,it.neighbor_list[iter_state.idx]), iter_state
# end
# CRCBS.get_possible_actions(env::CBSLowLevelEnv,s::CBS_State) = ActionIter(env,s.vtx,outneighbors(env.graph,s.vtx))
# CRCBS.get_next_state(env::CBSLowLevelEnv,s::CBS_State,a::CBS_Action) = CBS_State(a.dst)
# CRCBS.get_next_state(s::CBS_State,a::CBS_Action) = CBS_State(a.dst)
# CRCBS.get_transition_cost(env::CBSLowLevelEnv,s::CBS_State,a::CBS_Action,sp::CBS_State) = 1
# function CRCBS.violates_constraints(env::CBSLowLevelEnv, path::Path{CBS_State,CBS_Action}, s::CBS_State, a::CBS_Action, sp::CBS_State)
#     t = length(path) + 1
#     # if get(env.constraints.state_constraints,StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t),false)
#     if get(env.constraints.state_constraints,StateConstraint(get_agent_id(env.constraints),sp,t),false)
#         return true
#     else
#         v1 = s
#         v2 = sp
#         # if get(env.constraints.action_constraints,ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t),false)
#         if get(env.constraints.action_constraints,ActionConstraint(get_agent_id(env.constraints),v1,v2,t),false)
#             return true
#         end
#     end
#     return false
# end
# function CRCBS.check_termination_criteria(env::CBSLowLevelEnv,cost,path,s)
#     if length(path) > 40
#         print("TERMINATING SEARCH BECAUSE PATH LENGTH HAS REACHED ARBITRARY LIMIT OF 40")
#         return true
#     end
#     return false
# end
#
# """ Type alias for a path through the graph """
# const CBSPath = Path{CBS_State,CBS_Action}
#
# """ Returns an invalid StateConflict """
# # invalid_state_conflict() = StateConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
# invalid_state_conflict() = StateConflict{CBS_State,CBS_State}()
#
# """
#     Detect a `StateConflict` between two CBS path nodes.
# """
# function detect_state_conflict(n1::PathNode{CBS_State,CBS_Action},n2::PathNode{CBS_State,CBS_Action})
#     if n1.sp.vtx == n2.sp.vtx
#         return true
#     end
#     return false
# end
#
# """ Returns an invalid ActionConflict """
# # invalid_action_conflict() = ActionConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
# invalid_action_conflict() = ActionConflict{CBS_State,CBS_State}()
#
# """
#     Detect an `ActionConflict` between two CBS path nodes.
# """
# function detect_action_conflict(n1::PathNode{CBS_State,CBS_Action},n2::PathNode{CBS_State,CBS_Action})
#     if (n1.a.src == n2.a.dst) && (n1.a.dst == n2.a.src)
#         return true
#     end
#     return false
# end

# """
#     A lookup table to store all conflicts that have been detected
# """
# @with_kw struct ConflictTable
#     state_conflicts::Dict{Tuple{Int,Int},Vector{StateConflict}} = Dict{Tuple{Int,Int},Vector{StateConflict}}()
#     action_conflicts::Dict{Tuple{Int,Int},Vector{ActionConflict}} = Dict{Tuple{Int,Int},Vector{ActionConflict}}()
# end
#
# """ helper for retrieving conflicts associated with agents i and j """
# function get_conflicts(conflict_table::ConflictTable,i::Int,j::Int)
#     if i < j
#         state_conflicts = get(conflict_table.state_conflicts, (i,j), Vector{StateConflict}())
#         action_conflicts = get(conflict_table.action_conflicts, (i,j), Vector{ActionConflict}())
#     else
#         return get_conflicts(conflict_table,j,i)
#     end
#     return state_conflicts, action_conflicts
# end
#
# """
#     helper to insert conflicts into ConflictTable
# """
# function add_conflict!(conflict_table::ConflictTable,conflict)
#     i = agent1_id(conflict)
#     j = agent2_id(conflict)
#     if conflict_type(conflict) == StateConflict
#         push!(conflict_table.state_conflicts[(i,j)], conflict)
#     elseif conflict_type(conflict_type) == ActionConflict
#         push!(conflict_table.action_conflicts[(i,j)], conflict)
#     end
# end
#
# """ add detected conflicts to conflict table """
# function detect_conflicts!(conflicts::ConflictTable,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)
#     # state conflict
#     if detect_state_conflict(n1,n2)
#         add_conflict!(conflicts,StateConflict(
#             agent1_id = i,
#             agent2_id = j,
#             node1 = n1,
#             node2 = n2,
#             t = t
#         ))
#     end
#     if detect_action_conflict(n1,n2)
#         add_conflict!(conflicts,ActionConflict(
#             agent1_id = i,
#             agent2_id = j,
#             node1 = n1,
#             node2 = n2,
#             t = t
#         ))
#     end
# end
#
# """
#     returns the next conflict (temporally) that occurs in a conflict table
# """
# function get_next_conflict(conflict_table::ConflictTable)
#     conflicts = sort(union(conflict_table.state_conflicts,conflict_table.action_conflicts))
#     return get(conflicts,1,Conflict())
#     # state_conflicts = Vector{StateConflict}()
#     # for (k,v) in conflict_table.state_conflicts
#     #     if length(v) > 0
#     #         push!(state_conflicts, v[1])
#     #     end
#     # end
#     # sort!(state_conflicts, by=c->time_of(c))
#     # action_conflicts = Vector{ActionConflict}()
#     # for (k,v) in conflict_table.action_conflicts
#     #     if length(v) > 0
#     #         push!(action_conflicts, v[1])
#     #     end
#     # end
#     # sort!(action_conflicts, by=c->time_of(c))
#     #
#     # state_conflict = get(state_conflicts, 1, invalid_state_conflict())
#     # action_conflict = get(action_conflicts, 1, invalid_action_conflict())
#     # if is_valid(state_conflict) && is_valid(action_conflict)
#     #     if time_of(state_conflict) <= time_of(action_conflict)
#     #         return state_conflict, invalid_action_conflict()
#     #     else
#     #         return invalid_state_conflict(), action_conflict
#     #     end
#     # end
#     # return state_conflict, action_conflict
# end
#
# function Base.copy(c::ConflictTable)
#    c_new = ConflictTable()
#    for (k,v) in c.state_conflicts
#        c_new.state_conflicts[k] = copy(v)
#    end
#    for (k,v) in c.action_conflicts
#        c_new.action_conflicts[k] = copy(v)
#    end
#    return c_new
# end
#
# """
#     Returns a `ConflictTable` of all conflicts that occur in a given solution
#
#     args:
#     - conflict_table        a `ConflictTable` to store the detected conflicts
#     - paths:                a list of `Path`s, one for each individual agent
#     - idxs                  (optional) a list of agent ids for which to check
#                             collisions against all other agents
# """
# function detect_conflicts(paths::LowLevelSolution, idxs=collect(1:length(paths)))
#     conflict_table = ConflictTable()
#     detect_conflicts!(conflict_table,paths,idxs)
# end

# """
#     returns the next conflict (temporally) that occurs in a conflict table
# """
# function get_next_conflicts(conflict_table::ConflictTable)
#     state_conflicts = Vector{StateConflict}()
#     for (k,v) in conflict_table.state_conflicts
#         if length(v) > 0
#             push!(state_conflicts, v[1])
#         end
#     end
#     sort!(state_conflicts, by=c->time_of(c))
#     action_conflicts = Vector{ActionConflict}()
#     for (k,v) in conflict_table.action_conflicts
#         if length(v) > 0
#             push!(action_conflicts, v[1])
#         end
#     end
#     sort!(action_conflicts, by=c->time_of(c))
#
#     state_conflict = get(state_conflicts, 1, invalid_state_conflict())
#     action_conflict = get(action_conflicts, 1, invalid_action_conflict())
#     if is_valid(state_conflict) && is_valid(action_conflict)
#         if time_of(state_conflict) <= time_of(action_conflict)
#             return state_conflict, invalid_action_conflict()
#         else
#             return invalid_state_conflict(), action_conflict
#         end
#     end
#     return state_conflict, action_conflict
# end
#
# """
#     Returns a `StateConflict` and an `ActionConflict` next conflicts.
#     The function returns after finding the FIRST conflice (StateConflict or
#         ActionConflict), which means that at least one of the returned conflicts
#         will always be invalid. The rational for returning both anyway is to
#         preserve stability of the function's return type.
#     If state_conflict and action_conflict are both invalid, the search has reached
#         the end of the paths.
#
#     args:
#     - t_:       time index at which to begin the search
#     - i_:       index of path 1 at which to begin the search
#     - j_:       index of path 2 at which to begin the search
#     - tmax:     maximum lookahead time (defaults to the length of the longest
#         path)
#     Search begins at time `t_`, `paths[i_]`, `paths[j_]`, then returns after
#         finding the first conflict.
# """
# function get_next_conflicts(paths::LowLevelSolution,
#         i_::Int=1,
#         j_::Int=2,
#         t_::Int=1,
#         tmax::Int=maximum([length(p) for p in paths])
#         )
#     state_conflict = invalid_state_conflict()
#     action_conflict = invalid_action_conflict()
#
#     # extend paths so that they all match in length
#     extended_paths = [extend_path(path,tmax) for path in paths]
#
#     # begin search from time t, paths[i_], paths[j_]
#     t = t_; i = i_; j_ = max(j_,i+1)
#
#     path1 = get(extended_paths,i,CBSPath()) # in case i is beyond the length of paths
#     n1 = get_path_node(path1,t)
#     for j in j_:length(extended_paths)
#         path2 = extended_paths[j]
#         n2 = get_path_node(path2,t)
#         if detect_state_conflict(n1,n2)
#             state_conflict = StateConflict(i,j,get_sp(n1),get_sp(n2),t)
#             return state_conflict, action_conflict
#         elseif detect_action_conflict(n1,n2)
#             action_conflict = ActionConflict(i,j,get_s(n1),get_s(n2),t)
#             return state_conflict, action_conflict
#         end
#     end
#     # Continue search from next time step
#     for t in t_+1:tmax
#         for (i,path1) in enumerate(extended_paths)
#             n1 = get_path_node(path1,t)
#             for j in i+1:length(extended_paths)
#                 path2 = extended_paths[j]
#                 n2 = get_path_node(path2,t)
#                 if detect_state_conflict(n1,n2)
#                     state_conflict = StateConflict(i,j,get_sp(n1),get_sp(n2),t)
#                     return state_conflict, action_conflict
#                 elseif detect_action_conflict(n1,n2)
#                     action_conflict = ActionConflict(i,j,get_s(n1),get_s(n2),t)
#                     return state_conflict, action_conflict
#                 end
#             end
#         end
#     end
#     return state_conflict, action_conflict
# end
#
# """
#     Returns a list of all conflicts that occur in a given solution
#
#     args:
#     - paths:        a list of graph edges to be traversed by the agents
# """
# function get_all_conflicts(paths::LowLevelSolution)
#     state_conflicts = Vector{StateConflict}()
#     action_conflicts = Vector{ActionConflict}()
#     for (i,path1) in paths
#         for (j,path2) in paths
#             if j <= i
#                 continue
#             end
#             s_conflicts, a_conflicts = detect_conflicts(path1,path2,i,j)
#             union!(state_conflicts, s_conflicts)
#             union!(action_conflicts, a_conflicts)
#         end
#     end
#     sort!(state_conflicts,by=c->time_of(c))
#     sort!(action_conflicts,by=c->time_of(c))
#     return state_conflicts, action_conflicts
# end
#
# """
#     Returns a list of all conflicts that occur in a given solution
#
#     args:
#     - paths:        a list of graph edges to be traversed by the agents
# """
# function get_conflicts(paths::LowLevelSolution)
#     # TODO Make this way faster
#     state_conflicts = Vector{StateConflict}()
#     action_conflicts = Vector{ActionConflict}()
#     t_max = maximum([length(path) for path in paths])
#     nc, ec = get_next_conflicts(paths)
#     while true
#         if is_valid(nc)
#             push!(state_conflicts, nc)
#             conflict = nc
#         elseif is_valid(ec)
#             push!(action_conflicts, ec)
#             conflict = ec
#         else
#             break
#         end
#         nc, ec = get_next_conflicts(
#             paths,
#             agent1_id(conflict),
#             agent2_id(conflict)+1,
#             time_of(conflict),
#             t_max
#             )
#     end
#     return state_conflicts, action_conflicts
# end

# """
#     Helper function for reversing an `ActionConstraint`
# """
# flip(c::ActionConstraint) = ActionConstraint(get_agent_id(c),c.v2,c.v1,c.t)

#
# """
#     A node of a constraint tree. Each node has a set of constraints, a candidate
#     solution (set of robot paths), and a cost
# """
# @with_kw mutable struct ConstraintTreeNode{S,A,E<:AbstractLowLevelEnv{S,A}} # CBS High Level Node
#     # Environment
#     env::E
#     # maps agent_id to the set of constraints involving that agent
#     constraints::Dict{Int,ConstraintDict} = Dict{Int,ConstraintDict}()
#     # maintains a list of all conflicts
#     conflict_table::ConflictTable = ConflictTable()
#     # set of paths (one per agent) through graph
#     solution::LowLevelSolution = LowLevelSolution{S,A}()
#     # cost = sum([length(path) for path in solution])
#     cost::Int = -1
#     # index of parent node
#     parent::Int = -1
#     # indices of two child nodes
#     children::Tuple{Int,Int} = (-1,-1)
#     # unique id
#     id::Int = -1
# end

# """
#     Returns an empty `ConstraintTreeNode`
# """
# function empty_constraint_node()
#     ConstraintTreeNode()
# end

# """
#     Construct an empty `ConstraintTreeNode` from a `MAPF` instance
# """
# function initialize_root_node(mapf::MAPF)
#     ConstraintTreeNode(
#         solution = LowLevelSolution{CBS_State,CBS_Action}([CBSPath() for a in 1:num_agents(mapf)]),
#         constraints = Dict{Int,ConstraintDict}(
#             i=>ConstraintDict(a=i) for i in 1:length(mapf.starts)
#             ),
#         id = 1)
# end
#
# """
#     Initialize a new `ConstraintTreeNode` with the same `solution` and
#     `constraints` as the parent node
# """
# function initialize_child_node(parent_node::ConstraintTreeNode)
#     ConstraintTreeNode(
#         solution = copy(parent_node.solution),
#         constraints = copy(parent_node.constraints),
#         conflict_table = copy(parent_node.conflict_table),
#         parent = parent_node.id
#     )
# end


# """ Abstract type for algorithms that solve MAPF instances """
# abstract type AbstractMAPFSolver end

# """
#     The Conflict-Based Search algorithm for multi-agent path finding - Sharon et
#     al 2012
#
#     https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
# """
# struct CBS <: AbstractMAPFSolver end

# """
#     Returns a low level solution for a MAPF with constraints
# """
# function low_level_search!(
#     solver::CBS,
#     mapf::MAPF,
#     node::ConstraintTreeNode,
#     idxs=collect(1:num_agents(mapf)),
#     path_finder=A_star)
#     # Only compute a path for the indices specified by idxs
#     for i in idxs
#         # TODO allow passing custom heuristic
#         path = path_finder(
#             CBSLowLevelEnv(
#                 graph = mapf.graph,
#                 constraints = get_constraints(node,i),
#                 goal = mapf.goals[i],
#                 agent_idx = i
#                 ),
#                 mapf.starts[i],
#                 s -> states_match(s,mapf.goals[i])
#             )
#         node.solution[i] = path
#     end
#     node.cost = get_cost(node.solution)
#     # TODO check if solution is valid
#     return true
# end

# """
#     Run Conflict-Based Search on an instance of MAPF
# """
# function (solver::CBS)(mapf::MAPF,path_finder=A_star)
#     # priority queue that stores nodes in order of their cost
#     # node_list = Vector{ConstraintTreeNode}()
#     priority_queue = PriorityQueue{ConstraintTreeNode,Int}()
#
#     root_node = initialize_root_node(mapf)
#     low_level_search!(solver,mapf,root_node)
#     detect_conflicts!(root_node.conflict_table,root_node.solution)
#     if is_valid(root_node.solution,mapf)
#         @show root_node
#         enqueue!(priority_queue, root_node => root_node.cost)
#         # push!(node_list,root_node)
#     end
#
#     k = 0
#     while length(priority_queue) > 0
#         k += 1
#         node = dequeue!(priority_queue)
#         # check for conflicts
#         state_conflict, action_conflict = get_next_conflicts(node.conflict_table)
#         if is_valid(state_conflict)
#             @show constraints = generate_constraints_from_conflict(state_conflict)
#         elseif is_valid(action_conflict)
#             @show constraints = generate_constraints_from_conflict(action_conflict)
#         else
#             print("Optimal Solution Found! Cost = ",node.cost,"\n")
#             return node.solution, node.cost
#         end
#
#         # generate new nodes from constraints
#         for constraint in constraints
#             new_node = initialize_child_node(node)
#             # new_node.id = length(node_list) + 1
#             if add_constraint!(new_node,constraint,mapf)
#                 low_level_search!(solver,mapf,new_node,[get_agent_id(constraint)])
#                 detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)]) # update conflicts related to this agent
#                 if is_valid(new_node.solution, mapf)
#                     enqueue!(priority_queue, new_node => new_node.cost)
#                     # push!(node_list, new_node)
#                 end
#             end
#         end
#         if k > 5
#             break
#         end
#     end
#     print("No Solution Found. Returning default solution")
#     return LowLevelSolution(), typemax(Int)
# end

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
