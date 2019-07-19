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
function construct_distance_array(G,goal)
    if goal.vtx != -1 && nv(G) > goal.vtx
        d = gdistances(G,goal.vtx)
    else
        d = Vector{Float64}()
    end
    return d
end
@with_kw struct LowLevelEnv{G <: AbstractGraph} <: AbstractLowLevelEnv{State,Action}
    graph::G                    = Graph()
    constraints::ConstraintTable = ConstraintTable()
    goal::State                 = State()
    agent_idx::Int              = -1
    # helpers
    dists::Vector{Float64}      = construct_distance_array(graph,goal)
end
function CRCBS.build_env(mapf::MAPF, node::ConstraintTreeNode, idx::Int)
    LowLevelEnv(
        graph = mapf.graph,
        constraints = get_constraints(node,idx),
        goal = mapf.goals[idx],
        agent_idx = idx
        )
end
CRCBS.heuristic(env::LowLevelEnv,s) = env.dists[s.vtx]

CRCBS.states_match(s1::State,s2::State) = (s1.vtx == s2.vtx)
function CRCBS.is_goal(env::LowLevelEnv,s::State)
    if states_match(s, env.goal)
        ###########################
        # Cannot terminate if there is a constraint on the goal state in the
        # future (e.g. the robot will need to move out of the way so another
        # robot can pass)
        for constraint in env.constraints.sorted_state_constraints
            if s.t < get_time_of(constraint)
                if states_match(s, get_sp(constraint.v))
                    # @show s.t, get_time_of(constraint)
                    return false
                end
            end
        end
        ###########################
        return true
    end
    return false
end
CRCBS.wait(s::State) = Action(e=Edge(s.vtx,s.vtx))

# get_possible_actions
struct ActionIter
    # env::LowLevelEnv
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
CRCBS.get_possible_actions(env::LowLevelEnv,s::State) = ActionIter(s.vtx,outneighbors(env.graph,s.vtx))
CRCBS.get_next_state(s::State,a::Action) = State(a.e.dst,s.t+a.Δt)
CRCBS.get_next_state(env::LowLevelEnv,s::State,a::Action) = get_next_state(s,a)
CRCBS.get_transition_cost(env::LowLevelEnv,s::State,a::Action,sp::State) = 1
function CRCBS.violates_constraints(env::LowLevelEnv, path::Path{State,Action}, s::State, a::Action, sp::State)
    t = length(path) + 1
    if StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.state_constraints
        # @show s,a,sp
        return true
    elseif ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.action_constraints
        # @show s,a,sp
        return true
    end
    return false

    # cs = StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t)
    # constraints = env.constraints.sorted_state_constraints
    # idx = max(1, find_index_in_sorted_array(constraints, cs)-1)
    # for i in idx:length(constraints)
    #     c = constraints[i]
    #     if c == cs
    #         @show s,a,sp
    #         return true
    #     end
    #     if c.t < cs.t
    #         break
    #     end
    # end
    # ca = StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t)
    # constraints = env.constraints.sorted_action_constraints
    # idx = max(1, find_index_in_sorted_array(constraints, ca)-1)
    # for i in idx:length(constraints)
    #     c = constraints[i]
    #     if c == ca
    #         @show s,a,sp
    #         return true
    #     end
    #     if c.t < ca.t
    #         break
    #     end
    # end
    # return false
end
CRCBS.check_termination_criteria(env::LowLevelEnv,cost,path,s) = false

""" Type alias for a path through the graph """
const CBSPath = Path{State,Action}

""" Helper for displaying Paths """
function convert_to_vertex_lists(path::Path)
    vtx_list = [n.sp.vtx for n in path.path_nodes]
    if length(path) > 0
        vtx_list = [get_s(get_path_node(path,1)).vtx, vtx_list...]
    end
    vtx_list
end
function convert_to_vertex_lists(solution::LowLevelSolution)
    return [convert_to_vertex_lists(path) for path in solution]
end

""" Returns an invalid StateConflict """
invalid_state_conflict() = Conflict{PathNode{State,Action},PathNode{State,Action}}(conflict_type=STATE_CONFLICT)

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
invalid_action_conflict() = Conflict{PathNode{State,Action},PathNode{State,Action}}(conflict_type=ACTION_CONFLICT)

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
function CRCBS.initialize_root_node(mapf::MAPF)
    ConstraintTreeNode(
        solution = LowLevelSolution{State,Action}([CBSPath() for a in 1:num_agents(mapf)]),
        constraints = Dict{Int,ConstraintTable}(
            i=>ConstraintTable(a=i) for i in 1:length(mapf.starts)
            ),
        id = 1)
end

"""
    `default_solution(solver::CBS_Solver, mapf::MAPF)`

    Defines what is returned by the solver in case of failure to find a feasible
    solution.
"""
CRCBS.default_solution(solver::CBS_Solver, mapf::MAPF) = LowLevelSolution{State,Action}(), typemax(Int)

end
