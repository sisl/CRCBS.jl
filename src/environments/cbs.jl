export
    GraphEnv

# CBS submodule
module GraphEnv

using ..CRCBS
using Parameters, LightGraphs, DataStructures

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
# State
@with_kw struct State
    vtx::Int        = -1 # vertex of graph
    t::Int          = -1
end
get_vtx(s::State) = s.vtx
get_t(s::State) = s.t
Base.string(s::State) = "(v=$(get_vtx(s)),t=$(get_t(s)))"
# Action
@with_kw struct Action
    e::Edge{Int}    = Edge(-1,-1)
    dt::Int         = 1
end
get_e(a::Action) = a.e
get_dt(a::Action) = a.dt
Base.string(a::Action) = "(e=$(get_e(a).src) → $(get_e(a).dst))"
# LowLevelEnv
@with_kw struct LowLevelEnv{C<:AbstractCostModel,H<:AbstractCostModel,G<:AbstractGraph,T} <: AbstractLowLevelEnv{State,Action,C}
    graph::G                    = Graph()
    constraints::T              = ConstraintTable{PathNode{State,Action}}()
    goal::State                 = State()
    agent_idx::Int              = -1
    heuristic::H                = NullHeuristic() #PerfectHeuristic(graph,Vector{Int}(),Vector{Int}())
    cost_model::C               = SumOfTravelTime()
end
CRCBS.get_possible_actions(env::LowLevelEnv,s::State) = map(v->Action(e=Edge(get_vtx(s),v)),outneighbors(env.graph,get_vtx(s)))
CRCBS.get_next_state(s::State,a::Action) = State(get_e(a).dst,get_t(s)+get_dt(a))
CRCBS.get_next_state(env::LowLevelEnv,s::State,a::Action) = get_next_state(s,a)
CRCBS.wait(s::State) = Action(e=Edge(get_vtx(s),get_vtx(s)))
CRCBS.wait(env::LowLevelEnv,s::State) = Action(e=Edge(get_vtx(s),get_vtx(s)))
CRCBS.get_cost_model(env::E) where {E<:LowLevelEnv} = env.cost_model
function CRCBS.get_transition_cost(env::E,c::TravelTime,s::State,a::Action,sp::State) where {E<:LowLevelEnv}
    return cost_type(c)(get_dt(a))
end
function CRCBS.get_transition_cost(env::E,c::C,s::State,a::Action,sp::State) where {E<:LowLevelEnv,C<:ConflictCostModel}
    return get_conflict_value(c, env.agent_idx, get_vtx(sp), get_t(sp))
end
CRCBS.get_heuristic_model(env::E) where {E<:LowLevelEnv} = env.heuristic
function CRCBS.get_transition_cost(env::E,c::TravelDistance,s::State,a::Action,sp::State) where {E<:LowLevelEnv}
    return (get_vtx(s) == get_vtx(sp)) ? 0.0 : 1.0
end
CRCBS.get_heuristic_cost(env::E,s::State) where {E<:LowLevelEnv} = CRCBS.get_heuristic_cost(env,get_heuristic_model(env),s)
function CRCBS.get_heuristic_cost(env::E,h::H,s::State) where {E<:LowLevelEnv,H<:Union{PerfectHeuristic,DefaultPerfectHeuristic}}
    get_heuristic_cost(h, get_vtx(env.goal), get_vtx(s))
end
function CRCBS.get_heuristic_cost(env::E,h::H,s::State) where {E<:LowLevelEnv, H<:ConflictTableHeuristic}
    get_heuristic_cost(h, env.agent_idx, get_vtx(s), get_t(s))
end
# states_match
CRCBS.states_match(s1::State,s2::State) = (get_vtx(s1) == get_vtx(s2))
CRCBS.states_match(env::LowLevelEnv,s1::State,s2::State) = (get_vtx(s1) == get_vtx(s2))
################################################################################
######################## Low-Level (Independent) Search ########################
################################################################################
# is_goal
function CRCBS.is_goal(env::LowLevelEnv,s::State)
    if states_match(s, env.goal)
        if get_t(s) >= get_t(env.goal)
            return true
        end
    end
    return false
end
function CRCBS.violates_constraints(env::LowLevelEnv, s::State, a::Action, sp::State)
    t = get_t(sp)
    if StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.state_constraints
        return true
    elseif ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.action_constraints
        return true
    end
    return false
end
function CRCBS.build_env(mapf::MAPF{E,S,G}, node::ConstraintTreeNode, idx::Int) where {S,G,E<:LowLevelEnv}
    t_goal = -1
    for constraint in sorted_state_constraints(get_constraints(node,idx))
        sp = get_sp(constraint.v)
        if states_match(mapf.goals[idx], sp)
            # @show get_t(s), get_time_of(constraint)
            t_goal = max(t_goal,get_t(sp)+1)
        end
    end
    typeof(mapf.env)(
        graph = mapf.env.graph,
        constraints = get_constraints(node,idx),
        goal = State(mapf.goals[idx],t=t_goal),
        agent_idx = idx,
        heuristic = get_heuristic_model(mapf.env),  # TODO update the heuristic model
        cost_model = get_cost_model(mapf.env)       # TODO update the cost model
        )
end

################################################################################
###################### Conflict-Based Search (High-Level) ######################
################################################################################
# detect_state_conflict
function CRCBS.detect_state_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if get_vtx(n1.sp) == get_vtx(n2.sp) && get_t(n1.sp) == get_t(n2.sp)
        return true
    end
    return false
end
CRCBS.detect_state_conflict(env::LowLevelEnv,n1::PathNode{State,Action},n2::PathNode{State,Action}) = detect_state_conflict(n1,n2)
# detect_action_conflict
function CRCBS.detect_action_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if (get_e(n1.a).src == get_e(n2.a).dst) && (get_e(n1.a).dst == get_e(n2.a).src) && (get_t(n1.sp) == get_t(n2.sp))
        return true
    end
    return false
end
CRCBS.detect_action_conflict(env::LowLevelEnv,n1::PathNode{State,Action},n2::PathNode{State,Action}) = detect_action_conflict(n1,n2)

################################################################################
############################### HELPER FUNCTIONS ###############################
################################################################################
""" Helper for displaying Paths """
function CRCBS.convert_to_vertex_lists(path::Path{State,Action})
    vtx_list = [get_vtx(n.sp) for n in path.path_nodes]
    if length(path) > 0
        vtx_list = [get_s(get_path_node(path,1)).vtx, vtx_list...]
    end
    vtx_list
end
function CRCBS.convert_to_vertex_lists(solution::L) where {T,C,L<:LowLevelSolution{State,Action,T,C}}
    return [convert_to_vertex_lists(path) for path in get_paths(solution)]
end

end
