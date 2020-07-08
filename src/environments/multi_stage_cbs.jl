export
    MultiStageCBS

module MultiStageCBS

using ..CRCBS
using Parameters, LightGraphs, DataStructures

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
# state
@with_kw struct State
    vtx::Int    = -1 # vertex of graph
    stage::Int  = -1 # which stage of the sequence
    t::Int      = -1
end
Base.string(s::State) = "(v=$(s.vtx),stage=$(s.stage),t=$(s.t))"
CRCBS.is_valid(state::State) = state.vtx > 0
# action
@with_kw struct Action
    e::Edge{Int}    = Edge(-1,-1)
    Δt::Int         = 1
end
Base.string(a::Action) = "(e=$(a.e.src) → $(a.e.dst))"
@with_kw struct LowLevelEnv{C<:AbstractCostModel,H<:LowLevelSearchHeuristic,G<:AbstractGraph,T} <: AbstractLowLevelEnv{State,Action,C}
    graph::G                        = Graph()
    constraints::T                  = ConstraintTable{PathNode{State,Action}}()
    goal_sequence::Vector{State}    = Vector{State}()
    agent_idx::Int                  = -1
    cost_model::C                   = SumOfTravelTime()
    heuristic::H                    = NullHeuristic() # MultiStagePerfectHeuristic(graph,Vector{Vector{Int}}())
end
CRCBS.get_possible_actions(env::LowLevelEnv,s::State) = map(v->Action(e=Edge(s.vtx,v)),outneighbors(env.graph,s.vtx))
function CRCBS.get_next_state(s::State,a::Action)
    @assert(is_valid(s))
    @assert(s.vtx == a.e.src)
    State(a.e.dst, s.stage, s.t+a.Δt)
end
function CRCBS.get_next_state(env::E,s::State,a::Action) where {E<:LowLevelEnv}
    @assert(is_valid(s))
    @assert(s.stage <= length(env.goal_sequence))
    stage = s.stage
    if states_match(s, env.goal_sequence[s.stage])
        stage = min(stage+1, length(env.goal_sequence))
    end
    return State(a.e.dst, stage, s.t+a.Δt)
end
CRCBS.wait(s::State) = Action(e=Edge(s.vtx,s.vtx))
CRCBS.wait(env::E,s::State) where {E<:LowLevelEnv} = Action(e=Edge(s.vtx,s.vtx))
CRCBS.get_cost_model(env::E) where {E<:LowLevelEnv} = env.cost_model
function CRCBS.get_transition_cost(env::E,c::TravelTime,s::State,a::Action,sp::State) where {E<:LowLevelEnv}
    cost_type(c)(a.Δt)
end
CRCBS.get_heuristic_model(env::E) where {E<:LowLevelEnv} = env.heuristic
CRCBS.get_heuristic_cost(env::E,s::State) where {E<:LowLevelEnv} = CRCBS.get_heuristic_cost(env,get_heuristic_model(env),s)
function CRCBS.get_heuristic_cost(env::E,h::MultiStagePerfectHeuristic,s::State) where {E<:LowLevelEnv}
    get_heuristic_cost(h, env.agent_idx, s.stage, s.vtx)
end
################################################################################
######################## Low-Level (Independent) Search ########################
################################################################################
# build_env
function CRCBS.build_env(mapf::MAPF{E,S,G}, node::N, idx::Int)  where {S,G,E <: LowLevelEnv,N<:ConstraintTreeNode}
    goals = deepcopy(mapf.goals[idx])
    g = goals[end]
    t_goal = -1
    for constraint in sorted_state_constraints(get_constraints(node,idx))
        sp = get_sp(constraint.v)
        if states_match(g,sp)
            # @show s.t, get_time_of(constraint)
            t_goal = max(t_goal,sp.t+1)
        end
    end
    goals[end] = State(g,t=t_goal)
    E(
        graph = mapf.env.graph,
        constraints = get_constraints(node,idx),
        goal_sequence = goals,
        agent_idx = idx,
        cost_model = get_cost_model(mapf.env),
        heuristic = get_heuristic_model(mapf.env)
        )
end
# heuristic
# states_match
CRCBS.states_match(s1::State,s2::State) = (s1.vtx == s2.vtx)
function CRCBS.is_consistent(path::Path,start::State,goals::Vector{State})
    valid = true
    stage = 1
    if states_match(start,goals[stage])
        stage += 1
    end
    for k in 1:length(path)
        node = get_path_node(path,k)
        if states_match(get_sp(node),goals[stage])
            stage += 1
        end
        if stage > length(goals)
            return true
        end
    end
    return false
end
# is_goal
function CRCBS.is_goal(env::E,s::State) where {E<:LowLevelEnv}
    if s.stage == length(env.goal_sequence)
        if states_match(s, env.goal_sequence[s.stage])
            if s.t >= env.goal_sequence[s.stage].t
                return true
            end
        end
    end
    return false
end
# violates_constraints
function CRCBS.violates_constraints(env::E, s::State, a::Action, sp::State) where {E<:LowLevelEnv}
    t = sp.t
    if StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.state_constraints
        return true
    elseif ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.action_constraints
        return true
    end
    return false
end

################################################################################
###################### Conflict-Based Search (High-Level) ######################
################################################################################
# detect_state_conflict
function CRCBS.detect_state_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if n1.sp.vtx == n2.sp.vtx
        return true
    end
    return false
end
function CRCBS.detect_state_conflict(env::E,n1::PathNode{State,Action},n2::PathNode{State,Action}) where {E<:LowLevelEnv}
    detect_state_conflict(n1,n2)
end
# detect_action_conflict
function CRCBS.detect_action_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if (n1.a.e.src == n2.a.e.dst) && (n1.a.e.dst == n2.a.e.src)
        return true
    end
    return false
end
function CRCBS.detect_action_conflict(env::E,n1::PathNode{State,Action},n2::PathNode{State,Action}) where {E<:LowLevelEnv}
    detect_action_conflict(n1,n2)
end

################################################################################
############################### HELPER FUNCTIONS ###############################
################################################################################
""" Helper for displaying Paths """
function CRCBS.convert_to_vertex_lists(path::Path{State,Action,C}) where {C}
    vtx_list = [n.sp.vtx for n in path.path_nodes]
    if length(path) > 0
        vtx_list = [get_s(get_path_node(path,1)).vtx, vtx_list...]
    end
    vtx_list
end

end # end module MultiStageCBS
