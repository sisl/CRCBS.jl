export
    MultiStageCBS

module MultiStageCBS

using ..CRCBS
using Parameters, LightGraphs, DataStructures

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
# state
@with_kw struct State{S}
    s::S        = S()
    stage::Int  = 1 # which stage of the sequence
end
get_stage(s::State) = s.stage
get_state(s) = s
get_state(s::State) = s.s
Base.string(s::State) = string("s=",string(get_state(s)),", stage=",get_stage(s))
CRCBS.is_valid(state::State) = state.vtx > 0
# action
@with_kw struct LowLevelEnv{E}
    envs::Vector{E} = Vector{E}()
end
function first_env(env::LowLevelEnv)
    @assert(length(env.envs) >= 1)
    env.envs[get_stage(s)]
end
function current_env(env::LowLevelEnv,s::State)
    @assert(get_stage(s) <= length(env.envs))
    env.envs[get_stage(s)]
end
state_type(env::LowLevelEnv) = State{state_type(first_env(env))}
action_type(env::LowLevelEnv) = action_type(first_env(env))
cost_type(env::LowLevelEnv) = cost_type(first_env(env))
CRCBS.get_possible_actions(env::LowLevelEnv,s::State) = get_possible_actions(current_env(env,s),get_state(s))
function CRCBS.get_next_state(s::State,a)
    State(get_next_state(get_state(s),a),get_stage(s))
end
function CRCBS.get_next_state(env::LowLevelEnv,s::State,a)
    sp = get_next_state(current_env(env,s),get_state(s),a)
    stage = get_stage(s)
    if is_goal(current_env(env,s),sp)
        stage = min(stage+1, length(env.goal_sequence))
    end
    return State(sp,stage)
end
CRCBS.wait(s::State) = wait(get_state(s))
CRCBS.wait(env::LowLevelEnv,s::State) = wait(current_env(env,s),get_state(s))
CRCBS.get_cost_model(env::LowLevelEnv) = get_cost_model(first_env(env))
CRCBS.get_transition_cost(env::LowLevelEnv,s::State,a,sp::State) = get_transition_cost(current_env(env,s),get_state(s),a,get_state(sp))
CRCBS.get_heuristic_model(env::LowLevelEnv) = get_heuristic_model(first_env(env))
CRCBS.get_heuristic_cost(env::LowLevelEnv,s::State) = get_heuristic_cost(current_env(env,s),get_state(s))
################################################################################
######################## Low-Level (Independent) Search ########################
################################################################################
struct GoalSequence{S}
    goals::Vector{S}
end
# build_env
function CRCBS.build_env(mapf::MAPF{E,S,G}, node::ConstraintTreeNode, idx::Int) where {E,S,G<:GoalSequence}
    goals = mapf.goals[idx].goals
    envs = map(g->build_env(MAPF(mapf.env,get_starts(mapf),g)), goals)
    LowLevelEnv(envs)
end
# heuristic
# states_match
CRCBS.states_match(s1::State,s2::State) = states_match(get_state(s),get_state(s))
function CRCBS.is_consistent(path::Path,start::S,goals::GoalSequence{S}) where {S}
    valid = true
    stage = 1
    if states_match(start,goals.goals[stage])
        stage += 1
    end
    for k in 1:length(path)
        node = get_path_node(path,k)
        if states_match(get_sp(node),goals.goals[stage])
            stage += 1
        end
        if stage > length(goals)
            return true
        end
    end
    return false
end
# is_goal
function CRCBS.is_goal(env::LowLevelEnv,s::State)
    if s.stage == length(env.goal_sequence)
        return is_goal(current_env(env,s),get_state(s))
    end
    return false
end
# violates_constraints
CRCBS.violates_constraints(env::LowLevelEnv, s::State, a, sp::State) = violates_constraints(current_env(env,s),get_state(s),a,get_state(sp))

################################################################################
###################### Conflict-Based Search (High-Level) ######################
################################################################################
reduce_node(n::PathNode{State,A}) where {A} = PathNode(
    get_state(get_s(n)),get_a(n),get_state(get_sp(n)))
function CRCBS.detect_state_conflict(n1::PathNode{State,A},n2::PathNode{State,A}) where {A}
    detect_state_conflict(reduce_node(n1),reduce_node(n2))
end
function CRCBS.detect_action_conflict(n1::PathNode{State,A},n2::PathNode{State,A}) where {A}
    detect_action_conflict(reduce_node(n1),reduce_node(n2))
end

################################################################################
############################### HELPER FUNCTIONS ###############################
################################################################################
""" Helper for displaying Paths """
function CRCBS.convert_to_vertex_lists(path::Path{State{S},A,C}) where {S,A,C}
    p = Path{S,A,C}(map(n->reduce_node(n),path.path_nodes))
    convert_to_vertex_lists(p)
end

end # end module MultiStageCBS
