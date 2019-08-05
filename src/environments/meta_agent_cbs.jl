export
    MetaAgentCBS

# CBS submodule
module MetaAgentCBS

using ..CRCBS
using Parameters, LightGraphs, DataStructures


@with_kw struct State{S}
    states::Vector{S} = Vector{S}()
end
@with_kw struct Action{A}
    actions::Vector{A} = Vector{A}()
end

# const State{S} = Vector{S}
# const Action{A} = Vector{A}

@with_kw struct LowLevelEnv{S,A,C,E<:AbstractLowLevelEnv{S,A,C}} <: AbstractLowLevelEnv{State{S},Action{A},C}
    envs::Vector{E}             = Vector{CBS.LowLevelEnv}()
    # graph::G                    = Graph()
    # # starts::Vector{S}           = Vector{S}()
    # goal::State             = Vector{G}()
    # # helper for meta-agent grouping
    # groups::Vector{Vector{Int}} = Vector{Vector{Int}}()
end
construct_meta_env(envs::Vector{E} where {E <: CBS.LowLevelEnv}) = LowLevelEnv{CBS.State,CBS.Action,DefaultPathCost,CBS.LowLevelEnv}(envs=envs)
construct_meta_env(envs::Vector{E} where {E <: MultiStageCBS.LowLevelEnv}) = LowLevelEnv{MultiStageCBS.State,MultiStageCBS.Action,DefaultPathCost,MultiStageCBS.LowLevelEnv}(envs=envs)
################################################################################
################################################################################
################################################################################

"""
    `split_path(path::Path{State{S},Action{A}}) where {S,A}`

    Helper for breaking a meta-agent path into a set of single-agent paths
"""
function split_path(path::Path{State{S},Action{A}}) where {S,A}
    N = length(get_s(get_path_node(path, 1)).states)
    paths = [Path{S,A}() for i in 1:N]
    for t in 1:length(path)
        path_node = get_path_node(path, t)
        for i in 1:N
            push!(
                paths[i],
                PathNode(
                    get_s(path_node).states[i],
                    get_a(path_node).actions[i],
                    get_sp(path_node).states[i]
                    )
                )
        end
    end
    return paths
end

function CRCBS.low_level_search!(
    solver::MetaAgentCBS_Solver,
    mapf::M where {M<:AbstractMAPF},
    node::ConstraintTreeNode,
    idxs::Vector{Int}=collect(1:num_agents(mapf));
    heuristic=heuristic,
    path_finder=A_star)
    # Only compute a path for the indices specified by idxs
    for i in idxs
        group = node.groups[i]
        env = construct_meta_env([build_env(mapf, node, j) for j in group])
        start = State([get_start(mapf,j) for j in group])

        path = path_finder(env, start, heuristic)
        paths = split_path(path)
        for (idx,j) in enumerate(group)
            set_solution_path!(node.solution, paths[idx], j)
        end
    end
    node.cost = get_cost(node.solution)
    # TODO check if solution is valid
    return true
end

function CRCBS.heuristic(env::LowLevelEnv,state::S) where {S <: State}
    c = 0
    for (e,s) in zip(env.envs, state.states)
        c += heuristic(e,s)
    end
    return c
end
function CRCBS.states_match(s1::State,s2::State)
    for (state1, state2) in zip(s1.states,s2.states)
        if !states_match(state1,state2)
            return false
        end
    end
    return true
end
function CRCBS.is_goal(env::LowLevelEnv,state::State)
    for (e,s) in zip(env.envs, state.states)
        if !is_goal(e,s)
            return false
        end
    end
    return true
end
CRCBS.wait(s::State) = Action([CRCBS.wait(s_) for s_ in s.states])
# get_possible_actions
struct ActionIter{A}
    action_lists::Vector{Vector{A}}
end
const ActionIterState = Vector{Int}
function Base.iterate(it::ActionIter)
    iter_state = ActionIterState(ones(Int, length(it.action_lists)))
    if length(iter_state) > 0
        iter_state[end] = 0
        return iterate(it, iter_state)
    else
        return nothing
    end
end
function Base.iterate(iter::ActionIter,iter_state::ActionIterState)
    iter_state = deepcopy(iter_state)
    i = length(iter_state)
    while i > 0
        if iter_state[i] < length(iter.action_lists[i])
            iter_state[i] += 1
            action = Action([iter.action_lists[j][idx] for (j,idx) in enumerate(iter_state)])
            return action, iter_state
        else
            iter_state[i] = 1
            i -= 1
        end
    end
    return nothing
end
function CRCBS.get_possible_actions(env::LowLevelEnv,s::State)
    return ActionIter([collect(get_possible_actions(env_,s_)) for (env_,s_) in zip(env.envs, s.states)])
    # return Base.Iterators.product([get_possible_actions(env_,s_) for (env_,s_) in zip(env.envs, s)]...)
end
Base.length(iter::ActionIter) = product([length(a) for a in iter.action_lists])
CRCBS.get_next_state(s::State,a::Action) = State(
    [get_next_state(s_,a_) for (s_,a_) in zip(s.states,a.actions)]
    )
CRCBS.get_next_state(env::LowLevelEnv,s::State,a::Action) = State(
    [get_next_state(e_,s_,a_) for (e_,s_,a_) in zip(env.envs,s.states,a.actions)]
    )
CRCBS.get_transition_cost(env::LowLevelEnv,s::State,a::Action,sp::State) = sum(
    [get_transition_cost(e_,s_,a_,sp_) for (e_,s_,a_,sp_) in zip(env.envs,s.states,a.actions,sp.states)]
)
function CRCBS.violates_constraints(env::LowLevelEnv, path, s::State, a::Action, sp::State)
    for (i, (e_,s_,a_,sp_)) in enumerate(zip(env.envs,s.states,a.actions,sp.states))
        if violates_constraints(e_,path,s_,a_,sp_)
            return true
        end
        # check for self conflict
        P1 = PathNode(s_,a_,sp_)
        for (j, (s2_,a2_,sp2_)) in enumerate(zip(s.states,a.actions,sp.states))
            if i != j
                P2 = PathNode(s2_,a2_,sp2_)
                if detect_state_conflict(P1,P2) || detect_action_conflict(P1,P2)
                    return true
                end
            end
        end
    end
    return false
end
CRCBS.check_termination_criteria(env::LowLevelEnv,cost,path,s) = false

end
