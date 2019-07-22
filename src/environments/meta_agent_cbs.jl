export
    MetaAgentCBS

# CBS submodule
module MetaAgentCBS

using ..CRCBS
using Parameters, LightGraphs, DataStructures

const State{S} = Vector{S}
const Action{A} = Vector{A}

@with_kw struct LowLevelEnv{S,A,E <: AbstractLowLevelEnv{S,A}} <: AbstractLowLevelEnv{State{S},Action{A}}
    envs::Vector{E} = Vector{CBS.LowLevelEnv}()
end
function CRCBS.build_env(mapf::M where M <: MetaMAPF, node::ConstraintTreeNode, idxs::Vector{Int})
    LowLevelEnv{CBS.State,CBS.Action,CBS.LowLevelEnv}([build_env(mapf.mapf,node,idx) for idx in idxs])
end
function CRCBS.heuristic(env::LowLevelEnv,state::S) where {S <: State}
    c = 0
    for (e,s) in zip(env.envs, state)
        c += heuristic(e,s)
    end
    return c
end
function CRCBS.states_match(s1::State,s2::State)
    for (state1, state2) in zip(s1,s2)
        if !states_match(state1,state2)
            return false
        end
    end
    return true
end
function CRCBS.is_goal(env::LowLevelEnv,state::State)
    for (e,s) in zip(env.envs, state)
        if !is_goal(e,s)
            return false
        end
    end
    return true
end
CRCBS.wait(s::State) = Action([CRCBS.wait(s_) for s_ in s])
# get_possible_actions
struct ActionIter{A}
    action_lists::Vector{Vector{A}}
end
# struct ActionIterState
#     iter_list::Vector{Int}
# end
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
    return ActionIter([collect(get_possible_actions(env_,s_)) for (env_,s_) in zip(env.envs, s)])
    # return Base.Iterators.product([get_possible_actions(env_,s_) for (env_,s_) in zip(env.envs, s)]...)
end
Base.length(iter::ActionIter) = product([length(a for a in iter.action_lists)])
CRCBS.get_next_state(s::State,a::Action) = State(
    [get_next_state(s_,a_) for (s_,a_) in zip(s,a)]
    )
CRCBS.get_next_state(env::LowLevelEnv,s::State,a::Action) = State(
    [get_next_state(e_,s_,a_) for (e_,s_,a_) in zip(env.envs,s,a)]
    )
CRCBS.get_transition_cost(env::LowLevelEnv,s::State,a::Action,sp::State) = sum(
    [get_transition_cost(e_,s_,a_,sp_) for (e_,s_,a_,sp_) in zip(env.envs,s,a,sp)]
)
function CRCBS.violates_constraints(env::LowLevelEnv, path, s::State, a::Action, sp::State)
    for (e_,s_,a_,sp_) in zip(env.envs,s,a,sp)
        if violates_constraints(e_,path,s_,a_,sp_)
            return true
        end
    end
    return false
end
CRCBS.check_termination_criteria(env::LowLevelEnv,cost,path,s) = false

"""
    `split_path(path::Path{State{S},Action{A}}) where {S,A}`

    Helper for breaking a meta-agent path into a set of single-agent paths
"""
function split_path(path::Path{State{S},Action{A}}) where {S,A}
    N = length(get_s(get_path_node(path, 1)))
    paths = [Path{S,A}() for i in 1:N]
    for t in 1:length(path)
        path_node = get_path_node(path, t)
        for i in 1:N
            push!(
                paths[i],
                PathNode(
                    get_s(path_node)[i],
                    get_a(path_node)[i],
                    get_sp(path_node)[i]
                    )
                )
        end
    end
    return paths
end

function CRCBS.set_solution_path!(solution, meta_path, idxs::Vector{Int})
    paths = split_path(meta_path)
    for (i, path) in zip(idxs, paths)
        set_solution_path!(solution, path, i)
    end
    return solution
end

"""
    Construct an empty `ConstraintTreeNode` from a `MAPF` instance
"""
function CRCBS.initialize_root_node(mapf::MetaMAPF)
    initialize_root_node(mapf.mapf)
end

CRCBS.default_solution(solver::MetaAgentCBS_Solver, mapf::M where {M <: MetaMAPF}) = LowLevelSolution{State{CBS.State},Action{CBS.Action}}(), typemax(Int)

end
