export
    AbstractLowLevelEnv,
    action_type,
    state_type,

    PathNode,
    get_s,
    get_a,
    get_sp,

    Path,
    get_path_node,
    get_action,
    extend_path,
    extend_path!,
    get_initial_state,
    get_final_state,

    PathCost,

    states_match,
    is_goal,
    wait,
    get_possible_actions,
    get_next_state,
    get_transition_cost,
    get_path_cost,
    get_heuristic,
    violates_constraints,
    check_termination_criteria

"""
    `AbstractLowLevelEnv{S,A}`

    Defines a prototype environment for low level search (searching for a path
    for a single agent).

    S is the State type and A is the action type. Both S and A must be default
    constructible (i.e. you can call S() and A() without throwing errors)

    In general, a concrete subtype of `AbstractLowLevelEnv`
"""
abstract type AbstractLowLevelEnv{S,A} end
action_type(env::E where {S,A,E <: AbstractLowLevelEnv{S,A}}) = A
state_type(env::E where {S,A,E <: AbstractLowLevelEnv{S,A}}) = S

"""
    PathNode: includes current state `s`, action `a`, next state `sp`
"""
@with_kw struct PathNode{S,A}
    s::S = S() # state
    a::A = A() # action
    sp::S = S() # next state
end
get_s(p::PathNode) = p.s
get_a(p::PathNode) = p.a
get_sp(p::PathNode) = p.sp

"""
    `Path{S,A}`

    Encodes a motion plan as a sequence of `PathNode{S,A}`s
"""
const Path{S,A} = Vector{PathNode{S,A}}

"""
    returns the `PathNode` (s,a,s') corresponding to step `t` of `path`

    If `t` is greater than the length of `path`, the `PathNode` returned
    is (s,wait(s),s) corresponding to waiting at that node of the path
"""
function get_path_node(path::Path,t)
    if t <= length(path)
        return path[t]
    else
        t₀ = length(path)
        sp = get_final_state(path)
        for τ in length(path)+1:t
            s = sp
            a = wait(s)
            sp = get_next_state(s,a)
        end
        return PathNode(s,a,sp)
    end
end

"""
    `get_action(path,t)`

    Returns the action at time `t`.

    If `t` is greater than the length of `path`, the returned action defaults to
    a `wait` action.
"""
function get_action(path::Path{S,A}, t::Int) where {S,A}
    get_a(get_path_node(path,t))
    # if length(path) < t
    #     return wait(get_final_state(path))
    # elseif length(path) > 0
    #     return get_a(path[t])
    # else
    #     a = invalid_action(t) # must be overridden
    #     @assert(typeof(a) == A)
    #     return a
    # end
end

"""
    extend_path!(path,T)

    Extends `path` to match a given length `T` by adding `PathNode`s
    corresponding to waiting at the final state.

    args:
    - path      the path to be extended
    - the desired length of the new path
"""
function extend_path!(path::Path,T::Int)
    while length(path) < T
        s = get_final_state(path)
        a = wait(s)
        push!(path,PathNode(s,wait(s),get_next_state(s,a)))
    end
    return path
end

"""
    extend_path(path,T)

    Extends a copy of `path` to match a given length `T` by adding `PathNode`s
    corresponding to waiting at the final state.

    args:
    - path      the path to be extended
    - the desired length of the new path
"""
function extend_path(path::Path,T::Int)
    new_path = copy(path)
    while length(new_path) < T
        s = get_final_state(new_path)
        a = wait(s)
        push!(new_path,PathNode(s,wait(s),get_next_state(s,a)))
    end
    return new_path
end

get_initial_state(path::Path{S,A}) where {S,A} = get_s(get(path,1,PathNode{S,A}()))
get_final_state(path::Path{S,A}) where {S,A} = get_sp(get(path,length(path),PathNode{S,A}()))

""" Type alias for a list of agent paths """
const LowLevelSolution{S,A} = Vector{Path{S,A}}

const PathCost = Int



abstract type CostModel end

struct MakeSpan <: CostModel end

################################################################################
######################## General Methods to Implement ##########################
################################################################################

"""
    state_match(s1::S,s2::S)

    returns true if s1 and s2 match (not necessarily the same as equal)
"""
function states_match end

"""
    is_goal(env,s)

    Returns true if state `s` satisfies the goal condition of environment `env`
"""
function is_goal end

"""
    wait(s)

    returns an action that corresponds to waiting at state s
"""
function wait end

"""
    get_possible_actions(env::E <: AbstractLowLevelEnv{S,A}, s::S)

    return type must support iteration
"""
function get_possible_actions end

"""
    get_next_state(env::E <: AbstractLowLevelEnv{S,A}, s::S, a::A)

    returns a next state s
"""
function get_next_state end

"""
    get_transition_cost(env::E <: AbstractLowLevelEnv{S,A},s::S,a::A,sp::S)

    return scalar cost for transitioning from `s` to `sp` via `a`
"""
function get_transition_cost end

"""
    get_path_cost(env::E <: AbstractLowLevelEnv{S,A},path::Path{S,A})

    get the cost associated with a search path so far
"""
function get_path_cost end

"""
    violates_constraints(env::E <: AbstractLowLevelEnv{S,A},path::Path{S,A},s::S,a::A,sp::S)

    returns `true` if taking action `a` from state `s` violates any constraints
    associated with `env`
"""
function violates_constraints end

"""
    check_termination_criteria(env::E <: AbstractLowLevelEnv{S,A}, cost, path::Path{S,A}, s::S)

    returns true if any termination criterion is satisfied
"""
function check_termination_criteria end

# detect_action_conflict
# detect_state_conflict
