export
    DefaultState,
    DefaultAction,
    DefaultPathCost,
    AbstractLowLevelEnv,
    DefaultEnvironment,
    action_type,
    state_type,

    PathNode,
    get_s,
    get_a,
    get_sp,
    DefaultPathNode,

    Path,
    get_path_node,
    get_action,
    extend_path,
    extend_path!,
    get_initial_state,
    get_final_state,

    LowLevelSolution,
    AbstractMAPFSolver,

    initialize_mapf,
    build_env,
    initialize_root_node,
    states_match,
    is_goal,
    wait,
    get_possible_actions,
    get_next_state,
    get_transition_cost,
    get_path_cost,
    heuristic,
    # get_heuristic,
    violates_constraints,
    check_termination_criteria,
    solve!,
    default_solution

struct DefaultState end
struct DefaultAction end
const DefaultPathCost = Int
wait(DefaultState) = DefaultAction()
get_next_state(s::DefaultState,a::DefaultAction) = DefaultState()
get_next_state(env,s::DefaultState,a::DefaultAction) = DefaultState()

"""
    `AbstractLowLevelEnv{S,A,C}`

    Defines a prototype environment for low level search (searching for a path
    for a single agent).

    S is the State type, A is the action type, and C is the cost type. Both S
    and A must be default constructible (i.e. you can call S() and A() without
    throwing errors)

    In general, a concrete subtype of `AbstractLowLevelEnv`
"""
abstract type AbstractLowLevelEnv{S,A,C} end
struct DefaultEnvironment <: AbstractLowLevelEnv{DefaultState,DefaultAction,DefaultPathCost} end
action_type(env::E where {S,A,C,E <: AbstractLowLevelEnv{S,A,C}}) = A
state_type(env::E where {S,A,C,E <: AbstractLowLevelEnv{S,A,C}}) = S

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
const DefaultPathNode = PathNode{DefaultState,DefaultAction}

"""
    `Path{S,A}`

    Encodes a motion plan as a sequence of `PathNode{S,A}`s
"""
# const Path{S,A} = Vector{PathNode{S,A}}
abstract type AbstractPath end
@with_kw mutable struct Path{S,A} <: AbstractPath
    s0          ::S                     = S()
    path_nodes  ::Vector{PathNode{S,A}} = Vector{PathNode{S,A}}()
    cost        ::Int                   = 0
end
Path(v::Vector{PathNode{S,A}}) where {S,A} = Path(get_s(get(v,0,PathNode{S,A}())),v,0)
Base.cat(p::Path{S,A},x::PathNode{S,A},i...) where {S,A} = Path{S,A}(p.s0,cat(p.path_nodes,x,dims=1),p.cost)
Base.get(p::Path,i,default) = get(p.path_nodes,i,default)
Base.getindex(p::Path,i) = getindex(p.path_nodes,i)
Base.setindex!(p::Path,x,i) = setindex!(p.path_nodes,x,i)
Base.length(p::Path) = length(p.path_nodes)
Base.push!(p::Path,x) = push!(p.path_nodes,x)
# num_steps(p::Path) = length(p.path_nodes)
get_cost(p::Path) = p.cost
Base.copy(p::Path) = Path(p.s0,copy(p.path_nodes),p.cost)

function get_initial_state(path::Path{S,A}) where {S,A}
    if length(path) > 0
        return get_s(get(path,1,PathNode{S,A}()))
    else
        return path.s0
    end
end
function get_final_state(path::Path{S,A}) where {S,A}
    if length(path) > 0
        return get_sp(get(path,length(path),PathNode{S,A}()))
    else
        return path.s0
    end
end

"""
    returns the `PathNode` (s,a,s') corresponding to step `t` of `path`

    If `t` is greater than the length of `path`, the `PathNode` returned
    is (s,wait(s),s) corresponding to waiting at that node of the path
"""
function get_path_node(path::Path{S,A},t::Int) where {S,A}
    if t <= length(path)
        return path[t]
    else
        t₀ = length(path)
        if t₀ == 0
            sp = get_initial_state(path)
        else
            node = get(path,length(path),PathNode{S,A}())
            s = get_s(node)
            a = get_a(node)
            sp = get_sp(node)
        end
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
function get_action(path::Path, t::Int)
    get_a(get_path_node(path,t))
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
    extend_path!(new_path,T)
    return new_path
#     while length(new_path) < T
#         s = get_final_state(new_path)
#         a = wait(s)
#         push!(new_path,PathNode(s,wait(s),get_next_state(s,a)))
#     end
#     return new_path
end


""" Type alias for a list of agent paths """
const LowLevelSolution{S,A} = Vector{Path{S,A}}


# abstract type CostModel end
#
# struct MakeSpan <: CostModel end

""" Abstract type for algorithms that solve MAPF instances """
abstract type AbstractMAPFSolver end

################################################################################
######################## General Methods to Implement ##########################
################################################################################

# Other methods to override that are implemented as defaults in common.jl:
# - detect_conflicts!(conflict_table,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)

"""
    `initialize_mapf`
"""
function initialize_mapf end

"""
    `build_env(mapf::AbstractMAPF, node::ConstraintTreeNode, idx::Int)`

    Constructs a new low-level search environment for a conflict-based search
    mapf solver
"""
function build_env end

"""
    `initialize_root_node`

    Construct an empty `ConstraintTreeNode` from a `AbstractMAPF` instance
"""
function initialize_root_node end

"""
    `state_match(s1::S,s2::S)`

    returns true if s1 and s2 match (not necessarily the same as equal)
"""
function states_match end

"""
    `is_goal(env,s)`

    Returns true if state `s` satisfies the goal condition of environment `env`
"""
function is_goal end

"""
    `wait(s)`

    returns an action that corresponds to waiting at state s
"""
function wait end

"""
    `get_possible_actions(env::E <: AbstractLowLevelEnv{S,A,C}, s::S)`

    return type must support iteration
"""
function get_possible_actions end

"""
    `get_next_state(env::E <: AbstractLowLevelEnv{S,A,C}, s::S, a::A)`

    returns a next state s
"""
function get_next_state end

"""
    `get_transition_cost(env::E <: AbstractLowLevelEnv{S,A,C},s::S,a::A,sp::S)`

    return scalar cost for transitioning from `s` to `sp` via `a`
"""
function get_transition_cost end

"""
    `get_path_cost(env::E <: AbstractLowLevelEnv{S,A,C},path::Path{S,A})`

    get the cost associated with a search path so far
"""
function get_path_cost end

"""
    `heuristic(env::E <: AbstractLowLevelEnv{S,A,C},state::S)`

    get a heuristic "cost-to-go" from `state`
"""
function heuristic end

"""
    `violates_constraints(env::E <: AbstractLowLevelEnv{S,A,C},
        path::Path{S,A},s::S,a::A,sp::S)`

    returns `true` if taking action `a` from state `s` violates any constraints
    associated with `env`
"""
function violates_constraints end

"""
    `check_termination_criteria(env::E <: AbstractLowLevelEnv{S,A,C}, cost,
        path::Path{S,A}, s::S)`

    returns true if any termination criterion is satisfied
"""
function check_termination_criteria end

"""
    `solve!(solver, env)`

    Compute a solution from a solver and env
"""
function solve! end

"""
    `default_solution(solver::AbstractMAPFSolver, mapf::AbstractMAPF)`

    Defines what is returned by the solver in case of failure to find a feasible
    solution.
"""
function default_solution end
