export
    PathNode,
    get_s,
    get_a,
    get_sp
"""
    `PathNode{S,A}`

    Includes current state `s`, action `a`, next state `sp`
"""
@with_kw struct PathNode{S,A}
    s::S = S() # state
    a::A = A() # action
    sp::S = S() # next state
end
get_s(p::P) where {P<:PathNode} = p.s
get_a(p::P) where {P<:PathNode} = p.a
get_sp(p::P) where {P<:PathNode} = p.sp

export
    Path,
    node_type,
    get_cost,
    get_initial_state,
    get_final_state,
    get_path_node,
    get_action,
    extend_path!,
    extend_path
"""
    `Path{S,A,C}`

    Encodes a motion plan as a sequence of `PathNode{S,A}`s
"""
abstract type AbstractPath end
@with_kw mutable struct Path{S,A,C} <: AbstractPath
    s0          ::S                     = S()
    path_nodes  ::Vector{PathNode{S,A}} = Vector{PathNode{S,A}}()
    cost        ::C                     = C(0)
end
node_type(p::Path{S,A,C}) where {S,A,C}     = PathNode{S,A}
Path(v::Vector{P}) where {P<:PathNode}      = Path(get_s(get(v,0,P())),v,0)
Base.cat(p::P,x::N,i...) where {P<:Path,N<:PathNode} = P(p.s0,cat(p.path_nodes,x,dims=1),p.cost)
Base.get(p::P,i,default) where {P<:Path}    = get(p.path_nodes,i,default)
Base.getindex(p::P,i) where {P<:Path}       = getindex(p.path_nodes,i)
Base.setindex!(p::P,x,i) where {P<:Path}    = setindex!(p.path_nodes,x,i)
Base.length(p::P) where {P<:Path}           = length(p.path_nodes)
Base.push!(p::P,x) where {P<:Path}          = push!(p.path_nodes,x)
get_cost(p::P) where {P<:Path}              = p.cost
Base.copy(p::P) where {P<:Path}             = Path(p.s0,copy(p.path_nodes),p.cost)

function get_initial_state(path::P) where {P<:Path}
    if length(path) > 0
        return get_s(get(path,1,node_type(path)()))
    else
        return path.s0
    end
end
function get_final_state(path::P) where {P<:Path}
    if length(path) > 0
        return get_sp(get(path,length(path),node_type(path)()))
    else
        return path.s0
    end
end

"""
    returns the `PathNode` (s,a,s') corresponding to step `t` of `path`

    If `t` is greater than the length of `path`, the `PathNode` returned
    is (s,wait(s),s) corresponding to waiting at that node of the path
"""
function get_path_node(path::P,t::Int) where {P<:Path}
    if t <= length(path)
        return path[t]
    else
        t₀ = length(path)
        if t₀ == 0
            sp = get_initial_state(path)
        else
            node = get(path,length(path),node_type(path)())
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
function get_action(path::P, t::Int) where {P<:Path}
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
function extend_path!(path::P,T::Int) where {P<:Path}
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
function extend_path(path::P,T::Int) where {P<:Path}
    new_path = copy(path)
    extend_path!(new_path,T)
    return new_path
end

export
    AbstractCostModel,
    cost_type
"""
    `AbstractCostModel{T}`
"""
abstract type AbstractCostModel{T} end
get_cost_type(model::M) where {T,M<:AbstractCostModel{T}} = T

export
    LowLevelSolution,
    get_paths,
    get_costs,
    get_cost,
    set_solution_path!,
    set_path_cost!
"""
    `LowLevelSolution{S,A,T,C}`

    Contains a list of agent paths and the associated costs.
    Params:
    - `S` is the state type
    - `A` is the action type
    - `T` is the cost type
    - `C` is the cost model type
    Elements:
    - `paths::Vector{Path{S,A,T}}` is the vector of paths
    - `costs::Vector{T}` is the vector of costs, one per path
    - `cost::T` is the total cost for the entire solution
"""
@with_kw mutable struct LowLevelSolution{S,A,T,C<:AbstractCostModel{T}}
    paths::Vector{Path{S,A,T}}  = Vector{Path{S,A,T}}()
    costs::Vector{T}            = Vector{T}(map(i->get_initial_cost(C()),1:length(paths)))
    cost::T                     = get_initial_cost(C())
end
Base.copy(solution::L) where {L <: LowLevelSolution} = L(
    paths=copy(solution.paths),
    costs=copy(solution.costs),
    cost=copy(solution.cost))
get_paths(solution::L) where {L <: LowLevelSolution} = solution.paths
get_costs(solution::L) where {L <: LowLevelSolution} = solution.costs
get_cost(solution::L) where {L <: LowLevelSolution} = sum([length(p) for p in get_paths(solution)])
function set_solution_path!(solution::L, path::P, idx::Int) where {L<:LowLevelSolution, P<:Path}
    solution.paths[idx] = path
    return solution
end
function set_path_cost!(solution::L, cost::C, idx::Int) where {L<:LowLevelSolution,C}
    solution.costs[idx] = cost
    return solution
end

export
    AbstractLowLevelEnv,
    action_type,
    state_type,
    get_cost_model,
    get_cost_type
"""
    `AbstractLowLevelEnv{S,A,C}`

    Defines a prototype environment for low level search (searching for a path
    for a single agent).

    `S` is the State type, `A` is the action type, and `C` is the cost type. All
    three must be default constructible (i.e. you can call `S()`, `A()` and `C()`
    without throwing errors)

    In general, a concrete subtype of `AbstractLowLevelEnv` may include a graph
    whose edges are traversed by agents.
"""
abstract type AbstractLowLevelEnv{S,A,C} end
action_type(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = A
state_type(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = S
""" Override this method for when the cost model has arguments """
get_cost_model(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = C()
get_cost_type(env::E) where {S,A,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}} = T


export
    AbstractMAPFSolver
""" Abstract type for algorithms that solve MAPF instances """
abstract type AbstractMAPFSolver end
