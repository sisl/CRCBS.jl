################################################################################
##############################    Tuple Stuff    ###############################
################################################################################
for op in [:typemin,:typemax]
    @eval Base.$op(::Type{Tuple{A,B,C,D,E,F}}) where {A,B,C,D,E,F}    = map(i->$op(i),(A,B,C,D,E,F))
    @eval Base.$op(::Type{Tuple{A,B,C,D,E}}) where {A,B,C,D,E}        = map(i->$op(i),(A,B,C,D,E))
    @eval Base.$op(::Type{Tuple{A,B,C,D}}) where {A,B,C,D}            = map(i->$op(i),(A,B,C,D))
    @eval Base.$op(::Type{Tuple{A,B,C}}) where {A,B,C}                = map(i->$op(i),(A,B,C))
    @eval Base.$op(::Type{Tuple{A,B}}) where {A,B}                    = map(i->$op(i),(A,B))
    @eval Base.$op(::Type{Tuple{A}}) where {A}                        = map(i->$op(i),(A,))
end
Base.Tuple{A,B,C,D,E,F}() where {A,B,C,D,E,F}    = map(i->i(0),(A,B,C,D,E,F))
Base.Tuple{A,B,C,D,E}() where {A,B,C,D,E}        = map(i->i(0),(A,B,C,D,E))
Base.Tuple{A,B,C,D}() where {A,B,C,D}            = map(i->i(0),(A,B,C,D))
Base.Tuple{A,B,C}() where {A,B,C}                = map(i->i(0),(A,B,C))
Base.Tuple{A,B}() where {A,B}                    = map(i->i(0),(A,B))
Base.Tuple{A}() where {A}                        = map(i->i(0),(A))

Base.typemin(::Type{NTuple{N,R}}) where {N,R} = NTuple{N,R}(map(i->typemin(R),1:N))
Base.typemax(::Type{NTuple{N,R}}) where {N,R} = NTuple{N,R}(map(i->typemax(R),1:N))
Base.NTuple{N,R}() where {N,R} = NTuple{N,R}(map(i->R(0), 1:N))
Base.typemin(c::Tuple) = typemin(typeof(c))
Base.typemax(c::Tuple) = typemax(typeof(c))

for op = (:(>), :(<), :(>=), :(<=))
    eval(quote
        Base.$op(a::T,b::R) where {N,T<:Tuple,R<:Real} = $op(a[1],b)
        Base.$op(b::R,a::T) where {N,T<:Tuple,R<:Real} = $op(b,a[1])
    end)
end

################################################################################
###############################    Path Stuff    ###############################
################################################################################

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
state_type(p::PathNode{S,A}) where {S,A} = S
action_type(p::PathNode{S,A}) where {S,A} = A
Base.string(p::PathNode) = "$(string(get_s(p))) -- $(string(get_a(p))) -- $(string(get_sp(p)))"

export
    Path,
    node_type,
    get_cost,
    set_cost!,
    get_initial_state,
    get_final_state,
    get_start_time,
    get_end_time,
    get_path_node,
    get_action,
    extend_path!,
    extend_path

abstract type AbstractPath end

"""
    `Path{S,A,C}`

    Encodes a motion plan as a sequence of `PathNode{S,A}`s
"""
@with_kw mutable struct Path{S,A,C} <: AbstractPath
    path_nodes  ::Vector{PathNode{S,A}} = Vector{PathNode{S,A}}()
    s0          ::S                     = get_s(get(path_nodes, 1, PathNode{S,A}()))
    cost        ::C                     = typemax(C)
end
Path(v::Vector{P}) where {P<:PathNode}      = Path(path_nodes=v,s0=get_s(get(v,1,P())),cost=0.0)
state_type(p::Path{S,A,C}) where {S,A,C}    = S
action_type(p::Path{S,A,C}) where {S,A,C}   = A
cost_type(p::Path{S,A,C}) where {S,A,C}     = C
node_type(p) = PathNode{state_type(p),action_type(p)}
Base.cat(p::P,x::N,i...) where {P<:Path,N<:PathNode} = P(s0=p.s0,path_nodes=cat(p.path_nodes,x,dims=1),cost=p.cost)
Base.get(p::P,i,default=node_type(p)) where {P<:Path}    = get(p.path_nodes,i,default)
Base.getindex(p::P,i) where {P<:Path}       = getindex(p.path_nodes,i)
Base.setindex!(p::P,x,i) where {P<:Path}    = setindex!(p.path_nodes,x,i)
Base.length(p::P) where {P<:Path}           = length(p.path_nodes)
function Base.push!(p::P,n::N) where {P<:Path,N<:PathNode}
    if length(p.path_nodes) == 0
        p.s0 = n.s
    end
    push!(p.path_nodes,n)
end
get_cost(p::P) where {P<:Path}              = p.cost
function set_cost!(p::P) where {P<:Path}
    p.cost = cost
end
Base.copy(p::P) where {P<:Path}             = Path(s0=p.s0,path_nodes=copy(p.path_nodes),cost=p.cost)

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


export
    TimeIndexedState,
    get_start_index,
    get_index_from_time,
    get_end_index

get_start_index(path::P) where {P<:Path} = 0
get_end_index(path::P) where {P<:Path} = length(path) + get_start_index(path)
get_index_from_time(path::P,t::Int) where {P<:Path} = t - get_start_index(path)
abstract type TimeIndexedState end
get_start_index(path::P) where {S<:TimeIndexedState,A,C,P<:Path{S,A,C}} = 1 - get_time_index(path.s0)

"""
    returns the `PathNode` (s,a,s') corresponding to step `t` of `path`

    If `t` is greater than the length of `path`, the `PathNode` returned
    is (s,wait(s),s) corresponding to waiting at that node of the path.

    Assumes that all paths start at t = 1
"""
# function get_path_node(path::P,t::Int) where {P<:Path}
#     if t <= length(path)
#         return path[t]
#     else
#         t₀ = length(path)
#         if t₀ == 0
#             sp = get_initial_state(path)
#         else
#             node = get(path,length(path),node_type(path)())
#             s = get_s(node)
#             a = get_a(node)
#             sp = get_sp(node)
#         end
#         for τ in length(path)+1:t
#             s = sp
#             a = wait(s)
#             sp = get_next_state(s,a)
#         end
#         return PathNode(s,a,sp)
#     end
# end
function get_path_node(path::P,t::Int) where {P<:Path}
    t_idx = get_index_from_time(path,t)
    if 1 <= t_idx <= length(path)
        return path[t_idx]
    else
        t₀ = get_index_from_time(path,get_end_index(path))
        if t₀ <= 0
            node = node_type(path)(sp=get_initial_state(path))
            s = get_s(node)
            a = get_a(node)
            sp = get_sp(node)
        else
            node = get(path,length(path),node_type(path)(s=get_initial_state(path)))
            s = get_s(node)
            a = get_a(node)
            sp = get_sp(node)
        end
        for τ in t₀+1:t_idx
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
    # while length(path) < T
    while get_index_from_time(path,get_end_index(path)) < T
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
function extend_path(path::P,args...) where {P<:Path}
    new_path = copy(path)
    extend_path!(new_path,args...)
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
cost_type(model::M) where {T,M<:AbstractCostModel{T}} = T

export
    LowLevelSolution,
    get_paths,
    get_path_costs,
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
    cost_model::C               = C() # TODO C() is a problem
    costs::Vector{T}            = Vector{T}(map(i->get_initial_cost(cost_model),1:length(paths)))
    cost::T                     = get_initial_cost(cost_model)
end
state_type(s::LowLevelSolution{S,A,T,C}) where {S,A,T,C}    = S
action_type(s::LowLevelSolution{S,A,T,C}) where {S,A,T,C}   = A
cost_type(s::LowLevelSolution{S,A,T,C}) where {S,A,T,C}     = T
Base.copy(solution::L) where {L <: LowLevelSolution} = L(
    paths=copy(solution.paths), # NOTE don't want to needlessly copy paths between search nodes
    cost_model=deepcopy(solution.cost_model),
    costs=copy(solution.costs),
    cost=deepcopy(solution.cost),
    )
get_paths(solution::L) where {L <: LowLevelSolution}        = solution.paths
get_path_costs(solution::L) where {L <: LowLevelSolution}   = solution.costs
get_cost(solution::L) where {L <: LowLevelSolution}         = solution.cost
get_cost_model(solution::L) where {L <: LowLevelSolution}   = solution.cost_model
function set_solution_path!(solution::L, path::P, idx::Int) where {L<:LowLevelSolution, P<:Path}
    solution.paths[idx] = path
    return solution
end
function set_path_cost!(solution::L, cost::C, idx::Int) where {L<:LowLevelSolution,C}
    solution.costs[idx] = cost
    return solution
end
function set_cost!(solution::L,cost) where {L<:LowLevelSolution}
    solution.cost = cost
    return solution
end

export
    AbstractLowLevelEnv,
    action_type,
    state_type,
    get_cost_model,
    get_heuristic_model,
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
cost_type(env::E) where {E<:AbstractLowLevelEnv} = cost_type(get_cost_model(env))
""" Override this method for when the cost model has arguments """
get_cost_model(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = C()
function get_heuristic_model end
cost_type(env::E) where {S,A,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}} = T
get_cost_type(env::E) where {S,A,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}} = T


export
    AbstractMAPFSolver,
    NullSolver

""" Abstract type for algorithms that solve MAPF instances """
abstract type AbstractMAPFSolver end
struct NullSolver <: AbstractMAPFSolver end
