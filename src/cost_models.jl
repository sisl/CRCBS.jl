export
    get_initial_cost,
    get_infeasible_cost,
    # get_transition_cost,
    accumulate_cost,
    add_heuristic_cost,
    aggregate_costs,

    AggregationFunction,
    MaxCost,
    SumCost,

    FullCostModel,
    get_aggregation_function,
    get_cost_model,
    get_cost_type,

    MetaCostModel,
    MetaCost,
    CompositeCostModel,
    construct_composite_cost_model,

    LowLevelCostModel,

    TravelTime,
    TravelDistance,
    FinalTime,
    NullCost,
    MakeSpan,
    SumOfTravelDistance,
    SumOfTravelTime


get_initial_cost(mapf::M) where {M<:AbstractMAPF}           = get_initial_cost(mapf.env)
get_initial_cost(env::E) where {E<:AbstractLowLevelEnv}     = get_initial_cost(get_cost_model(env))
get_initial_cost(model::C) where {C<:AbstractCostModel}     = get_cost_type(model)(0)

get_infeasible_cost(mapf::M) where {M<:AbstractMAPF}        = get_infeasible_cost(mapf.env)
get_infeasible_cost(env::E) where {E<:AbstractLowLevelEnv}  = get_infeasible_cost(get_cost_model(env))
get_infeasible_cost(model::C) where {C<:AbstractCostModel}  = typemax(get_cost_type(model))

get_transition_cost(env::E,s,a) where {E<:AbstractLowLevelEnv} = get_transition_cost(env,get_cost_model(env),s,a,get_next_state(env,s,a))
get_transition_cost(env::E,s,a,sp) where {E<:AbstractLowLevelEnv} = get_transition_cost(env,get_cost_model(env),s,a,sp)

"""
    `accumulate_cost(model,cost,transition_cost)`

    Defines the way that a `transition_cost` updates the `current_cost` of a
    `Path`.
"""
function accumulate_cost end
accumulate_cost(env::E, cost, transition_cost) where {E<:AbstractLowLevelEnv} = accumulate_cost(get_cost_model(env), cost, transition_cost)

"""
    `add_heuristic_cost(cost_model,heuristic_model,cost,h_cost)`

    Defines how costs from multiple distinct paths are combined. For example,
    the `SumOfTravelTime` objective requires that we sum the costs of individual
    travel times. On the other hand, `MakeSpan` means that we take the maximum
    completion time of all paths.
"""
function add_heuristic_cost end
add_heuristic_cost(m::C, cost, h_cost) where {C<:AbstractCostModel} = cost + h_cost
add_heuristic_cost(m::C, h::H, cost, h_cost) where {C<:AbstractCostModel,H<:AbstractCostModel} = cost + h_cost
add_heuristic_cost(env::E, cost, h_cost) where {E<:AbstractLowLevelEnv} = add_heuristic_cost(get_cost_model(env),cost,h_cost)

"""
    `aggregate_costs(m::C, costs::Vector{T}) where {T,C<:AbstractCostModel{T}}`

    Defines how costs from multiple distinct paths are combined. For example,
    the `SumOfTravelTime` objective requires that we sum the costs of individual
    travel times. On the other hand, `MakeSpan` means that we take the maximum
    completion time of all paths.
"""
function aggregate_costs end
"""
    a special version of aggregate_costs for the meta_env
"""
aggregate_costs_meta(m::AbstractCostModel,args...) = aggregate_costs(m,args...)

abstract type AggregationFunction end
struct MaxCost <: AggregationFunction end
(f::MaxCost)(costs...) = maximum(costs...)
struct SumCost <: AggregationFunction end
(f::SumCost)(costs...) = sum(costs...)

"""
    `FullCostModel{F,T,M<:AbstractCostModel{T}} <: AbstractCostModel{T}`

    The `FullCostModel` defines all the behavior required for running CBS-based
    algorithms.

    Elements:
    - f::F must be callable, and defines how a vector of path costs (e.g., the
        paths of a solution) should be combined into a single cost that reflects
        the cost of the entire path group together
    - model::M<:AbstractCostModel{T} defines how the cost is computed during low
    level search (when individual paths are being compared against each other).
"""
struct FullCostModel{F,T,M<:AbstractCostModel{T}} <: AbstractCostModel{T}
    f::F        # the aggregation function
    model::M    # the low level cost model
end
get_aggregation_function(m::C) where {C<:FullCostModel} = m.f
get_cost_model(m::C) where {C<:FullCostModel} = m.model
aggregate_costs(m::C, costs::Vector{T}) where {T,C<:FullCostModel}  = m.f(costs)
accumulate_cost(model::M, c, c2) where {M<:FullCostModel}           = accumulate_cost(model.model,c,c2)
get_initial_cost(model::F) where {F<:FullCostModel}                 = get_initial_cost(model.model)
get_infeasible_cost(model::F) where {F<:FullCostModel}              = get_infeasible_cost(model.model)
add_heuristic_cost(model::F,cost,h_cost) where {F<:FullCostModel}   = add_heuristic_cost(model.model,cost,h_cost)
get_transition_cost(env::E,model::F,s,a,sp) where {E<:AbstractLowLevelEnv,F<:FullCostModel} = get_transition_cost(env,model.model,s,a,sp)

"""
    `CompositeCost{T}`
"""
struct CompositeCostModel{M<:Tuple,T<:Tuple} <: AbstractCostModel{T}
    cost_models::M
end
function construct_composite_cost_model(args...)
    models = Tuple(args)
    for m in models
        @assert typeof(m) <: AbstractCostModel
    end
    cost_types = map(m->get_cost_type(m),models)
    CompositeCostModel{typeof(models),Tuple{cost_types...}}(models)
end
function get_transition_cost(env::E,model::C,s,a,sp) where {S,A,C<:CompositeCostModel,E<:AbstractLowLevelEnv{S,A,C}}
    get_cost_type(model)(map(m->get_transition_cost(env,m,s,a,sp), model.cost_models))
end
function accumulate_cost(model::C, cost::T, transition_cost::T) where {T,M,C<:CompositeCostModel{M,T}}
    new_cost = map(x->accumulate_cost(x[1],x[2],x[3]),
    zip(model.cost_models, cost, transition_cost))
    T(new_cost)
end
function aggregate_costs(model::C, costs::Vector{T}) where {T,M,C<:CompositeCostModel{M,T}}
    aggregated_costs = map(
        i->aggregate_costs(model.cost_models[i], map(c->c[i], costs)), 1:length(model.cost_models))
    T(aggregated_costs)
end
function aggregate_costs_meta(model::C, costs::Vector{T}) where {T,M,C<:CompositeCostModel{M,T}}
    aggregated_costs = map(
        i->aggregate_costs_meta(model.cost_models[i], map(c->c[i], costs)), 1:length(model.cost_models))
    T(aggregated_costs)
end
function get_initial_cost(model::C) where {T,M,C<:CompositeCostModel{M,T}}
    T(map(m->get_initial_cost(m),model.cost_models))
end
function get_infeasible_cost(model::C) where {T,M,C<:CompositeCostModel{M,T}}
    T(map(m->get_infeasible_cost(m),model.cost_models))
end
function add_heuristic_cost(model::C, cost::T, h_cost) where {T,M,C<:CompositeCostModel{M,T}}
    T(map(
        i->add_heuristic_cost(model.cost_models[i], cost[i], h_cost[i]),
        1:length(cost)
        ))
end

"""
    `MetaCost`

    `MetaCost` maintaining separate costs for individual agents that have been
    combined into a MetaAgent.
    - independent_costs::Vector{T} a vector of costs, 1 per agent
    - total_cost::T the total cost, which reflects the combined cost (its
        interpretation depends on the `MetaCostModel` used to define cost-
        upating behavior)
"""
struct MetaCost{T}
    independent_costs::Vector{T}
    total_cost::T
end
Base.isless(m1::MetaCost,m2::MetaCost) = m1.total_cost < m2.total_cost

"""
    `MetaCostModel`

    Defines the cost-updating behavior of `MetaCost` for MetaAgent applications.
"""
struct MetaCostModel{T,M<:AbstractCostModel{T}} <: AbstractCostModel{MetaCost{T}}
    model::M
    num_agents::Int
end
aggregate_costs(m::C, costs::Vector{T}) where {T,C<:MetaCostModel} = aggregate_costs(m.model, costs)
function add_heuristic_cost(m::C, cost, h_cost) where {C<:MetaCostModel}
    costs = map(i->add_heuristic_cost(
        m.model,
        cost.independent_costs[i],
        h_cost[i]),1:m.num_agents)
    MetaCost(costs, aggregate_costs_meta(m.model, costs))
end
function accumulate_cost(model::M, cost::MetaCost{T}, transition_cost::Vector{T}) where {T,M<:MetaCostModel}
    new_costs = Vector{T}()
    for (i,(c1,c2)) in enumerate(zip(
        cost.independent_costs,
        transition_cost))
        push!(new_costs, accumulate_cost(model.model, c1, c2))
    end
    total_cost = aggregate_costs_meta(model.model,new_costs)
    new_cost = MetaCost{T}(new_costs,total_cost)
    return new_cost
end
function accumulate_cost(model::M, cost::MetaCost{T}, transition_cost::MetaCost{T}) where {T,M<:MetaCostModel}
    accumulate_cost(model, cost, transition_cost.independent_costs)
end
function get_initial_cost(model::C) where {C<:MetaCostModel}
    costs = map(a->get_initial_cost(model.model),1:model.num_agents)
    MetaCost(costs,aggregate_costs(model,costs))
end
function get_infeasible_cost(model::C) where {C<:MetaCostModel}
    costs = map(a->get_infeasible_cost(model.model),1:model.num_agents)
    MetaCost(costs,aggregate_costs(model,costs))
end

################################################################################
############################## Atomic Cost Models ##############################
################################################################################

"""
    `LowLevelCostModel{C}`

    The low level cost model defines the objective to be optimized by the
    solver at the low level. An optimal low level solver will return a path if a
    feasible path exists) of minimal cost under the objective specified by the
    associated LowLevelCostModel.
    The parameter `C` defines the `cost_type` of the objective. The following
    functions must be implemented for a `LowLevelCostModel` to be used:
    * `get_initial_cost(model::LowLevelCostModel, env::LowLevelEnv)` - returns
    the initial_cost for a path
    * `get_transition_cost(model::LowLevelCostModel{C},path::Path,s::S,a::A,
        sp::S) where {S,A,C}` - defines the cost associated with taking action
        `a` from state `s` to arrive in state `sp` according to the objective
        defined by `model` given that `s` is the "tip" of `path`.
    * `accumulate_cost(model::LowLevelCostModel{C}, current_cost::C,
        transition_cost::C)` - defines how cost accumulates as new `PathNode`s
        are added to a Path.
"""
abstract type LowLevelCostModel{T} <: AbstractCostModel{T} end

export
    TransformCostModel

struct TransformCostModel{T,M<:LowLevelCostModel{T}} <: LowLevelCostModel{T}
    f::Function
    model::M
end
get_transition_cost(env,m::TransformCostModel,args...) = m.f(get_transition_cost(env,m.model,args...))
accumulate_cost(m::TransformCostModel, args...)     = accumulate_cost(m.model,args...)
get_initial_cost(m::TransformCostModel,args...)     = get_initial_cost(m.model,args...)
get_infeasible_cost(m::TransformCostModel)          = get_infeasible_cost(m.model)
add_heuristic_cost(m::TransformCostModel,args...)   = add_heuristic_cost(m.model,args...)

struct TravelTime       <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelTime,      cost,transition_cost) = cost+transition_cost

struct TravelDistance   <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelDistance,  cost,transition_cost) = cost+transition_cost

struct FinalTime        <: LowLevelCostModel{Float64} end
accumulate_cost(model::FinalTime,       cost,transition_cost) = cost+transition_cost

struct NullCost         <: LowLevelCostModel{Float64} end
get_transition_cost(env::E,model::F,s,a,sp) where {E<:AbstractLowLevelEnv,F<:NullCost} = 0.0
accumulate_cost(model::NullCost,        cost,transition_cost) = cost

export
    DeadlineCost,
    set_deadline!,
    FullDeadlineCost,
    # WeightedDeadlinesCost,
    MultiDeadlineCost,
    SumOfMakeSpans

abstract type AbstractDeadlineCost <: LowLevelCostModel{Float64} end
"""
    `DeadlineCost`

    `DeadlineCost` is identical to `TravelTime`, except for the behavior of
    `add_heuristic_cost`.

    add_heuristic_cost: `c = max(0.0, t + Δt - deadline)`
"""
mutable struct DeadlineCost     <: AbstractDeadlineCost
    deadline::Float64 # deadline
    m::TravelTime
end
DeadlineCost(deadline::R) where {R<:Real} = DeadlineCost(deadline,TravelTime())
function set_deadline!(m::DeadlineCost,t_max)
    m.deadline = minimum(t_max)
    return m
end
set_deadline!(m::C,args...) where {C<:AbstractCostModel} = nothing
set_deadline!(m::C,args...) where {C<:FullCostModel} = set_deadline!(m.model,args...)
set_deadline!(m::C,args...) where {C<:MetaCostModel} = set_deadline!(m.model,args...)
function set_deadline!(m::C,args...) where {C<:CompositeCostModel}
    for model in m.cost_models
        set_deadline!(model,args...)
    end
end
get_initial_cost(m::C) where {C<:AbstractDeadlineCost} = get_initial_cost(m.m)
get_transition_cost(env::E,model::M,args...) where {E<:AbstractLowLevelEnv,M<:AbstractDeadlineCost} = get_transition_cost(env,model.m,args...)
accumulate_cost(model::C,cost,transition_cost) where {C<:AbstractDeadlineCost} = accumulate_cost(model.m,cost,transition_cost)
# add_heuristic_cost(m::C, cost, h_cost) where {C<:DeadlineCost} = max(0.0, cost + h_cost + m.deadline - m.stage_deadline) # assumes heuristic is PerfectHeuristic
add_heuristic_cost(m::C, cost, h_cost) where {C<:DeadlineCost} = max(0.0, cost + h_cost - m.deadline) # assumes heuristic is PerfectHeuristic
FullDeadlineCost(model::DeadlineCost) = FullCostModel(costs->max(0.0, maximum(costs)),model)

"""
    `MultiDeadlineCost`
"""
struct MultiDeadlineCost{F} <: AbstractDeadlineCost
    f::F # aggregation function
    tF::Vector{Float64}
    root_nodes::Vector{Int}
    weights::Vector{Float64}
    deadlines::Vector{Float64}
    m::TravelTime
end
const SumOfMakeSpans = MultiDeadlineCost{SumCost}
const MakeSpan = MultiDeadlineCost{MaxCost}
SumOfMakeSpans(tF,root_nodes,weights,deadlines) = MultiDeadlineCost(SumCost(),Float64.(tF),root_nodes,Float64.(weights),Float64.(deadlines),TravelTime())
SumOfMakeSpans() = MultiDeadlineCost(SumCost(),Float64[],Int[],Float64[],Float64[],TravelTime())
MakeSpan(tF,root_nodes,weights,deadlines) = MultiDeadlineCost(MaxCost(),Float64.(tF),root_nodes,Float64.(weights),Float64.(deadlines),TravelTime())
MakeSpan() = MultiDeadlineCost(MaxCost(),Float64[],Int[],Float64[],Float64[],TravelTime())
function set_deadline!(m::M,t_max) where {M<:MultiDeadlineCost}
    m.deadlines .= t_max
    return m
end
add_heuristic_cost(m::C, cost, h_cost) where {C<:MultiDeadlineCost} = m.f(m.weights .* max.(0.0, cost .+ h_cost .- m.deadlines)) # assumes heuristic is PerfectHeuristic
aggregate_costs(m::C, costs::Vector{T}) where {T,C<:MultiDeadlineCost}  = m.f(m.tF[m.root_nodes] .* m.weights) # TODO this is why A_star is failing on MetaEnv!
aggregate_costs_meta(m::C, costs::Vector{T}) where {T,C<:MultiDeadlineCost}  = maximum(costs)

# MakeSpan(model::FinalTime=FinalTime()) = FullCostModel(maximum,model)
SumOfTravelDistance(model::TravelDistance=TravelDistance()) = FullCostModel(sum,model)
SumOfTravelTime(model::TravelTime=TravelTime()) = FullCostModel(sum, model)

################################################################################
############################# HardConflictHeuristic ############################
################################################################################
export
    HardConflictTable,
    get_time_horizon,
    get_planned_vtx,
    reset_path!,
    set_path!,
    partially_set_path!,
    get_conflict_value,
    get_conflicting_paths,
    construct_empty_lookup_table

"""
    `HardConflictTable`

    Stores a lookup table of planned paths for all agents.
    When agent `i` queries the table, `table.paths[i]` (the existing path plan
    for agent `i`) must be subtracted so that the agent does not try to avoid
    conflicts with itself.
"""
@with_kw struct HardConflictTable{V<:AbstractVector,M<:AbstractMatrix}
    paths   ::Vector{V} = Vector{SparseVector{Int,Int}}()
    CAT     ::M         = SparseMatrixCSC(zeros(2,2)) # global table
    start_time::Int     = 0
end
get_time_horizon(h::T) where {T<:HardConflictTable} = size(h.CAT,2)
function get_planned_vtx(h::T,agent_idx::Int,t::Int) where {T<:HardConflictTable}
    t_idx = t + (1-h.start_time)
    if agent_idx == -1
        return 0
    else
        return get(h.paths[agent_idx], t_idx, -1)
    end
end
function reset_path!(h::T,path_idx::Int) where {V,M,T<:HardConflictTable{V,M}}
    # first remove the old path from the lookup table
    for (t,vtx) in enumerate(h.paths[path_idx])
        if vtx > 0
            h.CAT[vtx,t] = h.CAT[vtx,t] - 1
        end
    end
    # initialize an empty new path
    h.paths[path_idx] = V(zeros(Int,get_time_horizon(h)))
    return h
end
function set_path!(h::T,path_idx::Int,path::Vector{Int},start_time::Int=0) where {V,M,T<:HardConflictTable{V,M}}
    # println("Updating conflict table with path ", path, " for agent ",path_idx)
    reset_path!(h,path_idx)
    # now add new path vtxs to new path and lookup table
    for (i,vtx) in enumerate(path)
        t = (start_time-h.start_time) + i
        h.paths[path_idx][t] = vtx
        h.CAT[vtx,t] = h.CAT[vtx,t] + 1
    end
    h
end
"""
    `partially_set_path!`

    Only replaces the cached path from start_time to length(path). Useful if you
    want the remainder of the cached path to stay in the lookup table (i.e. for
    repairing an existing plan).
"""
function partially_set_path!(h::T,path_idx::Int,path::Vector{Int},start_time::Int=0) where {V,M,T<:HardConflictTable{V,M}}
    # now add new path vtxs to new path and lookup table
    t0 = (start_time-h.start_time)
    for (i,vtx) in enumerate(path)
        t = t0 + i
        # try
            old_vtx = h.paths[path_idx][t]
        if old_vtx != 0
            h.CAT[old_vtx,t] = h.CAT[old_vtx,t] - 1
        end
        h.paths[path_idx][t] = vtx
        h.CAT[vtx,t] = h.CAT[vtx,t] + 1
        # catch e
        #     throw(e)
        # end
    end
    h
end
function get_conflict_value(h::HardConflictTable,agent_idx::Int,vtx::Int,t::Int)
    t_idx = t + (1-h.start_time)
    try
        # c = h.CAT[vtx,t_idx]
        c = get(h.CAT, (vtx,t_idx), 0)
        if get_planned_vtx(h,agent_idx,t) == vtx # conflict with own trajectory
            c = c - 1
        end
        return c
    catch e
        @show vtx,t_idx,size(h.CAT)
        throw(e)
    end
end
"""
    `get_conflicting_paths`

    operates over a lookup table and returns a dictionary mapping path index to
    the time index at which the conflict occurred
"""
function get_conflicting_paths(ct::T) where {T<:HardConflictTable}
    conflict_idxs = map(idx->Int.([idx.I...]), findall(ct.CAT .> 1))
    agent_id_to_time_idx = Dict{Int,Int}()
    for idx in conflict_idxs
        vtx = idx[1]
        t = idx[2]+ct.start_time-1
        for (agent_id,path) in enumerate(ct.paths)
            if get_planned_vtx(ct,agent_id,t) == vtx
                agent_id_to_time_idx[agent_id] = t
            end
        end
    end
    agent_id_to_time_idx
end
"""
    `construct_empty_lookup_table(G,T::Int)`

    Returns an empty lookup table.
"""
construct_empty_lookup_table(V::Int,T::Int) = SparseMatrixCSC(zeros(V,T))
construct_empty_lookup_table(graph::G,T::Int) where {G<:AbstractGraph} = construct_empty_lookup_table(nv(graph),T)
function HardConflictTable(graph::G,T::Int,num_agents::Int) where {G<:AbstractGraph}
    HardConflictTable(
        paths = map(i->SparseVector(zeros(Int,T)),1:num_agents),
        CAT = construct_empty_lookup_table(graph,T)
        )
end

################################################################################
############################### SoftConflictHeuristic ##############################
################################################################################
export
    SoftConflictTable,
    get_fat_path,
    add_fat_path_to_table!,
    populate_soft_lookup_table!

"""
    `SoftConflictTable`
"""
@with_kw struct SoftConflictTable{M<:AbstractMatrix}
    CAT::M = SparseMatrixCSC(zeros(2,2)) # global table
end
get_time_horizon(h::SoftConflictTable) = size(h.CAT,2)
get_conflict_value(h::SoftConflictTable,vtx::Int,t::Int) = h.CAT[vtx,t]
get_conflict_value(h::SoftConflictTable,agent_idx::Int,vtx::Int,t::Int) = h.CAT[vtx,t]

"""
    `get_fat_path(G,D,start_vtx,goal_vtx)`

    returns a fat path through `G` from `start_vtx` to `goal_vtx`. Each set
    of vertices in the fat path contains all vertices with distance d1 from
    start_vtx and distance d2 to goal_vtx, where d1+d2 == the length of the
    shortest path(s) from `start_vtx` to `goal_vtx`

    G is a graph, D is the distance matrix
"""
function get_fat_path(G,D,start_vtx::Int,goal_vtx::Int)
    fat_path = Vector{Set{Int}}([Set{Int}(start_vtx)])
    for i in 1:D[start_vtx,goal_vtx]
        next_set = Set{Int}()
        for src_vtx in fat_path[end]
            for dst_vtx in outneighbors(G,src_vtx)
                if D[dst_vtx,goal_vtx] <= D[src_vtx,goal_vtx] - 1
                    push!(next_set, dst_vtx)
                end
            end
        end
        push!(fat_path, next_set)
    end
    fat_path
end
"""
    `add_fat_path_to_table(CAT,fat_path)`
"""
function add_fat_path_to_table!(CAT,t0,fat_path)
    for t in 1:length(fat_path)
        idxs = collect(fat_path[t])
        if t+t0 > 0
            CAT[idxs,t+t0] .+= 1.0/length(idxs)
        end
    end
end
"""
    `populate_soft_lookup_table!(CAT,start_times,start_vtxs,goal_vtxs)`
"""
function populate_soft_lookup_table!(CAT,G,D,start_times,start_vtxs,goal_vtxs)
    path_list = [t=>get_fat_path(G,D,s,g) for (s,t,g) in zip(start_vtxs,start_times,goal_vtxs)]
    for (t0,fat_path) in path_list
        add_fat_path_to_table!(CAT,t0,fat_path)
    end
    CAT
end
"""
    `construct_empty_lookup_table(graph,T::Int)`

    Returns a soft lookup table to encode possible paths for each agent through
    `graph`. The argument `T` defines the time horizon of the lookup table.
"""
function SoftConflictTable(graph,T::Int)
    SoftConflictTable(construct_empty_lookup_table(graph,T))
end
"""
    `construct_and_populate_soft_lookup_table!`
"""
function SoftConflictTable(graph,start_times::Vector{Int},start_vtxs::Vector{Int},goal_vtxs::Vector{Int};
        T = Int(round(maximum(start_times) + nv(graph))))
    CAT = construct_empty_lookup_table(graph,T)
    D = get_dist_matrix(graph)
    populate_soft_lookup_table!(CAT,graph,D,start_times,start_vtxs,goal_vtxs)
    SoftConflictTable(CAT)
end

################################################################################
############################# ConflictCostModel ############################
################################################################################
export
    ConflictCostModel,
    HardConflictCost,
    SoftConflictCost

struct ConflictCostModel{T<:Union{HardConflictTable,SoftConflictTable}} <: LowLevelCostModel{Float64}
    table::T
end
get_conflict_value(h::H, args...) where {H<:ConflictCostModel} = get_conflict_value(h.table, args...)
accumulate_cost(h::H,cost,transition_cost) where {H<:ConflictCostModel} = cost + transition_cost

HardConflictCost(args...) = FullCostModel(sum,ConflictCostModel(HardConflictTable(args...)))
SoftConflictCost(args...) = FullCostModel(sum,ConflictCostModel(SoftConflictTable(args...)))

get_time_horizon(h::H) where {H<:ConflictCostModel} = get_time_horizon(h.table)
get_planned_vtx(h::H,args...) where {T<:HardConflictTable,H<:ConflictCostModel}  = get_planned_vtx(h.table,args...)
reset_path!(h::H,args...) where {H<:ConflictCostModel}    = reset_path!(h.table,args...)
reset_path!(h::H,args...) where {H<:AbstractCostModel}    = nothing
reset_path!(h::FullCostModel{F,T,M},args...) where {F,T,M<:ConflictCostModel} = reset_path!(h.model,args...)
function reset_path!(h::H,args...) where {H<:CompositeCostModel}
    for m in h.cost_models
        reset_path!(m,args...)
    end
end
set_path!(h::H,args...) where {H<:ConflictCostModel}    = set_path!(h.table,args...)
set_path!(h::H,args...) where {H<:AbstractCostModel}    = nothing
set_path!(h::FullCostModel{F,T,M},args...) where {F,T,M<:ConflictCostModel} = set_path!(h.model,args...)
function set_path!(h::H,args...) where {H<:CompositeCostModel}
    for m in h.cost_models
        set_path!(m,args...)
    end
end
partially_set_path!(h::H,args...) where {H<:ConflictCostModel}    = partially_set_path!(h.table,args...)
partially_set_path!(h::H,args...) where {H<:AbstractCostModel}    = nothing
partially_set_path!(h::FullCostModel{F,T,M},args...) where {F,T,M<:ConflictCostModel} = partially_set_path!(h.model,args...)
function partially_set_path!(h::H,args...) where {H<:CompositeCostModel}
    for m in h.cost_models
        partially_set_path!(m,args...)
    end
end
