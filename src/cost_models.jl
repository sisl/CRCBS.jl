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
    DeadlineCost,
        set_deadline!,

    FullDeadlineCost,
    MakeSpan,
    SumOfTravelDistance,
    SumOfTravelTime


get_initial_cost(mapf::M) where {M<:AbstractMAPF}           = get_initial_cost(mapf.env)
get_initial_cost(env::E) where {E<:AbstractLowLevelEnv}     = get_initial_cost(get_cost_model(env))
get_initial_cost(model::C) where {C<:AbstractCostModel}     = get_cost_type(model)(0)

get_infeasible_cost(mapf::M) where {M<:AbstractMAPF}        = get_infeasible_cost(mapf.env)
get_infeasible_cost(env::E) where {E<:AbstractLowLevelEnv}  = get_infeasible_cost(get_cost_model(env))
get_infeasible_cost(model::C) where {C<:AbstractCostModel}  = typemax(get_cost_type(model))

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

abstract type AggregationFunction end
struct MaxCost <: AggregationFunction end
(f::MaxCost)(costs) = maximum(costs)
struct SumCost <: AggregationFunction end
(f::SumCost)(costs) = sum(costs)

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
aggregate_costs(m::C, costs::Vector{T}) where {T,C<:FullCostModel} = m.f(costs)
accumulate_cost(model::M, c, c2) where {M<:FullCostModel} = accumulate_cost(model.model,c,c2)
get_initial_cost(model::F) where {F<:FullCostModel} = get_initial_cost(model.model)
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
    MetaCost(costs, aggregate_costs(m, costs))
end
function accumulate_cost(model::M, cost::MetaCost{T}, transition_cost::Vector{T}) where {T,M<:MetaCostModel}
    new_costs = Vector{T}()
    for (i,(c1,c2)) in enumerate(zip(
        cost.independent_costs,
        transition_cost))
        push!(new_costs, accumulate_cost(model.model, c1, c2))
    end
    total_cost = aggregate_costs(model.model,new_costs)
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

struct TravelTime       <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelTime,      cost,transition_cost) = cost+transition_cost

struct TravelDistance   <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelDistance,  cost,transition_cost) = cost+transition_cost

struct FinalTime        <: LowLevelCostModel{Float64} end
accumulate_cost(model::FinalTime,       cost,transition_cost) = cost+transition_cost

struct NullCost         <: LowLevelCostModel{Float64} end
get_transition_cost(env::E,model::F,s,a,sp) where {E<:AbstractLowLevelEnv,F<:NullCost} = 0.0
accumulate_cost(model::NullCost,        cost,transition_cost) = cost

"""
    `DeadlineCost`

    `DeadlineCost` is identical to `TravelTime`, except for the behavior of
    `add_heuristic_cost`.

    add_heuristic_cost: `c = max(0.0, t + Δt - deadline)`
"""
mutable struct DeadlineCost     <: LowLevelCostModel{Float64}
    deadline::Float64 # deadline
    # stage_deadline::Float64
    m::TravelTime
end
# DeadlineCost(deadline::R) where {R<:Real} = DeadlineCost(deadline,deadline,TravelTime())
DeadlineCost(deadline::R) where {R<:Real} = DeadlineCost(deadline,TravelTime())
# function set_deadline!(m::DeadlineCost,t_max,t_stage=t_max)
function set_deadline!(m::DeadlineCost,t_max)
    m.deadline = t_max
    # m.stage_deadline = t_stage
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
# get_initial_cost(m::DeadlineCost) = - get_cost_type(m)(m.deadline)
get_initial_cost(m::DeadlineCost) = get_initial_cost(m.m)
get_transition_cost(env::E,model::M,args...) where {E<:AbstractLowLevelEnv,M<:DeadlineCost} = get_transition_cost(env,model.m,args...)
accumulate_cost(model::DeadlineCost,cost,transition_cost) = accumulate_cost(model.m,cost,transition_cost)
# add_heuristic_cost(m::C, cost, h_cost) where {C<:DeadlineCost} = max(0.0, cost + h_cost + m.deadline - m.stage_deadline) # assumes heuristic is PerfectHeuristic
add_heuristic_cost(m::C, cost, h_cost) where {C<:DeadlineCost} = max(0.0, cost + h_cost - m.deadline) # assumes heuristic is PerfectHeuristic
FullDeadlineCost(model::DeadlineCost) = FullCostModel(costs->max(0.0, maximum(costs)),model)

MakeSpan(model::FinalTime=FinalTime()) = FullCostModel(MaxCost(),model)
SumOfTravelDistance(model::TravelDistance=TravelDistance()) = FullCostModel(SumCost(),model)
SumOfTravelTime(model::TravelTime=TravelTime()) = FullCostModel(SumCost(), model)
