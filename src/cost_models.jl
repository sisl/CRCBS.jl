export
    # AbstractCostModel,
    HighLevelCostModel,
    MakeSpan,
    SumOfTravelDistance,
    SumOfTravelTime,

    CombinerFunction,
    MaxCost,
    SumCost,

    FullCostModel,

    LowLevelCostModel,
    TravelTime,
    TravelDistance,
    FinalTime,
    NullCost,
    MetaCostModel,
    MetaCost,
    CompositeCostModel,
    construct_composite_cost_model,

    accumulate_cost,
    add_heuristic_cost,
    combine_costs,

    get_initial_cost,
    get_infeasible_cost

"""
    `HighLevelCostModel{C}`

    The high level cost model defines the objective to be optimized by the
    solver at the high level. An optimal solver will return a solution (if a
    feasible solution exists) of minimal cost under the objective specified by
    the associated HighLevelCostModel.
    The parameter `C` defines the `cost_type` of the objective.
"""
abstract type HighLevelCostModel{T} <: AbstractCostModel{T} end

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
struct TravelDistance   <: LowLevelCostModel{Float64} end
struct FinalTime        <: LowLevelCostModel{Float64} end
struct NullCost         <: LowLevelCostModel{Float64} end

abstract type CombinerFunction end
struct MaxCost <: CombinerFunction end
(f::MaxCost)(costs) = maximum(costs)
struct SumCost <: CombinerFunction end
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
    f::F        # the combiner function
    model::M    # the low level cost model
end
cost_model(m::C) where {C<:FullCostModel} = m.model
cost_type(m::FullCostModel{F,T,M}) where {F,T,M} = T

MakeSpan(model::FinalTime=FinalTime()) = FullCostModel(MaxCost(),model)
SumOfTravelDistance(model::TravelDistance=TravelDistance()) = FullCostModel(SumCost(),model)
SumOfTravelTime(model::TravelTime=TravelTime()) = FullCostModel(SumCost(), model)

get_initial_cost(env::E) where {E<:AbstractLowLevelEnv}     = get_initial_cost(cost_model(env))
get_initial_cost(model::C) where {C<:AbstractCostModel}     = cost_type(model)(0)
get_infeasible_cost(env::E) where {E<:AbstractLowLevelEnv}  = get_infeasible_cost(cost_model(env))
get_infeasible_cost(model::C) where {C<:AbstractCostModel}  = typemax(cost_type(model))

"""
    `accumulate_cost(model,cost,transition_cost)`

    Defines the way that a `transition_cost` updates the `current_cost` of a
    `Path`.
"""
function accumulate_cost end
"""
    `add_heuristic_cost(cost_model,heuristic_model,cost,h_cost)`

    Defines how costs from multiple distinct paths are combined. For example,
    the `SumOfTravelTime` objective requires that we sum the costs of individual
    travel times. On the other hand, `MakeSpan` means that we take the maximum
    completion time of all paths.
"""
function add_heuristic_cost end
"""
    `combine_costs(m_hi_level::H, m_lo_level::C, costs::Vector{T}) where {T,H<:HighLevelCostModel,C<:AbstractCostModel{T}}`

    Defines how costs from multiple distinct paths are combined. For example,
    the `SumOfTravelTime` objective requires that we sum the costs of individual
    travel times. On the other hand, `MakeSpan` means that we take the maximum
    completion time of all paths.
"""
function combine_costs end

get_transition_cost(env::E,s,a,sp) where {E<:AbstractLowLevelEnv} = get_transition_cost(env,cost_model(env),s,a,sp)
get_transition_cost(env::E,s,a,sp) where {S,A,E<:AbstractLowLevelEnv{S,A,NullCost}} = 0.0

accumulate_cost(env::E, cost, transition_cost) where {E<:AbstractLowLevelEnv} = accumulate_cost(cost_model(env), cost, transition_cost)
accumulate_cost(model::M, c, c2) where {M<:FullCostModel} = accumulate_cost(cost_model(model),c,c2)

accumulate_cost(model::TravelTime,      cost,transition_cost) = cost+transition_cost
accumulate_cost(model::TravelDistance,  cost,transition_cost) = cost+transition_cost
accumulate_cost(model::FinalTime,       cost,transition_cost) = cost+transition_cost
accumulate_cost(model::NullCost,        cost,transition_cost) = cost

combine_costs(m::C, costs::Vector{T}) where {T,C<:FullCostModel} = m.f(costs)

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
function accumulate_cost(model::M, cost::MetaCost{T}, transition_cost::Vector{T}) where {T,M<:MetaCostModel}
    new_costs = Vector{T}()
    for (i,(c1,c2)) in enumerate(zip(
        cost.independent_costs,
        transition_cost))
        push!(new_costs, accumulate_cost(model.model, c1, c2))
    end
    total_cost = combine_costs(model.model,new_costs)
    new_cost = MetaCost{T}(new_costs,total_cost)
    return new_cost
end
function accumulate_cost(model::M, cost::MetaCost{T}, transition_cost::MetaCost{T}) where {T,M<:MetaCostModel}
    accumulate_cost(model, cost, transition_cost.independent_costs)
end

"""
    `CompositeCost{T}`
"""
struct CompositeCostModel{M<:Tuple,T<:Tuple} <: LowLevelCostModel{T}
    cost_models::M
end
function construct_composite_cost_model(args...)
    models = Tuple(args)
    for m in models
        @assert typeof(m) <: AbstractCostModel
    end
    cost_types = map(m->cost_type(m),models)
    CompositeCostModel{typeof(models),Tuple{cost_types...}}(models)
end
function get_transition_cost(env::E,model::C,s,a,sp) where {S,A,C<:CompositeCostModel,E<:AbstractLowLevelEnv{S,A,C}}
    cost_type(model)(map(c->get_transition_cost(env,m,s,a,sp), model.cost_models)...)
end
function accumulate_cost(model::C, cost::T, transition_cost::T) where {T,M,C<:CompositeCostModel{M,T}}
    new_cost = map(x->accumulate_cost(x[1],x[2],x[3]),
    zip(model.cost_models, cost, transition_cost))
    cost_type(model)(new_cost)
end
function combine_costs(model::C, costs::Vector{T}) where {T,M,C<:CompositeCostModel{M,T}}
    combined_costs = map(
        i->combine_costs(model.cost_models[i], map(c->c[i], costs)), 1:length(model.cost_models))
    cost_type(model)(combine_costs)
end
function get_initial_cost(model::M) where {M<:CompositeCostModel}
    cost_type(model)(map(m->get_initial_cost(m),model.cost_models))
end
function get_infeasible_cost(model::M) where {M<:CompositeCostModel}
    cost_type(model)(map(m->get_infeasible_cost(m),model.cost_models))
end
