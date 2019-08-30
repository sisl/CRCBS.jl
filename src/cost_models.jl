export
    # AbstractCostModel,
    HighLevelCostModel,
    MakeSpan,
    SumOfDistanceTraveled,
    SumOfTravelTime,

    LowLevelCostModel,
    TravelDistance,
    TravelTime,
    CostCache,

    cost_model,
    get_initial_cost,
    get_infeasible_cost,
    get_cost,
    accumulate_cost

"""
    HighLevelCostModel{C}

    The high level cost model defines the objective to be optimized by the
    solver at the high level. An optimal solver will return a solution (if a
    feasible solution exists) of minimal cost under the objective specified by
    the associated HighLevelCostModel.
    The parameter `C` defines the `cost_type` of the objective.
"""
abstract type HighLevelCostModel{T} <: AbstractCostModel{T} end
cost_type(model::M) where {T,M<:AbstractCostModel{T}} = T
struct MakeSpan <: HighLevelCostModel{Float64} end
struct SumOfDistanceTraveled <: HighLevelCostModel{Float64} end
struct SumOfTravelTime <: HighLevelCostModel{Float64} end
struct CompositeHighLevelCost <: HighLevelCostModel{Float64}
    # cost = a*MakeSpan + b*SumOfDistanceTraveled + c*CompositeHighLevelCost
    a::Float64
    b::Float64
    c::Float64
end

"""
    LowLevelCostModel{C}

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

"""
    `cost_model(env::E)`

    Override this method for when the cost model has arguments
"""
cost_model(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = C()
accumulate_cost(env::E, cost, transition_cost) where {E<:AbstractLowLevelEnv} = accumulate_cost(cost_model(env), cost, transition_cost)
function accumulate_cost(model::C, cost, transition_cost) where {C<:AbstractCostModel}
    return cost_type(model)(cost + transition_cost)
end
function get_transition_cost(env::E,s,a,sp) where {E<:AbstractLowLevelEnv}
    return get_transition_cost(env,cost_model(env),s,a,sp)
end
get_initial_cost(env::E) where {E<:AbstractLowLevelEnv}     = cost_type(env)(0)
get_initial_cost(model::C) where {C<:AbstractCostModel}     = cost_type(model)(0)
get_infeasible_cost(env::E) where {E<:AbstractLowLevelEnv}  = typemax(cost_type(env))
get_infeasible_cost(model::C) where {C<:AbstractCostModel}  = typemax(cost_type(model))

"""
    `TravelTime <: LowLevelCostModel{Float64}`
"""
struct TravelTime <: LowLevelCostModel{Float64} end

"""
    `TravelDistance <: LowLevelCostModel{Float64}`
"""
struct TravelDistance <: LowLevelCostModel{Float64} end

"""
    `NullCost`
"""
struct NullCost <: LowLevelCostModel{Float64} end
get_transition_cost(env::E,s,a,sp) where {S,A,E<:AbstractLowLevelEnv{S,A,NullCost}} = 0.0

"""
    `MetaCost`

    This will be used in maintaining separate costs for individual agents that have
    been combined into a MetaAgent.
"""
struct MetaCost{T}
    independent_costs::Vector{T}
    total_costs::T
end
Base.isless(m1::MetaCost,m2::MetaCost) = m1.total_costs < m2.total_costs

"""
    `MetaAgentCost`
"""
struct MetaAgentCost{T} <: LowLevelCostModel{MetaCost{T}} 
    num_agents::Int
end
# function accumulate_cost(model::MetaAgentCost{T}, cost::MetaCost{T}, transition_cost::MetaCost{T}) where {T}
#     new_cost = deepcopy(cost)
#     return cost_type(model)(cost + transition_cost)
# end

"""
    `CompositeCost{T}`
"""
struct CompositeCost{T<:Tuple} <: LowLevelCostModel{Float64}
    cost_models::T
end
construct_composite_cost(args...) = CompositeCost(Tuple(args))
function get_transition_cost(env::E,s,a,sp) where {S,A,C<:CompositeCost,E<:AbstractLowLevelEnv{S,A,C}}
    m = cost_model(env)
    Vector{Float64}(map(c->get_transition_cost(env,c,s,a,sp), m.cost_models))
end
