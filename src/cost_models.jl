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

# """
#     CostCache{M <: LowLevelCostModel,T}
#
#     For storing the incrementally modified cost of a path (rather than
#     recomputing the cost over the entire path at each time step)
# """
# struct CostCache{M <: LowLevelCostModel,T}
#     cost::T
# end

"""
    `cost_model(env::E)`

    Override this method for when the cost model has arguments
"""
cost_model(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = C()
accumulate_cost(env::E, cost, transition_cost) where {E<:AbstractLowLevelEnv} = accumulate_cost(cost_model(env), cost, transition_cost)
get_initial_cost(env::E) where {E<:AbstractLowLevelEnv}     = cost_type(env)(0)
get_initial_cost(model::C) where {C<:AbstractCostModel}     = cost_type(model)(0)
get_infeasible_cost(env::E) where {E<:AbstractLowLevelEnv}  = typemax(cost_type(env))
get_infeasible_cost(model::C) where {C<:AbstractCostModel}  = typemax(cost_type(model))

"""
    `TravelTime <: LowLevelCostModel{Float64}`
"""
struct TravelTime <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelTime, cost, transition_cost) = cost_type(model)(cost + transition_cost)

"""
    `TravelDistance <: LowLevelCostModel{Float64}`
"""
struct TravelDistance <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelDistance, cost, transition_cost) = cost_type(model)(cost + transition_cost)
