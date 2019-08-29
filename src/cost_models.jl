export
    HighLevelCostModel,
    MakeSpan,
    SumOfDistanceTraveled,
    SumOfTravelTime,

    LowLevelCostModel,
    TravelDistance,
    TravelTime,
    CostCache

"""
    HighLevelCostModel{C}

    The high level cost model defines the objective to be optimized by the
    solver at the high level. An optimal solver will return a solution (if a
    feasible solution exists) of minimal cost under the objective specified by
    the associated HighLevelCostModel.
    The parameter `C` defines the `cost_type` of the objective.
"""
abstract type HighLevelCostModel{C} end
cost_type(model::HighLevelCostModel{C}) where {C} = C
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
abstract type LowLevelCostModel{C} end
cost_type(model::LowLevelCostModel{C}) where {C} = C
struct TravelDistance <: LowLevelCostModel{Float64} end
struct TravelTime <: LowLevelCostModel{Float64} end
get_initial_cost(model::TravelTime) = 0.0

"""
    CostCache{M <: LowLevelCostModel,T}

    For storing the incrementally modified cost of a path (rather than
    recomputing the cost over the entire path at each time step)
"""
struct CostCache{M <: LowLevelCostModel,T}
    cost::T
end

function get_cost(model::TravelTime,path::Path)
    return length(path)
end
function accumulate_cost(model::TravelTime, current_cost::C, transition_cost::C) where {C}
    return current_cost + transition_cost
end
function get_transition_cost end
