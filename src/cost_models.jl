export
    HighLevelCostModel,
    MakeSpan,
    SumOfDistanceTraveled,
    SumOfTravelTime,

    LowLevelCostModel,
    TravelDistance,
    TravelTime,
    CostCache

abstract type HighLevelCostModel end

struct MakeSpan <: HighLevelCostModel end
struct SumOfDistanceTraveled <: HighLevelCostModel end
struct SumOfTravelTime <: HighLevelCostModel end
struct CompositeHighLevelCost <: HighLevelCostModel
    # cost = a*MakeSpan + b*SumOfDistanceTraveled + c*CompositeHighLevelCost
    a::Float64
    b::Float64
    c::Float64
end

abstract type LowLevelCostModel end
struct TravelDistance <: LowLevelCostModel end
struct TravelTime <: LowLevelCostModel end

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
