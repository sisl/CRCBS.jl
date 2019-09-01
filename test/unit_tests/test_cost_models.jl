# TravelTime
let
    model = TravelTime()
    cost = 1.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model,cost,transition_cost)
    @test new_cost == cost + transition_cost
end
# TravelDistance
let
    model = TravelDistance()
    cost = 1.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model,cost,transition_cost)
    @test new_cost == cost + transition_cost
end
# NullCost
let
    model = NullCost()
    cost = 1.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model,cost,transition_cost)
    @test new_cost == cost
end
# MetaCost
let
    t0 = 0.0
    num_agents = 3
    cost = MetaCost{Float64}([1.0,3.0,4.0],0.0)
    transition_cost = [1.0,1.0,1.0]
    let
        FullCostModel(MaxCost(),FinalTime())
        FullCostModel(maximum,FinalTime())
        MakeSpan()
        SumOfTravelTime()
        SumOfTravelDistance()
    end
    let
        model = MetaCostModel(MakeSpan(),num_agents)
        new_cost = accumulate_cost(model,cost,transition_cost)
        @test new_cost.total_cost == 5.0
    end
    let
        model = MetaCostModel(SumOfTravelTime(),num_agents)
        new_cost = accumulate_cost(model,cost,transition_cost)
        @test new_cost.total_cost == 11.0
    end
    let
        model = FullCostModel(max,MetaCostModel(SumOfTravelTime(),num_agents))
        get_cost_model(model)
        new_cost = accumulate_cost(model,cost,transition_cost)
        @test new_cost.total_cost == 11.0
    end
    # @test MetaCost{Float64}([1.0],0.0) < MetaCost{Float64}([0.0],1.0)
end
# CompositeCost
let
    model = construct_composite_cost_model(TravelTime(),TravelTime(),TravelDistance())
    cost = (0.0,0.0,0.0)
    transition_cost = (1.0,1.0,1.0)
    @test accumulate_cost(model, cost,transition_cost) == (1.0,1.0,1.0)
    get_initial_cost(model)
    get_infeasible_cost(model)
end