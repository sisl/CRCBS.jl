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
# DeadlineCost
let
    t_max = 10.0
    model = DeadlineCost(t_max)
    cost = get_initial_cost(model)
    @test cost == 0.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model, cost, transition_cost)
    @test new_cost == 1.0
    h_cost = 8.0
    @test add_heuristic_cost(model, cost, h_cost) == 0.0
    h_cost = 11.0
    @test add_heuristic_cost(model, cost, h_cost) == 1.0
    set_deadline!(model,0.0)
    @test add_heuristic_cost(model, cost, h_cost) == cost + h_cost
    # set_deadline!(model,t_max,t_max-4)
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
let
    cost_model = construct_composite_cost_model(
        FullDeadlineCost(DeadlineCost(10)),
        FullCostModel(sum,NullCost()),
        SumOfTravelTime()
    )
    get_infeasible_cost(cost_model)

end
