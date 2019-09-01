module heuristicsTests

using Parameters
using CRCBS

@with_kw struct State
    v::Int = -1
end
@with_kw struct Action
    v::Int = -1
end
struct GraphEnv <: AbstractLowLevelEnv{State,Action,TravelTime}
    G
end
get_heuristic_cost(env::GraphEnv,s::State) = get_heuristic_cost()

end #module

let
    PerfectHeuristic()
    SoftConflictTable()
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    start_times = [0,-2,2]
    starts = [1,2,3]
    goals = [4,5,6]
    let
        h = PerfectHeuristic(G,starts,goals)
        @test get_heuristic_cost(h,goals[1],starts[1]) == gdistances(G,starts[1])[goals[1]]
    end
    let
        h = SoftConflictTable(G,start_times,starts,goals)
        @test get_heuristic_cost(h,starts[1],1) >= 1.0
    end
    let
        h1 = PerfectHeuristic(G,starts,goals)
        h2 = SoftConflictTable(G,start_times,starts,goals)
        # h = TieBreakerHeuristic(h1,h2)
    end
    let
        h = construct_composite_heuristic(
            NullHeuristic(),
            PerfectHeuristic(G,starts,goals)
        )
        cost = get_heuristic_cost(h,goals[1],starts[1])
        @test cost[1] == 0.0
        @test cost[2] ==  gdistances(G,starts[1])[goals[1]]
    end
    # let
    #     solver = CBS_Solver()
    #     G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    #     starts = [CBS.State(1,0),CBS.State(2,0)]
    #     goals = [CBS.State(vtx=6),CBS.State(vtx=5)]
    #     cost_model = construct_composite_cost_model(
    #         FullCostModel(sum,NullCost()),
    #         FullCostModel(sum,TravelTime())
    #     )
    #     heuristic = construct_composite_heuristic(
    #         NullHeuristic(),
    #         PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    #     )
    #     env = CBS.LowLevelEnv(graph=G,cost_model=cost_model,h=heuristic)
    #     mapf = initialize_mapf(env,starts,goals)
    #     solution, cost = CRCBS.solve!(solver,mapf)
    # end
    # let
    #     env = GraphEnv(G)
    #     h1 = PerfectHeuristic(G,starts,goals)
    #     h2 = SoftConflictTable(G,start_times,starts,goals)
    #     h = CompositeHeuristic(h1,h2)
    #     @test get_heuristic_cost(h1,goals[1],starts[1]) == gdistances(G,starts[1])[goals[1]]
    #     @test get_heuristic_cost(h2,starts[1],1) >= 1.0
    # end
end

# let
#     solver = CBS_Solver()
#     G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
#     env = CBS.LowLevelEnv(graph=G,h=TieBreakerHeuristic(),initial_cost=TieBreakerCost())
#     starts = [CBS.State(1,0),CBS.State(2,0)]
#     goals = [CBS.State(vtx=6),CBS.State(vtx=5)]
#     mapf = initialize_mapf(env,starts,goals)
#     node = CBS.initialize_root_node(mapf)
#     CRCBS.solve!(solver,mapf)
# end
