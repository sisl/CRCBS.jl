let
    PerfectHeuristic()
    SoftConflictTable()
    TieBreakerHeuristic()
    TieBreakerCost()
    TieBreakerCost(0)
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
        h = TieBreakerHeuristic(h1,h2)
    end
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
