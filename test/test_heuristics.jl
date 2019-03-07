let
    h = PerfectHeuristic([1.0,2.0,3.0])
    @test h(2) == 2.0
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    mapf = MAPF(G.graph, [1,2], [5,6])
    h = PerfectHeuristic(mapf,1)
end
