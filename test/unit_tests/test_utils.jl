let
    S = CBSEnv.State
    A = CBSEnv.Action
    vtx_grid = initialize_regular_vtx_grid(;n_obstacles_x=1,n_obstacles_y=1)
    #  1   2   3   4   5   6
    #  7   8   9  10  11  12
    # 13  14   0   0  15  16
    # 17  18   0   0  19  20
    # 21  22  23  24  25  26
    # 27  28  29  30  31  32
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [CBSEnv.State(1,0),CBSEnv.State(2,0)]
    goals = [CBSEnv.State(vtx=6),CBSEnv.State(vtx=5)]
    cost_model = SumOfTravelTime()
    env = CBSEnv.LowLevelEnv(graph=G,cost_model=cost_model)
    mapf = MAPF(env,starts,goals)

    get_infeasible_solution(mapf)

    solution = get_initial_solution(mapf)
    path = get_paths(solution)[1]
    for (t,path) in zip([4,5],get_paths(solution))
        extend_path!(env,path,t)
        @test length(path) == t
        @test get_cost(path) == length(path)
        trim_path!(env,path,t-1)
        @test length(path) == t-1
        @test get_cost(path) == length(path)
    end
    T = 2
    trim_solution!(env,solution,T)
    for path in get_paths(solution)
        @test length(path) == T
        @test get_cost(path) == T
    end
    @test get_cost(solution) == T*num_agents(mapf)


end
let
    mapf = init_mapf_1()
    env = mapf.env
    path = Path{CBSEnv.State,CBSEnv.Action,Float64}(
        s0 = CBSEnv.State(1,0)
    )
    for a in [
        CBSEnv.Action(Edge(1,2),1),
        CBSEnv.Action(Edge(2,3),1),
        CBSEnv.Action(Edge(3,4),1),
        ]
        s = get_final_state(path)
        sp = get_next_state(env,s,a)
        add_to_path!(path,env,s,a,sp)
    end
    get_initial_state(path)
    @show convert_to_vertex_lists(path)
    trim_path!(env,path,1)
    string(path.path_nodes[1])
    get_initial_state(path)
    @show convert_to_vertex_lists(path)
    extend_path!(env,path,3)
    @show convert_to_vertex_lists(path)
end
# let
#     v = [0,1,2,4,5,6]
#     L = length(v)
#     x = 3
#     @test find_index_in_sorted_array(v,x) == 4
#     insert_to_sorted_array!(v,x)
#     @test length(v) == L + 1
#     v[4] == x
# end
# let
#     v = [0,1,1,1,1,2]
#     L = length(v)
#     x = 1
#     @test find_index_in_sorted_array(v,x) == 2
# end
# let
#     v = Vector{Int}()
#     insert_to_sorted_array!(v,1)
#     @test length(v) == 1
# end
