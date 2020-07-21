export init_mapf_problem

function init_mapf_problem(graph,start_vtxs,goal_vtxs,cost_model=SumOfTravelTime(),args...)
    env = CBSEnv.LowLevelEnv(
        graph = graph,
        cost_model = cost_model,
        heuristic = PerfectHeuristic(graph,start_vtxs,goal_vtxs)
    )
    starts = map(s->CBSEnv.State(vtx=s,t=0),start_vtxs)
    goals = map(s->CBSEnv.State(vtx=s,t=0),goal_vtxs)
    MAPF(env,starts,goals)
end

export
    init_mapf_1,
    init_mapf_2,
    init_mapf_3

function init_mapf_1(args...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    starts = [1,4]
    goals = [13,16]
    # graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

"""
    switch places
"""
function init_mapf_2(args...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    starts = [1,4]
    goals = [4,1]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

"""
    congested
"""
function init_mapf_3(args...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    starts = [1,2,3,4,5,6,7,8]
    goals = [13,14,15,16,9,10,11,12]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end
