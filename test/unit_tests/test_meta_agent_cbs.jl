let
    N = 2
    envs = [GraphEnv.LowLevelEnv() for i in 1:N]
    idxs = [1,2]
    env = MetaAgentCBS.construct_meta_env(envs,idxs)
    env = MetaAgentCBS.construct_meta_env(envs, idxs,get_cost_model(envs[1]))
    state = MetaAgentCBS.State([GraphEnv.State() for i in 1:N])
    action = MetaAgentCBS.Action([GraphEnv.Action() for i in 1:N])

    @test states_match(state, state)
    @test states_match(state, get_next_state(state, CRCBS.wait(state)))
    @test states_match(state, get_next_state(env, state, CRCBS.wait(state)))

    get_transition_cost(env, state, CRCBS.wait(state), get_next_state(state, CRCBS.wait(state)))
end
let
    G = Graph(3)
    add_edge!(G,1,1)
    add_edge!(G,1,2)
    add_edge!(G,1,3)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    add_edge!(G,3,3)
    cbs_env = GraphEnv.LowLevelEnv(graph=G)

    env = MetaAgentCBS.construct_meta_env([cbs_env, cbs_env],[1,2])
    state = MetaAgentCBS.State([GraphEnv.State(vtx=1), GraphEnv.State(vtx=2)])
    action_count = 0
    for a in get_possible_actions(env, state)
        action_count += 1
    end
    @test action_count == 9
end
let
    N = 2
    envs = [GraphEnv.LowLevelEnv() for i in 1:N]
    env = MetaAgentCBS.construct_meta_env(envs,[1,2], get_cost_model(envs[1]))
    get_cost_model(env)
    get_initial_cost(env)
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    starts = [GraphEnv.State(1,0),GraphEnv.State(2,0)]
    goals = [GraphEnv.State(vtx=6),GraphEnv.State(vtx=5)]
    heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    env = GraphEnv.LowLevelEnv(graph=G,heuristic=heuristic)
    mapf = MAPF(env,starts,goals)
    let
        solver = MetaAgentCBS_Solver(beta=0)
        node = initialize_root_node(solver,mapf)
        # Combine groups artificially
        group_idx = 1
        node.solution.group_idxs[group_idx] = [1,2]
        deleteat!(node.solution.group_idxs,2)
        CRCBS.build_env(solver,mapf,node,group_idx)
        @test low_level_search!(solver, mapf, node, [group_idx])
        # @show convert_to_vertex_lists(node.solution.solution)
        detect_conflicts!(node,[group_idx])
        for i in node.solution.group_idxs[group_idx]
            for j in node.solution.group_idxs[group_idx]
                @test count_conflicts(node,i,j) == 0
            end
        end
    end
    let
        solver = MetaAgentCBS_Solver(beta=1)
        node = initialize_root_node(solver,mapf)
        # Combine groups artificially
        env = CRCBS.build_env(solver,mapf,node,2)
        get_start(mapf,env,2)
        low_level_search!(solver, mapf, node)
        # @show convert_to_vertex_lists(node.solution.solution)
        group_idx = 1
        detect_conflicts!(node,[group_idx])
        for i in node.solution.group_idxs[group_idx]
            for j in node.solution.group_idxs[group_idx]
                @test count_conflicts(node,i,j) == 0
            end
        end
    end
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    starts = [GraphEnv.State(1,0),GraphEnv.State(2,0)]
    goals = [GraphEnv.State(vtx=6),GraphEnv.State(vtx=5)]
    heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    env = GraphEnv.LowLevelEnv(graph=G,heuristic=heuristic)
    mapf = MAPF(env,starts,goals)
    solver = MetaAgentCBS_Solver(beta=1)
    set_verbosity!(solver,global_verbosity())
    solution, cost = CRCBS.solve!(solver,mapf)
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    starts = [MultiStageCBS.State(vtx=1,stage=1,t=0),MultiStageCBS.State(vtx=2,stage=1,t=0)]
    goals = [[MultiStageCBS.State(vtx=6)],[MultiStageCBS.State(vtx=5)]]
    heuristic = MultiStagePerfectHeuristic(
        G, map(g->map(s->s.vtx, g), goals)
    )
    env = MultiStageCBS.LowLevelEnv(graph=G,heuristic=heuristic)
    mapf = MAPF(env,starts,goals)
    solver = MetaAgentCBS_Solver(beta=1)
    set_verbosity!(solver,global_verbosity())
    set_iteration_limit!(solver,10)
    solution, cost = CRCBS.solve!(solver,mapf)
end
