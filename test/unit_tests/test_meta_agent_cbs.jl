let
    N = 2
    env = MetaAgentCBS.LowLevelEnv{CBS.State,CBS.Action,CBS.LowLevelEnv}([CBS.LowLevelEnv() for i in 1:N])
    state = MetaAgentCBS.State([CBS.State() for i in 1:N])
    action = MetaAgentCBS.Action([CBS.Action() for i in 1:N])

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
    cbs_env = CBS.LowLevelEnv(graph=G)

    env = MetaAgentCBS.LowLevelEnv{CBS.State,CBS.Action,CBS.LowLevelEnv}([cbs_env, cbs_env])
    state = MetaAgentCBS.State([CBS.State(vtx=1), CBS.State(vtx=2)])
    action_count = 0
    for a in get_possible_actions(env, state)
        action_count += 1
    end
    @test action_count == 9
end
let
    # solver = CBS.CBSsolver()
    solver = MetaAgentCBS_Solver(1)
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    mapf = MetaMAPF(MAPF(
        G,
        [CBS.State(1,0),CBS.State(2,0)],
        [CBS.State(vtx=6),CBS.State(vtx=5)]
        ))
    node = initialize_root_node(mapf)
    env = build_env(mapf,node,[1,2])
    CRCBS.solve!(solver,mapf)
end
