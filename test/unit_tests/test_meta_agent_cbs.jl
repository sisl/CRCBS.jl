let
    N = 2
    env = MetaAgentCBS.LowLevelEnv{CBS.State,CBS.Action,DefaultPathCost,CBS.LowLevelEnv{DefaultPathCost,Graph}}(
        [CBS.LowLevelEnv{DefaultPathCost,Graph}() for i in 1:N])
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
    cbs_env = CBS.LowLevelEnv{DefaultPathCost,typeof(G)}(graph=G)

    env = MetaAgentCBS.LowLevelEnv{state_type(cbs_env),action_type(cbs_env),cost_type(cbs_env),typeof(cbs_env)}([cbs_env, cbs_env])
    state = MetaAgentCBS.State([CBS.State(vtx=1), CBS.State(vtx=2)])
    action_count = 0
    for a in get_possible_actions(env, state)
        action_count += 1
    end
    @test action_count == 9
end
let
    solver = MetaAgentCBS_Solver(1)
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    env = CBS.LowLevelEnv{DefaultPathCost,typeof(G)}(graph=G)
    starts = [CBS.State(1,0),CBS.State(2,0)]
    goals = [CBS.State(vtx=6),CBS.State(vtx=5)]
    mapf = initialize_mapf(env,starts,goals)

    CRCBS.solve!(solver,mapf)
end
let
    solver = MetaAgentCBS_Solver(1)
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    env = MultiStageCBS.LowLevelEnv{typeof(G)}(graph=G)
    starts = [MultiStageCBS.State(vtx=1,stage=1,t=0),MultiStageCBS.State(vtx=2,stage=1,t=0)]
    goals = [[MultiStageCBS.State(vtx=6)],[MultiStageCBS.State(vtx=5)]]
    mapf = initialize_mapf(env,starts,goals)

    CRCBS.solve!(solver,mapf)
end
