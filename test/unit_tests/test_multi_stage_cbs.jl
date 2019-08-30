let
    s = MultiStageCBS.State(vtx=1)
    @test states_match(s,s)
    a = MultiStageCBS.Action()
    env = MultiStageCBS.LowLevelEnv()

    a = CRCBS.wait(s)
    # sp = get_next_state(s,a)
    # sp = get_next_state(env,s,a)
    # @test states_match(s,sp)
    # @test is_goal(env,s)
end
let
    s = MultiStageCBS.State()
    a = MultiStageCBS.Action()
    env = MultiStageCBS.LowLevelEnv()
    # get_next_state(s,a)
    # get_transition_cost(env,s,a,s)
end
let
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = MultiStageCBS.LowLevelEnv(graph=G)
    for a in get_possible_actions(env,MultiStageCBS.State(vtx=2))
        @test a.e.src == 2
        @test a.e.dst ∈ [1,2,3]
    end
end
let
    # TEST CONSTRAINTS
    S = MultiStageCBS.State
    A = MultiStageCBS.Action
    P = PathNode{S,A}
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = MultiStageCBS.LowLevelEnv(graph=G)

    paths = [
        Path{S,A,Float64}()
    ]
end
let
    P = PathNode{MultiStageCBS.State,MultiStageCBS.Action}()
    id1 = 1
    id2 = 2

    conflict_table = ConflictTable()
    conflict = Conflict(conflict_type=STATE_CONFLICT,agent1_id=id1,agent2_id=id2,node1=P,node2=P)
    @test is_valid(conflict)
    add_conflict!(conflict_table,conflict)
    @test length(conflict_table.state_conflicts) == 1
    conflict = Conflict(conflict_type=ACTION_CONFLICT,agent1_id=id1,agent2_id=id2,node1=P,node2=P)
    @test is_valid(conflict)
    add_conflict!(conflict_table,conflict)
    @test length(conflict_table.state_conflicts) == 1
    get_conflicts(conflict_table,id1,id2)
    conflict = MultiStageCBS.get_next_conflict(conflict_table)
    @test !(conflict == nothing)
end
let
    graph = Graph(3)
    add_edge!(graph,1,2)
    add_edge!(graph,1,3)
    constraints = ConstraintTable()
    env = MultiStageCBS.LowLevelEnv() #graph=graph,constraints=constraints)
    # s = State(1)
    # for a in get_possible_actions(env,s)
    #     @test typeof(a) == Action
    # end
end
let
    # solver = MultiStageCBS.CBS_Solver()
    solver = CBS_Solver()
    G = initialize_regular_grid_graph(;n_obstacles_x=2,n_obstacles_y=2,obs_offset = [2;2])
    starts = [   MultiStageCBS.State(vtx=1,stage=1,t=0),
        MultiStageCBS.State(vtx=2,stage=1,t=0),
        MultiStageCBS.State(vtx=26,stage=1,t=10),
        MultiStageCBS.State(vtx=167,stage=1,t=10),
        MultiStageCBS.State(vtx=41,stage=1,t=4)    ]
    goals = [   [MultiStageCBS.State(vtx=6),MultiStageCBS.State(vtx=171)],
        [MultiStageCBS.State(vtx=5),MultiStageCBS.State(vtx=90)],
        [MultiStageCBS.State(vtx=100),MultiStageCBS.State(vtx=175)],
        [MultiStageCBS.State(vtx=85)],
        [MultiStageCBS.State(vtx=91),MultiStageCBS.State(vtx=1)]    ]
    env = MultiStageCBS.LowLevelEnv(graph=G)
    mapf = initialize_mapf(env,starts,goals)
    node = initialize_root_node(mapf)
    solution, cost = CRCBS.solve!(solver,mapf);
end
