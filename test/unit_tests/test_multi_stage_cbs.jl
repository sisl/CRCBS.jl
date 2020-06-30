let
    s = MultiStageCBS.State(vtx=1)
    @test states_match(s,s)
    a = MultiStageCBS.Action()
    env = MultiStageCBS.LowLevelEnv()

    a = CRCBS.wait(s)
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
end
let
    # solver = MultiStageCBS.CBS_Solver()
    solver = CBS_Solver()
    vtx_grid = initialize_regular_vtx_grid(;n_obstacles_x=2,n_obstacles_y=2,obs_offset = [1;1])
    #  1   2   3   4   5   6   7   8   9  10
    # 11  12  13  14  15  16  17  18  19  20
    # 21  22   0   0  23  24   0   0  25  26
    # 27  28   0   0  29  30   0   0  31  32
    # 33  34  35  36  37  38  39  40  41  42
    # 43  44  45  46  47  48  49  50  51  52
    # 53  54   0   0  55  56   0   0  57  58
    # 59  60   0   0  61  62   0   0  63  64
    # 65  66  67  68  69  70  71  72  73  74
    # 75  76  77  78  79  80  81  82  83  84
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [
        MultiStageCBS.State(vtx=1,stage=1,t=0),
        MultiStageCBS.State(vtx=2,stage=1,t=0),
        # MultiStageCBS.State(vtx=26,stage=1,t=10),
        # MultiStageCBS.State(vtx=19,stage=1,t=10),
        # MultiStageCBS.State(vtx=41,stage=1,t=4)
        ]
    goals = [
        [MultiStageCBS.State(vtx=14),MultiStageCBS.State(vtx=81)],
        [MultiStageCBS.State(vtx=5),MultiStageCBS.State(vtx=55)],
        # [MultiStageCBS.State(vtx=60),MultiStageCBS.State(vtx=52)],
        # [MultiStageCBS.State(vtx=61)],
        # [MultiStageCBS.State(vtx=32),MultiStageCBS.State(vtx=1)]
        ]
    let
        heuristic = MultiStagePerfectHeuristic(
            G, map(g->map(s->s.vtx, g), goals)
        )
        env = MultiStageCBS.LowLevelEnv(graph=G,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
        node = initialize_root_node(mapf)
        solution, cost = CRCBS.solve!(solver,mapf;verbose=true)
    end
    let
        env = MultiStageCBS.LowLevelEnv(
            graph=G,
            cost_model = construct_composite_cost_model(
                FullCostModel(sum,NullCost()),
                FullCostModel(sum,TravelTime())
                ),
            heuristic = construct_composite_heuristic(
                NullHeuristic(),
                MultiStagePerfectHeuristic(G, map(g->map(s->s.vtx, g), goals))
                )
            )
        mapf = MAPF(env,starts,goals)
        node = initialize_root_node(mapf)
        solution, cost = CRCBS.solve!(solver,mapf);
    end
end
