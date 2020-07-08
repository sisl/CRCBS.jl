let
    s = GraphEnv.State()
    @test states_match(s,s)
    a = GraphEnv.Action()
    env = GraphEnv.LowLevelEnv()

    a = CRCBS.wait(s)
    sp = get_next_state(s,a)
    sp = get_next_state(env,s,a)
    @test states_match(s,sp)
    @test is_goal(env,s)
end
let
    s = GraphEnv.State()
    a = GraphEnv.Action()
    env = GraphEnv.LowLevelEnv()
    get_next_state(s,a)
    get_transition_cost(env,s,a,s)
end
let
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = GraphEnv.LowLevelEnv(graph=G)
    for a in get_possible_actions(env,GraphEnv.State(2,0))
        @test a.e.src == 2
        @test a.e.dst ∈ [1,2,3]
    end
end
# test conflict detection
let
    path1 = Path{GraphEnv.State,GraphEnv.Action,Float64}()
    push!(path1,PathNode(
        GraphEnv.State(1,0),
        GraphEnv.Action(Edge(1,2),0),
        GraphEnv.State(2,1)
    ))
    # push!(path1,PathNode(
    #     GraphEnv.State(2,1),
    #     GraphEnv.Action(Edge(2,3),1),
    #     GraphEnv.State(3,2)
    # ))
    path2 = Path{GraphEnv.State,GraphEnv.Action,Float64}()
    push!(path2,PathNode(
        GraphEnv.State(2,1),
        GraphEnv.Action(Edge(2,3),1),
        GraphEnv.State(3,2)
    ))
    conflict_table = detect_conflicts([path1,path2])
end
let
    # TEST CONSTRAINTS
    S = GraphEnv.State
    A = GraphEnv.Action
    P = PathNode{S,A}
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = GraphEnv.LowLevelEnv(graph=G)

    paths = [
        Path{S,A,Float64}()
    ]
end
let
    P = PathNode{GraphEnv.State,GraphEnv.Action}()
    id1 = 1
    id2 = 2

    N = node_type(P)
    conflict_table = ConflictTable{Conflict{N,N}}()
    conflict = Conflict(conflict_type=STATE_CONFLICT,agent1_id=id1,agent2_id=id2,node1=P,node2=P)
    @test CRCBS.is_valid(conflict)
    add_conflict!(conflict_table,conflict)
    @test length(conflict_table.state_conflicts) == 1
    conflict = Conflict(conflict_type=ACTION_CONFLICT,agent1_id=id1,agent2_id=id2,node1=P,node2=P)
    @test CRCBS.is_valid(conflict)
    add_conflict!(conflict_table,conflict)
    @test length(conflict_table.state_conflicts) == 1
    get_conflicts(conflict_table,id1,id2)
    conflict = GraphEnv.get_next_conflict(conflict_table)
    @test !(conflict == nothing)
end
let
    GraphEnv.LowLevelEnv(cost_model=SumOfTravelTime())
end
let
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    T = 10
    n_agents = 2
    env = GraphEnv.LowLevelEnv(graph=G,agent_idx=1,cost_model=HardConflictCost(G,T,n_agents))
    # path1 = Path{GraphEnv.State,GraphEnv.Action,Float64}(s0=GraphEnv.State(5,0))
    # path1 = Path{GraphEnv.State,GraphEnv.Action,Float64}(s0=GraphEnv.State(5,0))
    set_path!(get_cost_model(env),2,[2,6,10,14])
    @test get_transition_cost(env,GraphEnv.State(5,0),GraphEnv.Action(Edge(5,6),1),GraphEnv.State(6,1)) == 1
end
# solve!
let
    vtx_grid = initialize_regular_vtx_grid(;n_obstacles_x=1,n_obstacles_y=1)
    #  1   2   3   4   5   6
    #  7   8   9  10  11  12
    # 13  14   0   0  15  16
    # 17  18   0   0  19  20
    # 21  22  23  24  25  26
    # 27  28  29  30  31  32
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [GraphEnv.State(1,0),GraphEnv.State(2,0)]
    goals = [GraphEnv.State(vtx=6),GraphEnv.State(vtx=5)]
    heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    env = GraphEnv.LowLevelEnv(graph=G,heuristic=heuristic)
    mapf = MAPF(env,starts,goals)
    default_solution(mapf)
    println("Testing CBS")
    # low_level_search
    # let
    #     solver = CBSSolver()
    #     node = GraphEnv.initialize_root_node(mapf)
    #     initialize_child_search_node(node)
    #     low_level_search!(solver,mapf,node)
    # end
    # high_level_search
    let
        solver = CBSSolver()
        set_iteration_limit!(solver,0)
        @test_throws SolverException CRCBS.solve!(solver,mapf)
    end
    let
        solver = CBSSolver()
        set_verbosity!(solver,global_verbosity())
        set_iteration_limit!(solver,100)
        solution, cost = CRCBS.solve!(solver,mapf)
        @test cost == 10
    end
    # with CompositeCost and CompositeHeuristic
    let
        cost_model = construct_composite_cost_model(
            FullCostModel(sum,NullCost()),
            FullCostModel(sum,TravelTime())
        )
        heuristic = construct_composite_heuristic(
            NullHeuristic(),
            PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
        )
        env = GraphEnv.LowLevelEnv(graph=G,cost_model=cost_model,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        solution, cost = CRCBS.solve!(solver,mapf)
    end
    let
        deadline = ne(G)+1.0
        cost_model = construct_composite_cost_model(
            FullDeadlineCost(DeadlineCost(deadline)),
            FullCostModel(sum,NullCost()),
            SumOfTravelTime()
        )
        heuristic_model = construct_composite_heuristic(
            PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals)),
            HardConflictHeuristic(G,ne(G),num_agents(mapf)),
            PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals)),
        )
        env = GraphEnv.LowLevelEnv(graph=G,cost_model=cost_model,heuristic=heuristic_model)
        mapf = MAPF(env,starts,goals)
        default_solution(mapf)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        solution, cost = CRCBS.solve!(solver,mapf)
    end
end
