let
    s = CBS.State()
    @test states_match(s,s)
    a = CBS.Action()
    env = CBS.LowLevelEnv()

    a = CRCBS.wait(s)
    sp = get_next_state(s,a)
    sp = get_next_state(env,s,a)
    @test states_match(s,sp)
    @test is_goal(env,s)
end
let
    s = CBS.State()
    a = CBS.Action()
    env = CBS.LowLevelEnv()
    get_next_state(s,a)
    get_transition_cost(env,s,a,s)
end
let
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = CBS.LowLevelEnv(graph=G)
    for a in get_possible_actions(env,CBS.State(2,0))
        @test a.e.src == 2
        @test a.e.dst ∈ [1,2,3]
    end
end
# test conflict detection
let
    path1 = Path{CBS.State,CBS.Action,Float64}()
    push!(path1,PathNode(
        CBS.State(1,0),
        CBS.Action(Edge(1,2),0),
        CBS.State(2,1)
    ))
    # push!(path1,PathNode(
    #     CBS.State(2,1),
    #     CBS.Action(Edge(2,3),1),
    #     CBS.State(3,2)
    # ))
    path2 = Path{CBS.State,CBS.Action,Float64}()
    push!(path2,PathNode(
        CBS.State(2,1),
        CBS.Action(Edge(2,3),1),
        CBS.State(3,2)
    ))
    conflict_table = detect_conflicts([path1,path2])
end
let
    # TEST CONSTRAINTS
    S = CBS.State
    A = CBS.Action
    P = PathNode{S,A}
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = CBS.LowLevelEnv(graph=G)

    paths = [
        Path{S,A,Float64}()
    ]
end
let
    P = PathNode{CBS.State,CBS.Action}()
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
    conflict = CBS.get_next_conflict(conflict_table)
    @test !(conflict == nothing)
end
let
    graph = Graph(3)
    add_edge!(graph,1,2)
    add_edge!(graph,1,3)
    constraints = ConstraintTable()
    env = CBS.LowLevelEnv() #graph=graph,constraints=constraints)
end
let
    env = CBS.LowLevelEnv(cost_model=SumOfTravelTime())
end
# solve!
let
    solver = CBS_Solver()
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    starts = [CBS.State(1,0),CBS.State(2,0)]
    goals = [CBS.State(vtx=6),CBS.State(vtx=5)]
    num_agents = 2
    # low_level_search
    let
        heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
        env = CBS.LowLevelEnv(graph=G,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
        default_solution(mapf)
        node = CBS.initialize_root_node(mapf)
        initialize_child_search_node(node)
        low_level_search!(solver,mapf,node)
        # high_level_search
        solution, cost = CRCBS.solve!(solver,mapf)
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
        env = CBS.LowLevelEnv(graph=G,cost_model=cost_model,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
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
            HardConflictTable(G,ne(G),num_agents),
            PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals)),
        )
        env = CBS.LowLevelEnv(graph=G,cost_model=cost_model,heuristic=heuristic_model)
        mapf = MAPF(env,starts,goals)
        default_solution(mapf)
        solution, cost = CRCBS.solve!(solver,mapf)
    end
end
