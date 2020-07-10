module CommonTests

using Parameters, CRCBS

@with_kw struct State
    v::Int = -1
end
@with_kw struct Action
    v::Int = -1
end
CRCBS.states_match(s1::State,s2::State) = (s1.v == s2.v)
CRCBS.detect_state_conflict(n1::P,n2::P) where {P<:PathNode{State,Action}} = get_sp(n1) == get_sp(n2)
CRCBS.detect_action_conflict(n1::P,n2::P) where {P<:PathNode{State,Action}} = get_a(n1) == get_a(n2)

function test_conflict(conflict::Conflict)
    CRCBS.conflict_type(conflict)
    CRCBS.agent1_id(conflict)
    CRCBS.agent2_id(conflict)
    CRCBS.node1(conflict)
    CRCBS.node2(conflict)
    CRCBS.state1(conflict)
    CRCBS.state2(conflict)
    CRCBS.action1(conflict)
    CRCBS.action2(conflict)
    CRCBS.next_state1(conflict)
    CRCBS.next_state2(conflict)
    CRCBS.time_of(conflict)
end

end #module

let
    S = CBSEnv.State
    A = CBSEnv.Action
    P = PathNode{S,A}
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [CBSEnv.State(1,0),CBSEnv.State(2,0)]
    goals = [CBSEnv.State(vtx=5),CBSEnv.State(vtx=6)]
    heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    env = CBSEnv.LowLevelEnv(graph=G,heuristic=heuristic)
    mapf = MAPF(env,starts,goals)
    let
        root_node = initialize_root_node(mapf)
        paths = [
            Path([P(S(1,0),A(Edge(1,2),0),S(2,1))]),
            Path([P(S(2,0),A(Edge(2,2),0),S(2,1))]),
        ]
        set_solution_path!(root_node.solution,paths[1],1)
        set_solution_path!(root_node.solution,paths[2],2)
        child1 = initialize_child_search_node(root_node)
        child2 = initialize_child_search_node(root_node)
        detect_conflicts!(child1,[1])
        @test count_conflicts(child1) == 1
        @test count_conflicts(child1,1,2) == 1
        @test count_conflicts(child1,2,1) == 1
        @test count_conflicts(child1,[1,2],[1,2]) == 1

        @test count_conflicts(child2) == 0
        @test count_conflicts(root_node) == 0

        @test convert_to_vertex_lists(
            get_paths(child1.solution)[1]
            ) == convert_to_vertex_lists(
            get_paths(root_node.solution)[1]
            )

        set_solution_path!(child1.solution,Path([P()]),1)
        @test convert_to_vertex_lists(
            get_paths(child1.solution)[1]
            ) != convert_to_vertex_lists(
            get_paths(root_node.solution)[1]
            )

        detect_conflicts!(child1,[1])
        # reset_conflict_table!(child1,1,2)
        @test count_conflicts(child1) == 0
    end
end

let
    # P = PathNode{CommonTests.State,CommonTests.Action}
    # State = CommonTests.State
    # Action = CommonTests.Action

    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action

    let


        starts = [State(1),State(2)]
        goals = [State(3),State(4)]
        paths = [
            Path([
                P(State(1),Action(1),State(2)),
                P(State(2),Action(2),State(3))
                ]),
            Path([
                P(State(2),Action(2),State(3)),
                P(State(3),Action(3),State(4))
                ]),
            ]
        @test CRCBS.is_valid(paths[1],starts[1],goals[1])
        @test CRCBS.is_valid(paths,starts,goals)
    end
    let
        conflict = Conflict{P,P}()
        CommonTests.test_conflict(conflict)
        @test !CRCBS.is_valid(conflict)

        conflict = DefaultConflict()
        CommonTests.test_conflict(conflict)
        @test !CRCBS.is_valid(conflict)
        @test !(DefaultConflict() < DefaultConflict())
    end
    let
        conflict_table = ConflictTable{Conflict{P,P}}()
        @test count_conflicts(conflict_table) == 0
        conflict = Conflict{P,P}(
            STATE_CONFLICT,1,2,P(State(1),Action(1),State(1)),P(State(1),Action(1),State(1)),1
        )
        add_conflict!(conflict_table,conflict)
        @test count_conflicts(conflict_table) == 1
        @test count_conflicts(conflict_table,1,2) == 1

        conflict = Conflict{P,P}(
            ACTION_CONFLICT,1,2,P(State(1),Action(1),State(1)),P(State(1),Action(1),State(1)),1
        )
        add_conflict!(conflict_table,conflict)
        @test count_conflicts(conflict_table) == 2

        get_conflicts(conflict_table,1,2)
        get_conflicts(conflict_table,2,1)
        reset_conflict_table!(conflict_table,1,2)
        @test count_conflicts(conflict_table) == 0
    end
    let
        goals = [State(),State()]
        starts = [State(),State()]
        paths = [Path([P(),P()]),Path([P(),P()])]

        @test detect_state_conflict(paths[1],paths[2],1)
        @test detect_action_conflict(paths[1],paths[2],1)
    end
    let
        goals = [State(1),State(4)]
        starts = [State(5),State(2)]
        paths = [
            Path([
                P(State(1),Action(1),State(2)),
            ]),
            Path([
                P(State(3),Action(2),State(2)),
            ])
            ]
        conflict_table = ConflictTable{Conflict{P,P}}()
        detect_conflicts!(conflict_table,paths[1],paths[2],1,2)

        @test length(conflict_table.state_conflicts[(1,2)]) == 1
        @test length(conflict_table.action_conflicts[(1,2)]) == 0
        @test conflict_type(get_next_conflict(conflict_table)) == STATE_CONFLICT
        reset_conflict_table!(conflict_table,2,1)
        @test length(conflict_table.state_conflicts[(1,2)]) == 0

        conflict_table = ConflictTable{Conflict{P,P}}()
        paths = [
            Path([
                P(State(1),Action(1),State(2)),
            ]),
            Path([
                P(State(3),Action(1),State(4)),
            ])
            ]
        detect_conflicts!(conflict_table,paths[1],paths[2],1,2)
        @test length(conflict_table.action_conflicts[(1,2)]) == 1
        @test conflict_type(get_next_conflict(conflict_table)) == ACTION_CONFLICT

        reset_conflict_table!(conflict_table,1,2) # should do nothing
        @test length(conflict_table.action_conflicts[(1,2)]) == 0
    end
    let
        goals = [State(1),State(4)]
        starts = [State(5),State(2)]
        paths = [
            Path([
                P(State(1),Action(1),State(2)),
            ]),
            Path([
                P(State(3),Action(2),State(2)),
            ])
            ]
        conflict_table = detect_conflicts(paths)
        @test length(conflict_table.state_conflicts[(1,2)]) == 1
        @test length(conflict_table.action_conflicts[(1,2)]) == 0
        @test conflict_type(get_next_conflict(conflict_table)) == STATE_CONFLICT

        # check if new non-conflicting paths will remove the conflicts previously discovered
        paths = [Path([P(State(1),Action(1),State(2))]),
                Path([P(State(3),Action(2),State(4))])]
        detect_conflicts!(conflict_table,paths)
        @test length(conflict_table.state_conflicts[(1,2)]) == 0
        @test length(conflict_table.action_conflicts[(1,2)]) == 0
    end
end
let
    S = CBSEnv.State
    A = CBSEnv.Action
    P = PathNode{S,A}

    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    env = CBSEnv.LowLevelEnv(graph=G,agent_idx=1)
    isa(state_space_trait(env),DiscreteSpace)
    let
        for v in vertices(G)
            s = S(v,1)
            idx,t = serialize(env,s,s.t)
            sp, _ = deserialize(env,S(),idx,t)
            @test get_vtx(s) == get_vtx(sp)
        end
        for e in edges(G)
            a = A(e,1)
            idx,t = serialize(env,a,1)
            ap, _ = deserialize(env,A(),idx,t)
            @test get_e(a) == get_e(ap)
        end
    end
    let
        for c in [state_constraint(1,2,3),action_constraint(1,2,3)]
            @test CRCBS.get_agent_id(c) == c.a
        end
    end
    let
        for constraints in [
            DiscreteConstraintTable(env,1),
            ConstraintTable{P}(a = 1)
            ]
            # add a constraint whose agent id does not match (should throw an error)
            s = S(1,1)
            a = A(Edge(1,2),1)
            sp = S(2,2)
            c = state_constraint(get_agent_id(constraints)+1,P(),2)
            @test_throws AssertionError add_constraint!(env,constraints,c)
            @test_throws AssertionError CRCBS.has_constraint(env,constraints,c)
            # add a constraint whose agent id DOES match
            c = state_constraint(get_agent_id(constraints),P(s,a,sp),2)
            add_constraint!(env,constraints,c)
            @test CRCBS.has_constraint(env,constraints,c)
            sc,ac = CRCBS.search_constraints(env,constraints,get_path_node(c))
            @test length(sc) == 1
            @test length(ac) == 0
            # # add a constraint whose agent id DOES match
            c = action_constraint(get_agent_id(constraints),P(s,a,sp),2)
            add_constraint!(env,constraints,c)
            @test CRCBS.has_constraint(env,constraints,c)
            sc,ac = CRCBS.search_constraints(env,constraints,get_path_node(c))
            @test length(sc) == 1
            @test length(ac) == 1
            # # query sorted constraints
            c = sorted_state_constraints(env,constraints)[1]
            @test get_sp(c) == sp
            c = sorted_action_constraints(env,constraints)[1]
            @test get_a(c) == a
        end
    end
end
let
    S = CommonTests.State
    A = CommonTests.Action
    P = PathNode{S,A}
    let
        conflict = Conflict{P,P}(
            STATE_CONFLICT,1,2,P(S(1),A(1),S(1)),P(S(1),A(1),S(1)),1
        )
        constraints = generate_constraints_from_conflict(conflict)
        @test length(constraints) == 2
        for c in constraints
            @test is_state_constraint(c)
        end
        conflict = Conflict{P,P}(
            ACTION_CONFLICT,1,2,P(S(1),A(1),S(1)),P(S(1),A(1),S(1)),1
        )
        constraints = generate_constraints_from_conflict(conflict)
        @test length(constraints) == 2
        for c in constraints
            @test !is_state_constraint(c)
        end
    end
end
