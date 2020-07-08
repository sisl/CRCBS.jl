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
    S = GraphEnv.State
    A = GraphEnv.Action
    P = PathNode{S,A}
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [GraphEnv.State(1,0),GraphEnv.State(2,0)]
    goals = [GraphEnv.State(vtx=5),GraphEnv.State(vtx=6)]
    heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    env = GraphEnv.LowLevelEnv(graph=G,heuristic=heuristic)
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
    let
        state_constraint = StateConstraint(1,2,3)
        @test CRCBS.get_agent_id(state_constraint) == state_constraint.a
        action_constraint = ActionConstraint(1,2,3)
        @test CRCBS.get_agent_id(action_constraint) == action_constraint.a
    end
    let
        constraints = ConstraintTable{P}(a = 1)
        @test get_agent_id(constraints) == constraints.a
        # add a constraint whose agent id does not match (should throw an error)
        @test_throws AssertionError add_constraint!(constraints, StateConstraint(get_agent_id(constraints)+1,P(),2))
        @test length(constraints.state_constraints) == 0
        @test length(constraints.action_constraints) == 0
        # add a constraint whose agent id DOES match
        add_constraint!(constraints, StateConstraint(get_agent_id(constraints),P(),1))
        @test length(constraints.state_constraints) == 1
        @test length(constraints.action_constraints) == 0
        # add a constraint whose agent id DOES match
        add_constraint!(constraints, ActionConstraint(get_agent_id(constraints),P(),1))
        @test length(constraints.action_constraints) == 1
    end
    let
        conflict = Conflict{P,P}(
            STATE_CONFLICT,1,2,P(State(1),Action(1),State(1)),P(State(1),Action(1),State(1)),1
        )
        constraints = generate_constraints_from_conflict(conflict)
        @test length(constraints) == 2
        for c in constraints
            @test typeof(c) <: StateConstraint
        end
        conflict = Conflict{P,P}(
            ACTION_CONFLICT,1,2,P(State(1),Action(1),State(1)),P(State(1),Action(1),State(1)),1
        )
        constraints = generate_constraints_from_conflict(conflict)
        @test length(constraints) == 2
        for c in constraints
            @test typeof(c) <: ActionConstraint
        end
    end
end
