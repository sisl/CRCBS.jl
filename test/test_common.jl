module CommonTests

using Parameters, CRCBS

@with_kw struct State
    v::Int = -1
end
@with_kw struct Action
    v::Int = -1
end
CRCBS.states_match(s1::State,s2::State) = (s1.v == s2.v)

function CRCBS.detect_conflicts!(
    c::ConflictTable,
    n1::PathNode{State,Action},
    n2::PathNode{State,Action},
    i::Int,
    j::Int,
    t::Int)
    if n1.sp == n2.sp
        add_conflict!(c,Conflict(STATE_CONFLICT,i,j,n1,n2,t))
    elseif n1.a == n2.a
        add_conflict!(c,Conflict(ACTION_CONFLICT,i,j,n1,n2,t))
    end
end

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
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
    goals = [State(),State()]
    starts = [State(),State()]
    paths = [Path([P(),P()]),Path([P(),P()])]
    @test is_valid(paths[1],starts[1],goals[1])
    @test is_valid(paths,starts,goals)
end
let
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
    conflict = Conflict{P,P}()
    CommonTests.test_conflict(conflict)
    @test !is_valid(conflict)

    conflict = DefaultConflict()
    CommonTests.test_conflict(conflict)
    @test !is_valid(conflict)
    @test !(DefaultConflict() < DefaultConflict())
end
let
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
    conflict_table = ConflictTable()
    conflict = Conflict{P,P}(
        STATE_CONFLICT,1,2,P(State(1),Action(1),State(1)),P(State(1),Action(1),State(1)),1
    )
    add_conflict!(conflict_table,conflict)
    @test(length(conflict_table.state_conflicts) == 1)

    conflict = Conflict{P,P}(
        ACTION_CONFLICT,1,2,P(State(1),Action(1),State(1)),P(State(1),Action(1),State(1)),1
    )
    add_conflict!(conflict_table,conflict)
    @test(length(conflict_table.action_conflicts) == 1)

    get_conflicts(conflict_table,1,2)
    get_conflicts(conflict_table,2,1)
end
let
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
    goals = [State(),State()]
    starts = [State(),State()]
    paths = [Path([P(),P()]),Path([P(),P()])]

    @test_throws ErrorException detect_state_conflict(paths[1],paths[2],1)
    @test_throws ErrorException detect_action_conflict(paths[1],paths[2],1)
end
let
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
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
    conflict_table = ConflictTable()
    detect_conflicts!(conflict_table,paths[1],paths[2],1,2)

    @test length(conflict_table.state_conflicts[(1,2)]) == 1
    @test length(conflict_table.action_conflicts[(1,2)]) == 0
    @test conflict_type(get_next_conflict(conflict_table)) == STATE_CONFLICT
    reset_conflict_table!(conflict_table,2,1)
    @test length(conflict_table.state_conflicts[(1,2)]) == 0

    conflict_table = ConflictTable()
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
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
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
    constraints = ConstraintTable(a = 1)
    @test get_agent_id(constraints) == constraints.a
    # add a constraint whose agent id does not match (should throw an error)
    @test_throws ErrorException add_constraint!(constraints, StateConstraint(get_agent_id(constraints)+1,2,2))
    @test length(constraints.state_constraints) == 0
    @test length(constraints.action_constraints) == 0
    # add a constraint whose agent id DOES match
    add_constraint!(constraints, StateConstraint(get_agent_id(constraints),1,1))
    @test length(constraints.state_constraints) == 1
    @test length(constraints.action_constraints) == 0
    # add a constraint whose agent id DOES match
    add_constraint!(constraints, ActionConstraint(get_agent_id(constraints),1,1))
    @test length(constraints.action_constraints) == 1
end
let
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
    node = ConstraintTreeNode{State,Action}()
    node.constraints[1] = ConstraintTable(a = 1)
    @test get_cost(node) == 0
    get_constraints(node,1)
    @test add_constraint!(node,StateConstraint(1,1,1))
    @test add_constraint!(node,ActionConstraint(1,1,1))
    @test length(get_constraints(node,1).state_constraints) == 1
    @test length(get_constraints(node,1).action_constraints) == 1

end
let
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    Action = CommonTests.Action
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
