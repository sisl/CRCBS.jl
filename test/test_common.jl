module CommonTests

using Parameters, CRCBS

@with_kw struct State
    v::Int = -1
end
@with_kw struct Action
    v::Int = -1
end
CRCBS.states_match(s1::State,s2::State) = (s1.v == s2.v)

@with_kw struct ConflictTable
    state_conflicts::Set{Conflict} = Set{Conflict}()
    action_conflicts::Set{Conflict} = Set{Conflict}()
end

function CRCBS.detect_conflicts!(c::ConflictTable,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)
    if n1 == n2
        add_conflict!(c,Conflict(STATE_CONFLICT,i,j,n1,n2,t))
    end
end

function add_conflict!(c::ConflictTable,conflict::Conflict)
    if conflict_type(conflict) == STATE_CONFLICT
        push!(c.state_conflicts, conflict)
        return true
    elseif conflict_type(conflict) == ACTION_CONFLICT
        push!(c.action_conflicts, conflict)
        return true
    end
    return false
end

end #module

let
    P = PathNode{CommonTests.State,CommonTests.Action}
    State = CommonTests.State
    goals = [State(),State()]
    starts = [State(),State()]
    paths = [[P(),P()],[P(),P()]]
    @test is_valid(paths[1],starts[1],goals[1])
    @test is_valid(paths,starts,goals)

    conflict = Conflict{P,P}()
    @test !is_valid(conflict)
    @test !is_valid(StateConflict{P,P}())
    @test !is_valid(ActionConflict{P,P}())

    conflict_table = CommonTests.ConflictTable()
    CommonTests.add_conflict!(conflict_table,conflict)
    detect_conflicts!(conflict_table,paths)

    constraint_dict = ConstraintDict(a = 1)
    state_constraint = StateConstraint{Int}(1,2,3)
    action_constraint = ActionConstraint{Int}(1,2,3)

    add_constraint!(constraint_dict, state_constraint)
    add_constraint!(constraint_dict, action_constraint)

    new_dict = merge(constraint_dict,constraint_dict)
    add_constraint!(new_dict, StateConstraint{Int}(1,2,3))

    node_dict = Dict{Int,ConstraintDict}()
    merge(node_dict,node_dict)

end
