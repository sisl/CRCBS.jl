module CBS

using Parameters, LightGraphs, CRCBS

@with_kw struct State
    vtx::Int        = -1 # vertex of graph
    t::Int          = -1
end
@with_kw struct Action
    e::Edge{Int}    = Edge(-1,-1)
    Δt::Int         = 1
end
@with_kw struct CBSLowLevelEnv{G <: AbstractGraph,C} <: AbstractLowLevelEnv{State,Action}
    graph::G                    = Graph()
    constraints::ConstraintDict = ConstraintDict()
    goal::State             = State()
    agent_idx::Int              = -1
end

CRCBS.states_match(s1::State,s2::State) = (s1.vtx == s2.vtx)
CRCBS.wait(s::State) = Action(e=Edge(s.vtx,s.vtx))

# get_possible_actions
struct ActionIter
    env::CBSLowLevelEnv
    s::Int # source state
    neighbor_list::Vector{Int} # length of target edge list
end
struct ActionIterState
    idx::Int # idx of target node
end
function Base.iterate(it::ActionIter)
    iter_state = ActionIterState(0)
    return iterate(it,iter_state)
end
function Base.iterate(it::ActionIter, iter_state::ActionIterState)
    iter_state = ActionIterState(iter_state.idx+1)
    if iter_state.idx > length(it.neighbor_list)
        return nothing
    end
    Action(e=Edge(it.s,it.neighbor_list[iter_state.idx])), iter_state
end
CRCBS.get_possible_actions(env::CBSLowLevelEnv,s::State) = ActionIter(env,s.vtx,outneighbors(env.graph,s.vtx))
CRCBS.get_next_state(env::CBSLowLevelEnv,s::State,a::Action) = State(a.dst)
CRCBS.get_next_state(s::State,a::Action) = State(a.dst)
CRCBS.get_transition_cost(env::CBSLowLevelEnv,s::State,a::Action,sp::State) = 1
function CRCBS.violates_constraints(env::CBSLowLevelEnv, path::Path{State,Action}, s::State, a::Action, sp::State)
    t = length(path) + 1
    if get(env.constraints.state_constraints,StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t),false)
        return true
    elseif get(env.constraints.action_constraints,ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t),false)
        return true
    end
    return false
end
CRCBS.check_termination_criteria(env::CBSLowLevelEnv,cost,path,s) = false

""" Type alias for a path through the graph """
const CBSPath = Path{State,Action}

""" Returns an invalid StateConflict """
# invalid_state_conflict() = StateConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
invalid_state_conflict() = Conflict{PathNode{State,State},PathNode{State,State}}(conflict_type=STATE_CONFLICT)

"""
    Detect a `StateConflict` between two CBS path nodes.
"""
function detect_state_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if n1.sp.vtx == n2.sp.vtx
        return true
    end
    return false
end

""" Returns an invalid ActionConflict """
# invalid_action_conflict() = ActionConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
invalid_action_conflict() = ActionConflict{State,State}()

"""
    Detect an `ActionConflict` between two CBS path nodes.
"""
function detect_action_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if (n1.a.src == n2.a.dst) && (n1.a.dst == n2.a.src)
        return true
    end
    return false
end

""" add detected conflicts to conflict table """
function CRCBS.detect_conflicts!(conflicts::ConflictTable,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)
    # state conflict
    if detect_state_conflict(n1,n2)
        add_conflict!(conflicts,Conflict(
            STATE_CONFLICT,
            agent1_id = i,
            agent2_id = j,
            node1 = n1,
            node2 = n2,
            t = t
        ))
    end
    if detect_action_conflict(n1,n2)
        add_conflict!(conflicts,Conflict(
            ACTION_CONFLICT,
            agent1_id = i,
            agent2_id = j,
            node1 = n1,
            node2 = n2,
            t = t
        ))
    end
end

end # module

let
    s = CBS.State()
    @test states_match(s,s)
    a = CBS.Action()
    a = CRCBS.wait(s)
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
# let
#     graph = Graph(3)
#     add_edge!(graph,1,2)
#     add_edge!(graph,1,3)
#     constraints = ConstraintDict()
#     env = CBSLowLevelEnv(graph,constraints)
#     s = State(1)
#     for a in get_possible_actions(env,s)
#         @test typeof(a) == Action
#     end
# end
