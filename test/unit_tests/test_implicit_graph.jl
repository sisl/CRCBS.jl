module ImplicitGraphsTests

using CRCBS, LightGraphs

struct State
    v::Int # node index
    t::Int # time step
end
State() = State(-1,-1)
struct Action
    e::Edge
    Δt::Int # traversal_time
end
Action() = Action(Edge(-1,-1),-1)
struct TestEnv{G} <: AbstractLowLevelEnv{State,Action,TravelTime}
    g::G
    constraints::Set{Int}
    goal_state::State
end
CRCBS.is_goal(env::TestEnv,s::State) = (s.v == env.goal_state.v)
struct ActionIter
    env::TestEnv
    s::Int # source state
    edge_list::Vector{Int} # length of target edge list
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
    if iter_state.idx > length(it.edge_list)
        return nothing
    end
    Action(Edge(it.s,it.edge_list[iter_state.idx]),1.0), iter_state
end
CRCBS.get_possible_actions(env::TestEnv,s::State) = ActionIter(env,s.v,outneighbors(env.g,s.v))
CRCBS.get_next_state(env::TestEnv,s::State,a::Action) = State(a.e.dst,s.t+a.Δt)
CRCBS.get_transition_cost(env::TestEnv,s::State,a::Action,sp::State) = 1.0
CRCBS.violates_constraints(env::TestEnv,s::State,a::Action,sp::State) = s.v in env.constraints ? true : false
function CRCBS.check_termination_criteria(solver, env::TestEnv, cost, s::State)
    if cost >= 20
        return true
    end
    return false
end

CRCBS.get_heuristic_cost(env::TestEnv,s) = gdistances(env.g,env.goal_state.v)[s.v]
end

let
    G = cycle_graph(10)
    rem_edge!(G,1,10)
    add_edge!(G,1,5)
    add_edge!(G,3,6)
    add_edge!(G,6,8)
    add_edge!(G,8,10)
    goal_state = ImplicitGraphsTests.State(8,-1)
    env = ImplicitGraphsTests.TestEnv(G,Set{Int}(5),goal_state)
    start_state = ImplicitGraphsTests.State(1,0)

    let
        path, path_cost = CRCBS.a_star(env,start_state)
        @test length(path) == 4
        @test get_cost(path) == 4
        # Now run a_star again on top of the previous path
        goal_state2 = ImplicitGraphsTests.State(10,-1)
        env2 = ImplicitGraphsTests.TestEnv(G,Set{Int}(5),goal_state2)
        path2, path_cost  = CRCBS.a_star(env2,path,path_cost)
        @test get_cost(path2) == 5
        @test length(path2) == 5
    end
end
