let
    # const State = Int
    struct State
        v::Int # node index
        t::Int # time step
    end
    struct Action
        e::Edge
        Δt::Int # traversal_time
    end
    struct GraphEnv{G} <: AbstractEnv{State,Action}
        g::G
        constraints::Set{Int}
    end

    struct ActionIter
        env::GraphEnv
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
    CRCBS.get_possible_actions(env::GraphEnv,s::State) = ActionIter(env,s.v,outneighbors(env.g,s.v))
    CRCBS.get_next_state(env::GraphEnv,s::State,a::Action) = State(a.e.dst,s.t+a.Δt)
    CRCBS.get_transition_cost(env::GraphEnv,s::State,a::Action,sp::State) = 1.0
    CRCBS.get_path_cost(env::GraphEnv,path::Path{State,Action}) = get_terminal_state(path).t
    CRCBS.violates_constraints(env::GraphEnv,path::Path{State,Action},s::State,a::Action,sp::State) = s.v in env.constraints ? true : false
    function CRCBS.check_termination_criteria(env::GraphEnv, cost, path::Path{State,Action},s::State)
        if cost >= 20
            return true
        end
        return false
    end

    G = CycleGraph(10)
    rem_edge!(G,1,10)
    add_edge!(G,1,5)
    add_edge!(G,3,6)
    add_edge!(G,6,8)
    add_edge!(G,8,10)
    env = GraphEnv(G,Set{Int}(5))
    start_state = State(1,0)
    goal_state = State(10,-1)

    dists = gdistances(G,goal_state.v)
    heuristic(s) = dists[s.v]
    is_goal(s::State) = (s.v == goal_state.v)

    A_star(env,start_state,is_goal,heuristic)
end
