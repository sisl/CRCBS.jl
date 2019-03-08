
const State = Int
const Path = Vector{State}
const Action = Edge
const PathCost = Int
abstract type Env{S,A} end
action_type(env::E where {S,A,E <: Env{S,A}}) = A
state_type(env::E where {S,A,E <: Env{S,A}}) = S
const Env = Graph

get_possible_actions(env::Env,state::State) = []
# transition cost
get_transition_cost(env::Env,state::State,action::Action) = 1.0
# g(n) = cost of the path from the start node to n,
get_path_cost(env::Env,path::Path) = 0.0
get_next_state(env::Env,state::State,action::Action) = State()
# get_cost_to_go(state::State,goal::State) = 0.0
get_heuristic(env::Env,state::State,goals::State) = 0.0
# h(n) = heuristic estimate of cost from n to goal
# f(n) = g(n) + h(n)


function A_star_impl!(env::E where E <: Env,# the graph
    t, # the end vertex
    frontier,               # an initialized heap containing the active vertices
    constraints::C where C,
    colormap::Vector{UInt8},  # an (initialized) color-map to indicate status of vertices
    distmx::AbstractMatrix,
    heuristic::Function)

    E = action_type(env)

    @inbounds while !isempty(frontier)
        (cost_so_far, path, u) = dequeue!(frontier)
        if u == t
            return path
        end

        for v in LightGraphs.outneighbors(g, u)
            # Skip node if it violates any of the constraints
            if violates_constraints(constraints,v,path)
                continue
            end
            if get(colormap, v, 0) < 2
                dist = distmx[u, v]
                colormap[v] = 1
                new_path = cat(path, E(u, v), dims=1)
                path_cost = cost_so_far + dist
                enqueue!(frontier,
                    (path_cost, new_path, v),
                    path_cost + heuristic(v)
                )
            end
        end
        colormap[u] = 2
    end
    Vector{E}()
end



mutable struct Node
    state::State
    f_score::Cost # total cost f(n) = g(n) + h(n)
    g_score::Cost # cost so far
end

mutable struct Neighbor
    state::State # target state (reached by applyin action from current state)
    action::Action # action to get from current state to state
    cost::Cost # cost of traversing taking this action
end
function cost_to_go(state::State)
    return 0.0
end

"""
    We want an A_star algorithm that can operate on implicitly defined graphs
    (we want to have an implicit cartesian product graph)
"""
function A_star(env,start_state,solution,initial_cost=0.0)
    open_set = PriorityQueue{State,Node}()
    closed_set = Set{State}()
    came_from = Dict{State,Tuple{State,Action,PathCost,PathCost}}()

    start_node = Node(start_state, cost_to_go(start_state), initial_cost)
    enqueue!(open_set, start_state=>start_node)

end
