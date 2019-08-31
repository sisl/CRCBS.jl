export
    A_star_impl!,
    A_star

"""
    The internal loop of the A* algorithm.
"""
function A_star_impl!(env::E, frontier, explored::Set{S}, heuristic::Function) where {S,A,T,C<:AbstractCostModel{T},E <: AbstractLowLevelEnv{S,A,C}}
    while !isempty(frontier)
        (cost_so_far, path, s) = dequeue!(frontier)
        if is_goal(env,s)
            return path, cost_so_far # TODO return the cost as well
        elseif check_termination_criteria(env,cost_so_far,path,s)
            break
        end

        for a in get_possible_actions(env,s)
            sp = get_next_state(env,s,a)
            # Skip node if it violates any of the constraints
            if violates_constraints(env,path,s,a,sp)
                continue
            end
            if !(sp in explored)
                new_path = cat(path, PathNode(s, a, sp))
                new_path.cost = accumulate_cost(env, cost_so_far, get_transition_cost(env,s,a,sp))
                # enqueue!(frontier, (new_path.cost, new_path, sp) => new_path.cost + heuristic(env,sp))
                enqueue!(frontier, (new_path.cost, new_path, sp) => add_heuristic_cost(env, new_path.cost, heuristic(env,sp)))
            end
        end
        push!(explored,s)
    end
    Path{S,A,T}(), get_initial_cost(env)
end

# g(n) = cost of the path from the start node to n,
# h(n) = heuristic estimate of cost from n to goal
# f(n) = g(n) + h(n)

"""
    `A_star(env,start_state,heuristic)`

    A generic implementation of the [A* search algorithm](http://en.wikipedia.org/wiki/A%2A_search_algorithm)
    that operates on an Environment and initial state.

    args:
    - `env::E <: AbstractLowLevelEnv{S,A,C}`
    - `start_state::S`
    - `heuristic::Function=(env,s)->C(0)`

    The following methods must be implemented:
    - is_goal(env::E,s::S)
    - check_termination_criteria(env::E,cost::C,path::Path{S,A,C},state::S)
    - get_possible_actions(env::E,s::S)
    - get_next_state(env::E,s::S,a::A,sp::S)
    - get_transition_cost(env::E,s::S,a::A)
    - violates_constraints(env::E,s::S,path::Path{S,A,C})
"""
function A_star(env::E,start_state::S,heuristic::Function=(env,s)->C(0)) where {S,A,T,C<:AbstractCostModel{T},E <: AbstractLowLevelEnv{S,A,C}}
    # initial_cost = C(0) # TODO require default constructible cost
    initial_cost = get_initial_cost(env)
    frontier = PriorityQueue{Tuple{T, Path{S,A,T}, S}, T}()
    enqueue!(frontier, (initial_cost, Path{S,A,T}(s0=start_state,cost=initial_cost), start_state)=>initial_cost)
    explored = Set{S}()
    A_star_impl!(env,frontier,explored,heuristic)
end
