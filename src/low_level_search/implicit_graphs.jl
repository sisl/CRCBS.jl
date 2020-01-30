export
    A_star_impl!,
    A_star

CRCBS.check_termination_criteria(solver,env,cost_so_far,path,s) = CRCBS.check_termination_criteria(env,cost_so_far,path,s)

export
    logger_enter_a_star!,
    logger_exit_a_star!,
    logger_step_a_star!,
    logger_find_constraint_a_star!,
    logger_enqueue_a_star!

function logger_enter_a_star!(logger)
    nothing
end
function logger_exit_a_star!(logger, path, cost, status)
    if status == false
        println("A*: Returning Infeasible")
    end
    nothing
end
function logger_step_a_star!(logger, path, s, q_cost)
    nothing
end
function logger_find_constraint_a_star!(logger,env,path,s,a,sp)
    nothing
end
function logger_enqueue_a_star!(logger,env,path,s,a,sp)
    nothing
end

"""
    The internal loop of the A* algorithm.

    # g(n) = cost of the path from the start node to n,
    # h(n) = heuristic estimate of cost from n to goal
    # f(n) = g(n) + h(n)
"""
function A_star_impl!(solver, env::E, frontier, explored::Set{S}, heuristic::Function;verbose=false) where {S,A,T,C<:AbstractCostModel{T},E <: AbstractLowLevelEnv{S,A,C}}
    logger_enter_a_star!(solver)
    opt_status = false
    while !isempty(frontier)
        (cost_so_far, path, s), q_cost = dequeue_pair!(frontier)
        logger_step_a_star!(solver,path,s,q_cost)
        if is_goal(env,s)
            opt_status = true
            logger_exit_a_star!(solver,path,cost_so_far,opt_status)
            return path, cost_so_far
        elseif check_termination_criteria(solver,env,cost_so_far,path,s)
            break
        end

        for a in get_possible_actions(env,s)
            sp = get_next_state(env,s,a)
            # Skip node if it violates any of the constraints
            if violates_constraints(env,path,s,a,sp)
                logger_find_constraint_a_star!(solver,env,path,s,a,sp)
                continue
            end
            if !(sp in explored)
                new_path = cat(path, PathNode(s, a, sp))
                new_path.cost = accumulate_cost(env, get_cost(path), get_transition_cost(env,s,a,sp))
                logger_enqueue_a_star!(logger,env,new_path,s,a,sp)
                enqueue!(frontier, (new_path.cost, new_path, sp) => add_heuristic_cost(env, new_path.cost, heuristic(env,sp)))
            end
        end
        push!(explored,s)
    end
    # println("A*: Returning Infeasible")
    path = Path{S,A,T}(cost=get_infeasible_cost(env))
    cost = path.cost # get_infeasible_cost(env)
    logger_exit_a_star!(solver,path,cost,opt_status)
    return path, cost
end


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
function A_star(solver, env::E,path::P,heuristic::Function,initial_cost::T=get_cost(path);verbose=false) where {S,A,T,P<:Path{S,A,T},C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    frontier = PriorityQueue{Tuple{T, P, S}, T}()
    enqueue!(frontier, (initial_cost, path, get_final_state(path))=>initial_cost)
    explored = Set{S}()
    A_star_impl!(solver,env,frontier,explored,heuristic;verbose=verbose)
end
function A_star(env::E,path::P,heuristic::Function,initial_cost::T=get_cost(path);verbose=false) where {S,A,T,P<:Path{S,A,T},C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    A_star(NullSolver(),env,path,heuristic,initial_cost;verbose=verbose)
end
function A_star(solver, env::E,start_state::S,heuristic::Function,initial_cost::T=get_initial_cost(env);verbose=false) where {S,A,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    path = Path{S,A,T}(s0=start_state,cost=initial_cost)
    A_star(solver,env,path,heuristic,initial_cost;verbose=verbose)
end
function A_star(env::E,start_state::S,heuristic::Function,initial_cost::T=get_initial_cost(env);verbose=false) where {S,A,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    path = Path{S,A,T}(s0=start_state,cost=initial_cost)
    A_star(env,path,heuristic,initial_cost;verbose=verbose)
end
