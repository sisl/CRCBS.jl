export AbstractAStarPlanner
abstract type AbstractAStarPlanner end
check_termination_criteria(solver::A, env, cost_so_far, s) where {A<:AbstractAStarPlanner} =  iterations(solver) > iteration_limit(solver)
function logger_step_a_star!(solver::A, env, base_path, s, q_cost) where {A<:AbstractAStarPlanner}
    increment_iteration_count!(solver)
    log_info(2,solver,"A* iter $(iterations(solver)): s = $(string(s)), q_cost = $q_cost")
end
function logger_enter_a_star!(solver::A) where {A<:AbstractAStarPlanner}
    log_info(1,solver,"A*: entering...")
    @assert(iterations(solver) == 0, "A*: ERROR: iterations = $(iterations(solver)) at entry")
end
function logger_enqueue_a_star!(solver::A, env, s, a, sp, h_cost) where {A<:AbstractAStarPlanner}
    log_info(2,solver,"A* exploring $(string(s)) -- $(string(sp)), h_cost = $h_cost")
end
function logger_exit_a_star!(solver::A, path, cost, status) where {A<:AbstractAStarPlanner}
    # empty!(solver.search_history)
    if status == false
        log_info(-1,solver,"A*: failed to find feasible path. Returning path of cost $cost")
    else
        log_info(0,solver,"A*: returning optimal path with cost $cost")
    end
end

export AStar

"""
    AStar

A* Path Planner.
Fields:
- logger
- replan : if true, planner will replan with an empty conflict table following
    timeout.
"""
@with_kw struct AStar{C} <: AbstractAStarPlanner
    logger::SolverLogger{C} = SolverLogger{C}()
    replan::Bool            = false
end
AStar() = AStar{Float64}()

export
    a_star_impl!,
    a_star

check_termination_criteria(solver,env,cost_so_far,path,s) = check_termination_criteria(env,cost_so_far,path,s)

export
    logger_enter_a_star!,
    logger_exit_a_star!,
    logger_step_a_star!,
    logger_find_constraint_a_star!,
    logger_enqueue_a_star!

function logger_enter_a_star!(logger,args...)
    nothing
end
function logger_exit_a_star!(logger, path, cost, status, args...)
    if status == false
        println("A*: Returning Infeasible")
    end
    nothing
end
function logger_step_a_star!(logger, s, q_cost, args...)
    nothing
end
function logger_find_constraint_a_star!(logger,env,s,a,sp,args...)
    nothing
end
function logger_enqueue_a_star!(logger,env,s,a,sp,args...)
    nothing
end

"""
    reconstruct path by working backward from the terminal state
"""
function reconstruct_path!(path,predecessor_map,s,cost)
    node_sequence = Vector{node_type(path)}()
    while haskey(predecessor_map, s)
        path_node = predecessor_map[s]
        push!(node_sequence, path_node)
        s = get_s(path_node)
    end
    reverse!(node_sequence)
    for n in node_sequence
        push!(path,n)
    end
    set_cost!(path,cost)
    return path
    # prepend!(node_sequence, path.path_nodes)
    # return Path(path_nodes=node_sequence, s0=get_initial_state(path), cost=cost)
end

"""
    The internal loop of the A* algorithm.

    # g(n) = cost of the path from the start node to n,
    # h(n) = heuristic estimate of cost from n to goal
    # f(n) = g(n) + h(n)
"""
function a_star_impl!(solver, env::E, base_path, frontier, explored) where {E <: AbstractLowLevelEnv}
    logger_enter_a_star!(solver)
    cost_map = Dict{state_type(env),cost_type(env)}() # TODO: get some speedup by converting states to indices (then use a vector instead of a dict)
    predecessor_map = Dict{state_type(env),node_type(env)}() # TODO: get some speedup by converting states to indices (then use a vector instead of a dict)
    default_cost = get_infeasible_cost(env)
    opt_status = false
    while !isempty(frontier)
        (cost_so_far, s), q_cost = dequeue_pair!(frontier)
        logger_step_a_star!(solver,env,base_path,s,q_cost)
        if is_goal(env,s)
            opt_status = true
            r_path = reconstruct_path!(base_path,predecessor_map,s,cost_so_far)
            logger_exit_a_star!(solver,r_path,cost_so_far,opt_status)
            return r_path, cost_so_far
        elseif check_termination_criteria(solver,env,cost_so_far,s)
            break
        end

        for a in get_possible_actions(env,s)
            sp = get_next_state(env,s,a)
            if violates_constraints(env,s,a,sp) # Skip node if it violates any of the constraints
                logger_find_constraint_a_star!(solver,env,s,a,sp)
                continue
            end
            if !(sp in explored)
                new_cost = accumulate_cost(env, cost_so_far, get_transition_cost(env,s,a,sp))
                if new_cost < get(cost_map, sp, default_cost)
                    cost_map[sp] = new_cost
                    predecessor_map[sp] = PathNode(s, a, sp) # track predecessor
                    h_cost = add_heuristic_cost(env, new_cost, get_heuristic_cost(env,sp))
                    logger_enqueue_a_star!(solver,env,s,a,sp,h_cost)
                    enqueue!(frontier, (new_cost, sp) => h_cost)
                end
            end
        end
        push!(explored,s)
    end
    # println("A*: Returning Infeasible")
    path = path_type(env)(cost=get_infeasible_cost(env))
    cost = path.cost # get_infeasible_cost(env)
    logger_exit_a_star!(solver,path,cost,opt_status)
    return path, cost
end


"""
    a_star(env,start_state)

    A generic implementation of the [A* search algorithm](http://en.wikipedia.org/wiki/A%2A_search_algorithm)
    that operates on an Environment and initial state.

    args:
    - `env::E <: AbstractLowLevelEnv`
    - `start_state`

    The following methods must be implemented:
    - is_goal(env::E,s::S)
    - check_termination_criteria(env::E,cost::C,path::Path{S,A,C},state::S)
    - get_possible_actions(env::E,s::S)
    - get_next_state(env::E,s::S,a::A,sp::S)
    - get_transition_cost(env::E,s::S,a::A)
    - violates_constraints(env::E,s::S,path::Path{S,A,C})
"""
function a_star(solver, env::E,path::P,initial_cost=get_cost(path)) where {E<:AbstractLowLevelEnv,P<:AbstractPath}
    frontier = PriorityQueue{Tuple{cost_type(env), state_type(env)}, cost_type(env)}()
    enqueue!(frontier, (initial_cost, get_final_state(path))=>initial_cost)
    explored = Set{state_type(env)}()
    a_star_impl!(solver,env,path,frontier,explored)
end
function a_star(env::E,path::P,initial_cost=get_cost(path)) where {E<:AbstractLowLevelEnv,P<:AbstractPath}
    a_star(nothing,env,path,initial_cost)
end
function a_star(solver, env::E,start_state,initial_cost=get_initial_cost(env)) where {E<:AbstractLowLevelEnv}
    path = path_type(env)(s0=start_state,cost=initial_cost)
    a_star(solver,env,path,initial_cost)
end
function a_star(env::E,start_state,initial_cost=get_initial_cost(env)) where {E<:AbstractLowLevelEnv}
    path = path_type(env)(s0=start_state,cost=initial_cost)
    a_star(env,path,initial_cost)
end
