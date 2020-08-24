export
    solve!,
    CBSSolver,
    MetaAgentCBS_Solver

function solve! end
function low_level_search! end

"""
    `solve!(solver, args ...)`

    Run the algorithm represented by `solver` on an instance of a Multi-Agent
    Path-Finding problem.
"""
function solve!(solver, args...)
    throw(ArgumentError(string(
        "function CRCBS.solve!(solver::", typeof(solver),",...) not defined.",
        " You must explicitly override CRCBS.solve!() for solvers of type",
        typeof(solver)
        )))
end

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
    BiLevelPlanner,
    AbstractCBSSolver,
    low_level

abstract type BiLevelPlanner end
abstract type AbstractCBSSolver <: BiLevelPlanner end
low_level(solver::BiLevelPlanner) = solver.low_level_planner
function set_best_cost!(solver::AbstractCBSSolver,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(low_level(solver),cost)
end
function hard_reset_solver!(solver::AbstractCBSSolver)
    hard_reset_solver!(get_logger(solver))
    hard_reset_solver!(low_level(solver))
end

export CBSSolver

"""
    CBSSolver

Path planner that employs Conflict-Based Search
"""
@with_kw struct CBSSolver{L,C} <: AbstractCBSSolver
    low_level_planner::L    = AStar()
    logger::SolverLogger{C} = SolverLogger{cost_type(low_level_planner)}()
end
CBSSolver(planner) = CBSSolver(low_level_planner=planner)
# CBSSolver(mapf::AbstractMAPF) = CBSSolver(low_level_planner=AStar{cost_type(mapf)}())
for op in [:set_deadline!,:set_runtime_limit!,:set_verbosity!]
    eval(quote
        $op(solver::CBSSolver,args...) = begin
            $op(get_logger(solver),args...)
            $op(solver.low_level_planner,args...)
        end
    end)
end

################################################################################
############################# CBS Logger Interface #############################
################################################################################
function enter_cbs!(solver)
    reset_solver!(solver)
end
function logger_cbs_add_constraint!(solver,new_node,constraint) end
function logger_dequeue_cbs_node!(solver,node)
    if verbosity(solver) >= 1
        println("CBS: Current paths: ")
        for (i,path) in enumerate(get_paths(node.solution))
            println("\t",i,": ",convert_to_vertex_lists(path))
        end
    end
    enforce_iteration_limit(solver)
end
function logger_exit_cbs_optimal!(solver,node)
    set_best_cost!(solver,get_cost(node))
    if verbosity(solver) >= 0
        println("Optimal Solution Found! Cost = $(get_cost(node))")
    end
end
function logger_cbs_add_constraint!(solver,node,constraint,mapf)
    increment_iteration_count!(solver)
    enforce_time_limit(solver)
    enforce_iteration_limit(solver)
    if verbosity(solver) == 1
        println("CBS: adding constraint ",string(constraint))
    elseif verbosity(solver) == 2
        println("CBS: adding constraint:")
        println("\t",string(constraint))
        println("CBS: constraints in node:")
        for i in 1:num_agents(mapf)
            for constraint in sorted_state_constraints(mapf,node,i)
                println("\t",string(constraint))
            end
            for constraint in sorted_action_constraints(mapf,node,i)
                println("\t",string(constraint))
            end
        end
    end
end

################################################################################
################################ CBS Interface #################################
################################################################################
export low_level_search!

"""
    `low_level_search!(
        solver,
        mapf::AbstractMAPF,
        node::ConstraintTreeNode,
        idxs=collect(1:num_agents(mapf)),
        path_finder=a_star)`

    Returns a low level solution for a MAPF with constraints. The heuristic
    function for cost-to-go is user-defined and environment-specific
"""
function low_level_search!(
    solver, mapf::M, node::N, idxs=collect(1:num_agents(mapf));
    path_finder=a_star
    ) where {M<:AbstractMAPF,N<:ConstraintTreeNode}
    # Only compute a path for the indices specified by idxs
    for i in idxs
        env = build_env(solver, mapf, node, i)
        reset_solver!(low_level(solver))
        path, cost = path_finder(low_level(solver), env, get_start(mapf,env,i))
        set_solution_path!(node.solution, path, i)
        set_path_cost!(node.solution, cost, i)
    end
    set_cost!(node, aggregate_costs(get_cost_model(mapf.env),get_path_costs(node.solution)))
    return is_consistent(node.solution,mapf)
end
# low_level_search!(solver::CBSSolver,args...) = low_level_search!(low_level(solver),args...)
build_env(solver, mapf, node, i) = build_env(mapf, node, i)

"""
    get_agent_idxs(solver,node,mapf,constraint)

Part of CBS interface. Defaults to return the index of a single agent affected
by a constraint. Can be overridden to return the index of e.g., a "meta-agent"
(group of agents).
"""
function get_agent_idxs(solver,node,mapf,constraint)
    [get_agent_id(constraint)]
end

"""
    cbs_bypass!(solver,mapf,node,conflict,priority_queue)

Part of CBS interface. Defaults to false, but can be overridden to modify the
priority_queue and/or bypass the branching step of CBS
"""
cbs_bypass!(solver,mapf,node,priority_queue) = false

"""
    cbs_update_conflict_table!(solver,mapf,node,constraint)

Allows for flexible conflict-updating dispatch. This function is called within
    within the default `cbs_branch!()` method.
"""
function cbs_update_conflict_table!(solver,mapf,node,constraint)
    detect_conflicts!(node,[get_agent_id(constraint)]) # update conflicts related to this agent
end

"""
    cbs_branch!(solver,mapf,node,conflict,priority_queue)

Part of CBS interface. Defaults to splitting on the conflict and adding two
nodes to the priority_queue, where each of the child nodes has one of the new
complementary constraints.
"""
function cbs_branch!(solver,mapf,node,conflict,priority_queue)
    constraints = generate_constraints_from_conflict(conflict)
    for constraint in constraints
        new_node = initialize_child_search_node(solver,mapf,node)
        if add_constraint!(mapf,new_node,constraint)
            logger_cbs_add_constraint!(solver,new_node,constraint,mapf)
            consistent_flag = low_level_search!(solver, mapf, new_node,[get_agent_id(constraint)])
            if consistent_flag
                cbs_update_conflict_table!(solver,mapf,new_node,constraint)
                enqueue!(priority_queue, new_node => get_cost(new_node))
            end
        end
    end
    priority_queue
end

"""
    Conflict-Based Search

    Sharon et al 2012
    https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
"""
function cbs!(solver,mapf)
    enter_cbs!(solver)
    node = initialize_root_node(solver,mapf)
    priority_queue = PriorityQueue{typeof(node),cost_type(node)}()
    consistent_flag = low_level_search!(solver,mapf,node)
    if consistent_flag
        detect_conflicts!(node)
        enqueue!(priority_queue, node => get_cost(node))
    end

    try
        while ~isempty(priority_queue)
            # node = cbs_dequeue_and_preprocess!(solver,priority_queue,mapf)
            node = dequeue!(priority_queue)
            logger_dequeue_cbs_node!(solver,node)
            # check for conflicts
            conflict = get_next_conflict(node.conflict_table)
            if !is_valid(conflict)
                logger_exit_cbs_optimal!(solver,node)
                return node.solution, get_cost(node)
            elseif ~cbs_bypass!(solver,mapf,node,priority_queue)
                cbs_branch!(solver,mapf,node,conflict,priority_queue)
            end
        end
    catch e
        if isa(e,SolverException)
            showerror(stderr,e)
        else
            rethrow(e)
        end
    end
    println("No Solution Found. Returning default solution")
    return default_solution(mapf)
end

function solve!(solver::CBSSolver, mapf::M where {M<:AbstractMAPF}, path_finder=a_star;verbose=false)
    cbs!(solver,mapf)
end
