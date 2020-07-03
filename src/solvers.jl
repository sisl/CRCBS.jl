export
    solve!,
    CBS_Solver,
    ICBS_Solver,
    MetaAgentCBS_Solver

function solve! end
function low_level_search! end

"""
    `solve!(solver::AbstractMAPFSolver, args ...)`

    Run the algorithm represented by `solver` on an instance of a Multi-Agent
    Path-Finding problem.
"""
function solve!(solver::AbstractMAPFSolver, args ...)
    throw(ArgumentError(string(
        "function CRCBS.solve!(solver::", typeof(solver),",...) not defined.",
        " You must explicitly override CRCBS.solve!() for solvers of type",
        typeof(solver)
        )))
end

export AbstractAStarPlanner
abstract type AbstractAStarPlanner end
check_termination_criteria(solver::A,env,cost_so_far,s) where {A<:AbstractAStarPlanner} =  iterations(solver) > iteration_limit(solver)
function logger_step_a_star!(solver::A, env, base_path, s, q_cost) where {A<:AbstractAStarPlanner}
    increment_iteration_count!(solver)
    log_info(2,solver,"A* iter $(iterations(solver)): s = $(string(s)), q_cost = $q_cost")
end
function logger_enter_a_star!(solver::A) where {A<:AbstractAStarPlanner}
    log_info(1,solver,"A*: entering...")
    @assert(iterations(solver) == 0, "A*: ERROR: iterations = $(iterations(solver)) at entry")
end
function logger_enqueue_a_star!(solver::A,env,s,a,sp,h_cost) where {A<:AbstractAStarPlanner}
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

export CBS_Solver

"""
    CBS_Solver

Path planner that employs Conflict-Based Search
"""
@with_kw struct CBS_Solver{L,C} <: AbstractCBSSolver
    low_level_planner::L    = AStar()
    logger::SolverLogger{C} = SolverLogger{cost_type(low_level_planner)}()
end
CBS_Solver(planner) = CBS_Solver(low_level_planner=planner)

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
    if verbosity(solver) >= 0
        println("Optimal Solution Found! Cost = $(get_cost(node))")
    end
end
function logger_cbs_add_constraint!(solver,node,constraint,mapf)
    increment_iteration_count!(solver)
    if verbosity(solver) == 1
        println("CBS: adding constraint ",string(constraint))
    elseif verbosity(solver) == 2
        println("CBS: adding constraint:")
        println("\t",string(constraint))
        println("CBS: constraints in node:")
        for i in 1:num_agents(mapf)
            for constraint in sorted_state_constraints(node,i)
                println("\t",string(constraint))
            end
            for constraint in sorted_action_constraints(node,i)
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
        solver::S where {S<:AbstractMAPFSolver},
        mapf::M where {M<:AbstractMAPF},
        node::ConstraintTreeNode,
        idxs=collect(1:num_agents(mapf)),
        path_finder=A_star)`

    Returns a low level solution for a MAPF with constraints. The heuristic
    function for cost-to-go is user-defined and environment-specific
"""
function low_level_search!(
    solver::S, mapf::M, node::N, idxs::Vector{Int}=collect(1:num_agents(mapf));
    heuristic=get_heuristic_cost, path_finder=A_star, verbose=false
    ) where {S<:AbstractAStarPlanner,M<:AbstractMAPF,N<:ConstraintTreeNode}
    # Only compute a path for the indices specified by idxs
    for i in idxs
        env = build_env(solver, mapf, node, i)
        reset_solver!(solver)
        path, cost = path_finder(solver, env, get_start(mapf,env,i), heuristic)
        set_solution_path!(node.solution, path, i)
        set_path_cost!(node.solution, cost, i)
    end
    set_cost!(node, aggregate_costs(get_cost_model(mapf.env),get_path_costs(node.solution)))
    return is_consistent(node.solution,mapf)
end
low_level_search!(solver::CBS_Solver,args...) = low_level_search!(low_level(solver),args...)
build_env(solver, mapf, node, i) = build_env(mapf, node, i)

# """
#     cbs_dequeue_and_preprocess(solver::CBS_Solver,priority_queue,mapf)
#
# Part of CBS interface. Defaults to dequeueing the next node from
# `priority_queue`. This function can be overridden as part of implementing e.g.,
# Meta-Agent CBS.
# """
# function cbs_dequeue_and_preprocess!(solver,priority_queue,mapf)
#     node = dequeue!(priority_queue)
# end

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
priority_queue and/or bypass the branching step of CBS.
"""
cbs_bypass!(solver,mapf,node,priority_queue) = false

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
        if add_constraint!(new_node,constraint)
            logger_cbs_add_constraint!(solver,new_node,constraint,mapf)
            consistent_flag = low_level_search!(solver, mapf, new_node,[get_agent_id(constraint)])
            if consistent_flag
                detect_conflicts!(new_node,[get_agent_id(constraint)]) # update conflicts related to this agent
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
    println("No Solution Found. Returning default solution")
    return default_solution(mapf)
end

function solve!(solver::CBS_Solver, mapf::M where {M<:AbstractMAPF}, path_finder=A_star;verbose=false)
    cbs!(solver,mapf)
end

"""
    MetaAgentCBS_Solver

Path planner that employs Meta Agent Conflict-Based Search
"""
@with_kw struct MetaAgentCBS_Solver{L,C} <: AbstractCBSSolver
    low_level_planner::L    = AStar()
    logger::SolverLogger{C} = SolverLogger{cost_type(low_level_planner)}()
    beta::Int               = 1
end
MetaAgentCBS_Solver(planner) = MetaAgentCBS_Solver(low_level_planner=planner)

export MetaSolution

"""
    MetaSolution{S}

Wrapper for a LowLevelSolution that allows for keeping track of groups.
"""
struct MetaSolution{S}
    solution::S
    group_idxs::Vector{Vector{Int}}
end
MetaSolution(mapf::AbstractMAPF) = MetaSolution(
    get_initial_solution(mapf),
    map(i->[i],1:num_agents(mapf))
    )
for op in [
    :state_type,
    :action_type,
    :cost_type,
    :get_paths,
    :get_path_costs,
    :get_cost,
    :get_cost_model,
    :set_solution_path!,
    :set_path_cost!,
    :set_cost!,
    :is_consistent,
    :is_valid,
    ]
    @eval $op(s::MetaSolution,args...) = $op(s.solution,args...)
end
Base.copy(solution::L) where {L<:MetaSolution} = L(
    copy(solution.solution),
    deepcopy(solution.group_idxs)
    )

function initialize_root_node(solver::MetaAgentCBS_Solver,mapf::AbstractMAPF,solution=MetaSolution(mapf))
    initialize_root_node(mapf,solution)
end

"""
    `combine_agents(conflict_table, groups::Vector{Vector{Int}})`

    Helper for merging two (meta) agents into a meta-agent
"""
function combine_agents!(solver, node)
    groups = node.solution.group_idxs
    N = length(groups)
    conflict_counts = zeros(Int,N,N)
    for (i,idxs1) in enumerate(groups)
        for (j,idxs2) in enumerate(groups)
            conflict_counts[i,j] += count_conflicts(node.conflict_table,idxs1,idxs2)
        end
    end
    idx = argmax(conflict_counts).I
    i = minimum(idx)
    j = maximum(idx)
    if conflict_counts[i,j] > solver.beta
        log_info(1,solver,"Conflict Limit exceeded. Merging agents ", groups[i], " and ", groups[j],"\n")
        groups = deepcopy(groups)
        groups[i] = [groups[i]..., groups[j]...]
        deleteat!(groups, j)
        node.groups = groups
        return groups, i
    end
    return groups, -1
end

function get_group_index(groups, agent_idx)
    group_idx = -1
    for i in 1:length(groups)
        if agent_idx in groups[i]
            group_idx = i
            break
        end
    end
    return group_idx
end
function get_group_index(solution::MetaSolution, agent_idx)
    get_group_index(solution.group_idxs,agent_idx)
end

function cbs_bypass!(solver::MetaAgentCBS_Solver,mapf,node,priority_queue)
    groups, group_idx = combine_agents!(solver, node)
    if group_idx > 0 # New Meta Agent has been constructed
        new_node = initialize_child_search_node(node)
        low_level_search!(solver, mapf, new_node, [group_idx])
        for agent_id in node.groups[group_idx]
            detect_conflicts!(new_node.conflict_table,new_node.solution,agent_id)
        end
        if is_valid(new_node.solution, mapf)
            enqueue!(priority_queue, new_node => get_cost(new_node))
        end
        return true
    end
    return false
end

function CRCBS.low_level_search!(solver::MetaAgentCBS_Solver,mapf,node,idxs=collect(1:num_agents(mapf)))
    group_idxs = union(map(i->get_group_index(node.solution,i), idxs))
    low_level_search!(low_level(solver),mapf,node,group_idxs)
end

function detect_conflicts!(conflict_table,solution::MetaSolution,
        group_idxs=collect(1:length(solution.group_idxs)),
        args...;
        kwargs...
    )
    for group_idx in group_idxs
        for idx in solution.group_idxs[group_idx]
            detect_conflicts!(conflict_table,solution.solution,idx,args...;kwargs...)
        end
    end
end

solve!(solver::MetaAgentCBS_Solver, mapf) = cbs!(solver,mapf)
