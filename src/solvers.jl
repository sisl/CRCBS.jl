export
    solve!,
    CBS_Solver,
    ICBS_Solver,
    MetaAgentCBS_Solver

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

"""
    The Conflict-Based Search algorithm for multi-agent path finding - Sharon et
    al 2012

    https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
"""
struct CBS_Solver <: AbstractMAPFSolver end

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
    ) where {S<:AbstractMAPFSolver,M<:AbstractMAPF,N<:ConstraintTreeNode}
    # Only compute a path for the indices specified by idxs
    for i in idxs
        env = build_env(mapf, node, i)
        # Solve!
        path, cost = path_finder(env, get_start(mapf,env,i), heuristic)
        set_solution_path!(node.solution, path, i)
        set_path_cost!(node.solution, cost, i)
    end
    set_cost!(node.solution, aggregate_costs(get_cost_model(mapf.env),get_path_costs(node.solution)))
    set_cost!(node, get_cost(node.solution))
    return is_consistent(node.solution,mapf)
end

function enter_cbs!(solver) end
verbosity(solver) = 1
function logger_dequeue_cbs_node!(solver,node)
    if verbosity(solver) >= 1
        println("CBS: Current paths: ")
        for (i,path) in enumerate(get_paths(node.solution))
            println("\t",i,": ",convert_to_vertex_lists(path))
        end
    end
end
function logger_exit_cbs_optimal!(solver,node)
    println("Optimal Solution Found! Cost = $(get_cost(node))")
end
function logger_cbs_add_constraint!(solver,new_node,constraint)
    if verbosity(solver) > 1
        println("CBS: Constraint agent id = $(get_agent_id(constraint)),",
            " time index = $(get_time_of(constraint))")
    end
end

"""
    Conflict-Based Search

    Sharon et al 2012
    https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
"""
function cbs!(solver,mapf)
    enter_cbs!(solver)
    root_node = initialize_root_node(mapf)
    priority_queue = PriorityQueue{typeof(root_node),get_cost_type(mapf.env)}()
    consistent_flag = low_level_search!(solver,mapf,root_node)
    detect_conflicts!(root_node)
    if consistent_flag
        enqueue!(priority_queue, root_node => get_cost(root_node))
    end

    while ~isempty(priority_queue)
        node = dequeue!(priority_queue)
        logger_dequeue_cbs_node!(solver,node)
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        if !is_valid(conflict)
            logger_exit_cbs_optimal!(solver,node)
            return node.solution, get_cost(node)
        end
        # otherwise, create constraints and branch
        # branch!(solver,node,priority_queue,mapf)
        constraints = generate_constraints_from_conflict(conflict)
        for constraint in constraints
            new_node = initialize_child_search_node(node)
            if add_constraint!(new_node,constraint)
                logger_cbs_add_constraint!(solver,new_node,constraint)
                consistent_flag = low_level_search!(solver, mapf, new_node,[get_agent_id(constraint)])
                detect_conflicts!(new_node,[get_agent_id(constraint)]) # update conflicts related to this agent
                if consistent_flag
                    enqueue!(priority_queue, new_node => get_cost(new_node))
                end
            end
        end
    end
    println("No Solution Found. Returning default solution")
    return default_solution(mapf)
end

function solve!(solver::CBS_Solver, mapf::M where {M<:AbstractMAPF}, path_finder=A_star;verbose=false)
    cbs!(solver,mapf)
end

"""
    The Meta-Agent Conflict-Based Search algorithm for multi-agent path finding
    - Sharon et al 2012

    The parameter `beta` is specifies how many conflicts are allowed between
    two agents before they must be merged into a "meta-agent".

    http://faculty.cse.tamu.edu/guni/Papers/SOCS12-MACBS.pdf
"""
struct MetaAgentCBS_Solver <: AbstractMAPFSolver
    beta::Int # conflict limit
end

"""
    `combine_agents(conflict_table, groups::Vector{Vector{Int}})`

    Helper for merging two (meta) agents into a meta-agent
"""
function combine_agents!(node, beta)
    groups = node.groups
    N = length(groups)
    conflict_counts = zeros(Int,N,N)
    for (i,idxs1) in enumerate(groups)
        for (j,idxs2) in enumerate(groups)
            conflict_counts[i,j] += count_conflicts(node.conflict_table,idxs1,idxs2)
        end
    end
    idx = argmax(conflict_counts)
    i = min(idx[1],idx[2]); j = max(idx[1],idx[2]);
    if conflict_counts[i,j] > beta
        print("Conflict Limit exceeded. Merging agents ", groups[i], " and ", groups[j],"\n")
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

function solve!(solver::MetaAgentCBS_Solver, mapf::M where {M<:AbstractMAPF}, path_finder=A_star)
    root_node = initialize_root_node(mapf)
    priority_queue = PriorityQueue{typeof(root_node),Int}()
    root_node.groups = [[i] for i in 1:num_agents(mapf)]
    low_level_search!(solver,mapf,root_node)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => get_cost(root_node))
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        if !is_valid(conflict)
            print("Optimal Solution Found! Cost = ",get_cost(node),"\n")
            return node.solution, get_cost(node)
        end
        # check if a new meta-agent needs to be formed
        groups, group_idx = combine_agents!(node, solver.beta)
        if group_idx > 0 # New Meta Agent has been constructed
            new_node = initialize_child_search_node(node)
            low_level_search!(solver, mapf, new_node, [group_idx])
            for agent_id in node.groups[group_idx]
                detect_conflicts!(new_node.conflict_table,new_node.solution,agent_id)
            end
            if is_valid(new_node.solution, mapf)
                enqueue!(priority_queue, new_node => get_cost(new_node))
            end
        else # generate new nodes from constraints
            constraints = generate_constraints_from_conflict(conflict)
            for constraint in constraints
                new_node = initialize_child_search_node(node)
                if add_constraint!(new_node,constraint)
                    group_idx = get_group_index(groups, get_agent_id(constraint))
                    low_level_search!(solver, mapf, new_node, [group_idx])
                    for agent_id in node.groups[group_idx]
                        detect_conflicts!(new_node.conflict_table,new_node.solution,agent_id)
                    end
                    if is_valid(new_node.solution, mapf)
                        enqueue!(priority_queue, new_node => get_cost(new_node))
                    end
                end
            end
        end
    end
    print("No Solution Found. Returning default solution")
    return default_solution(solver, mapf)
end
