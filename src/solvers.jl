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

function CRCBS.solve!(solver::CBS_Solver, mapf::M where {M<:AbstractMAPF}, path_finder=A_star)
    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,get_cost_type(mapf.env)}()

    root_node = initialize_root_node(mapf)
    low_level_search!(solver,mapf,root_node;path_finder=path_finder)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        if !is_valid(conflict)
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            return node.solution, node.cost
        end
        # otherwise, create constraints and branch
        constraints = generate_constraints_from_conflict(conflict)
        for constraint in constraints
            new_node = initialize_child_search_node(node)
            if add_constraint!(new_node,constraint)
                low_level_search!(solver, mapf, new_node,[get_agent_id(constraint)]; path_finder=path_finder)
                detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)]) # update conflicts related to this agent
                if is_valid(new_node.solution, mapf)
                    # TODO update env (i.e. update heuristic, etc.)
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
    end
    print("No Solution Found. Returning default solution")
    return default_solution(mapf)
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

function CRCBS.solve!(solver::MetaAgentCBS_Solver, mapf::M where {M<:AbstractMAPF}, path_finder=A_star)
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    root_node = initialize_root_node(mapf)
    root_node.groups = [[i] for i in 1:num_agents(mapf)]
    low_level_search!(solver,mapf,root_node)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        if !is_valid(conflict)
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            return node.solution, node.cost
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
                enqueue!(priority_queue, new_node => new_node.cost)
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
                        enqueue!(priority_queue, new_node => new_node.cost)
                    end
                end
            end
        end
    end
    print("No Solution Found. Returning default solution")
    return default_solution(solver, mapf)
end

# """
#     The Improved Conflict-Based Search Algorithm - Boyarski et al 2015
#
#     https://www.ijcai.org/Proceedings/15/Papers/110.pdf
# """
# struct ICBS_Solver <: AbstractMAPFSolver end
#
# function solve!(solver::ICBS_Solver, mapf::MAPF, path_finder=A_star)    # priority queue that stores nodes in order of their cost
#     priority_queue = PriorityQueue{ConstraintTreeNode,Int}()
#
#     root_node = initialize_root_node(mapf)
#     low_level_search!(mapf,root_node)
#     if is_valid(root_node.solution,mapf)
#         enqueue!(priority_queue, root_node => root_node.cost)
#     end
#
#     while length(priority_queue) > 0
#         node = dequeue!(priority_queue)
#         # check for conflicts
#         state_conflicts, action_conflicts = get_conflicts(node.solution)
#         # state_conflict, action_conflict = get_next_conflicts(node.solution)
#         if is_valid(state_conflict)
#             constraints = generate_constraints_from_conflict(state_conflict)
#         elseif is_valid(action_conflict)
#             constraints = generate_constraints_from_conflict(action_conflict)
#         else
#             print("Optimal Solution Found! Cost = ",node.cost,"\n")
#             return node.solution, node.cost
#         end
#
#         # generate new nodes from constraints
#         for constraint in constraints
#             new_node = initialize_child_search_node(node)
#             if add_constraint!(new_node,constraint,mapf)
#                 low_level_search!(mapf,new_node,[get_agent_id(constraint)])
#                 if is_valid(new_node.solution, mapf)
#                     enqueue!(priority_queue, new_node => new_node.cost)
#                 end
#             end
#         end
#     end
#     print("No Solution Found. Returning default solution")
#     return LowLevelSolution(), typemax(Int)
# end
