export
    solve!,
    CBS_Solver,
    ICBS_Solver

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
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()
    # node_list = Vector{ConstraintTreeNode}()

    root_node = initialize_root_node(mapf)
    low_level_search!(solver,mapf,root_node)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if is_valid(root_node.solution,mapf)
        # @show root_node
        enqueue!(priority_queue, root_node => root_node.cost)
        # push!(node_list,root_node)
    end

    # k = 0
    while length(priority_queue) > 0
        # @show k += 1
        node = dequeue!(priority_queue)
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        # @show conflict.node1.sp, conflict.agent1_id
        if is_valid(conflict)
            constraints = generate_constraints_from_conflict(conflict)
        else
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            # for (i,p) in enumerate(node.solution)
            #     @show i=>[n.sp.vtx for n in p.path_nodes]
            # end
            return node.solution, node.cost
        end

        # generate new nodes from constraints
        for constraint in constraints
            new_node = initialize_child_search_node(node)
            # new_node.id = length(node_list) + 1
            if add_constraint!(new_node,constraint)
                low_level_search!(solver,mapf,new_node,[get_agent_id(constraint)])
                # for (i,p) in enumerate(new_node.solution)
                #     @show i=>[n.sp.vtx for n in p.path_nodes]
                # end
                detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)]) # update conflicts related to this agent
                if is_valid(new_node.solution, mapf)
                    # @show new_node.constraints
                    enqueue!(priority_queue, new_node => new_node.cost)
                    # push!(node_list, new_node)
                end
            end
        end
        # if k > 5
        #     break
        # end
    end
    # @show length(node_list)
    print("No Solution Found. Returning default solution")
    return default_solution(solver, mapf)
end

"""
    The Improved Conflict-Based Search Algorithm - Boyarski et al 2015

    https://www.ijcai.org/Proceedings/15/Papers/110.pdf
"""
struct ICBS_Solver <: AbstractMAPFSolver end

function solve!(solver::ICBS_Solver, mapf::MAPF, path_finder=A_star)    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    root_node = initialize_root_node(mapf)
    low_level_search!(mapf,root_node)
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        # check for conflicts
        state_conflicts, action_conflicts = get_conflicts(node.solution)
        # state_conflict, action_conflict = get_next_conflicts(node.solution)
        if is_valid(state_conflict)
            constraints = generate_constraints_from_conflict(state_conflict)
        elseif is_valid(action_conflict)
            constraints = generate_constraints_from_conflict(action_conflict)
        else
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            return node.solution, node.cost
        end

        # generate new nodes from constraints
        for constraint in constraints
            new_node = initialize_child_search_node(node)
            if add_constraint!(new_node,constraint,mapf)
                low_level_search!(mapf,new_node,[get_agent_id(constraint)])
                if is_valid(new_node.solution, mapf)
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
    end
    print("No Solution Found. Returning default solution")
    return LowLevelSolution(), typemax(Int)
end
