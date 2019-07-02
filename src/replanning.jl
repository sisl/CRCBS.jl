
struct NewAgent
    # New start positions
    starts::Vector{Int}
    # start times = time at which the robots appear on the graph and suddenly need to accomplish task k
    start_times::Vector{Float64}
    # goals now contains lists, thanks to which we can set intermediate goals (pick up object, etc)
    goals::Vector{Array{Int64,1}}
    # Each intermediary goal takes some additional time for task completion, which is stored here
    goal_completion_times::Vector{Array{Float64,1}}
end

function replan_CTCBS(mapf::MAPF,new_agent,time,node::ConstraintTreeNode,distmx_DP,path_finder=LightGraphs.a_star)

    # Max number of iterations before we consider STTCBS has "failed" for time reasons
    max_iterations = 100

    # Counter for the number of iterations
    iteration_count = 0

    # The priority queue stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()

    # Time we spent calculating conflict probability (by counting or by integrating)
    countingtime = 0.0

    # Time spent finding paths
    time_spent_on_astar = 0.0

    # We collect the number of interactions we find
    num_interactions = [0,0]

    # Root node R containing 0 constraints
    root_node = node

    # If the solution enables all agents to reach their goals, we add it to the queue.
    if is_valid(root_node.solution,mapf)
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    # Loop through the Priority Queue by best-first search
    while length(priority_queue) > 0 && iteration_count < max_iterations

        print("\n \n")

        # Get lowest cost node from the queue
        node = dequeue!(priority_queue)

        # Print current solution
        #println("Solution being examined:")
        #println(node.solution)

        # Run our model to find the most likely conflict
        node_conflict, edge_conflict, counting_deltat,conflict_params = count_most_likely_conflicts!(mapf,node.solution,num_interactions)

        # Add the time we spent finding probabilities to countingtime
        countingtime += counting_deltat

        # One of node or edge conflicts should be invalid (one conflict is more likely than the other).
        # Therefore we create constraints for the one of them that is the most likely, thus valid
        if is_valid(node_conflict)

            # Print
            println("Agents ", node_conflict.agent1_id, " and ", node_conflict.agent2_id, " conflict at node ", node_conflict.node_id)

            # Create two node constraints
            constraints = generate_constraints_from_conflict(node,node_conflict,mapf.t_delay,conflict_params,mapf.epsilon)
            println("Constraints")
            println(constraints)

        elseif is_valid(edge_conflict)

            # Print
            println("Agents ", edge_conflict.agent1_id, " and ", edge_conflict.agent2_id, " conflict at edge ", edge_conflict.node1_id, " - ", edge_conflict.node2_id)

            # Create two edge conflicts
            constraints = generate_constraints_from_conflict(node,edge_conflict,mapf.t_delay,conflict_params,mapf.epsilon)
            println("Constraints")
            println(constraints)

        else

            # If we have found no conflict with probability greater than ϵ, we found the optimal solution!
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            print("Time spent on probability count: ", countingtime, " \n")
            print("Time spent on path finding: ", time_spent_on_astar, " \n")
            return (node.solution, node.cost,countingtime,time_spent_on_astar,num_interactions,iteration_count,node)
        end

        # generate new nodes from constraints
        for constraint in constraints

            # create node with all of the previous constraints
            new_node = initialize_child_node(node)

            # Try to add the new constraint: this will fail if you try to steer
            # away an agent from its goal (final, not intermediary).
            # The algorithm can therefore fail if two agents are assigned the
            # same final goal and are found to conflict while arriving there.
            if add_constraint!(new_node,constraint,mapf)
                _,_,astartime = low_level_search!(mapf,new_node,distmx,distmx_DP,[get_agent_id(constraint)])
                sleep(0.01)

                # Add time spent on astar
                time_spent_on_astar += astartime

                #If new solution is valid, add it to the queue
                if is_valid(new_node.solution, mapf)
                    #print("Consequently we found the solutions: \n")
                    #print(new_node.solution, "\n")
                    #print("Adding new node to priority queue","\n")
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
        iteration_count += 1
    end

    # If the max number of iterations has passed or if the priority queue has emptied, failure
    print("No Solution Found. Returning default solution")
    return (LowLevelSolution(), typemax(Int),countingtime,time_spent_on_astar,num_interactions[1],iteration_count)
end
