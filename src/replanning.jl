"""Structures and functions used for replanning"""


struct NewAgents
    # New start positions
    start::Vector{Int}
    # start times = time at which the robots appear on the graph and suddenly need to accomplish task k
    start_times::Vector{Float64}
    # goals now contains lists, thanks to which we can set intermediate goals (pick up object, etc)
    goals::Vector{Array{Int64,1}}
    # Each intermediary goal takes some additional time for task completion, which is stored here
    goal_completion_times::Vector{Array{Float64,1}}
end

struct NewAgent
    # New start position
    start::Int
    # start time = time at which the robot appears on the graph and suddenly needs to accomplish task k
    start_time::Float64
    # goals now are lists, thanks to which we can set intermediate goals (pick up object, etc)
    goals::Array{Int64,1}
    # Each intermediary goal takes some additional time for task completion, which is stored here
    goal_completion_times::Array{Float64,1}
end

function merge_MAPFs!(mapf::MAPF,newagents::NewAgents)

    # Concatenate data for these agents
    starts = vcat(mapf.starts,newagents.starts)
    start_times = vcat(mapt.start_times,newagents.start_times)
    goals = vcat(mapf.goals,newagents.goals)
    goal_completion_times = vcat(mapf.goal_completion_times,newagents.goal_completion_times)

    # Save new data into mapf
    mapf.starts = starts
    mapf.start_times = start_times
    mapf.goals = goals
    mapf.goal_completion_times = goal_completion_times

    return # No need to return the mapf, it is modified
end

function reassign_agent_MAPF!(mapf::MAPF,newagent::NewAgent,id::Int64)

    # Replace data for this agent
    mapf.starts[id] = newagent.start
    mapf.start_times[id] = newagent.start_time
    mapf.goals[id] = newagent.goals
    mapf.goal_completion_times[id] = newagent.goal_completion_times

    return # No need to return the mapf, it is modified
end

function jump_in_time!(mapf,time,node)
    """We delete all constraints that happen before time and erase all agents that have reached their goals"""

    # First, we delete all constraints that happen before time
    for agent in keys(node.constraints)
        for v in keys(node.constraints[agent].node_constraints)
            t = node.constraints[agent].node_constraints[v]
            if t < time
                delete!(node.constraints[agent].node_constraints,v)
            end
        end
        for e in keys(node.constraints[agent].edge_constraints)
            t = node.constraints[agent].edge_constraints[e]
            if t < time
                delete!(node.constraints[agent].edge_constraints,e)
            end
        end
    end

    # Simulate where everyone is currently, then
    # robot Ri starts at node j at time ti and its path is now [j j2 j3...]


    # Now the new mapf.starts and mapf.start times could be updated for the old robots with the time of
    # their departure (or arrival + expected stay) from start_node, and start_node itself (current node)



    return
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

function run_particles_until_t!(mapf::MAPF, solution::LowLevelSolution,end_time::Float64)
    """Simulates 1 particle into the given solution.
    returns solution_times, a vector of arrays
    Also returns the truncated solution that is has not yet been through, and
    the new starting time for the first of the nodes."""
    # solution is a vector of vectors of edges, we return for each node
    # the time at which the robot arrives and the time at which it leaves in a
    # tuple. So there are num_edges + 1 tuples in a vector of solution_times.
    lambda = mapf.lambda
    # For each robot, we have an array with dimension 1 being the node index,
    # dimension 2 being the start or end node, and dimension 3 being the index
    # of the particle.
    solution_times = Vector{Array{Float64,3}}()
    num_robots = length(solution)

    for (agent_id, agent_solution) in enumerate(solution)

        #Initialize array of times
        num_edges = length(agent_solution)
        time_array = zeros(Float64, num_edges+1,2) #We took away the last dimension which was number of particles

        # We start at the agent's starting departure time.
        time = mapf.start_times[agent_id]

        # All particles start at the first node at the same starting time.
        time_array[1,1] = time

        # To keep track of the intermediate goals, let's set:
        advancement=1 # Looking for goal 1

        for (edge_idx,edge) in enumerate(agent_solution)

            # In case this edge is the last one we start traversing, we want to
            # record the start time to set it as the new start time.
            arrive_at_node1_time = time

            # ----------------------- Node 1 ---------------------------- #

            #Get the info for the delay of the distribution at this node
            node1_delay = get_prop(mapf.graph, edge.src,:n_delay)
            distrib = Gamma(node1_delay,lambda)

            # Generate a random delay from this Distribution
            node_time = rand(distrib,num_particles)

            # What if our node is an intermediary goal? then we add goal
            # completion time
            task_completion_time = 0.0
            if edge.src == mapf.goals[agent_id][advancement]
                # What if it was actually the final goal?
                if length(mapf.goals[agent_id]) == advancement
                    # This means the goal was actually our first node
                    # The agent thus stops moving and becomes part of the
                    # dynamic environment as long as it does not have any new task
                    # assigned to it. The new task will be assigned before
                    # merging mapfs and the corresponding agent will be removed.
                    mapf.start_time[agent_id] = 0.0
                    mapf.goals[agent_id] = [edge.src]
                    mapf.goal_completion_times[agent_id] = [500.0]
                    mapf.starts[agent_id] = edge.src
                    # Finally, because of this, if the agent is idle and awaiting
                    # a new task, then everyone else will plan to avoid it.



                else
                    # We can get the task completion time
                    task_completion_time = mapf.goal_completion_times[agent_id][advancement]
                    advancement += 1
                end
            end

            # Advance time with delay at node
            time = time + node_time + task_completion_time

            # Update time array with the time at which particles quit node 1
            time_array[edge_idx,2] = time

            # ------------------- Along the edge ------------------------ #

            t_edge = get_prop(mapf.graph,edge,:weight) #scalar
            edge_time = t_edge*ones(Float64,num_particles)

            # Advance time for the particles
            time = time + edge_time

            # If time is bigger than end time, we will not say that the robot
            # has arrived at end_time + remaining. While this would not be a
            # problem in simulation, while we actually run the system we don't
            # want to suppose what will happen and take it as valid. (+ we need
            # the occupancy information at the edge for the remaining time.)
            # Finally, we stop at the previous node and set the start time accordingly
            if time > end_time

                # Set new start time to time at which it entered node 1
                mapf.start_time = arrive_at_node1_time

                # Truncate every node it has traversed from the solution
                # If first iteration, edge_idx = 1, the solution is returned whole
                solution[agent_id] = solution[agent_id][edge_idx:end]

                # Quit the for so that we push time_array into solution_times
                break
            end


            # Add the deterministic time delay to indicate when the particle
            # enters the second node of the edge
            time_array[edge_idx+1,1] = time


            # ----------------------- Node 2 ---------------------------- #
            # ------------- Special case of target node ----------------- #
            if edge_idx == length(agent_solution)
                # Get the info for the delay of the distribution at this node
                node1_delay = get_prop(mapf.graph, edge.dst,:n_delay)
                distrib = Gamma(node1_delay,lambda)

                # Generate a random delay from this Distribution
                node_time = rand(distrib,num_particles)

                # Advance time for the particles
                time = time + node_time

                # Update time array with the time at which particles quit node 2
                time_array[edge_idx+1,2] = time
            end

        end
        push!(solution_times,time_array)

    end
    return solution_times
end
