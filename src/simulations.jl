export
    run_experiment_set_CRCBS,
    Experiment_Set,
    Experiment

struct Experiment_parameters
    name::String
    num_agents::Tuple
    grid_x::Tuple
    grid_y::Tuple
    filling_density::Tuple
    lambda::Float64
    epsilon::Float64
    t_delay::Float64
    num_experiments::Int64
end

struct Experiment
    """Trials identically conducted"""
    Id::Int64
    CBS::Bool
    success::Bool

    # Experiment properties
    num_trials::Int64
    num_agents::Int64
    cbs_time_steps::Int64
    mapf::MAPF

    # Result storage
    solving_time::Float64 #Time it took to find a solution
    solution_cost::Float64 #Cost of computer solution in ideal case
    global_cp::Float64
    conflict_counts_locally::Vector{Float64}
    probability_error::Tuple{Float64,Float64} #Mean, std

end

mutable struct Experiment_Set
    name::String
    data::Vector{Experiment}
end

function save_experiment_set(Exp::Experiment_Set)
    jldopen(string("../experiments/results/001",Exp.name,".jld"),"w") do file
        addrequire(file,CRCBS)
        write(file,"data",Exp)
    end
end

# returns solution_times
function run_particles(mapf::MAPF, solution::LowLevelSolution,num_particles::Int64)
    """Simulates n particles into the given solution.
    returns solution_times, a vector of arrays"""
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
        time_array = zeros(Float64, num_edges+1,2,num_particles)

        time = 0.0*ones(Float64,num_particles) #For the moment.

        # All particles start at the first node at the same starting time.
        time_array[1,1,:] = time

        for (edge_idx,edge) in enumerate(agent_solution)

            # ----------------------- Node 1 ---------------------------- #

            #Get the info for the delay of the distribution at this node
            node1_delay = get_prop(mapf.graph, edge.src,:n_delay)
            distrib = Gamma(node1_delay,lambda)

            # Generate a random delay from this Distribution
            node_time = rand(distrib,num_particles)

            # Advance time for the particles
            time = time + node_time

            # Update time array with the time at which particles quit node 1
            time_array[edge_idx,2,:] = time

            # ------------------- Along the edge ------------------------ #

            t_edge = get_prop(mapf.graph,edge,:weight) #scalar
            edge_time = t_edge*ones(Float64,num_particles)

            # Advance time for the particles
            time = time + edge_time

            # Add the deterministic time delay to indicate when num_particles
            # enter the second node of the edge
            time_array[edge_idx+1,1,:] = time

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
                time_array[edge_idx+1,2,:] = time
            end

        end

    end
    return solution_times
end

# returns (global_cp, conflict_counts_locally, mean_prob_err,std_prob_err)
function get_conflict_stats(mapf::MAPF,paths::LowLevelSolution,solution_times::Vector{Array{Float64,3}},num_particles::Int64)
    """We return the global conflict probability, a list of local conflict
    likelihoods, and for sanity check we also look at the error in between the
    calculated conflict probability and the statistic"""

    conflict_counts_locally = Vector{Int64}()
    conflict_probability_error = Vector{Float64}()

    epsilon = mapf.epsilon
    lambda = mapf.lambda

    clear_graph_occupancy!(mapf::MAPF)

    # The first step is to fill the graph with occupancy information
    for (robot_id,robotpath) in enumerate(paths)
        fill_graph_with_path!(robot_id,robotpath,mapf)
    end

    # ----- Node conflicts ----- #
    for v in vertices(mapf.graph)

        #Get node information
        nn = get_prop(mapf.graph, v, :n_delay)
        occupancy = get_prop(mapf.graph,v,:occupancy)

        # There is interaction at this node
        if length(occupancy) >= 2
            list_of_occupants = collect(keys(occupancy))
            pairs_of_occupants = collect(combinations(list_of_occupants,2))
            for (r1,r2) in pairs_of_occupants

                #Get Collision Probability
                (n1,t1) = occupancy[r1]
                (n2,t2) = occupancy[r2]
                (cp,err) = get_collision_probability_node(n1,t1,n2,t2,nn,lambda)

                # Count conflicts
                #Index of node v traversal for robots 1 and 2
                print("\n")
                print(collect(paths[r1]))
                print("\n")
                print(typeof(Iterators.flatten(Array(paths[r1]))))
                print("\n")
                print(collect(Iterators.flatten(Array(paths[r1]))))
                r1_flat_path = collect(Iterators.flatten(paths[r1]))
                r2_flat_path = collect(Iterators.flatten(paths[r2]))
                v_idx_r1 = div(findfirst(r1_flat_path.==v),2)+1
                v_idx_r2 = div(findfirst(r2_flat_path.==v),2)+1

                r1_arrivals = solution_times[r1][v_idx_r1,1,:]
                r1_departures = solution_times[r1][v_idx_r1,2,:]
                r2_arrivals = solution_times[r2][v_idx_r2,1,:]
                r2_departures = solution_times[r2][v_idx_r2,2,:]

                num_conflicts = length(findall( ((r2_departures-r1_arrivals).>0) .& ((r1_departures-r2_arrivals).>0)   ))
                push!(conflict_counts_locally, num_conflicts)

                # Compare conflict occurence to probability
                experimental_local_conflict_probability = num_conflicts/num_particles
                probability_error = abs(experimental_local_conflict_probability-cp)

                push!(conflict_probability_error,probability_error)

            end
        end
    end

    # ----- Edge conflicts ----- #
    for e in edges(mapf.graph)

        #Get edge information. We consider confrontations as robots arriving in
        #opposite directions only
        t_edge = get_prop(mapf.graph,e,:weight)
        occupancy = get_prop(mapf.graph,e,:occupancy)
        reverse_edge = Edge(e.dst,e.src)
        reverse_occupancy = get_prop(mapf.graph,reverse_edge,:occupancy)

        # There is interaction at this edge
        if length(occupancy)*length(reverse_occupancy) >= 1
            occupants_1 = keys(occupancy)
            occupants_2 = keys(reverse_occupancy)
            for robot1_id in occupants_1
                for robot2_id in occupants_2
                    if robot1_id != robot2_id
                        # Get Collision Probability
                        (n1,t1) = occupancy[robot1_id]
                        (n2,t2) = reverse_occupancy[robot2_id]
                        (cp,err) = get_collision_probability_edge(n1,t1,n2,t2,t_edge,lambda)

                        # Get indexes for v1 and v2 for robot1 and robot2
                        r1_flat_path = collect(Iterators.flatten(paths[robot1_id]))
                        r2_flat_path = collect(Iterators.flatten(paths[robot2_id]))
                        v1_idx_r1 = div(findfirst(r1_flat_path.==e.src),2)+1
                        v1_idx_r2 = div(findfirst(r2_flat_path.==e.src),2)+1
                        v2_idx_r1 = div(findfirst(r1_flat_path.==e.dst),2)+1
                        v2_idx_r2 = div(findfirst(r2_flat_path.==e.dst),2)+1

                        #Count conflicts that arrive on the edge but not at the nodes
                        # robot 1 arrives at node 2 after robot 2 leaves node 2
                        # and robot 2 arrives at node 1 after robot 1 leaves node 2
                        r1_arrivals = solution_times[r1][v1_idx_r1,2,:]
                        r1_departures = solution_times[r1][v2_idx_r1,1,:]
                        r2_arrivals = solution_times[r2][v2_idx_r2,2,:]
                        r2_departures = solution_times[r2][v1_idx_r2,1,:]

                        num_conflicts = length(findall( ((r2_departures-r1_arrivals).>0) .& ((r1_departures-r2_arrivals).>0)   ))
                        push!(conflict_counts_locally, num_conflicts)

                        # Compare conflict occurence to probability
                        experimental_local_conflict_probability = num_conflicts/num_particles
                        probability_error = abs(experimental_local_conflict_probability-cp)

                        push!(conflict_probability_error,probability_error)

                    end
                end
            end
        end
    end

    # get the mean and std for conflict probability probability_error
    mean_prob_err = mean(conflict_probability_error)
    std_prob_err = std(conflict_probability_error,corrected=true)

    # Get the global conflict probability
    global_cp = 1 - prod(1 .- conflict_counts_locally)

    clear_graph_occupancy!(mapf)
    return (global_cp, conflict_counts_locally, mean_prob_err,std_prob_err)
end

# returns mapf
function create_grid_mapf(num_robots::Int64,
    grid_size::Tuple{Int64,Int64},
    filling_density::Float64,
    lambda::Float64,
    epsilon::Float64,
    t_delay::Float64
    )
    """Outputs the mapf to use for simulation"""
    G = initialize_full_grid_graph_CT(grid_size[1],grid_size[2],filling_density)

    #Choose 2*num_robots unique vertices at random
    startsandgoals = shuffle(vertices(G))[1:2*num_robots]
    starts = startsandgoals[1:num_robots]
    goals = startsandgoals[num_robots+1:end]

    mapf = MAPF(G,starts,goals,lambda,epsilon,t_delay)
    return mapf
end

function run_experiment_set_CRCBS(name::String,
    num_agents::Tuple,
    grid_x::Tuple,
    grid_y::Tuple,
    filling_density::Tuple,
    lambda::Float64,
    epsilon::Float64,
    t_delay::Float64,
    num_experiments::Int64,
    num_trials::Int64
    )

    print("Started \n")

    #Create the parameters of all experiments
    #num_agents
    num_agent_list = shuffle(collect(num_agents))[1:min(num_experiments,length(num_agents))]
    d = num_experiments-length(num_agents)
    append!(num_agent_list,sample(collect(num_agents),d*(sign(d)==1)))
    #grid size x
    grid_x_list = shuffle(collect(grid_x))[1:min(num_experiments,length(grid_x))]
    d = num_experiments-length(grid_x)
    append!(grid_x_list,sample(collect(grid_x),d*(sign(d)==1)))
    #grid size y
    grid_y_list = shuffle(collect(grid_y))[1:min(num_experiments,length(grid_y))]
    d = num_experiments-length(grid_y)
    append!(grid_y_list,sample(collect(grid_y),d*(sign(d)==1)))
    #filling density
    filling_density_list = shuffle(collect(filling_density))[1:min(num_experiments,length(filling_density))]
    d = num_experiments-length(filling_density)
    append!(filling_density_list,sample(collect(filling_density),d*(sign(d)==1)))

    # Save experiment parameters for when we will try doing it with CBS
    Expp = Experiment_parameters(name,num_agents,grid_x,grid_y,filling_density,lambda,epsilon,t_delay,num_experiments)

    print("Check1 \n")

    jldopen(string("../experiments/experiment_parameters/001",name,".jld"),"w") do file
        #addrequire(file,CRCBS)
        write(file,"parms",Expp)
    end



    #Now we can create the experiment set
    data = Vector{}()
    print("Check2 \n")
    for i in 1:num_experiments
        mapf = create_grid_mapf(num_agent_list[i],
        (grid_x_list[i],grid_y_list[i]),
        filling_density_list[i],
        lambda,epsilon,t_delay)

        # Run CRCBS
        b = @timed(CTCBS(mapf))

        #Get time, solution and cost
        execution_time = b[2]
        solution = b[1][1]
        cost = b[1][2]
        success = true
        if cost >=  typemax(Int)
            success = false
        end

        #Run particles for simulation
        solution_times = run_particles(mapf, solution,num_trials)

        #Get stats
        (global_cp, conflict_counts_locally, mean_prob_err,std_prob_err) = get_conflict_stats(mapf,solution,solution_times,num_trials)
        probability_error = (mean_prob_err,std_prob_err)

        #Create experiment instance
        print("type of i")
        print(typeof(i))
        new_experiment = Experiment(i,false,success,num_trials,num_agents,-1,mapf,execution_time,cost,global_cp,conflict_counts_locally,probability_error)

        # Add it to the list of experiments from the set
        push!(data, new_experiment)

    end

    print("Check3 \n")

    # save the set of experiments
    save_experiment_set(data)



end
