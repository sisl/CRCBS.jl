export
    run_experiment_set_CRCBS,
    run_convergence_tests_CRCBS,
    Experiment_parameters,
    run_problem

struct Experiment_parameters
    name::String
    lambda::Float64
    epsilon::Float64
    t_delay::Float64
    num_particles::Int64
    vs::Vector{Int}
    es::Vector{Tuple{Int,Int}}
    starts::Vector{Int}
    goals::Vector{Int}
end


# -------------------- SIMULATION  and PROCESSING ---------------------------- #
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
        push!(solution_times,time_array)

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
            println("Possible interactions")
            sleep(0.02)
            list_of_occupants = collect(keys(occupancy))
            pairs_of_occupants = collect(combinations(list_of_occupants,2))
            for (r1,r2) in pairs_of_occupants

                #println("Interaction!")

                #Get Collision Probability
                (n1,t1) = occupancy[r1]
                (n2,t2) = occupancy[r2]
                (cp,err) = get_collision_probability_node(n1,t1,n2,t2,nn,lambda)
                # if cp==0
                #     println("found probability of zero with parameters")
                #     println("n1: ", n1)
                #     println("t1: ", t1)
                #     println("n2: ", n2)
                #     println("t2: ", t2)
                #     println("nn: ", nn)
                #     println("lambda: ", lambda)
                # end


                # Count conflicts
                #Index of node v traversal for robots 1 and 2

                path_r1 = []
                for e in paths[r1]
                    append!(path_r1, e.src)
                    append!(path_r1, e.dst)
                end

                path_r2 = []
                for e in paths[r2]
                    append!(path_r2, e.src)
                    append!(path_r2, e.dst)
                end

                v_idx_r1 = div(findfirst(path_r1.==v),2)+1
                v_idx_r2 = div(findfirst(path_r2.==v),2)+1

                r1_arrivals = solution_times[r1][v_idx_r1,1,:]
                r1_departures = solution_times[r1][v_idx_r1,2,:]
                r2_arrivals = solution_times[r2][v_idx_r2,1,:]
                r2_departures = solution_times[r2][v_idx_r2,2,:]

                num_conflicts = length(findall( ((r2_departures-r1_arrivals).>0) .& ((r1_departures-r2_arrivals).>0)   ))

                if num_conflicts > 1
                    println("\n I found", num_conflicts, " conflicts at a node!")
                    println("This is out of ", num_particles, " trials, and the collision probability is ", cp)
                    #push!(conflict_counts_locally, num_conflicts)

                    push!(conflict_counts_locally, num_conflicts)

                    # Compare conflict occurence to probability
                    experimental_local_conflict_probability = num_conflicts/num_particles
                    probability_error = abs(experimental_local_conflict_probability-cp)

                    push!(conflict_probability_error,probability_error)
                end

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
        if length(occupancy)*length(reverse_occupancy) >= 1 && e.src < e.dst

            occupants_1 = keys(occupancy)
            occupants_2 = keys(reverse_occupancy)

            # println("Edge ", e, " found occupants:")
            # println(occupants_1)
            # println("and reverse occupants:")
            # println(occupants_2)

            for robot1_id in occupants_1
                for robot2_id in occupants_2
                    if robot1_id != robot2_id
                        println("Interaction at edge!")



                        # Get Collision Probability
                        (n1,t1) = occupancy[robot1_id]
                        (n2,t2) = reverse_occupancy[robot2_id]
                        (cp,err) = get_collision_probability_edge(n1,t1,n2,t2,t_edge,lambda)


                        path_r1 = []
                        for ed in paths[robot1_id]
                            append!(path_r1, ed.src)
                            append!(path_r1, ed.dst)
                        end

                        path_r2 = []
                        for ed in paths[robot2_id]
                            append!(path_r2, ed.src)
                            append!(path_r2, ed.dst)
                        end

                        # Get indexes for v1 and v2 for robot1 and robot2
                        v1_idx_r1 = div(findfirst(path_r1.==e.src),2)+1
                        v1_idx_r2 = div(findfirst(path_r2.==e.src),2)+1
                        v2_idx_r1 = div(findfirst(path_r1.==e.dst),2)+1
                        v2_idx_r2 = div(findfirst(path_r2.==e.dst),2)+1

                        #Count conflicts that arrive on the edge but not at the nodes
                        # robot 1 arrives at node 2 after robot 2 leaves node 2
                        # and robot 2 arrives at node 1 after robot 1 leaves node 2
                        r1_arrivals = solution_times[robot1_id][v1_idx_r1,2,:]
                        r1_departures = solution_times[robot1_id][v2_idx_r1,1,:]
                        r2_arrivals = solution_times[robot2_id][v2_idx_r2,2,:]
                        r2_departures = solution_times[robot2_id][v1_idx_r2,1,:]

                        num_conflicts = length(findall( ((r2_departures-r1_arrivals).>0) .& ((r1_departures-r2_arrivals).>0)   ))

                        if num_conflicts > 1
                            println("\n I found", num_conflicts, " conflicts!")
                            println("This is out of ", num_particles, " trials, and the collision probability is ", cp)
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
    end

    # get the mean and std for conflict probability probability_error
    mean_prob_err = mean(conflict_probability_error)
    std_prob_err = std(conflict_probability_error,corrected=true)

    # Get the global conflict probability
    global_cp = 1 - prod(1 .- conflict_counts_locally/num_particles)

    clear_graph_occupancy!(mapf)
    return (global_cp, conflict_counts_locally, mean_prob_err,std_prob_err)
end

# returns (global_cp, conflict_counts_locally, mean_prob_err,std_prob_err)
function push_conflict_stats!(mapf::MAPF,paths::LowLevelSolution,solution_times::Vector{Array{Float64,3}},num_particles_list,data)
    """Pushes probability convergence information into data"""

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
                # if cp==0
                #     println("found probability of zero with parameters")
                #     println("n1: ", n1)
                #     println("t1: ", t1)
                #     println("n2: ", n2)
                #     println("t2: ", t2)
                #     println("nn: ", nn)
                #     println("lambda: ", lambda)
                # end


                # Count conflicts
                #Index of node v traversal for robots 1 and 2
                path_r1 = []
                for e in paths[r1]
                    append!(path_r1, e.src)
                    append!(path_r1, e.dst)
                end

                path_r2 = []
                for e in paths[r2]
                    append!(path_r2, e.src)
                    append!(path_r2, e.dst)
                end

                v_idx_r1 = div(findfirst(path_r1.==v),2)+1
                v_idx_r2 = div(findfirst(path_r2.==v),2)+1

                r1_arrivals = solution_times[r1][v_idx_r1,1,:]
                r1_departures = solution_times[r1][v_idx_r1,2,:]
                r2_arrivals = solution_times[r2][v_idx_r2,1,:]
                r2_departures = solution_times[r2][v_idx_r2,2,:]

                # Looks at conflict occurences with gradually more information
                for num_trials in num_particles_list
                    num_conflicts = length(findall( ((r2_departures[1:num_trials]-r1_arrivals[1:num_trials]).>0) .& ((r1_departures[1:num_trials]-r2_arrivals[1:num_trials]).>0)   ))
                    experimental_probability = num_conflicts/num_trials
                    push!(data,[cp,experimental_probability,num_trials])
                end
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
        if length(occupancy)*length(reverse_occupancy) >= 1 && e.src < e.dst

            occupants_1 = keys(occupancy)
            occupants_2 = keys(reverse_occupancy)

            # println("Edge ", e, " found occupants:")
            # println(occupants_1)
            # println("and reverse occupants:")
            # println(occupants_2)

            for robot1_id in occupants_1
                for robot2_id in occupants_2
                    if robot1_id != robot2_id

                        # Get Collision Probability
                        (n1,t1) = occupancy[robot1_id]
                        (n2,t2) = reverse_occupancy[robot2_id]
                        (cp,err) = get_collision_probability_edge(n1,t1,n2,t2,t_edge,lambda)


                        path_r1 = []
                        for ed in paths[robot1_id]
                            append!(path_r1, ed.src)
                            append!(path_r1, ed.dst)
                        end

                        path_r2 = []
                        for ed in paths[robot2_id]
                            append!(path_r2, ed.src)
                            append!(path_r2, ed.dst)
                        end

                        # Get indexes for v1 and v2 for robot1 and robot2
                        v1_idx_r1 = div(findfirst(path_r1.==e.src),2)+1
                        v1_idx_r2 = div(findfirst(path_r2.==e.src),2)+1
                        v2_idx_r1 = div(findfirst(path_r1.==e.dst),2)+1
                        v2_idx_r2 = div(findfirst(path_r2.==e.dst),2)+1

                        #Count conflicts that arrive on the edge but not at the nodes
                        # robot 1 arrives at node 2 after robot 2 leaves node 2
                        # and robot 2 arrives at node 1 after robot 1 leaves node 2
                        r1_arrivals = solution_times[robot1_id][v1_idx_r1,2,:]
                        r1_departures = solution_times[robot1_id][v2_idx_r1,1,:]
                        r2_arrivals = solution_times[robot2_id][v2_idx_r2,2,:]
                        r2_departures = solution_times[robot2_id][v1_idx_r2,1,:]

                        # Looks at conflict occurences with gradually more information
                        for num_trials in num_particles_list
                            num_conflicts = length(findall( ((r2_departures[1:num_trials]-r1_arrivals[1:num_trials]).>0) .& ((r1_departures[1:num_trials]-r2_arrivals[1:num_trials]).>0)   ))
                            experimental_probability = num_conflicts/num_trials
                            push!(data,[cp,experimental_probability,num_trials])
                        end
                    end
                end
            end
        end
    end

    return
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

    print("Check1 \n")

    #Now we can create the experiment set
    data = DataFrame(name=String[],
    Id=Int64[],
    CBS=Bool[],
    success=Bool[],

    #MAPF contents
    lambda=Float64[],
    epsilon=Float64[],
    t_delay=Float64[],

    # Experiment properties
    num_trials=Int64[],
    num_agents=Int64[],
    cbs_time_steps=Int64[],
    num_nodes=Int64[],
    num_edges=Int64[],
    #mapf::MAPF

    # Result storage
    solving_time=Float64[], #Time it took to find a solution
    counting_time=Float64[],
    time_spent_on_astar=Float64[],
    num_interactions=Int64[],

    solution_cost=Float64[], #Cost of computer solution in ideal case
    global_cp=Float64[],
    conflict_counts_locally=Vector{Float64}[],
    probability_error=Tuple{Float64,Float64}[]) #Mean, std


    print("Check2 \n")
    sleep(0.05)

    for i in 1:num_experiments

        # Create MAPF
        mapf = create_grid_mapf(num_agent_list[i],
        (grid_x_list[i],grid_y_list[i]),
        filling_density_list[i],
        lambda,epsilon,t_delay)
        n_nodes = length(vertices(mapf.graph))
        n_edges = div(length(edges(mapf.graph)),2) #Edge 1 - 2 and 2 - 1 are the same

        # Save mapf for CBS experiment
        exp_parms = mapf_to_exp_parameters(string(name,"_",string(i)),num_trials,mapf)

        jldopen(string("../experiments/experiment_parameters/",string(name,"_",i),".jld"),"w") do file
            write(file,"parms",exp_parms)
        end

        # Save metagraph
        # This did not work
        # savegraph(string("../experiments/graphs/", string(name,"_",string(i)), ".mg"), mapf.graph)
        # Instead we will save a txt mapping the vertex number to its x and y
        graph_to_txt(mapf.graph,string(name,"_",string(i)))

        # Run CRCBS
        b = @timed(CTCBS(mapf))
        println("Time spent performing optimization: ", b[2])
        sleep(0.05)

        #Get time, solution and cost
        execution_time = b[2]
        solution = b[1][1]
        cost = b[1][2]
        success = true
        if cost >=  typemax(Int)
            success = false
        end

        solution_to_txt(solution, string(name,"_",string(i)))

        countingtime = b[1][3]
        time_spent_on_astar = b[1][4]
        num_interactions = b[1][5][1]

        #Run particles for simulation
        solution_times = run_particles(mapf, solution,num_trials)

        #Get stats
        (global_cp, conflict_counts_locally, mean_prob_err,std_prob_err) = get_conflict_stats(mapf,solution,solution_times,num_trials)
        probability_error = (mean_prob_err,std_prob_err)

        # Add it to the list of experiments from the set
        push!(data, [string(name,"_",string(i)),i,false,success,mapf.lambda,mapf.epsilon,mapf.t_delay,num_trials,num_agent_list[i],-1,n_nodes, n_edges,execution_time,countingtime,time_spent_on_astar,num_interactions,cost,global_cp,conflict_counts_locally,probability_error])

    end

    print("Check3 \n")

    return data
end

function run_convergence_tests_CRCBS(name::String,
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

    print("Check1 \n")

    #Now we can create the experiment set
    data = DataFrame(theoretical_probability = Float64[],
    experimental_probability = Float64[],
    num_samples = Int64[]) #Mean, std


    print("Check2 \n")

    num_trial_list = [1,5,10,20,50,100,200,500,1000,2000,5000,10000,20000,50000,100000]
    num_trials = 100000

    for i in 1:num_experiments

        # Create MAPF
        mapf = create_grid_mapf(num_agent_list[i],
        (grid_x_list[i],grid_y_list[i]),
        filling_density_list[i],
        lambda,epsilon,t_delay)
        n_nodes = length(vertices(mapf.graph))
        n_edges = div(length(edges(mapf.graph)),2) #Edge 1 - 2 and 2 - 1 are the same

        # Save mapf for CBS experiment
        exp_parms = mapf_to_exp_parameters(string(name,"_",string(i)),num_trials,mapf)

        # Run CRCBS
        b = @timed(CTCBS(mapf))
        println("Time spent performing optimization: ", b[2])

        #Get time, solution and cost
        execution_time = b[2]
        solution = b[1][1]
        cost = b[1][2]
        success = true
        if cost >=  typemax(Int)
            success = false
        end



        #Run particles for simulation
        solution_times = run_particles(mapf, solution, num_trials)

        #Get stats
        push_conflict_stats!(mapf,solution,solution_times,num_trial_list,data)

    end

    print("Check3 \n")

    return data
end
# ---------------------------------------------------------------------------- #


# ----------------------- DATA STORAGE FUNCTIONS ----------------------------- #

function mapf_to_exp_parameters(name,num_particles,mapf)
    vs = [v for v in vertices(mapf.graph)]
    es = [(e.src,e.dst) for e in edges(mapf.graph)]
    return Experiment_parameters(name,mapf.lambda,mapf.epsilon,mapf.t_delay,num_particles,vs,es,mapf.starts,mapf.goals)
end

function solution_to_txt(solution, id)
    io = open(string("../experiments/solutions/CRCBS_",string(id),".txt"), "w")
    for path in solution
        for e in path
            print(io, string(string(e.src), " ", string(e.dst), ";"))
        end
        println(io, "")
    end
    close(io)
    return
end

function graph_to_txt(G,filename)
    io = open(string("../experiments/graphs/",string(filename),".txt"), "w")
    for v in vertices(G)
        x = get_prop(G,v,:x)
        y = get_prop(G,v,:y)
        println(io,string(v," ",x," ",y))
    end
    close(io)
    return
end

function load_graph_dict(filename)
    f = open(string("../experiments/graphs/",string(filename),".txt"))
    Gdict = Dict()
    for line in eachline(f)
        values = split(line)
        v = parse(Int64,values[1])
        x = parse(Float64,values[2])
        y = parse(Float64,values[3])
        Gdict[v] = (x,y)
    end
    return Gdict
end

function load_experiment_parameters(file)
    """Creates an experiment set with all the experiment sets"""
    experiment_vector = Vector{}()


    exp_set = load(string("../experiments/experiment_parameters/",file,".jld"))
    #println(exp_set)
    parameters = exp_set["parms"]

    #parameters = Experiment_parameters(experiment_vector)
    return parameters
end

function run_problem(name; save_simulation=false)
    lambda = 1.0
    epsilon = 0.0001
    t_delay = 1.0
    exp_parms = load_experiment_parameters(name)
    G = MetaGraph()
    for v in exp_parms.vs
        add_vertex!(G)

    end
    for v in vertices(G)
        set_prop!(G, v,:n_delay, 1.0)
    end
    for e in exp_parms.es
        add_edge!(G,e[1],e[2])
        set_prop!(G, Edge(e[1],e[2]), :weight, 1.0)
    end
    mapf = MAPF(G,exp_parms.starts,exp_parms.goals,lambda,epsilon,t_delay)
    a = @timed(CTCBS(mapf))
    llsolution = a[1][1]
    cost = a[1][2]
    astartime = a[1][3]
    fnctime = a[1][4]
    computation_time = a[2]
    solution_to_txt(llsolution, name)

    if save_simulation
        solution_times = run_particles(mapf, llsolution,20) #20 particles
        jldopen(string("../experiments/simulations/",string(name,"_",i),".jld"),"w") do file
            write(file,"times",solution_times)
        end
    end


    return (llsolution,cost,computation_time)
end

function load_solution_times(file)
    """Creates an experiment set with all the experiment sets"""

    times_f = load(string("../experiments/simulations/",file,".jld"))
    #println(exp_set)
    solution_times = times_f["times"]

    #parameters = Experiment_parameters(experiment_vector)
    return solution_times
end
# ---------------------------------------------------------------------------- #
