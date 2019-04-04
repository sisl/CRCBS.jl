using LightGraphs, MetaGraphs
using JLD

struct Experiment
    """Trials identically conducted"""
    Id::Int64

    # Experiment properties
    num_trials::Int64
    num_agents::Int64
    cbs_time_steps::Int64
    mapf::MAPF

    # Result storage
    execution_time::Float64 #Time it took to find a solution
    conflicts::Vector

end

mutable struct Experiment_Set
    name::String
    data::Vector{Experiment}
end

function save_experiment_set(Exp::Experiment_Set)
    jldopen("/results/001"+Exp.name+".jld","w") do file
        addrequire(file,CRCBS)
        write(file,"data",Exp)
    end
end

function run_particles(mapf::MAPF, solution::LowLevelSolution,num_particles::Int64)
    """Simulates n particles into the given solution"""
    # Solution is a vector of vectors of edges, we return for each node
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
            node1_delay = get_prop(mapf.graph, edge[1],:n_delay)
            distrib = Gamma(n_delay,lambda)

            # Generate a random delay from this Distribution
            node_time = rand(d,num_particles)

            # Advance time for the particles
            time = time + node_time

            # Update time array with the time at which particles quit node 1
            time_array[2*edge_idx-1,2,:] = time

            # ------------------- Along the edge ------------------------ #

            t_edge = get_prop(mapf.graph,edge,:weight) #scalar
            edge_time = t_edge*ones(Float64,num_particles)

            # Advance time for the particles
            time = time + edge_time

            # Add the deterministic time delay to indicate when num_particles
            # enter the second node of the edge
            time_array[2*edge_idx,1,:] = time

            # ----------------------- Node 2 ---------------------------- #

            #Get the info for the delay of the distribution at this node
            node1_delay = get_prop(mapf.graph, edge[2],:n_delay)
            distrib = Gamma(n_delay,lambda)

            # Generate a random delay from this Distribution
            node_time = rand(d,num_particles)

            # Advance time for the particles
            time = time + node_time

            # Update time array with the time at which particles quit node 2
            time_array[2*edge_idx,2,:] = time

        end

        # Now that the time array is complete we push it into the vector
        push!(solution_times,time_array)

    end
    return solution_times
end


function count_conflicts(mapf::MAPF,paths::LowLevelSolution,solution_times::Vector{Array{Float64,3}},num_particles::Int64)

    conflicts = zeros(Int64,num_particles)

    clear_graph_occupancy!(mapf::MAPF)

    # The first step is to fill the graph with occupancy information
    for (robot_id,robotpath) in enumerate(paths)
        fill_graph_with_path!(robot_id,robotpath,mapf)
    end

    # STOPPED HERE CONTINUE TOMORROW


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
                (n1,t1) = occupancy[r1]
                (n2,t2) = occupancy[r2]
                (cp,err) = get_collision_probability_node(n1,t1,n2,t2,nn,lambda)

                # Conflict is likely and higher than all previously found
                if cp > maximum([epsilon,node_conflict_p])
                    node_conflict_p = cp
                    node_conflict = NodeConflict(r1,r2,v,t1,t2,cp)
                end
            end
        end


end
