using CRCBS
using LightGraphs, MetaGraphs
using GraphUtils
using DataFrames
using TOML

# initialize varying epsilon experiments
function gen_epsilon_experiments()
    GRID_SIZE = 6
    vtx_grid = GraphUtils.initialize_dense_vtx_grid(GRID_SIZE,GRID_SIZE)
    #   1   2   3   4   5   6
    #   7   8   9  10  11  12
    #  13  14  15  16  17  18
    #  19  20  21  22  23  24
    #  25  26  27  28  29  30
    #  31  32  33  34  35  36
    G = initialize_full_grid_graph_CT(GRID_SIZE-1,GRID_SIZE-1)
    agent_1_start_list  = [vtx_grid[i,1] for i in 2:GRID_SIZE-1]
    agent_1_goal_list   = [vtx_grid[i,GRID_SIZE] for i in 2:GRID_SIZE-1]
    agent_2_start_list  = [vtx_grid[1,i] for i in 2:GRID_SIZE-1]
    agent_2_goal_list   = [vtx_grid[GRID_SIZE,i] for i in 2:GRID_SIZE-1]
    epsilons = 0.025:0.025:0.975
    lambda = 0.5
    t_delay = 0.5

    config_dicts = []
    for (start1,goal1) in zip(agent_1_start_list,agent_1_goal_list)
        for (start2,goal2) in zip(agent_2_start_list,agent_2_goal_list)
            starts = [start1,start2]
            goals = [goal1,goal2]
            for epsilon in epsilons
                # mapf = MAPF(G,starts,goals,lambda,epsilon,t_delay)
                push!(config_dicts,
                    Dict(
                        :start1=>start1,
                        :start2=>start2,
                        :goal1=>goal1,
                        :goal2=>goal2,
                        :epsilon=>epsilon,
                        :lambda=>lambda,
                        :t_delay=>t_delay,
                        )
                )
            end
        end
    end
    return G, config_dicts
end

G, problem_configs = gen_epsilon_experiments()

base_path = joinpath("/home/kylebrown/Desktop/STTCBS/CRCBS.jl/experiments","epsilon_experiments")
base_problem_path = joinpath(base_path,"problems")
base_results_path = joinpath(base_path,"results")
mkpath(base_problem_path)
mkpath(base_results_path)

# write problem configs
for (i,config) in enumerate(problem_configs)
    prob_path = joinpath(base_problem_path,"problem_$i.toml")
    if !isfile(prob_path)
        open(prob_path,"w") do io
            TOML.print(io,Dict(string(k)=>v for (k,v) in config))
        end
    end
end

results_df = DataFrame(
    :start1=>Int[],
    :start2=>Int[],
    :goal1=>Int[],
    :goal2=>Int[],
    :epsilon=>Float64[],
    :lamba=>Float64[],
    :t_delay=>Float64[],
    :solution=>Vector{Vector{Int}}[],
    :global_cp=>Float64[],
    :conflict_counts_locally=>Int[],
    :mean_prob_err=>Float64[],
    :std_prob_err=>Float64[],

    :runtime=>Float64[],
    :countingtime=>Float64[],
    :time_spent_on_astar=>Float64[],
    :num_interactions=>Int[],
    :iteration_count=>Int[],
)

NUM_TRIALS = 100
for (i,config) in enumerate(problem_configs)
    # unpack
    start1 = config[:start1]
    start2 = config[:start2]
    goal1 = config[:goal1]
    goal2 = config[:goal2]
    epsilon = config[:epsilon]
    lambda = config[:lambda]
    t_delay = config[:t_delay]
    # define MAPF
    mapf = MAPF(G,[start1,start2],[goal1,goal2],lambda,epsilon,t_delay)
    # solve MAPF
    (solution, 
    cost, 
    countingtime, 
    time_spent_on_astar, 
    num_interactions, 
    iteration_count), runtime, _ = @timed CTCBS(mapf)
    # compute statistics
    solution_times = CRCBS.run_particles(mapf, solution, NUM_TRIALS)
    (global_cp, 
        conflict_counts_locally, 
        mean_prob_err, 
        std_prob_err) = CRCBS.get_conflict_stats(mapf,solution,solution_times,NUM_TRIALS)
    # save results
    results = Dict(
        :start1=>start1,
        :start2=>start2,
        :goal1=>goal1,
        :goal2=>goal2,
        :epsilon=>epsilon,
        :lamba=>lambda,
        :t_delay=>t_delay,
        :solution=>map(p->map(e->[e.src,e.dst],p),solution),
        :global_cp=>global_cp,
        :conflict_counts_locally=>conflict_counts_locally,
        :mean_prob_err=>mean_prob_err,
        :std_prob_err=>std_prob_err,

        :runtime=>runtime,
        :countingtime=>countingtime,
        :time_spent_on_astar=>time_spent_on_astar,
        :num_interactions=>num_interactions,
        :iteration_count=>iteration_count,
    )
    results_path = joinpath(base_results_path,"results_$i.toml")
    if !isfile(results_path)
        open(results_path,"w") do io
            TOML.print(io,Dict(string(k)=>val for (k,val) in results))
        end
    end
    
end


