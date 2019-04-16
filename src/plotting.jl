export
    plot_SVtime_vs_nrobots


function plot_SVtime_vs_nrobots(files)
    exp_set = load_experiment_set("svtimeVSnrobots", files)
    svtimes = []
    n_robots = []
    for exp in exp_set
        append!(svtimes, exp.solving_time)
        append!(n_robots, exp.num_agents)
    end
    plot(n_robots,svtimes)
end
