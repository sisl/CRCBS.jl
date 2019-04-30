export
    plot_SVtime_vs_nrobots,
    plot_optimal_nominal_paths


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

function plot_optimal_nominal_paths(file)
    Gdict = load_graph_dict(file)
    directory = string("../experiments/solutions/CRCBS_",file,".txt")
    f = open(directory)
    plot(title = string("Solution to ", file),leg=false)
    for (k,ln) in enumerate(eachline(f))
        edgelist = split(ln,";")[1:end-1]

        # Plot start point
        first_vertex = parse(Int64,split(edgelist[1]," ")[1])
        x = Gdict[first_vertex][1]
        y = Gdict[first_vertex][2]
        scatter!([x],[y],marker=:star5,markersize=12,markercolor=k)

        #println("List of edges: ", edgelist)

        # Plot edges
        for edge in edgelist
            vertices = split(edge, " ")
            #println("Vertices: ", vertices)
            v1 = parse(Int64, vertices[1])
            v2 = parse(Int64, vertices[2])
            x1 = Gdict[v1][1]
            y1 = Gdict[v1][2]
            x2 = Gdict[v2][1]
            y2 = Gdict[v2][2]
            plot!([x1,x2],[y1,y2],color=k)
        end

        # Plot goal point
        last_vertex = parse(Int64,split(edgelist[end]," ")[2])
        x = Gdict[last_vertex][1]
        y = Gdict[last_vertex][2]
        scatter!([x],[y],marker=:circle,markersize=15,markercolor=k)
    end

    # Save and close
    savefig(string("../experiments/plots/",file))
    close(f)
    return
end
