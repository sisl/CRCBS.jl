export
    plot_SVtime_vs_nrobots,
    plot_optimal_nominal_paths,
    plot_simulations


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

function plot_optimal_nominal_paths(file;type="CRCBS",graphfilename="",save=true)

    if graphfilename == ""
        graphfilename = file
    end

    # Load experiment parameters to find and draw edges
    exp_parms = load_experiment_parameters(graphfilename)



    Gdict = load_graph_dict(graphfilename)
    directory = string("../experiments/solutions/",type,"_",file,".txt")
    f = open(directory)
    myplot = plot(title = string(type, " Solution to ", file),leg=false)
    pp = 0
    if type == "CBS"
        pp = 3
    end

    # First draw all edges in gray
    for e in exp_parms.es
        x1 = Gdict[e[1]][1]
        y1 = Gdict[e[1]][2]
        x2 = Gdict[e[2]][1]
        y2 = Gdict[e[2]][2]
        plot!([x1,x2],[y1,y2], linewidth=0.2, color=RGB(0.5, 0.5, 0.5))
    end

    # Then draw paths, start and end points, and wait points
    for (k,ln) in enumerate(eachline(f))
        if k < 4 && type == "CBS"
            continue
        end
        edgelist = split(ln,";")[1:end-1]

        # Plot start point
        first_vertex = parse(Int64,split(edgelist[1]," ")[1])
        x = Gdict[first_vertex][1]
        y = Gdict[first_vertex][2]
        scatter!([x],[y],marker=:star5,markersize=12,markercolor=k-pp)

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
            plot!([x1,x2],[y1,y2],color=k-pp,linewidth=3)
            if v1 == v2
                scatter!([x1],[y1],marker=:diamond,markersize=5,markercolor=k-pp)
            end
        end

        # Plot goal point
        last_vertex = parse(Int64,split(edgelist[end]," ")[2])
        x = Gdict[last_vertex][1]
        y = Gdict[last_vertex][2]
        scatter!([x],[y],marker=:circle,markersize=15,markercolor=k-pp)
    end

    # Save and close
    if save == true
        savefig(string("../experiments/plots/",type,"_",file))
    end
    close(f)
    return myplot
end


function plot_simulations(file;type="CRCBS",graphfilename="",timehorizon=10,num_particles = 20,lambda = 1.0, epsilon = 0.0001)
    if graphfilename == ""
        graphfilename = file
    end

    # Load experiment parameters to find and draw edges
    exp_parms = load_experiment_parameters(graphfilename)

    # Load graph dictionary to get x y coordinates
    Gdict = load_graph_dict(graphfilename)
    directory = string("../experiments/solutions/",type,"_",file,".txt")
    f = open(directory)

    # turn txt into list of solutions
    llsolutionx = Vector{}()
    llsolutiony = Vector{}()
    for (k,ln) in enumerate(eachline(f))
        edgelist = split(ln,";")[1:end-1]
        array_k_x = []
        array_k_y = []

        # First vertex
        first_vertex = parse(Int64,split(edgelist[1]," ")[1])
        x = Gdict[first_vertex][1]
        y = Gdict[first_vertex][2]
        append!(array_k_x,x)
        append!(array_k_y,y)
        # Add the second vertex of each edge
        for edge in edgelist
            vertices = split(edge, " ")
            #println("Vertices: ", vertices)
            v2 = parse(Int64, vertices[2])
            x2 = Gdict[v2][1]
            y2 = Gdict[v2][2]
            append!(array_k_x,x2)
            append!(array_k_y,y2)
        end
        push!(llsolutionx, array_k_x)
        push!(llsolutiony, array_k_y)
    end
    close(f)

    # Find saved simulation data
    solution_times = load_solution_times(graphfilename)


    # Iterate through time
    list_of_times = range(0,10,step=0.5)
    for (k,t) in enumerate(list_of_times)

        # Plot the overall result
        myplot = plot_optimal_nominal_paths(file;type=type,graphfilename=graphfilename,save=false)

        # Plot the particles

        for (robot_idx,solution_per_robot) in enumerate(solution_times)
            xs = []
            ys = []
            for particle_idx = 1:shape(solution_times)[3]
                for (v_idx,time) in enumerate(collect(solution_per_robot[:,1,particle_idx]))
                    if t > time #we are after entering vertex v_idx
                        xleft = llsolutionx[robot_idx][v_idx]
                        yleft = llsolutiony[robot_idx][v_idx]
                        # Did we leave that vertex?
                        #If not, we are there
                        if t < solution_per_robot[v_idx,2,particle_idx]
                            tleft = solution_per_robot[v_idx,2,particle_idx]
                            x = xleft
                            y = yleft
                            continue
                        else #If yes, we are travelling to the next vertex
                            tright = try
                                solution_per_robot[v_idx+1,1,particle_idx]
                            catch
                                tleft
                            end

                            if t_left-t_right >= 0.001
                                xright = llsolutionx[robot_idx][v_idx+1]
                                yright = llsolutiony[robot_idx][v_idx+1]
                                alpha = (t-tleft)/(tright-tleft)
                                x = xleft + alpha*(xright-xleft)
                                y = yleft + alpha*(yright-yleft)
                            else
                                x = xleft
                                y = yleft
                            end
                            continue
                        end
                    end
                end
                append!(xs,x)
                append!(ys,y)
            end

            # Let's plot that color now that we have all particles for a robot at one time
            scatter!(xs,ys,marker=:circle,markersize=2,markercolor=robot_idx)
        end

        # We scattered all the points for all the robots, so now we save the plot
        savefig(string("../experiments/plots/",type,"_",file,"_time_",t))
    end
    return
end
