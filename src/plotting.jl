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

function plot_optimal_nominal_paths(file;type="CRCBS",graphfilename="",save=true,savedir="")

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
        savefig(string("../experiments/plots/",savedir,type,"_",file))
    end
    close(f)
    return myplot
end


function plot_simulations(file;type="CRCBS",graphfilename="",timehorizon=10,num_particles = 20,savedir="")
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
        if type == "CBS" && k < 4
            continue
        end
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
    solution_times = load_solution_times(file)


    # Iterate through time
    list_of_times = range(0,timehorizon,step=0.5)
    for (k,t) in enumerate(list_of_times)

        # Plot the overall result
        myplot = plot_optimal_nominal_paths(file;type=type,graphfilename=graphfilename,save=false)

        # Plot the particles

        for (robot_idx,solution_per_robot) in enumerate(solution_times)
            xs = []
            ys = []
            x = 0 #We need to define those outside the loop for them to be global variables
            y = 0
            #println("llsolutionx")
            #println(llsolutionx)
            found=false
            for particle_idx = 1:min(size(solution_per_robot)[3],num_particles)
                for (v_idx,time) in enumerate(collect(solution_per_robot[:,1,particle_idx]))
                    #println("t time")
                    #println(t)
                    #println(time)
                    if t <= time #we are before entering vertex v_idx
                        if v_idx == 1
                            #println("Found beginning")
                            x = llsolutionx[robot_idx][v_idx]
                            y = llsolutiony[robot_idx][v_idx]
                            found=true
                            break
                        else
                            #println("Found progression")
                            xleft = llsolutionx[robot_idx][v_idx-1]
                            yleft = llsolutiony[robot_idx][v_idx-1]
                            tleft = solution_per_robot[v_idx-1,2,particle_idx]
                            # Did we leave the previous vertex?
                            #If not, we are there
                            if t < solution_per_robot[v_idx-1,2,particle_idx]
                                x = xleft
                                y = yleft
                                found=true
                                break
                            else #If yes, we are travelling to the next vertex
                                #println("TRAVELLING TO NEXT VERTEX")
                                tright = solution_per_robot[v_idx,1,particle_idx]
                                #println("tright tleft t")
                                #println(tright)
                                #println(tleft)
                                #println(t)

                                if tright-tleft >= 0.001
                                    xright = llsolutionx[robot_idx][v_idx]
                                    yright = llsolutiony[robot_idx][v_idx]
                                    alpha = (t-tleft)/(tright-tleft)
                                    x = xleft + alpha*(xright-xleft)
                                    y = yleft + alpha*(yright-yleft)
                                    found=true
                                else
                                    x = xleft
                                    y = yleft
                                    found=true
                                end
                                break
                            end
                        end
                    end

                end
                if found == true # If we were bigger than all other values, we are at the last vertex (arrived)
                    #println("We found that we were at: ", x, " ", y)
                    append!(xs,x)
                    append!(ys,y)
                else
                    #println("We found that we were all at the last vertex")
                    x = llsolutionx[robot_idx][end]
                    y = llsolutiony[robot_idx][end]
                    append!(xs,x)
                    append!(ys,y)
                end

            end

            # Let's plot that color now that we have all particles for a robot at one time
            scatter!(xs,ys,marker=:circle,markersize=4,markercolor=robot_idx)
        end

        # We scattered all the points for all the robots, so now we save the plot
        savefig(string("../experiments/plots/", savedir,type,"_",file,"_time_",t))
    end
    return
end
