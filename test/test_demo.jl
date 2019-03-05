let
    G = initialize_regular_grid_graph()
    n_robots = 5
    n_goals = n_robots

    goals = Set{Int}()
    while length(goals) < n_goals
        push!(goals,rand(1:nv(G)))
    end
    starts = Set{Int}()
    while length(starts) < n_robots
        push!(starts,rand(1:nv(G)))
        setdiff!(starts,goals)
    end
    goals = collect(goals)
    starts = collect(starts)

    mapf = MAPF(G.graph,starts,goals)

    CBS(mapf)
end
