export
initialize_full_grid_graph_CT,
initialize_regular_grid_graph_CT

using Parameters
using DataStructures
using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors
using CRCBS

include("utils.jl")

function initialize_full_grid_graph(x_max=10.0,y_max=10.0,filling_density=1.0)
    dx = 1.0
    dy = 1.0
    x_pts=collect(0.0:dx:x_max)
    y_pts=collect(0.0:dy:y_max)
    G = MetaDiGraph() # navigation graph
    pts = []

    for x in x_pts
        for y in y_pts

            # Use the desired filling density to create the vertex or not
            p = rand() #Uniform distribution between 0 and 1
            if p <= filling_density
                push!(pts,[x;y])
                add_vertex!(G,Dict(:x=>x,:y=>y, :n_delay=>1.0))
            end
        end
    end
    kdtree = KDTree(hcat(pts...))
    # create edges of 4-connected grid
    for i in vertices(G)

        # Create waiting edges, going from i to i
        add_edge!(G,i,i)
        set_prop!(G, Edge(i,i), :weight, 1.0)

        for j in inrange(kdtree,pts[i],norm([dx;dy]))
            if i != j && !(has_edge(G,i,j))
                if abs(pts[i][1]-pts[j][1]) <= dx && pts[i][2] == pts[j][2]
                    add_edge!(G,i,j)
                    set_prop!(G, Edge(i,j), :weight, 1.0)
                    add_edge!(G,j,i)
                    set_prop!(G, Edge(j,i), :weight, 1.0)
                elseif abs(pts[i][2]-pts[j][2]) <= dy && pts[i][1] == pts[j][1]
                    add_edge!(G,i,j)
                    set_prop!(G, Edge(i,j), :weight, 1.0)
                    add_edge!(G,j,i)
                    set_prop!(G, Edge(j,i), :weight, 1.0)
                end
            end
        end
    end
    return G
end

function initialize_full_grid_graph_CT(x_max=10.0,y_max=10.0,filling_density=1.0)
    """Initializing a grid graph in CT.
        Considers that the nominal time to move from one edge to another is 1.
        Each node has a delay parameter n_d of 1.
        The output is a MetaGraph"""
    G = initialize_full_grid_graph(x_max,y_max,filling_density)
    for e in edges(G)
        set_prop!(G,e,:weight,1.0)
        set_prop!(G,e,:occupancy, Dict{Int64, Tuple{Int64,Float64}}())
    end
    for v in vertices(G)
        set_prop!(G,v,:n_delay,1.0)
        set_prop!(G,v,:occupancy,Dict{Int64, Tuple{Int64,Float64}}())
    end
    return G
end

function initialize_regular_grid_graph_CT(;
    n_obstacles_x=2,
    n_obstacles_y=2,
    obs_width = [2;2],
    obs_offset = [1;1],
    env_pad = [1;1],
    env_offset = [1.0,1.0],
    env_scale = 2.0 # this is essentially the robot diameter
    )
    """Returns a grid graph that represents a 2D environment with regularly spaced
        rectangular obstacles in CT.
        Considers that the nominal time to move from one edge to another is 1.
        Each node has a delay parameter n_d of 1.
        The output is a MetaGraph"""
    # generate occupancy grid representing the environment
    o = ones(Int,obs_width[1],obs_width[2]) # obstacle region
    op = pad_matrix(o,(obs_offset[1],obs_offset[2]),0) # padded obstacles region
    A = repeat(op,n_obstacles_x,n_obstacles_y)
    Ap = pad_matrix(A,(env_pad[1],env_pad[2]),0) # padded occupancy grid
    K = zeros(Int,size(Ap))
    G = MetaGraph()

    k = 0
    for i in 1:size(Ap,1)
        for j in 1:size(Ap,2)
            if Ap[i,j] == 0
                k += 1
                add_vertex!(G,
                    Dict(:x=>env_offset[1] + env_scale*(i-1),
                    :y=>env_offset[2] + env_scale*(j-1),
                    :n_delay=>1.0)
                    )
                add_edge!(G,nv(G),nv(G))
                set_prop!(G, Edge(nv(G),nv(G)), :weight, 1.0)
                K[i,j] = k
            end
        end
    end
    for i in 1:size(Ap,1)
        for j in 1:size(Ap,2)
            if Ap[i,j] == 0
                if j < size(Ap,2)
                    add_edge!(G,K[i,j],K[i,j+1])
                    set_prop!(G, Edge(K[i,j],K[i,j+1]), :weight, 1.0)
                end
                if j > 1
                    add_edge!(G,K[i,j],K[i,j-1])
                    set_prop!(G, Edge(K[i,j],K[i,j-1]), :weight, 1.0)
                end
                if i < size(Ap,1)
                    add_edge!(G,K[i,j],K[i+1,j])
                    set_prop!(G, Edge(K[i,j],K[i+1,j]), :weight, 1.0)
                end
                if i > 1
                    add_edge!(G,K[i,j],K[i-1,j])
                    set_prop!(G, Edge(K[i,j],K[i-1,j]), :weight, 1.0)
                end
            end
        end
    end
    return G
end
