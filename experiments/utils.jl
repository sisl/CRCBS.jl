export
    get_dist_matrix,
    pad_matrix,
    initialize_full_grid_graph,
    initialize_regular_grid_graph

"""
    Get the distance matrix corresponding to the edge weights of a graph
"""
function get_dist_matrix(G::AbstractGraph)
    distmx = zeros(Float64,nv(G),nv(G))
    for v in vertices(G)
        distmx[v,:] .= dijkstra_shortest_paths(G,v).dists
    end
    distmx
end

"""
    helper to pad a matrix with some value around the edges
"""
function pad_matrix(mat::Matrix{T}, pad_size::Tuple{Int,Int},pad_val::T) where T
    new_size = [size(mat)...] + 2*[pad_size...]
    A = fill!(typeof(mat)(undef,new_size[1],new_size[2]),pad_val)
    A[
        pad_size[1]+1:pad_size[1]+size(mat,1),
        pad_size[2]+1:pad_size[2]+size(mat,2)
    ] .= mat
    A
end

function compute_distance_matrix(graph::G where G,weight_mtx::M where M)
   D = zeros(Float64,nv(graph),nv(graph))
   for v1 in vertices(graph)
       ds = dijkstra_shortest_paths(graph,v1,weight_mtx)
       D[v1,:] = ds.dists
   end
   D
end

"""
    A dummy function for initializing a grid graph

    TODO: extend to incorporate obstacles
"""
function initialize_full_grid_graph()
    dx = 1.0
    dy = 1.0
    x_pts=collect(0.0:dx:5.0)
    y_pts=collect(0.0:dy:4.0)
    G = MetaGraph() # navigation graph
    pts = []
    for x in x_pts
        for y in y_pts
            if !((4.0 <= x <= 6.0) && (4.0 <= y <= 6.0)) # check obstacle condition
                push!(pts,[x;y])
                add_vertex!(G,Dict(:x=>x,:y=>y))
            end
        end
    end
    kdtree = KDTree(hcat(pts...))
    # create edges of 4-connected grid
    for i in vertices(G)
        for j in inrange(kdtree,pts[i],norm([dx;dy]))
            if i != j && !(has_edge(G,i,j))
                if abs(pts[i][1]-pts[j][1]) <= dx && pts[i][2] == pts[j][2]
                    add_edge!(G,i,j)
                elseif abs(pts[i][2]-pts[j][2]) <= dy && pts[i][1] == pts[j][1]
                    add_edge!(G,i,j)
                end
            end
        end
    end
    return G
end

"""
    Returns a grid graph that represents a 2D environment with regularly spaced
    rectangular obstacles
"""
function initialize_regular_grid_graph(;
    n_obstacles_x=2,
    n_obstacles_y=2,
    obs_width = [2;2],
    obs_offset = [1;1],
    env_pad = [1;1],
    env_offset = [1.0,1.0],
    env_scale = 2.0 # this is essentially the robot diameter
    )
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
                    :y=>env_offset[2] + env_scale*(j-1))
                    )
                add_edge!(G,nv(G),nv(G))
                K[i,j] = k
            end
        end
    end
    for i in 1:size(Ap,1)
        for j in 1:size(Ap,2)
            if Ap[i,j] == 0
                if j < size(Ap,2)
                    add_edge!(G,K[i,j],K[i,j+1])
                end
                if j > 1
                    add_edge!(G,K[i,j],K[i,j-1])
                end
                if i < size(Ap,1)
                    add_edge!(G,K[i,j],K[i+1,j])
                end
                if i > 1
                    add_edge!(G,K[i,j],K[i-1,j])
                end
            end
        end
    end
    G
end
