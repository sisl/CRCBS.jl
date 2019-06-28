export
    get_dist_matrix,
    pad_matrix,
    sample_wo_repl,
    compute_distance_matrix

"""
    Get the distance matrix corresponding to a LightGraph
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

"""
    Get the expected distance matrix corresponding to the edge weights of a graph
"""
function compute_distance_matrix(graph::G where G,weight_mtx::M where M)
   D = zeros(Float64,nv(graph),nv(graph))
   for v1 in vertices(graph)
       ds = dijkstra_shortest_paths(graph,v1,weight_mtx)
       D[v1,:] = ds.dists
   end
   D
end

function sample_wo_repl(A,n)
    sample = zeros(n)
    for i in 1:n
        sample[i] = splice!(A, rand(eachindex(A)))
    end
    return sample
end
