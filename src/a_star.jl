# A* shortest-path algorithm



function LightGraphs.a_star_impl!(g::AbstractGraph,# the graph
    t, # the end vertex
    frontier,               # an initialized heap containing the active vertices
    colormap::Vector{UInt8},  # an (initialized) color-map to indicate status of vertices
    constraints::C where C,
    distmx::AbstractMatrix,
    heuristic::Function)

    E = Edge{eltype(g)}

    @inbounds while !isempty(frontier)
        (cost_so_far, path, u) = dequeue!(frontier)
        if u == t
            return path
        end

        for v in LightGraphs.outneighbors(g, u)
            # Skip node if it violates any of the constraints
            if violates_constraint(constraints,v,path)
                continue
            end
            if get(colormap, v, 0) < 2
                dist = distmx[u, v]
                colormap[v] = 1
                new_path = cat(path, E(u, v), dims=1)
                path_cost = cost_so_far + dist
                enqueue!(frontier,
                    (path_cost, new_path, v),
                    path_cost + heuristic(v)
                )
            end
        end
        colormap[u] = 2
    end
    Vector{E}()
end

"""
    override A* to accept constraints

    a_star(g, s, t[, distmx][, heuristic])
Return a vector of edges comprising the shortest path between vertices `s` and `t`
using the [A* search algorithm](http://en.wikipedia.org/wiki/A%2A_search_algorithm).
An optional heuristic function and edge distance matrix may be supplied. If missing,
the distance matrix is set to [`LightGraphs.DefaultDistance`](@ref) and the heuristic is set to
`n -> 0`.
"""
function LightGraphs.a_star(g::AbstractGraph{U},  # the g
    s::Integer,                       # the start vertex
    t::Integer,                       # the end vertex
    constraints::C,         # constraints on which nodes can be traversed at which time
    distmx::AbstractMatrix{T}=weights(g),
    heuristic::Function=n -> zero(T)) where {C, T, U}

    E = Edge{eltype(g)}

    # if we do checkbounds here, we can use @inbounds in a_star_impl!
    checkbounds(distmx, Base.OneTo(nv(g)), Base.OneTo(nv(g)))
    # heuristic (under)estimating distance to target
    frontier = PriorityQueue{Tuple{T,Vector{E},U},T}()
    frontier[(zero(T), Vector{E}(), U(s))] = zero(T)
    colormap = empty_colormap(nv(g))
    colormap[s] = 1
    a_star_impl!(g, U(t), constraints, frontier, colormap, distmx, heuristic)
end
