export
    LowLevelSearchHeuristic,
    PerfectHeuristic

abstract type LowLevelSearchHeuristic end

"""
    `PerfectHeuristic`

    The Perfect Heuristic stores the exact distance between any vertex in a
    graph to all goal vertices specified during construction. The distances are
    stored in `dists`, a dictionary which maps a goal vertex `v` to a vector of
    distances from any other vertex in `1:nv(G)` to `v`.

    An example of how to access the distance from vertex `v1` to goal `v2`:
    `get_heuristic_cost(h,v1,v2) = h.dists[v2][v1]`
"""
struct PerfectHeuristic
    dists::Dict{Int,Vector{Float64}}
end
function PerfectHeuristic(graph,goals::Vector{Int})
    dists = Dict(v => gdistances(graph,v) for v in goals)
    PerfectHeuristic(dists)
end
get_heuristic_cost(h::PerfectHeuristic,goal_vtx::Int,vtx::Int) = h.dists[goal_vtx][vtx]
