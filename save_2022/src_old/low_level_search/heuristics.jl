export
    LowLevelSearchHeuristic,
    PerfectHeuristic,
    L2Heuristic,
    L1Heuristic

abstract type LowLevelSearchHeuristic end

"""
    A perfect heuristic based on distance
"""
struct PerfectHeuristic{V<:Vector} <: LowLevelSearchHeuristic
    distances::V
end
""" Functor method to get heuristic value for vertex v """
(h::PerfectHeuristic)(v::Int) = h.distances[v]

"""
    Construct a perfect distance heuristic from the goal node corresponding to
    the ith
"""
function PerfectHeuristic(mapf::MAPF,i::Int)
    PerfectHeuristic(dijkstra_shortest_paths(mapf.graph,mapf.goals[i]).dists)
end
