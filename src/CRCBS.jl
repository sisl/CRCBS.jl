module CRCBS

using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

export
    MAPF,
    CBSConstraint,
    ConstraintTreeNode,
    generate_random_grid_graph,
    generate_random_grid_graph_by_random_walk,
    initialize_grid_graph

"""
    A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
    a graph `G` whose edges have unit length, as well as a set of start and goal
    vertices on that graph
"""
struct MAPF # Multi Agent Path Finding Problem
    G::Graph
    starts::Vector{Int}
    goals::Vector{Int}
end

"""
    Encodes a conflict between two agents at a particular time
"""
struct Conflict
    agent_id1::Int
    agent_id2::Int
    node_id::Int
    t::Int
end
struct CBSConstraint
    a::Int # agent ID
    v::Int # vertex ID
    t::Int # time ID
end
struct ConstraintTreeNode
    constraints::Set{CBSConstraint}
    solution::Vector{Vector{Int}} # set of paths (one per agent) through graph
    cost::Int # cost = sum([length(path) for path in solution])
    parent::Int # index of parent node
    children::Tuple{Int,Int} # indices of two child nodes
    # is_goal::Bool
end

"""
    The Conflict-Based search algorithm for multi-agent path finding
"""
function CBS(mapf::MAPF)
    # TODO implement!
end

"""
    Initializes a grid graph

    TODO: extend to incorporate obstacles
"""
function initialize_grid_graph()
    dx = 1.0
    dy = 1.0
    x_pts=collect(0.0:dx:10.0)
    y_pts=collect(0.0:dy:10.0)
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

end # module
