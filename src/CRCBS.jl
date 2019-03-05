module CRCBS

using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

include("utils.jl")

export
    MAPF,
    NodeConflict,
    EdgeConflict,
    CBSConstraint,
    ConstraintTreeNode,
    CBS,

    generate_random_grid_graph,
    initialize_full_grid_graph,
    initialize_regular_grid_graph


"""
    A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
    a graph `G` whose edges have unit length, as well as a set of start and goal
    vertices on that graph. Note that this is the _labeled_ case, where each
    agent has a specific assigned destination.
"""
struct MAPF{G <: AbstractGraph} # Multi Agent Path Finding Problem
    graph::G
    starts::Vector{Int}
    goals::Vector{Int}
end

"""
    Encodes a conflict between two agents at a particular node at a particular
    time
"""
struct NodeConflict
    agent1_id::Int
    agent2_id::Int
    node_id::Int
    t::Int
end

"""
    Encodes a conflict between two agents at a particular edge at a particular
    time. This means that the agents are trying to swap places at time t.
"""
struct EdgeConflict
    agent1_id::Int
    agent2_id::Int
    node1_id::Int
    node2_id::Int
    t::Int
end

"""
    Encodes a constraint that agent `a` may not occupy vertex `v` at time `t`
"""
struct CBSConstraint
    a::Int # agent ID
    v::Int # vertex ID
    t::Int # time ID
end

"""
    A node of a constraint tree. Each node has a set of constraints, a candidate
    solution (set of robot paths), and a cost
"""
struct ConstraintTreeNode
    constraints::Set{CBSConstraint}
    solution::Vector{Vector{Edge}} # set of paths (one per agent) through graph
    cost::Int # cost = sum([length(path) for path in solution])
    parent::Int # index of parent node
    children::Tuple{Int,Int} # indices of two child nodes
    # is_goal::Bool
end
const NULL_NODE = -1

"""
    Returns a list of conflicts that occur in a given solution

    args:
    - paths:        a list of graph edges to be traversed by the agents
"""
function get_conflicts(paths::Vector{Vector{Edge}})
    # TODO Make this way faster
    node_conflicts = Vector{NodeConflict}()
    edge_conflicts = Vector{EdgeConflict}()
    max_length = maximum([length(path) for path in paths])
    for t in 1:max_length
        for (i,path1) in enumerate(paths)
            if length(path1) < t
                continue
            end
            for (j,path2) in enumerate(paths)
                if length(path2) < t
                    continue
                end
                if i != j
                    if path1[t].src == path2[t].src
                        # Same src node
                        push!(node_conflicts, NodeConflict(i,j,path1[t].src,t))
                    # elseif path1[t].dst == path2[t].dst
                    #     # Same destination node (redundant)
                    elseif (path1[t].src == path2[t].dst) && (path1[t].dst == path2[t].src)
                        # Same edge, opposite direction
                        push!(edge_conflicts, EdgeConflict(i,j,path1[t].src,path2[t].src,t))
                    end
                end
            end
        end
    end
    return node_conflicts, edge_conflicts
end

"""
    The Conflict-Based search algorithm for multi-agent path finding
"""
function CBS(mapf::MAPF,path_finder=LightGraphs.a_star)
    # TODO implement!
    G = mapf.graph # graph with no metadata
    starts = mapf.starts
    goals = mapf.goals
    # compute an initial solution
    solution = Vector{Vector{Edge}}()
    for i in 1:length(starts)
        path = path_finder(G,starts[i],goals[i])
        push!(solution,path)
    end
    cost = sum([length(path) for path in solution])
    #
    root_node = ConstraintTreeNode(
        Set{CBSConstraint}(),
        solution,
        cost,
        NULL_NODE,
        (NULL_NODE,NULL_NODE)
        )
    # check for conflicts
    node_conflicts, edge_conflicts = get_conflicts(solution)

end

"""
    A dummy function for initializing a grid graph

    TODO: extend to incorporate obstacles
"""
function initialize_full_grid_graph()
    dx = 1.0
    dy = 1.0
    x_pts=collect(0.0:dx:10.0)
    y_pts=collect(0.0:dy:10.0)
    G = MetaGraph() # navigation graph
    pts = []
    for x in x_pts
        for y in y_pts
            # if !((4.0 <= x <= 6.0) && (4.0 <= y <= 6.0)) # check obstacle condition
                push!(pts,[x;y])
                add_vertex!(G,Dict(:x=>x,:y=>y))
            # end
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
    n_obstacles_x=4,
    n_obstacles_y=3,
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


end # module
