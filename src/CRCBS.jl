module CRCBS

using LightGraphs, MetaGraphs
using LinearAlgebra
using NearestNeighbors

include("utils.jl")

export
    GraphPath,
    traversal_time,
    LowLevelSolution,
    MAPF,
    NodeConflict,
    invalid_node_conflict,
    EdgeConflict,
    invalid_edge_conflict,
    is_valid,
    CBSConstraint,
    ConstraintTreeNode,
    empty_constraint_node,
    get_next_conflicts,
    get_conflicts,
    get_first_conflicts,
    get_first_node_conflict,
    get_first_edge_conflict,
    low_level_search,
    CBS,

    generate_random_grid_graph,
    initialize_full_grid_graph,
    initialize_regular_grid_graph

"""
    type alias for a path through the graph
"""
const GraphPath = Vector{Edge}

"""
    returns the time for traversal of a GraphPath. Defaults to the length of the
    path, but it may be useful in case we need to override later
"""
traversal_time(path::GraphPath) = length(path)

"""
    type alias for a list of agent paths
"""
const LowLevelSolution = Vector{GraphPath}

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
    Returns an invalid NodeConflict
"""
function invalid_node_conflict()
    NodeConflict(-1,-1,-1,-1)
end

"""
    checks if a node conflict is valid
"""
function is_valid(conflict::NodeConflict)
    return (conflict.agent1_id != -1)
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
    returns an invalid EdgeConflict
"""
function invalid_edge_conflict()
    return EdgeConflict(-1,-1,-1,-1,-1)
end

"""
    checks if an edge node is invalid
"""
function is_valid(conflict::EdgeConflict)
    return (conflict.agent1_id != -1)
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
    Returns an empty `ConstraintTreeNode`
"""
function empty_constraint_node()
    ConstraintTreeNode(Set{CBSConstraint}(),Vector{Vector{Edge}}(),-1, -1, (-1,-1))
end

"""
    Returns a `NodeConflict` and an `EdgeConflict` next conflicts.
    The function returns after finding the FIRST conflice (NodeConflict or
        EdgeConflict), which means that at least one of the returned conflicts
        will always be invalid. The rational for returning both anyway is to
        preserve stability of the function's return type.
    If node_conflict and edge_conflict are both invalid, the search has reached
        the end of the paths.

    args:
    - t_:       time index at which to begin the search
    - i_:       index of path 1 at which to begin the search
    - j_:       index of path 2 at which to begin the search
    - tmax:     maximum lookahead time (defaults to the length of the longest
        path)
    Search begins at time `t_`, `paths[i_]`, `paths[j_]`, then returns after
        finding the first conflict.
"""
function get_next_conflicts(paths::LowLevelSolution,
        i_::Int=1,j_::Int=1,t_::Int=1,tmax::Int=maximum([traversal_time(p) for p in paths])
        )
    node_conflict = invalid_node_conflict()
    edge_conflict = invalid_edge_conflict()
    # begin search from time t, paths[i_], paths[j_]
    t = t_
    i = i_
    path1 = paths[i]
    if length(path1) < t
        # do nothing
    else
        for (j,path2) in enumerate(paths[j_:end])
            if path1[t].src == path2[t].src
                # Same src node
                node_conflict = NodeConflict(i,j,path1[t].src,t)
                return node_conflict, edge_conflict
            elseif (path1[t].src == path2[t].dst) && (path1[t].dst == path2[t].src)
                # Same edge, opposite direction
                edge_conflict = EdgeConflict(i,j,path1[t].src,path2[t].src,t)
                return node_conflict, edge_conflict
            end
        end
    end
    # continue search from time t = t_+1
    for t in t_+1:tmax
        for (i,path1) in enumerate(paths)
            if length(path1) < t
                continue
            end
            for (j,path2) in enumerate(paths[i:end])
                if path1[t].src == path2[t].src
                    # Same src node
                    node_conflict = NodeConflict(i,j,path1[t].src,t)
                    return node_conflict, edge_conflict
                elseif (path1[t].src == path2[t].dst) && (path1[t].dst == path2[t].src)
                    # Same edge, opposite direction
                    edge_conflict = EdgeConflict(i,j,path1[t].src,path2[t].src,t)
                    return node_conflict, edge_conflict
                end
            end
        end
    end
    return node_conflict, edge_conflict
end

"""
    Returns a list of all conflicts that occur in a given solution

    args:
    - paths:        a list of graph edges to be traversed by the agents
"""
function get_conflicts(paths::LowLevelSolution)
    # TODO Make this way faster
    node_conflicts = Vector{NodeConflict}()
    edge_conflicts = Vector{EdgeConflict}()
    t_max = maximum([length(path) for path in paths])
    nc, ec = get_next_conflicts(paths)
    c =
    while true
        if is_valid(nc)
            push!(node_conflicts, nc)
            conflict = nc
        elseif is_valid(ec)
            push!(edge_conflicts, ec)
            conflict = ec
        else
            break
        end
        nc, ec = get_next_conflicts(
            paths,
            conflict.agent1_id,
            conflict.agent2_id+1,
            conflict.t,
            t_max
            )
    end
    # for t in 1:t_max
    #     for (i,path1) in enumerate(paths)
    #         if length(path1) < t
    #             continue
    #         end
    #         for (j,path2) in enumerate(paths[i:end])
    #             if path1[t].src == path2[t].src
    #                 # Same src node
    #                 push!(node_conflicts, NodeConflict(i,j,path1[t].src,t))
    #             elseif (path1[t].src == path2[t].dst) && (path1[t].dst == path2[t].src)
    #                 # Same edge, opposite direction
    #                 push!(edge_conflicts, EdgeConflict(i,j,path1[t].src,path2[t].src,t))
    #             end
    #         end
    #     end
    # end
    return node_conflicts, edge_conflicts
end

# """
#     Returns a NodeConflict and an EdgeConflict. Only the first one discovered
#     will be valid
# """
# function get_first_conflicts(paths::LowLevelSolution)
#     # TODO Make this way faster
#     get_next_conflicts(paths::LowLevelSolution)
# end
#
# """
#     Returns the first `NodeConflict` encountered in a list of candidate paths
# """
# function get_first_node_conflict(paths::LowLevelSolution)
#     # TODO Make this way faster
#     node_conflicts = Vector{NodeConflict}()
#     edge_conflicts = Vector{EdgeConflict}()
#     max_length = maximum([length(path) for path in paths])
#     for t in 1:max_length
#         for (i,path1) in enumerate(paths)
#             if length(path1) < t
#                 continue
#             end
#             for (j,path2) in enumerate(paths[i:end])
#                 if length(path2) < t
#                     continue
#                 end
#                 if path1[t].src == path2[t].src
#                     # Same src node
#                     return NodeConflict(i,j,path1[t].src,t)
#                 end
#             end
#         end
#     end
#     return invalid_node_conflict()
# end
#
# """
#     Returns the first `EdgeConflict` encountered in a list of candidate paths
# """
# function get_first_edge_conflict(paths::LowLevelSolution)
#     # TODO Make this way faster
#     node_conflicts = Vector{NodeConflict}()
#     edge_conflicts = Vector{EdgeConflict}()
#     max_length = maximum([length(path) for path in paths])
#     for t in 1:max_length
#         for (i,path1) in enumerate(paths)
#             if length(path1) < t
#                 continue
#             end
#             for (j,path2) in enumerate(paths[i:end])
#                 if length(path2) < t
#                     continue
#                 end
#                 if (path1[t].src == path2[t].dst) && (path1[t].dst == path2[t].src)
#                     # Same src node
#                     return EdgeConflict(i,j,path1[t].src,path2[t].src,t)
#                 end
#             end
#         end
#     end
#     return invalid_node_conflict()
# end

"""
    Returns a low level solution for a MAPF with constraints
"""
function low_level_search(mapf::MAPF,node::ConstraintTreeNode,path_finder=LightGraphs.a_star)
    # compute an initial solution
    solution = LowLevelSolution()
    for i in 1:length(mapf.starts)
        # TODO allow passing custom heuristic
        path = path_finder(mapf.graph,mapf.starts[i],mapf.goals[i])
        push!(solution,path)
    end
    # sum of individual costs (SIC)
    cost = sum([length(path) for path in solution])
    # TODO check if solution is valid
    return solution, cost
end

"""
    The Conflict-Based search algorithm for multi-agent path finding
"""
function CBS(mapf::MAPF,path_finder=LightGraphs.a_star)
    # TODO implement!
    solution, cost = low_level_search(mapf,empty_constraint_node())
    root_node = ConstraintTreeNode(
        Set{CBSConstraint}(),
        solution,
        cost,
        NULL_NODE,
        (NULL_NODE,NULL_NODE)
        )
    # check for conflicts
    # node_conflicts, edge_conflicts = get_conflicts(solution)
    node_conflict, edge_conflict = get_next_conflicts(solution)
    if is_valid(node_conflict)
        # Create constraints from node_conflict
    elseif is_valid(edge_conflict)
        # Create constraints from edge_conflict
    else
        # Done!
        return solution, cost
    end

    # constraints = create_constraints_from_conflict(node_conflicts)
    #
    # p = PriorityQueue()
    # push!(p,root_node)
    # while length(p) > 0
    #     node = pop!(p)
    #     # check that this node is not redundant with parent node (that its constraint set is unique)
    #     # get low level solution
    #     solution = low_level_search(node)
    #     conflict = get_first_conflict(solution)
    #     # if (length(node_conflicts) == 0 && length(edge_conflicts) == 0)
    #     if !is_valid(conflict)
    #         return solution
    #     end
    #
    # end
    return solution, cost
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


end # module
