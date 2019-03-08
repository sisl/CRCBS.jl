let
    MAPF(Graph(), [1,2,3,4], [5,6,7,8])
end
let
    conflict = NodeConflict(1,2,3,1)
    @test is_valid(conflict)
    conflict = invalid_node_conflict()
    @test !is_valid(conflict)
    @test detect_node_conflict(Edge(1,2),Edge(3,2))
    @test detect_node_conflict([Edge(1,2),Edge(2,3)],[Edge(4,5),Edge(5,3)],2)
end
let
    conflict = EdgeConflict(1,2,3,4,1)
    @test is_valid(conflict)
    conflict = invalid_edge_conflict()
    @test !is_valid(conflict)
    @test detect_edge_conflict(Edge(1,2),Edge(2,1))
    @test detect_edge_conflict([Edge(3,2),Edge(2,1)],[Edge(4,1),Edge(1,2)],2)
end
let
    NodeConstraint(1,2,3)
    EdgeConstraint(1,2,3,4)
end
let
    node = empty_constraint_node()
    get_constraints(node,1)
end
let
    mapf = MAPF(Graph(), [1,2,3,4], [5,6,7,8])
    empty_constraint_node()
    node = initialize_root_node(mapf)
    add_constraint!(node,NodeConstraint(1,2,3),mapf)
    merge(node.constraints,node.constraints)
    # @test compare_constraint_nodes(ConstraintTreeNode(),ConstraintTreeNode())
end
let
    G = initialize_full_grid_graph()
    G = initialize_regular_grid_graph()
end
let
    mapf = MAPF(Graph(), [1,2], [3,4])
    solution = LowLevelSolution([GraphPath([Edge(1,3)]),GraphPath([Edge(2,4)])])
    @test is_valid(solution, mapf)
end
let
    #               6
    #               |
    #               7
    #               |
    #     1 -- 2 -- 3 -- 4 -- 5
    #               |
    #               8
    #               |
    #               9
    G = Graph(5)
    add_edge!(G,1,2)
    add_edge!(G,2,3)
    add_edge!(G,3,4)
    add_edge!(G,4,5)
    add_edge!(G,6,7)
    add_edge!(G,7,3)
    add_edge!(G,3,8)
    add_edge!(G,8,9)
    mapf = MAPF(G,[1,5],[5,1])
    node = initialize_root_node(mapf)
    low_level_search!(mapf,node)
    # check node conflict
    solution = LowLevelSolution([
        GraphPath([Edge(1,2),Edge(2,3),Edge(3,4),Edge(4,5)]),
        GraphPath([Edge(5,4),Edge(4,3),Edge(3,2),Edge(2,1)])
    ])
    node_conflict, edge_conflict = get_next_conflicts(solution)
    @test node_conflict.t == 2
    @test node_conflict.node_id == 3
    constraints = generate_constraints_from_conflict(node_conflict)
    add_constraint!(node,constraints[1],mapf)
    @test violates_constraints(node.constraints[1],3,[Edge(1,2)])
end
let
    #               6
    #               |
    #               7
    #               |
    #     1 -- 2 -- 3 -- 4 -- 5
    #               |
    #               8
    #               |
    #               9
    G = Graph(9)
    add_edge!(G,1,2)
    add_edge!(G,2,3)
    add_edge!(G,3,4)
    add_edge!(G,4,5)
    add_edge!(G,6,7)
    add_edge!(G,7,3)
    add_edge!(G,3,8)
    add_edge!(G,8,9)
    mapf = MAPF(G,[1,4],[4,1])
    node = initialize_root_node(mapf)
    low_level_search!(mapf,node)
    # check node conflict
    solution = LowLevelSolution([
        GraphPath([Edge(1,2),Edge(2,3),Edge(3,4)]),
        GraphPath([Edge(4,3),Edge(3,2),Edge(2,1)])
    ])
    node_conflict, edge_conflict = get_next_conflicts(solution)
    @test edge_conflict.t == 2
    @test edge_conflict.node1_id == 2
    @test edge_conflict.node2_id == 3
    constraints = generate_constraints_from_conflict(edge_conflict)
    add_constraint!(node,constraints[1],mapf)
    @test violates_constraints(node.constraints[1],3,[Edge(1,2)])
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    mapf = MAPF(G.graph, [1,2], [5,6])
    node = initialize_root_node(mapf)
    low_level_search!(mapf,node)
    node_conflicts, edge_conflicts = get_conflicts(node.solution)
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=2,n_obstacles_y=2)
    mapf = MAPF(G, [1,3,2], [5,6,7])
    CBS(mapf)
end
