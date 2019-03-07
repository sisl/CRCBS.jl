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
    constraint = CBSConstraint(1,2,3)
    # constraints = Set{CBSConstraint}([
    #     CBSConstraint(1,2,3),
    #     CBSConstraint(1,3,3),
    #     CBSConstraint(2,4,3)
    # ])
    # get_constraint_dict(constraints)
end
let
    node = empty_constraint_node()
    get_constraints(node,1)
end
let
    mapf = MAPF(Graph(), [1,2,3,4], [5,6,7,8])
    empty_constraint_node()
    node = ConstraintTreeNode(mapf)
    add_constraint!(node,CBSConstraint(1,2,3),mapf)
    combine_constraints(node.constraints,node.constraints)
    @test compare_constraint_nodes(ConstraintTreeNode(),ConstraintTreeNode())
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
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    mapf = MAPF(G.graph, [1,2], [5,6])
    node = ConstraintTreeNode(mapf)
    add_constraint!(node,CBSConstraint(1,2,3),mapf)
    # solution, cost = low_level_search(mapf,node)
    low_level_search!(mapf,node)
    get_cost(node.solution)

    node_conflict, edge_conflict = get_next_conflicts(node.solution)
    generate_constraints_from_conflict(node_conflict)
    generate_constraints_from_conflict(edge_conflict)
    @test !(is_valid(node_conflict) && is_valid(edge_conflict))

    node_conflicts, edge_conflicts = get_conflicts(node.solution)
    @test 1 == 1
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=2,n_obstacles_y=2)
    mapf = MAPF(G.graph, [1,3,2], [5,6,7])
    CBS(mapf)
end
