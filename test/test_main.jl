let
    G = initialize_full_grid_graph()
    G = initialize_regular_grid_graph()
    MAPF(G, [1,2,3,4], [5,6,7,8])
    mapf = MAPF(G.graph, [1,2,3,4], [5,6,7,8])
    NodeConflict(1,2,3,1)
    EdgeConflict(1,2,3,4,1)
    CBSConstraint(1,2,3)
    empty_constraint_node()
    solution, cost = low_level_search(mapf,empty_constraint_node())
    get_cost(solution)
    
    node_conflict, edge_conflict = get_next_conflicts(solution)
    generate_constraints_from_conflict(node_conflict)
    generate_constraints_from_conflict(edge_conflict)
    @test is_valid(node_conflict) || is_valid(edge_conflict)
    node_conflicts, edge_conflicts = get_conflicts(solution)

    CBS(mapf)

    @test 1 == 1
end
