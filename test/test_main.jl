let
    G = initialize_full_grid_graph()
    G = initialize_regular_grid_graph()
    MAPF(G, [1,2,3,4], [5,6,7,8])
    MAPF(G.graph, [1,2,3,4], [5,6,7,8])
    NodeConflict(1,2,3,1)
    EdgeConflict(1,2,3,4,1)
    CBSConstraint(1,2,3)
    ConstraintTreeNode(
        Set{CBSConstraint}(),
        Vector{Vector{Edge}}(),
        0,
        1,
        (-1,-1)
        )
    @test 1 == 1
end
