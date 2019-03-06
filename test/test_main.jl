let
    MAPF(Graph(), [1,2,3,4], [5,6,7,8])
end
let
    conflict = NodeConflict(1,2,3,1)
    @test is_valid(conflict)
end
let
    conflict = invalid_node_conflict()
    @test !is_valid(conflict)
end
let
    conflict = EdgeConflict(1,2,3,4,1)
    @test is_valid(conflict)
end
let
    conflict = invalid_edge_conflict()
    @test !is_valid(conflict)
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
# let
#     d1 = Dict{Int,Set{Float64}}()
#     d2 = Dict{Int,Set{Float64}}()
#     d1[1] = Set{Float64}([1.0,2.0,3.0])
#     d1[2] = Set{Float64}([1.0,3.0])
#     d2[2] = Set{Float64}([4.0,5.0])
#     d2[3] = Set{Float64}([1.0,3.0])
#     d3 = combine_constraints(d1,d2)
#     for (k,v) in d3
#         @test length(setdiff(v,union(get(d1,k,Set{Float64}()),get(d2,k,Set{Float64}())))) == 0
#     end
# end
let
    mapf = MAPF(Graph(), [1,2,3,4], [5,6,7,8])
    empty_constraint_node()
    node = ConstraintTreeNode(mapf)
    add_constraint!(node,CBSConstraint(1,2,3))
    # combine_constraints(node.constraints,node.constraints)
    @test compare_constraint_nodes(ConstraintTreeNode(),ConstraintTreeNode())
end
let
    G = initialize_full_grid_graph()
    G = initialize_regular_grid_graph()
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    mapf = MAPF(G.graph, [1,2], [5,6])
    node = ConstraintTreeNode(mapf)
    add_constraint!(node,CBSConstraint(1,2,3))
    solution, cost = low_level_search(mapf,node)
    get_cost(solution)

    node_conflict, edge_conflict = get_next_conflicts(solution)
    generate_constraints_from_conflict(node_conflict)
    generate_constraints_from_conflict(edge_conflict)
    @test is_valid(node_conflict) || is_valid(edge_conflict)

    node_conflicts, edge_conflicts = get_conflicts(solution)
    @test 1 == 1
end
let
    G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
    mapf = MAPF(G.graph, [1,2], [5,6])
    CBS(mapf)
end
