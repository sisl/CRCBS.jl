let
    basetup = (1,2.0,3,4.0,5,6.0)
    for j in 1:length(basetup)
        tup = tuple(basetup[1:j])
        for i in 1:length(tup)
            @test typemax(tup)[i] == typemax(typeof(tup[i]))
            @test typemin(tup)[i] == typemin(typeof(tup[i]))
        end
    end
end
let
    DefaultState()
    DefaultAction()
    p = DefaultPathNode()
    @test typeof(get_s(p)) == state_type(p) == DefaultState
    @test typeof(get_a(p)) == action_type(p) == DefaultAction
    @test typeof(get_sp(p)) == state_type(p) == DefaultState
end
let
    Path([DefaultPathNode()])
    p = Path{DefaultState,DefaultAction,Float64}(cost=0.0)
    @test state_type(p) == DefaultState
    @test action_type(p) == DefaultAction
    @test node_type(p) == PathNode{state_type(p),action_type(p)}
    @test cost_type(p) == Float64
    get(p,1,node_type(p))
    get(p,1)
    # @test get_cost(p) == 0
    @test length(p.path_nodes) == 0
    push!(p,PathNode{DefaultState,DefaultAction}())
    @test length(p.path_nodes) == 1
    p = cat(p,DefaultPathNode())
    @test length(p.path_nodes) == 2
    @test typeof(p[1]) == DefaultPathNode
    p[1] = DefaultPathNode()
    @test length(p) == length(p.path_nodes)
    p1 = copy(p)
    @test get_cost(p1) == get_cost(p)
    @test length(p1) == length(p)

    get_path_node(p,1)
    get_path_node(p,5)
    get_action(p,1)
    @test length(extend_path(p,5)) == 5
    extend_path!(p,5)
    @test length(p) == 5

    get_initial_state(p)
    get_final_state(p)
end
let
    cost_model = TravelTime()
    solution = LowLevelSolution{DefaultState,DefaultAction,get_cost_type(cost_model),TravelTime}()
    copy(solution)
end

# let
#     conflict = StateConflict(1,2,CBS_State(3),CBS_State(3),1)
#     @test is_valid(conflict)
#     conflict = invalid_state_conflict()
#     @test !is_valid(conflict)
#     # @test detect_state_conflict(Edge(1,2),Edge(3,2))
#     path1 = CBSPath([
#         PathNode{CBS_State,Edge{Int}}(CBS_State(1),Edge(1,2),CBS_State(2)),
#         PathNode{CBS_State,Edge{Int}}(CBS_State(2),Edge(2,3),CBS_State(3))
#         ])
#     path2 = CBSPath([
#         PathNode{CBS_State,Edge{Int}}(CBS_State(4),Edge(4,5),CBS_State(5)),
#         PathNode{CBS_State,Edge{Int}}(CBS_State(5),Edge(5,3),CBS_State(3))
#         ])
#     @test detect_state_conflict(path1,path2,2)
#     # @test detect_state_conflict([Edge(1,2),Edge(2,3)],[Edge(4,5),Edge(5,3)],2)
# end
# let
#     conflict = ActionConflict(1,2,CBS_State(3),CBS_State(4),1)
#     @test is_valid(conflict)
#     conflict = invalid_action_conflict()
#     @test !is_valid(conflict)
#     # @test detect_action_conflict(Edge(1,2),Edge(2,1))
#     path1 = CBSPath([
#         PathNode{CBS_State,Edge{Int}}(CBS_State(3),Edge(3,2),CBS_State(2)),
#         PathNode{CBS_State,Edge{Int}}(CBS_State(2),Edge(2,1),CBS_State(1))
#         ])
#     path2 = CBSPath([
#         PathNode{CBS_State,Edge{Int}}(CBS_State(4),Edge(4,1),CBS_State(1)),
#         PathNode{CBS_State,Edge{Int}}(CBS_State(1),Edge(1,2),CBS_State(2))
#         ])
#     @test detect_action_conflict(path1,path2,2)
#     # @test detect_action_conflict([Edge(3,2),Edge(2,1)],[Edge(4,1),Edge(1,2)],2)
# end
# let
#     StateConstraint(1,2,3)
#     ActionConstraint(1,2,3,4)
# end
# # let
#     # node = empty_constraint_node()
#     # get_constraints(node,1)
# # end
# let
#     mapf = MAPF(Graph(), [1,2,3,4], [5,6,7,8])
#     # empty_constraint_node()
#     node = initialize_root_node(mapf)
#     add_constraint!(node,StateConstraint(1,CBS_State(2),3),mapf)
#     merge(node.constraints,node.constraints)
#     # @test compare_constraint_nodes(ConstraintTreeNode(),ConstraintTreeNode())
# end
# let
#     mapf = MAPF(Graph(), [CBS_State(1),CBS_State(2)], [CBS_State(3),CBS_State(4)])
#     solution = LowLevelSolution([
#         CBSPath([
#             PathNode{CBS_State,Edge{Int}}(CBS_State(1),Edge(1,3),CBS_State(3))]),
#         CBSPath([
#             PathNode{CBS_State,Edge{Int}}(CBS_State(2),Edge(2,4),CBS_State(4))])
#             ])
#     @test is_valid(solution, mapf)
# end
# let
#     G = initialize_regular_grid_graph(;n_obstacles_x=2,n_obstacles_y=2)
#     mapf = MAPF(G, [CBS_State(1),CBS_State(2),CBS_State(3)], [CBS_State(7),CBS_State(6),CBS_State(5)])
#     node = initialize_root_node(mapf)
#     low_level_search!(mapf,node)
#     conflict_table = detect_conflicts(node.solution)
#     detect_conflicts!(conflict_table,node.solution,[1])
#     get_next_conflicts(conflict_table)
# end
# let
#     #               6
#     #               |
#     #               7
#     #               |
#     #     1 -- 2 -- 3 -- 4 -- 5
#     #               |
#     #               8
#     #               |
#     #               9
#     G = Graph(5)
#     add_edge!(G,1,2)
#     add_edge!(G,2,3)
#     add_edge!(G,3,4)
#     add_edge!(G,4,5)
#     add_edge!(G,6,7)
#     add_edge!(G,7,3)
#     add_edge!(G,3,8)
#     add_edge!(G,8,9)
#     mapf = MAPF(G,[CBS_State(1),CBS_State(5)],[CBS_State(5),CBS_State(1)])
#     node = initialize_root_node(mapf)
#     low_level_search!(mapf,node)
#     # check node conflict
#     solution = LowLevelSolution([
#         CBSPath([
#             PathNode{CBS_State,Edge{Int}}(CBS_State(1),Edge(1,2),CBS_State(2)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(2),Edge(2,3),CBS_State(3)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(3),Edge(3,4),CBS_State(4)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(4),Edge(4,5),CBS_State(5))
#             ]),
#         CBSPath([
#             PathNode{CBS_State,Edge{Int}}(CBS_State(5),Edge(5,4),CBS_State(4)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(4),Edge(4,3),CBS_State(3)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(3),Edge(3,2),CBS_State(2)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(2),Edge(2,1),CBS_State(1))
#             ])
#     ])
#     detect_conflicts!(node.conflict_table, solution)
#     state_conflict, action_conflict = get_next_conflicts(node.conflict_table)
#     @test state_conflict.t == 2
#     @test state_conflict.state1 == CBS_State(3)
#     constraints = generate_constraints_from_conflict(state_conflict)
#     add_constraint!(node,constraints[1],mapf)
#     # @test violates_constraints(node.constraints[1],3,[Edge(1,2)])
# end
# let
#     #               6
#     #               |
#     #               7
#     #               |
#     #     1 -- 2 -- 3 -- 4 -- 5
#     #               |
#     #               8
#     #               |
#     #               9
#     G = Graph(9)
#     add_edge!(G,1,2)
#     add_edge!(G,2,3)
#     add_edge!(G,3,4)
#     add_edge!(G,4,5)
#     add_edge!(G,6,7)
#     add_edge!(G,7,3)
#     add_edge!(G,3,8)
#     add_edge!(G,8,9)
#     mapf = MAPF(G,[CBS_State(1),CBS_State(4)],[CBS_State(4),CBS_State(1)])
#     node = initialize_root_node(mapf)
#     low_level_search!(mapf,node)
#     # check node conflict
#     # solution = LowLevelSolution([
#     #     CBSPath([Edge(1,2),Edge(2,3),Edge(3,4)]),
#     #     CBSPath([Edge(4,3),Edge(3,2),Edge(2,1)])
#     # ])
#     solution = LowLevelSolution([
#         CBSPath([
#             PathNode{CBS_State,Edge{Int}}(CBS_State(1),Edge(1,2),CBS_State(2)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(2),Edge(2,3),CBS_State(3)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(3),Edge(3,4),CBS_State(4))
#             ]),
#         CBSPath([
#             PathNode{CBS_State,Edge{Int}}(CBS_State(4),Edge(4,3),CBS_State(3)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(3),Edge(3,2),CBS_State(2)),
#             PathNode{CBS_State,Edge{Int}}(CBS_State(2),Edge(2,1),CBS_State(1))
#             ])
#     ])
#     state_conflict, action_conflict = get_next_conflicts(solution)
#     @test action_conflict.t == 2
#     @test action_conflict.state1 == CBS_State(2)
#     @test action_conflict.state2 == CBS_State(3)
#     constraints = generate_constraints_from_conflict(action_conflict)
#     add_constraint!(node,constraints[1],mapf)
#     # @test violates_constraints(node.constraints[1],3,[Edge(1,2)])
# end
# let
#     G = initialize_regular_grid_graph(;n_obstacles_x=1,n_obstacles_y=1)
#     mapf = MAPF(G.graph, [CBS_State(1),CBS_State(2)], [CBS_State(5),CBS_State(6)])
#     node = initialize_root_node(mapf)
#     low_level_search!(mapf,node)
#     state_conflicts, action_conflicts = get_conflicts(node.solution)
# end
# let
#     G = initialize_regular_grid_graph(;n_obstacles_x=2,n_obstacles_y=2)
#     mapf = MAPF(G, [CBS_State(1),CBS_State(3),CBS_State(2)], [CBS_State(5),CBS_State(6),CBS_State(7)])
#     solver = CBS()
#     solver(mapf)
# end
