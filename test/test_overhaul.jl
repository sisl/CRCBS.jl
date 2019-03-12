let
    s = CBS_State()
    @test states_match(s,s)
    a = CBS_Action()
    a = CRCBS.wait(s)
end
let
    graph = Graph(3)
    add_edge!(graph,1,2)
    add_edge!(graph,1,3)
    constraints = ConstraintDict()
    env = CBSLowLevelEnv(graph,constraints)
    s = CBS_State(1)
    for a in get_possible_actions(env,s)
        @test typeof(a) == CBS_Action
    end
end
