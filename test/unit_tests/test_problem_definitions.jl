let
    mapf = MAPF(DefaultEnvironment(), [1,2,3,4], [5,6,7,8])
    @test num_agents(mapf) == 4
    @test num_goals(mapf) == 4
end
let
    PC_MAPF()
end
