# For testing Priority Inheritance with Backtracking (PIBT)
let
    mapf = init_mapf_1()
    solver = PIBTPlanner{Float64}()
    cache = CRCBS.init_cache(solver,mapf)
    for i in 1:num_agents(mapf)
        @test states_match(cache.states[i],get_start(mapf,i))
        env = cache.envs[i]
        s = cache.states[i]
        a = CRCBS.wait(env,s)
        sp = get_next_state(env,s,a)
        CRCBS.reserve!(cache,env,s,a,sp)
        @test CRCBS.is_reserved(cache,env,s,a,sp)
        @test length(cache.occupied) == i
    end
end
let
    mapf = init_mapf_2()
    solver = PIBTPlanner{Float64}()
    set_iteration_limit!(solver,10)
    set_verbosity!(solver,0)
    solution, valid = pibt!(solver, mapf)
    @test valid
    # @show valid, convert_to_vertex_lists(solution)
end
let
    mapf = init_mapf_3()
    solver = PIBTPlanner{Float64}()
    set_iteration_limit!(solver,1000)
    set_verbosity!(solver,0)
    solution, valid = pibt!(solver, mapf)
    @test valid
    # @show valid, convert_to_vertex_lists(solution)
end
