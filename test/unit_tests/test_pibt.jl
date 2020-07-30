# For testing Priority Inheritance with Backtracking (PIBT)
let
    mapf = init_mapf_1()
    solver = PIBTPlanner{Float64}()
    cache = CRCBS.pibt_init_cache(solver,mapf)
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
# test PIBT on mapf test problems
let
    solver = PIBTPlanner{Float64}()
    set_iteration_limit!(solver,20)
    set_verbosity!(solver,0)
    for f in mapf_test_problems()
        mapf = f()
        reset_solver!(solver)
        solution, valid = pibt!(solver, mapf)
        @test valid
    end
end
# test PIBT with ragged plans on mapf test problems
let
    solver = PIBTPlanner{Float64}()
    set_iteration_limit!(solver,20)
    set_verbosity!(solver,0)
    for f in mapf_test_problems()
        mapf = f()
        reset_solver!(solver)
        solution, valid = pibt!(solver, mapf)
        @test valid
        # remove the path of robot 1 and replan
        p1 = get_paths(solution,1)
        empty!(p1.path_nodes)
        set_cost!(p1,get_initial_cost(mapf))
        p2 = deepcopy(get_paths(solution,2))
        # replan with the ragged plan
        solution, valid = pibt!(solver, mapf)
        @test valid
        p2b = get_paths(solution)[2]
        @test length(p2) == length(p2b)
        @test get_cost(p2) == get_cost(p2b)
        for (n2a,n2b) in zip(p2.path_nodes,p2b.path_nodes)
            @test get_s(n2a) == get_s(n2b)
            @test get_a(n2a) == get_a(n2b)
            @test get_sp(n2a) == get_sp(n2b)
        end
    end
end
