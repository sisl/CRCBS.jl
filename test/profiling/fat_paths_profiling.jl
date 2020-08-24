using CRCBS
using GraphUtils
using Test, Logging


# Problem Instances
scen_file = "/home/kylebrown/Repos/mapf_benchmarks/scenarios/scen-even/empty-8-8-even-10.scen"
map_path = "/home/kylebrown/Repos/mapf_benchmarks/maps/"
scenario = CRCBS.parse_mapf_scenario(scen_file,map_path)
base_mapf = CRCBS.construct_base_mapf(scenario)
mapf = CRCBS.gen_mapf_problem_from_scenario(base_mapf,scenario,1,2)

function profile_fat_paths_solver(solver,mapf,problem_id::Int,profile_config)
    fp_mapf = CRCBS.init_fat_path_mapf(mapf)
    reset_solver!(solver)
    (solution, cost), t, bytes, gctime, memallocs = @time solve!(solver,fp_mapf)
    results = (
        solution=solution,
        cost=cost,
        t=t,
        bytes=bytes,
        gctime=gctime,
        memallocs=memallocs
        )
end

function compile_results(solver,results,config)

end


solution, cost = solve!(solver,fp_mapf)
convert_to_vertex_lists(solution)
cost
