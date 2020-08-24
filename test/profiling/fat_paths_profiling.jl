using CRCBS
using GraphUtils
using Test, Logging
using TOML


function profile_solver!(solver,mapf)
    reset_solver!(solver)
    (solution, cost), t, bytes, gctime, memallocs = @timed solve!(solver,mapf)
    timer_results = (
        t=t,
        bytes=bytes,
        gctime=gctime,
        memallocs=memallocs
        )
    return solution, timer_results
end
function profile_fat_paths_solver!(solver,mapf)
    fp_mapf = CRCBS.init_fat_path_mapf(mapf)
    profile_solver!(solver,fp_mapf)
end

# Problem Instances
scen_file = joinpath(ENV["HOME"],"Repos/mapf_benchmarks/scenarios/scen-even/empty-8-8-even-10.scen")
map_path = joinpath(ENV["HOME"],"Repos/mapf_benchmarks/maps/")
scenario = CRCBS.parse_mapf_scenario(scen_file,map_path)
base_mapf = CRCBS.construct_base_mapf(scenario)
mapf = CRCBS.gen_mapf_problem_from_scenario(base_mapf,scenario,1,15)
fp_mapf = CRCBS.init_fat_path_mapf(mapf)
fat_path_solver = CBSSolver(AStar{cost_type(fp_mapf)}())
solver = CBSSolver(AStar{cost_type(mapf)}())

config = [
    RunTime(),IterationCount(),SolutionCost(),NumConflicts(),RobotPaths(),
    TimeOutStatus(),IterationMaxOutStatus()
]

solution, timer_results = profile_solver!(solver,mapf)
results_dict = CRCBS.compile_results(solver,config,solution,timer_results)

solution, timer_results = profile_fat_paths_solver!(fat_path_solver,mapf)
results_dict = CRCBS.compile_results(fat_path_solver,config,solution,timer_results)

results_path = joinpath(ENV["HOME"],".julia/dev/CRCBS/test/results.toml")
open(results_path,"w") do io
    TOML.print(io,results_dict)
end
toml_dict = TOML.parsefile(results_path)
load_feature(SolutionCost(),toml_dict)
