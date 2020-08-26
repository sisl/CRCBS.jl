using CRCBS
using GraphUtils
using Test, Logging
using TOML
using Parameters


"""
    FatPathsSolver{S}

A simple wrapper around a MAPF solver so that profile_solver! will dispatch
correctly (by transforming adding the fat paths cost to the problem before
solving it).
"""
@with_kw struct FatPathsSolver{S}
    solver::S = CBSSolver(AStar{NTuple{2,Float64}}())
end
function CRCBS.profile_solver!(solver::FatPathsSolver,mapf)
    fp_mapf = CRCBS.init_fat_path_mapf(mapf)
    CRCBS.profile_solver!(solver.solver,fp_mapf)
end
CRCBS.get_logger(solver::FatPathsSolver) = get_logger(solver.solver)
CRCBS.low_level(solver::FatPathsSolver) = low_level(solver.solver)


# Problem Instances
base_scen_path = joinpath(ENV["HOME"],"Repos/mapf_benchmarks/scenarios")
map_path = joinpath(ENV["HOME"],"Repos/mapf_benchmarks/maps/")
results_path = "/scratch/mapf_experiments"

EXPERIMENTS_DIR = "/scratch/mapf_experiments/fat_path_experiments"
PROBLEM_DIR = joinpath(EXPERIMENTS_DIR,"problems")
RESULTS_DIR = joinpath(EXPERIMENTS_DIR,"results")

config = (
    solver_configs = [
        (
            solver=CBSSolver(),
            results_path=joinpath(RESULTS_DIR,"CBSSolver")
        ),
        (
            solver=FatPathsSolver(),
            results_path=joinpath(RESULTS_DIR,"FatPathsSolver")
        ),
    ],
    problem_dir = PROBLEM_DIR,
    feats = [
        RunTime(),IterationCount(),SolutionCost(),NumConflicts(),RobotPaths(),
        TimeOutStatus(),IterationMaxOutStatus(),
        MemAllocs(),ByteCount(),GCTime()
    ]
)
for solver_config in config.solver_configs
    set_runtime_limit!(solver_config.solver,50)
    set_iteration_limit!(solver_config.solver,5000)
    set_iteration_limit!(low_level(solver_config.solver),1000)
end

BenchmarkInterface.generate_problem_files_from_moving_ai(
    [joinpath(base_scen_path,"scen-even/empty-8-8-even-10.scen")],
    map_path,
    PROBLEM_DIR
)

loader = BenchmarkInterface.init_mapf_loader(PROBLEM_DIR)

BenchmarkInterface.profile_with_skipping!(config,loader)
# run_profiling(config,loader)