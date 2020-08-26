# This module provides an interface to the Moving AI MAPF benchmark suite found
# at https://www.movingai.com/benchmarks/index.html
export BenchmarkInterface

module BenchmarkInterface

using GraphUtils
using TOML
using ..CRCBS

struct ScenarioAgentSpec
    start::Tuple{Int,Int}
    goal::Tuple{Int,Int}
end
struct MAPFScenario
    mapfile::String
    buckets::Vector{Vector{ScenarioAgentSpec}}
end

"""
    parse_map_file

Parses a .map file (see citation below) into an indicator grid. Each cell of the map is
encoded by one of the following characters:
    . - passable terrain
    G - passable terrain
    @ - out of bounds
    O - out of bounds
    T - trees (unpassable)
    S - swamp (passable from regular terrain)
    W - water (traversable, but not passable from terrain)

Returns an array of integers encoded as `IMPASSABLE=>1, FREE=>0` (we treat only
'G', 'S', and '.' as free).

@article{sturtevant2012benchmarks,
  title={Benchmarks for Grid-Based Pathfinding},
  author={Sturtevant, N.},
  journal={Transactions on Computational Intelligence and AI in Games},
  volume={4},
  number={2},
  pages={144 -- 148},
  year={2012},
  url = {http://web.cs.du.edu/~sturtevant/papers/benchmarks.pdf},
}
"""
function parse_map_file(filename,encoding=Dict('@'=>1,'O'=>1,'T'=>1,'W'=>1,'.'=>0,'G'=>0,'S'=>0))
    @assert splitext(filename)[end] == ".map"
    grid = zeros(Int,1,1)
    open(filename,"r") do io
        line = readline(io)
        if findfirst("type",line) == nothing
            throw(ErrorException("First line of map file should give type!"))
        end
        map_type = split(line)[end]
        if map_type != "octile"
            throw(ErrorException("Parser cannot handle non-octile maps!"))
        end
        line = readline(io)
        if findfirst("height",line) == nothing
            throw(ErrorException("Second line of map file should give height!"))
        end
        height = parse(Int,split(line)[end])
        line = readline(io)
        if findfirst("width",line) == nothing
            throw(ErrorException("Third line of map file should give width!"))
        end
        width = parse(Int,split(line)[end])
        @assert readline(io) == "map"
        grid = zeros(Int,height,width)
        for i in 1:height
            line = readline(io)
            for (j,c) in enumerate(line)
                if haskey(encoding,c)
                    grid[i,j] = encoding[c]
                else
                    throw(ErrorException("unrecognized character type $c"))
                end
            end
        end
    end
    return grid
end


"""
    parse_mapf_scenario(filename,map_path="")

Parses a .scen file into a set of 'buckets', where each bucket contains a list
of (start location,goal location) pairs. Each bucket can be used to instantiate
MAPF instances some (or all) of these pairs. The benchmarking approach proposed
on the benchmark website (https://www.movingai.com/benchmarks/index.html) is to
start with a 2-agent MAPF for each bucket, and increase the number of agents
until solver time out.
"""
function parse_mapf_scenario(filename,map_path="")
    @assert splitext(filename)[end] == ".scen"
    buckets = Dict{Int,Vector{ScenarioAgentSpec}}()
    mapfile = ""
    open(filename,"r") do io
        for (i,line) in enumerate(eachline(io))
            if i == 1
                if findfirst("version", line) != nothing
                    continue
                end
            end
            split_line = split(line)
            bucket = parse(Int,split_line[1])
            mapfile = split_line[2]
            map_width = parse(Int,split_line[3])
            map_height = parse(Int,split_line[4])
            start_x = parse(Int,split_line[5]) + 1
            start_y = parse(Int,split_line[6]) + 1
            goal_x = parse(Int,split_line[7]) + 1
            goal_y = parse(Int,split_line[8]) + 1
            optimal_length = parse(Float64,split_line[9])
            if !haskey(buckets,bucket)
                buckets[bucket] = valtype(buckets)()
            end
            push!(buckets[bucket],ScenarioAgentSpec(
                (start_x,start_y),
                (goal_x,goal_y)
            ))
        end
    end
    if isfile(map_path)
        mapfile = map_path
    else
        mapfile = joinpath(map_path,mapfile)
    end
    @assert isfile(mapfile)
    return MAPFScenario(
        mapfile,
        map(k->buckets[k],sort(collect(keys(buckets))))
    )
end

function generate_cbs_env(indicator_grid)
    CBSEnv.LowLevelEnv(graph=construct_factory_env_from_indicator_grid(indicator_grid))
end
function generate_state_map(env::GridFactoryEnvironment)
    (x,y) -> CBSEnv.State(env.vtx_map[x,y],0)
end

function construct_base_mapf(scen::MAPFScenario,env_generator=generate_cbs_env)
    grid = parse_map_file(scen.mapfile)
    env = env_generator(grid)
    return MAPF(env,Vector{state_type(env)}(),Vector{state_type(env)}())
end

function gen_mapf_problem_from_scenario(mapf::M,
        scen::MAPFScenario,
        bucket_id::Int,
        n_agents::Int,
        state_map=generate_state_map(mapf.env.graph),
        ) where {M<:MAPF}
    @assert(length(scen.buckets) >= bucket_id,
        "bucket_id $bucket_id out of range!")
    bucket = scen.buckets[bucket_id]
    @assert(length(bucket) >= n_agents,
        string("bucket only has $(length(bucket)) agents.",
        " Cannot instantiate a problem with $n_agents agents."))
    starts = Vector{state_type(mapf)}()
    goals = Vector{state_type(mapf)}()
    for i in 1:n_agents
        push!(starts, state_map(bucket[i].start...))
        push!(goals, state_map(bucket[i].goal...))
    end
    M(mapf.env,starts,goals)
end



"""
    MovingAIBenchmarkFile

Points to a TOML-formatted file that contains the following elements:
    scenario = "/path/to/scenario/file.scen"
    map_file = "/path/to/map/file.map"
    bucket_idx = 1 # an integer
    n_agents = 2 # an integer

Extension is .bm
"""
struct MovingAIBenchmarkFile
    path::String
end

function pad_file_number(id,pad=4)
    # hacky 0 padding to avoid dependency on Sprintf
    padding = prod(map(i->"0",1:(pad-length(string(id)))))
    return "$(padding)$(id)"
end

function generate_problem_files_from_moving_ai(
        scenario_paths,
        base_map_path,
        problem_dir
        ;
        verbose=true,
        pad=4,
        )
    problem_id = 1
    mkpath(problem_dir)
    for scen_file in scenario_paths
        scenario = parse_mapf_scenario(scen_file,base_map_path)
        for (bucket_idx,bucket) in enumerate(scenario.buckets)
            for n_agents in 2:length(bucket)
                config_dict = Dict(
                    "scenario" => scen_file,
                    "map_file" => scenario.mapfile,
                    "bucket_idx" => bucket_idx,
                    "n_agents" => n_agents,
                )
                problem_filename = joinpath(
                    problem_dir,"problem_$(pad_file_number(problem_id,pad)).bm")
                if isfile(problem_filename)
                    if verbose
                        println("File ",problem_filename," already exists.",
                            " Skipping...")
                    end
                else
                    open(problem_filename,"w") do io
                        TOML.print(io,config_dict)
                    end
                end
                problem_id += 1
            end
        end
    end
    return true
end

"""
    ProblemLoader

Can be queried via `get_problem(iterator,problem_filename)` to return MAPF
instances.
"""
struct ProblemLoader{M}
    scenarios::Dict{String,MAPFScenario}
    base_mapfs::Dict{String,M}
end

function init_mapf_loader(problem_dir)
    scenarios = Dict{String,MAPFScenario}()
    base_mapfs = Dict{String,MAPF}()
    for problem_file in readdir(problem_dir;join=true)
        problem_config = TOML.parsefile(problem_file)
        scen_file   = problem_config["scenario"]
        map_file    = problem_config["map_file"]
        if !haskey(scenarios,scen_file)
            scenarios[scen_file] = parse_mapf_scenario(
                scen_file, problem_config["map_file"]
            )
        end
        if !haskey(base_mapfs,map_file)
            base_mapfs[map_file] = construct_base_mapf(
                scenarios[scen_file]
            )
        end
    end
    return ProblemLoader(scenarios,base_mapfs)
end

function CRCBS.load_problem(loader::ProblemLoader,probfile)
    problem_config = TOML.parsefile(probfile)
    scen_file   = problem_config["scenario"]
    map_file    = problem_config["map_file"]
    bucket_idx  = problem_config["bucket_idx"]
    n_agents    = problem_config["n_agents"]
    if !haskey(loader.scenarios,scen_file)
        loader.scenarios[scen_file] = parse_mapf_scenario(
            scen_file, problem_config["map_file"]
        )
    end
    if !haskey(loader.base_mapfs,map_file)
        loader.base_mapfs[map_file] = construct_base_mapf(
            loader.scenarios[scen_file]
        )
    end
    scenario = loader.scenarios[scen_file]
    base_mapf = loader.base_mapfs[map_file]
    mapf = gen_mapf_problem_from_scenario(
        base_mapf, scenario, bucket_idx, n_agents)
    return mapf
end

end # moduler
