export FeatureExtractor

"""
    FeatureExtractor

Abstract type for features that may be reported about the solution to a PC-TAPF
    or sequential task assignment problem
"""
abstract type FeatureExtractor{T} end

export
    RunTime,
    MemAllocs,
    GCTime,
    ByteCount,
    SolutionCost,
    OptimalityGap,
    OptimalFlag,
    FeasibleFlag,
    NumConflicts,
    IterationCount

export
    TimeOutStatus,
    IterationMaxOutStatus

export
    RobotPaths,
    RobotSeparation

struct RunTime          <: FeatureExtractor{Float64} end
struct MemAllocs        <: FeatureExtractor{Float64} end
struct GCTime           <: FeatureExtractor{Float64} end
struct ByteCount        <: FeatureExtractor{Float64} end
struct SolutionCost     <: FeatureExtractor{Vector{Float64}} end
struct OptimalityGap    <: FeatureExtractor{Float64} end
struct OptimalFlag      <: FeatureExtractor{Bool} end
struct FeasibleFlag     <: FeatureExtractor{Bool} end
struct NumConflicts     <: FeatureExtractor{Int} end
struct IterationCount   <: FeatureExtractor{Int} end
struct TimeOutStatus    <: FeatureExtractor{Bool} end
struct IterationMaxOutStatus <: FeatureExtractor{Bool} end
struct RobotPaths       <: FeatureExtractor{Vector{Int}} end
struct RobotSeparation  <: FeatureExtractor{Vector{Int}} end

export
    extract_feature,
    load_feature

"""
    extract_feature(solution,extractor,solution,solve_time)

Function must be implemented to extract features from solutions
Args:
- solver (pc_tapf solver)
- extractor <: FeatureExtractor
- solution <: SearchEnv
- timer_results::NamedTuple{t,bytes,gctime,memallocs}
"""
function extract_feature end

extract_feature(solver,feats::Tuple,args...) = map(feat->extract_feature(solver,feat,args...),feats)

extract_feature(solver,::RunTime,       mapf,solution,timer_results) = timer_results.t
extract_feature(solver,::MemAllocs,     mapf,solution,timer_results) = timer_results.memallocs
extract_feature(solver,::GCTime,        mapf,solution,timer_results) = timer_results.gctime
extract_feature(solver,::ByteCount,     mapf,solution,timer_results) = timer_results.bytes
extract_feature(solver,::SolutionCost,  mapf,solution,timer_results) = get_cost(solution)
extract_feature(solver,::OptimalityGap, mapf,solution,timer_results) = optimality_gap(solver)
extract_feature(solver,::OptimalFlag,   mapf,solution,timer_results) = optimality_gap(solver) <= 0
extract_feature(solver,::FeasibleFlag,  mapf,solution,timer_results) = best_cost(solver) < typemax(typeof(best_cost(solver)))
extract_feature(solver,::NumConflicts,  mapf,solution,timer_results) = count_conflicts(detect_conflicts(solution))
extract_feature(solver,::IterationCount, mapf,solution,timer_results) = iterations(solver)
extract_feature(solver,::TimeOutStatus, mapf,solution,timer_results) = time_out_status(solver)
extract_feature(solver,::IterationMaxOutStatus, mapf,solution,timer_results) = iteration_max_out_status(solver)
extract_feature(solver,::RobotPaths,    mapf,solution,timer_results) = convert_to_vertex_lists(solution)
function extract_feature(solver,::RobotSeparation, mapf,solution,timer_results)
    separation_distances = Vector{Float64}()
    for (t,nodes) in enumerate(zip(map(p->p.path_nodes,get_paths(solution))...))
        d_min = Inf
        for (j,n1) in enumerate(nodes)
            for (k,n2) in enumerate(nodes)
                if k <= j
                    continue
                end
                d_min = min(get_distance(mapf,get_sp(n1),get_sp(n2)),d_min)
            end
        end
        push!(separation_distances,d_min)
    end
    return separation_distances
end

function load_feature(feat::FeatureExtractor, results)
    feature_key = string(typeof(feat))
    @assert(haskey(results,feature_key))
    return results[feature_key]
end

export compile_results

function compile_results(solver,feats,args...)
    results = Dict(string(typeof(feat))=>extract_feature(solver,feat,args...) for feat in feats)
    for (k,v) in results
        if isa(v,Tuple)
            results[k] = [v...] # because TOML doesn't handle tuples
        end
    end
    return results
end

export profile_solver!

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

export load_problem

"""
    load_problem(loader,filename)

Generic function that allows a problem instance to be retrived/constructed from
`loader` (which may cache information) and `filename::String` (which points to a
problem file).
"""
function load_problem end

export run_profiling

"""
    run_profiling(config,loader)

Profile the performance of one or more solvers on a set of problems defined
by files in a directory.
Args:
* `config`: A named tuple or struct with at least the following fields:
    - `problem_dir::String` : path to problem files.
    - `solver_configs` : A vector of objects, each of which has the fields:
        - `solver` : a solver complying with the CRCBS interface.
        - `results_path::String` : path at which to store results.
    - feats : A vector of `Feature` objects, which defines the specific
    features to be extracted and saved.
* `Loader`: Any kind of cache on which `load_problem(loader,problem_file)`
    can be called.
"""
function run_profiling(config,loader)
    for solver_config in config.solver_configs
        mkpath(solver_config.results_path)
    end
    for problem_file in readdir(config.problem_dir;join=true)
        problem_name = splitext(splitdir(problem_file)[end])[1]
        mapf = load_problem(loader,problem_file)
        for solver_config in config.solver_configs
            solver = solver_config.solver
            outfile = joinpath(solver_config.results_path,string(problem_name,".results"))
            solution, timer_results = profile_solver!(solver,mapf)
            results_dict = compile_results(
                solver,
                config.feats,
                mapf,
                solution,
                timer_results
                )
            results_dict["problem_file"] = problem_file
            open(outfile,"w") do io
                TOML.print(io,results_dict)
            end
        end
    end
end
