
export ScalarFeature

"""
    ScalarFeature

Abstract type for features that may be reported about the solution to a PC-TAPF
    or sequential task assignment problem
"""
abstract type ScalarFeature{T} end
abstract type VectorFeature{T} end

export
    RunTime,
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
    RobotPaths

struct RunTime          <: ScalarFeature{Float64} end
struct SolutionCost     <: ScalarFeature{Float64} end
struct OptimalityGap    <: ScalarFeature{Float64} end
struct OptimalFlag      <: ScalarFeature{Bool} end
struct FeasibleFlag     <: ScalarFeature{Bool} end
struct NumConflicts     <: ScalarFeature{Int} end
struct IterationCount   <: ScalarFeature{Int} end
struct TimeOutStatus    <: ScalarFeature{Bool} end
struct IterationMaxOutStatus <: ScalarFeature{Bool} end
struct RobotPaths       <: VectorFeature{Int} end

export
    extract_feature,
    load_feature

"""
    extract_feature(solution,model,solution,solve_time)

Function must be implemented to extract features from solutions
Args:
- solver (pc_tapf solver)
- model <: ScalarFeature
- solution <: SearchEnv
- timer_results::NamedTuple{t,bytes,gctime,memallocs}
"""
function extract_feature end

extract_feature(solver,feats::Tuple,args...) = map(feat->extract_feature(solver,feat,args...),feats)

extract_feature(solver,::RunTime,       solution,timer_results) = timer_results.t
extract_feature(solver,::SolutionCost,  solution,timer_results) = get_cost(solution)
extract_feature(solver,::OptimalityGap, solution,timer_results) = optimality_gap(solver)
extract_feature(solver,::OptimalFlag,   solution,timer_results) = optimality_gap(solver) <= 0
extract_feature(solver,::FeasibleFlag,  solution,timer_results) = best_cost(solver) < typemax(typeof(best_cost(solver)))
extract_feature(solver,::NumConflicts,  solution,timer_results) = count_conflicts(detect_conflicts(solution))
extract_feature(solver,::IterationCount, solution,timer_results) = iterations(solver)
extract_feature(solver,::TimeOutStatus, solution,timer_results) = time_out_status(solver)
extract_feature(solver,::IterationMaxOutStatus, solution,timer_results) = iteration_max_out_status(solver)
extract_feature(solver,::RobotPaths,    solution,timer_results) = convert_to_vertex_lists(solution)

function load_feature(feat::ScalarFeature, results)
    feature_key = string(typeof(feat))
    @assert(haskey(results,feature_key))
    return results[feature_key]
end

function compile_results(solver,feats,args...)
    results = Dict(string(typeof(feat))=>extract_feature(solver,feat,args...) for feat in feats)
    for (k,v) in results
        if isa(v,Tuple)
            results[k] = [v...]
        end
    end
    return results
end
