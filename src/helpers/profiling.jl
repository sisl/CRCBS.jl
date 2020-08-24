
export ScalarFeature

"""
    ScalarFeature

Abstract type for features that may be reported about the solution to a PC-TAPF
    or sequential task assignment problem
"""
abstract type ScalarFeature{T} end

export
    RunTime,
    SolutionCost,
    OptimalityGap,
    OptimalFlag,
    FeasibleFlag,
    NumConflicts

struct RunTime          <: ScalarFeature{Float64} end
struct SolutionCost     <: ScalarFeature{Float64} end
struct OptimalityGap    <: ScalarFeature{Float64} end
struct OptimalFlag      <: ScalarFeature{Bool} end
struct FeasibleFlag     <: ScalarFeature{Bool} end
struct NumConflicts     <: ScalarFeature{Int} end

export extract_feature

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

extract_feature(solver,feats::Tuple,args...) = Dict(string(feat)=>extract_feature(solver,feat,args...),feats)

extract_feature(solver,::RunTime,       solution,timer_results) = timer_results.t
extract_feature(solver,::SolutionCost,  solution,timer_results) = best_cost(solver)
extract_feature(solver,::OptimalityGap, solution,timer_results) = optimality_gap(solver)
extract_feature(solver,::OptimalFlag,   solution,timer_results) = optimality_gap(solver) <= 0
extract_feature(solver,::FeasibleFlag,  solution,timer_results) = best_cost(solver) < typemax(typeof(best_cost(solver)))
extract_feature(solver,::NumConflicts,  solution,timer_results) = count_conflicts(detect_conflicts(results.solution))
