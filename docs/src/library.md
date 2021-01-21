# Core Types and Methods

```@docs
PathNode
Path
get_s
get_a
get_sp
LowLevelSolution
```

## Core Interface
```@docs
is_goal
get_possible_actions
get_next_state
wait
get_transition_cost
get_path_cost
get_heuristic_cost
build_env
states_match
violates_constraints
check_termination_criteria
```

## Problem Definitions
```@docs
MAPF
```

## Cost Models and interface
```@docs
get_initial_cost
get_infeasible_cost
add_heuristic_cost
accumulate_cost
aggregate_costs
CompositCostModel
MetaCostModel
TravelTime
TravelDistance
CompositeHeuristic
NullHeuristic
PerfectHeuristic
EnvDistanceHeuristic
MultiStagePerfectHeuristic
```

## Solvers
```@docs
AStar
CBSSolver
MetaAgentCBS_Solver
PIBTPlanner
solve!
reset_solver!
hard_reset_solver!
SolverLogger
SolverWrapper
```

## Environments
```@docs
```

## Profiling
```@docs
FeatureExtractor
extract_feature
profile_solver!
load_problem
write_results
run_profiling
init_dataframe
construct_results_dataframe
construct_config_dataframe
```