export
    AbstractMAPF,
    MAPF,
    action_type,
    state_type,
    cost_type,
    num_agents,
    num_goals,
    get_starts,
    get_goals,
    get_start,
    get_goal,
    get_start

abstract type AbstractMAPF end

"""
    A MAPF is a Multi Agent Path Finding problem. It consists of an environment,
    `env`, through which a group of agents may navigate, as well as a list of
    start and goal states in that environment. Note that this is the _labeled_
    case, where each agent has a specific assigned destination.

    Elements:
    - env::E - the base environment
    - starts::Vector{S} - the vector of initial states
    - starts::Vector{G} - the vector of goals
"""
struct MAPF{E,S,G} <: AbstractMAPF # Multi Agent Path Finding Problem
    env     ::E           # Environment Type
    starts  ::Vector{S}   # Vector of initial states
    goals   ::Vector{G}   # Vector of goal states
end
action_type(mapf::MAPF)         = action_type(mapf.env)
state_type(mapf::MAPF)          = state_type(mapf.env)
cost_type(mapf::MAPF)           = cost_type(mapf.env)
num_agents(mapf::MAPF)          = length(mapf.starts)
num_goals(mapf::MAPF)           = length(mapf.goals)
get_starts(mapf::MAPF)          = mapf.starts
get_goals(mapf::MAPF)           = mapf.goals
get_start(mapf::MAPF, i)        = get_starts(mapf)[i]
get_goal(mapf::MAPF, i)         = get_goals(mapf)[i]
get_start(mapf::MAPF, env, i)   = get_start(mapf,i)
# TODO implement a check to be sure that no two agents have the same goal
