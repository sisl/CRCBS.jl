export
    AbstractMAPF,
    num_agents,
    num_goals,
    get_starts,
    get_goals,
    get_start,
    get_goal,

    MAPF,
    MultiMAPF,
    MetaMAPF

abstract type AbstractMAPF end
num_agents(mapf::AbstractMAPF)          = length(mapf.starts)
num_goals(mapf::AbstractMAPF)           = length(mapf.goals)
get_starts(mapf::AbstractMAPF)          = mapf.starts
get_goals(mapf::AbstractMAPF)           = mapf.goals
get_start(mapf::AbstractMAPF, i)        = get_starts(mapf)[i]
get_goal(mapf::AbstractMAPF, i)         = get_goals(mapf)[i]
get_start(mapf::AbstractMAPF, env, i)   = get_start(mapf,i)

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

struct MetaMAPF{M} <: AbstractMAPF
    mapf::M
end
num_agents(mapf::MetaMAPF)  = num_agents(mapf.mapf)
num_goals(mapf::MetaMAPF)   = num_goals(mapf.mapf)
get_starts(mapf::MetaMAPF)  = get_starts(mapf.mapf)
get_goals(mapf::MetaMAPF)   = get_goals(mapf.mapf)
