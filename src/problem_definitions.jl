export
    AbstractMAPF,
    num_agents,
    num_goals,
    get_starts,
    get_goals,

    MAPF,
    MultiMAPF,
    MetaMAPF

abstract type AbstractMAPF end
num_agents(mapf::AbstractMAPF) = length(mapf.starts)
num_goals(mapf::AbstractMAPF) = length(mapf.goals)
get_starts(mapf::AbstractMAPF) = mapf.starts
get_goals(mapf::AbstractMAPF) = mapf.goals
get_start(mapf::AbstractMAPF, i) = get_starts(mapf)[i]
get_goal(mapf::AbstractMAPF, i) = get_goals(mapf)[i]

"""
    A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
    a graph `G` whose edges have unit length, as well as a list of start and
    goal vertices on that graph. Note that this is the _labeled_ case, where
    each agent has a specific assigned destination.
"""
struct MAPF{S,G} <: AbstractMAPF # Multi Agent Path Finding Problem
    graph::G            # <: AbstractGraph
    starts::Vector{S}   # Vector of initial agent states
    goals::Vector{S}    # Vector of goal states
    # TODO store distance matrices here so they don't need to be regenerated at
    # each iteration by build_env()
end

"""
    MultiMAPF{S,G}

    The Multi-Stage Multi-Agent Path-Finding problem. Each agent is assigned an
    arbitrarily long vector of goal states to visit, and the optimal solution is
    the lowest-cost list of trajectories for which each agent visits its
    assigned goals in the specified order.
"""
struct MultiMAPF{S,G} <: AbstractMAPF
    graph::G # <: AbstractGraph
    starts::Vector{S}   # Vector of initial agent states
    goals::Vector{Vector{S}}    # Vector of goal states
    # TODO store distance matrices here so they don't need to be regenerated at
    # each iteration by build_env()
end

# """
#     MetaMAPF{S,G}
#
#     The MetaAgent Multi-Agent Path-Finding problem. Agents can be combined into
#     a "meta agent", meaning that low-level search is performed in their joint
#     state space.
# """
# struct MetaMAPF{S,G} <: AbstractMAPF
#     graph::G # <: AbstractGraph
#     starts::Vector{S}   # Vector of initial agent states
#     goals::Vector{S}    # Vector of goal states
#     # TODO store distance matrices here so they don't need to be regenerated at
#     # each iteration by build_env()
# end

struct MetaMAPF{M} <: AbstractMAPF
    mapf::M
end
num_agents(mapf::MetaMAPF)  = num_agents(mapf.mapf)
num_goals(mapf::MetaMAPF)   = num_goals(mapf.mapf)
get_starts(mapf::MetaMAPF)  = get_starts(mapf.mapf)
get_goals(mapf::MetaMAPF)   = get_goals(mapf.mapf)
