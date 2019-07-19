export
    AbstractMAPF,
    MAPF,
    MultiMAPF,
    num_agents,
    num_goals

abstract type AbstractMAPF end

"""
    A MAPF is an instance of a Multi Agent Path Finding problem. It consists of
    a graph `G` whose edges have unit length, as well as a list of start and
    goal vertices on that graph. Note that this is the _labeled_ case, where
    each agent has a specific assigned destination.
"""
struct MAPF{S,G} <: AbstractMAPF# Multi Agent Path Finding Problem
    graph::G # <: AbstractGraph
    starts::Vector{S}   # Vector of initial agent states
    goals::Vector{S}    # Vector of goal states
end
num_agents(mapf::AbstractMAPF) = length(mapf.starts)
num_goals(mapf::AbstractMAPF) = length(mapf.goals)

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
end
