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
    MetaMAPF,
    PC_TAPF

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

"""
    `PC_TAPF`

    Precedence-Constrained Multi-Agent Path-Finding Problem.
    Elements:
    * env::E - the base environment
    * `task_precedence_graph` - edges encode the precedence constraints between
    tasks.
    * `num_agents` - identifies number of agents involved
    * `agent_ids` - `agent_ids[i]` says which agent is assigned to task i
    * `start_times`- `start_times[i]` gives start time of task i
    * `deadlines` - `deadlines[i]` is deadline for task i
    * `durations` - `durations[i]` is minimum time expected to complete task i
    * `starts` - `starts[i]` gives the start state associated with path i
    * `goals` - `goals[i]` gives the goal associated with path i
"""
@with_kw struct PC_TAPF{E,GR<:AbstractGraph,S,G}
    env::E                      = nothing
    task_precedence_graph::GR   = DiGraph()
    num_agents ::Int            = 0
    agent_ids  ::Vector{Int}    = Vector{Int}() # tracks which robot is assigned to which task
    start_times::Vector{Int}    = Vector{Int}() # time when each task is scheduled to begin
    deadlines  ::Vector{Int}    = Vector{Int}() # deadline by which each task must be completed
    durations  ::Vector{Int}    = Vector{Int}() # expected duration for each task (slack[i] = (deadlines[i] - start_times[i]) - durations[i])
    starts     ::Vector{S}      = Vector{Int}() # start states of agents (includes start times, I suppose)
    goals      ::Vector{G}      = Vector{Int}() # goals assigned to agents
end
num_agents(mapf::M) where {M<:PC_TAPF}          = length(mapf.starts)
num_goals(mapf::M) where {M<:PC_TAPF}           = length(mapf.goals)
get_starts(mapf::M) where {M<:PC_TAPF}          = mapf.starts
get_goals(mapf::M) where {M<:PC_TAPF}           = mapf.goals
get_start(mapf::M, i) where {M<:PC_TAPF}        = get_starts(mapf)[i]
get_goal(mapf::M, i) where {M<:PC_TAPF}         = get_goals(mapf)[i]
get_start(mapf::M, env, i) where {M<:PC_TAPF}   = get_start(mapf,i)
