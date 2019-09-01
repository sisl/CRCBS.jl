################################################################################
############################### Path Constructors ##############################
################################################################################

################################################################################
######################### LowLevelSolution Constructors ########################
################################################################################
export
    get_initial_solution,
    get_infeasible_solution,
    default_solution
"""
    `get_initial_solution`
"""
function get_initial_solution(mapf::MAPF{E,S,G}) where {S,A,G,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    LowLevelSolution{S,A,T,C}(
        paths = map(i->Path{S,A,T}(
                    s0=get_start(mapf,i),
                    cost=get_initial_cost(mapf.env)),
                1:num_agents(mapf)),
        costs = Vector{T}(map(a->get_initial_cost(mapf.env),1:num_agents(mapf))),
        cost = get_initial_cost(mapf.env),
        cost_model = get_cost_model(mapf.env)
    )
end
"""
    `get_infeasible_solution`
"""
function get_infeasible_solution(mapf::MAPF{E,S,G}) where {S,A,G,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    LowLevelSolution{S,A,T,C}(
        paths = Vector{Path{S,A,T}}(map(a->Path{S,A,T}(),1:num_agents(mapf))),
        costs = Vector{T}(map(a->get_initial_cost(mapf.env),1:num_agents(mapf))),
        cost = get_initial_cost(mapf.env),
        cost_model = get_cost_model(mapf.env)
    )
end
"""
    `default_solution(solver::AbstractMAPFSolver, mapf::AbstractMAPF)`

    Defines what is returned by the solver in case of failure to find a feasible
    solution.
"""
function default_solution(mapf::MAPF{E,S,G}) where {S,A,G,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    return get_infeasible_solution(mapf), get_infeasible_cost(mapf.env)
end

################################################################################
################################## Utilities ###################################
################################################################################
export
    is_valid
"""
    Checks if an individual path satisfies start and end constraints
"""
function is_valid(path::P,start::S,goal::G) where {S,G,P<:Path}
    return (states_match(get_initial_state(path), start)
        && states_match(get_final_state(path), goal))
end
function is_valid(paths::Vector{P},starts::Vector{S},goals::Vector{G}) where {S,G,P <: Path}
    for (i,path) in enumerate(paths)
        if !is_valid(path,starts[i],goals[i])
            return false
        end
    end
    return true
end
function is_valid(solution::L,starts::Vector{S},goals::Vector{G}) where {S,G,L<:LowLevelSolution}
    is_valid(get_paths(solution),starts,goals)
end
function is_valid(solution::L,mapf::M) where {L<:LowLevelSolution, M<:AbstractMAPF}
    is_valid(solution,get_starts(mapf),get_goals(mapf))
end
