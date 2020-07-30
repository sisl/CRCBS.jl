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
        cost_model = get_cost_model(mapf.env),
        costs = Vector{T}(map(a->get_initial_cost(mapf.env),1:num_agents(mapf))),
        cost = get_initial_cost(mapf.env),
    )
end
"""
    `get_infeasible_solution`
"""
function get_infeasible_solution(mapf::MAPF{E,S,G}) where {S,A,G,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    LowLevelSolution{S,A,T,C}(
        paths = Vector{Path{S,A,T}}(map(a->Path{S,A,T}(cost=get_initial_cost(mapf.env)),1:num_agents(mapf))),
        cost_model = get_cost_model(mapf.env),
        costs = Vector{T}(map(a->get_initial_cost(mapf.env),1:num_agents(mapf))),
        cost = get_infeasible_cost(mapf.env),
    )
end
"""
    `default_solution(solver, mapf::AbstractMAPF)`

    Defines what is returned by the solver in case of failure to find a feasible
    solution.
"""
function default_solution(mapf::M) where {M<:AbstractMAPF}
    return get_infeasible_solution(mapf), get_infeasible_cost(mapf.env)
end

"""
    add_to_path!(path,env,s,a,sp)

Adds the new (s,a,sp) tuple and its cost (under env) to path.
"""
function add_to_path!(path::Path,env,s,a,sp)
    push!(path,PathNode(s,a,sp))
    set_cost!(path,accumulate_cost(env,get_cost(path),
        get_transition_cost(env,s,a,sp)))
    path
end

################################################################################
################################# Extend Paths #################################
################################################################################
export
    extend_path!

function extend_path!(env::E,path::P,T::Int) where {E<:AbstractLowLevelEnv,P<:Path}
    while get_index_from_time(path,get_end_index(path)) < T
        s = get_final_state(path)
        a = wait(s)
        sp = get_next_state(env,s,a)
        push!(path,PathNode(s,a,sp))
        set_cost!(path, accumulate_cost(env, get_cost(path), get_transition_cost(env,s,a,sp)))
    end
    return path
end
function extend_path(env::E,path::P,args...) where {E<:AbstractLowLevelEnv,P<:Path}
    new_path = copy(path)
    extend_path!(new_path,args...)
    return new_path
end

export sorted_actions

"""
    sorted_actions(env,s)

Return a vector of actions sorted lowest cost to highest cost.
"""
function sorted_actions(env,s)
    f = (s,a,sp)->add_heuristic_cost(env,get_transition_cost(env,s,a,sp),get_heuristic_cost(env,sp))
    sort(
        collect(get_possible_actions(env,s)),
        by=a->f(s,a,get_next_state(env,s,a))
    )
end

################################################################################
################################## Utilities ###################################
################################################################################
export
    is_consistent,
    is_valid


"""
    is_consistent(solution,mapf)

    Check if solution satisfies start and end constraints
"""
function is_consistent(path::P,start::S,goal::G) where {S,G,P<:Path}
    return (states_match(get_initial_state(path), start)
        && states_match(get_final_state(path), goal))

end
function is_consistent(paths::Vector{P},starts::Vector{S},goals::Vector{G}) where {S,G,P <: Path}
    for (i,path) in enumerate(paths)
        if !is_consistent(path,starts[i],goals[i])
            return false
        end
    end
    return true
end
function is_consistent(solution::L,starts::Vector{S},goals::Vector{G}) where {S,G,L<:LowLevelSolution}
    is_consistent(get_paths(solution),starts,goals)
end
function is_consistent(solution::L,mapf::M) where {L<:LowLevelSolution, M<:AbstractMAPF}
    is_consistent(solution,get_starts(mapf),get_goals(mapf))
end
is_valid(args...) = is_consistent(args...)

################################################################################
######################### Visualization and Debugging ##########################
################################################################################

export
    convert_to_vertex_lists

function convert_to_vertex_lists(path) end
function convert_to_vertex_lists(solution::S) where {S<:LowLevelSolution}
    return [convert_to_vertex_lists(path) for path in get_paths(solution)]
end
