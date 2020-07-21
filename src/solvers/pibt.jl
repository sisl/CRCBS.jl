# Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding
# Okumura et al, IJCAI 2019
# https://www.ijcai.org/Proceedings/2019/0076.pdf
export PIBTPlanner

@with_kw struct PIBTPlanner{C}
    logger::SolverLogger{C} = SolverLogger{C}()
end

"""
    PIBTCache{S,A}

Contains info to be passed along through recursive calls to the PIBT algorithm
for multi-agent path planning.
Info to be stored:
- current state of each agent (should be lined up at the same time step)
- priority of each agent
- the planned action (and associated next state) for each agent
- the search environment for each agent, which contains e.g., the agent's goal,
    cost_model, heuristic_model, etc.
- a conflict table of sorts to indicate which states/actions are reserved
"""
struct PIBTCache{E,S,A}
    envs::Vector{E}
    states::Vector{S}
    actions::Vector{A}
    priorities::Vector{Int}
    undecided::Set{Int} # agent ids
    occupied::Set{Int}
    timers::Vector{Int}
end
function init_cache(solver::PIBTPlanner,mapf::MAPF)
    node = initialize_root_node(solver,mapf)
    envs = map(i->build_env(solver,mapf,node,i), 1:num_agents(mapf))
    states = map(i->get_start(mapf,i), 1:num_agents(mapf))
    actions = map(i->wait(envs[i],states[i]), 1:num_agents(mapf))
    priorities = collect(1:num_agents(mapf))
    undecided = Set(collect(1:num_agents(mapf)))
    occupied = Set{Int}()
    timers = zeros(Int,num_agents(mapf))
    PIBTCache(
        envs,states,actions,priorities,undecided,occupied,timers
    )
end
function update_solution!(solver::PIBTPlanner,solution,cache)
    for (p,env,s,a) in zip(get_paths(solution),cache.envs,cache.states,cache.actions)
        sp = get_next_state(env,s,a)
        push!(p,PathNode(s,a,sp))
    end
    solution
end
function set_priorities!(solver::PIBTPlanner,cache)
    cache.priorities .= reverse(sortperm(
        map(i->(cache.timers[i],i), 1:length(cache.timers))
        ))
    log_info(3,solver,"priorities: ", cache.priorities)
    return cache
end
function update_cache!(solver::PIBTPlanner,mapf,cache,solution)
    union!(cache.undecided,Set(collect(1:num_agents(mapf))))
    empty!(cache.occupied)
    for (i,p) in enumerate(get_paths(solution))
        s = get_final_state(p)
        cache.states[i] = s
        cache.actions[i] = wait(cache.envs[i],cache.states[i])
        if is_goal(cache.envs[i],s) && is_valid(get_goal(cache.envs[i]))
            node = initialize_root_node(solver,mapf)
            cache.envs[i] = build_env(mapf,node,-1)
            cache.timers[i] = 0
        else
            cache.timers[i] += 1
        end
    end
    if any(cache.timers .== 0)
        set_priorities!(solver,cache)
    end
    return cache
end

function get_next_agent_id(solver,cache::PIBTCache)
    for i in sortperm(cache.priorities)
        if i in cache.undecided
            return i
        end
    end
    return -1
end

function sorted_actions(env,s)
    f = (s,a,sp)->add_heuristic_cost(env,get_transition_cost(env,s,a,sp),get_heuristic_cost(env,sp))
    sort(
        collect(get_possible_actions(env,s)),
        by=a->f(s,a,get_next_state(env,s,a))
    )
end

function is_reserved(cache::PIBTCache,env,s,a,sp)
    idx,_ = serialize(env,sp,-1)
    return (idx in cache.occupied)
end
function reserve!(cache::PIBTCache,env,s,a,sp)
    idx,_ = serialize(env,sp,-1)
    push!(cache.occupied,idx)
    return cache
end
function set_action!(cache::PIBTCache,i,a)
    cache.actions[i] = a
end
"""
    get_conflict_index(cache,i,s,a,sp)

Returns the index of an agent that currently occupies `sp`, or -1 if there is no
such agent.
"""
function get_conflict_index(cache,i,s,a,sp)
    for (k,sk) in enumerate(cache.states)
        if states_match(sp,sk) && k != i
            if k in cache.undecided
                return k
            end
        end
    end
    return -1
end

function is_consistent(cache::PIBTCache,mapf)
    for (env,s) in zip(cache.envs,cache.states)
        if !is_goal(env,s)
            return false
        end
    end
    return true
end

export pibt_step!

"""
    pibt_step!(solver,mapf,i,j=-1)

i is the id of the higher priority agent, j is the index of the lower priority
agent.
"""
function pibt_step!(solver,mapf,cache,i,j=-1)
    log_info(3,solver,"pibt_step!( ... i = ",i,", j = ",j," )")
    env = cache.envs[i]
    s = cache.states[i]
    sj = get(cache.states, j, state_type(mapf)())
    a_list = sorted_actions(env,s) # NOTE does NOT need to exclude wait()
    while ~isempty(a_list)
        a = a_list[1]
        sp = get_next_state(env,s,a)
        if !is_reserved(cache,env,s,a,sp) && !states_match(sp,sj)
            reserve!(cache,env,s,a,sp)
            log_info(3,solver,"reserve!( ... a = ",string(a),", sp = ",string(sp)," )")
            k = get_conflict_index(cache,i,s,a,sp)
            if k != -1
                log_info(3,solver,"get_conflict_index( i = ",i,", sp = ",string(sp)," ) : ",k)
                if pibt_step!(solver,mapf,cache,k,i)
                    set_action!(cache,i,a)
                    setdiff!(cache.undecided,i)
                    return true
                else
                    deleteat!(a_list,1)
                    break
                end
            else
                set_action!(cache,i,a)
                setdiff!(cache.undecided,i)
                return true
            end
        else
            log_info(3,solver,"illegal action ",string(a))
            deleteat!(a_list,1)
        end
    end
    set_action!(cache,i,wait(env,s))
    return false
end

export pibt!

function pibt!(solver, mapf)
    solution = get_initial_solution(mapf)
    node = initialize_root_node(mapf)
    cache = init_cache(solver,mapf)
    while !is_consistent(cache,mapf)
        try
            increment_iteration_count!(solver)
            enforce_iteration_limit(solver)
        catch e
            if isa(e,SolverException)
                bt = catch_backtrace()
                showerror(stdout,e)
                return solution, is_consistent(cache,mapf)
            else
                rethrow(e)
            end
        end
        log_info(3,solver,"PIBT iterations = ",iterations(solver))
        # update cache
        while !isempty(cache.undecided)
            i = get_next_agent_id(solver,cache)
            if ~pibt_step!(solver,mapf,cache,i)
                return solution, false
            end
        end
        # update solution with cache
        update_solution!(solver,solution,cache)
        log_info(3,solver,"solution: ",convert_to_vertex_lists(solution))
        update_cache!(solver,mapf,cache,solution)
    end
    return solution, is_consistent(cache,mapf)
end
