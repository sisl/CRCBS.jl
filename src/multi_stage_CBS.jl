export
    MultiStageCBS

# CBS submodule
module MultiStageCBS

using ..CRCBS
using Parameters, LightGraphs, DataStructures

@with_kw struct State
    vtx::Int    = -1 # vertex of graph
    stage::Int  = -1 # which stage of the sequence
    t::Int      = -1
end
@with_kw struct Action
    e::Edge{Int}    = Edge(-1,-1)
    Δt::Int         = 1
end
function construct_multi_stage_distance_array(G,goals)
    vtxs = [s.vtx for s in goals]
    dists = Vector{Vector{Float64}}()
    d = 0
    g = vtxs[end]
    while length(vtxs) > 0
        v = pop!(vtxs)
        d = gdistances(G,g)[v] + d
        push!(dists, gdistances(G,v).+ d)
        g = v
    end
    reverse(dists)
end
@with_kw struct LowLevelEnv{G <: AbstractGraph} <: AbstractLowLevelEnv{State,Action}
    graph::G                        = Graph()
    constraints::ConstraintTable    = ConstraintTable()
    goal_sequence::Vector{State}    = Vector{State}()
    agent_idx::Int                  = -1
    # helpers
    dists::Vector{Vector{Float64}} = construct_multi_stage_distance_array(graph,goal_sequence)
end
heuristic(env::LowLevelEnv,s) = env.dists[s.stage][s.vtx]

""" Type alias for a path through the graph """
const CBSPath = Path{State,Action}

""" Helper for displaying Paths """
function convert_to_vertex_lists(path::Path)
    vtx_list = [n.sp.vtx for n in path.path_nodes]
    if length(path) > 0
        vtx_list = [get_s(get_path_node(path,1)).vtx, vtx_list...]
    end
    vtx_list
end
function convert_to_vertex_lists(solution::LowLevelSolution)
    return [convert_to_vertex_lists(path) for path in solution]
end

CRCBS.states_match(s1::State,s2::State) = (s1.vtx == s2.vtx)
function CRCBS.is_valid(path::CBSPath,start::State,goals::Vector{State})
    valid = true
    stage = 1
    if states_match(start,goals[stage])
        stage += 1
    end
    for k in 1:length(path)
        node = get_path_node(path,k)
        if states_match(get_sp(node),goals[stage])
            stage += 1
        end
        if stage > length(goals)
            return true
        end
    end
    return false
end
function CRCBS.is_goal(env::LowLevelEnv,s::State)
    if states_match(s, env.goal_sequence[s.stage])
        ###########################
        # Cannot terminate if there is a constraint on the goal state in the
        # future (e.g. the robot will need to move out of the way so another
        # robot can pass)
        if s.stage == length(env.goal_sequence) # terminal goal state
            for constraint in env.constraints.sorted_state_constraints
                if s.t < get_time_of(constraint)
                    if states_match(s, get_sp(constraint.v))
                        # @show s.t, get_time_of(constraint)
                        return false
                    end
                end
            end
            return true # done!
        else
            return false # not done yet!
        end
        ###########################
    end
    return false
end
CRCBS.wait(s::State) = Action(e=Edge(s.vtx,s.vtx))

# get_possible_actions
struct ActionIter
    # env::LowLevelEnv
    s::Int # source state
    neighbor_list::Vector{Int} # length of target edge list
end
struct ActionIterState
    idx::Int # idx of target node
end
function Base.iterate(it::ActionIter)
    iter_state = ActionIterState(0)
    return iterate(it,iter_state)
end
function Base.iterate(it::ActionIter, iter_state::ActionIterState)
    iter_state = ActionIterState(iter_state.idx+1)
    if iter_state.idx > length(it.neighbor_list)
        return nothing
    end
    Action(e=Edge(it.s,it.neighbor_list[iter_state.idx])), iter_state
end
CRCBS.get_possible_actions(env::LowLevelEnv,s::State) = ActionIter(s.vtx,outneighbors(env.graph,s.vtx))
CRCBS.get_next_state(s::State,a::Action) = State(a.e.dst, s.stage, s.t+a.Δt)
function CRCBS.get_next_state(env::LowLevelEnv,s::State,a::Action)
    stage = s.stage
    if states_match(s, env.goal_sequence[s.stage])
        stage = min(stage+1, length(env.goal_sequence))
    end
    return State(a.e.dst, stage, s.t+a.Δt)
end
CRCBS.get_transition_cost(env::LowLevelEnv,s::State,a::Action,sp::State) = 1
function CRCBS.violates_constraints(env::LowLevelEnv, path::Path{State,Action}, s::State, a::Action, sp::State)
    t = length(path) + 1
    if StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.state_constraints
        # @show s,a,sp
        return true
    elseif ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.action_constraints
        # @show s,a,sp
        return true
    end
    return false

    # cs = StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t)
    # constraints = env.constraints.sorted_state_constraints
    # idx = max(1, find_index_in_sorted_array(constraints, cs)-1)
    # for i in idx:length(constraints)
    #     c = constraints[i]
    #     if c == cs
    #         @show s,a,sp
    #         return true
    #     end
    #     if c.t < cs.t
    #         break
    #     end
    # end
    # ca = StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t)
    # constraints = env.constraints.sorted_action_constraints
    # idx = max(1, find_index_in_sorted_array(constraints, ca)-1)
    # for i in idx:length(constraints)
    #     c = constraints[i]
    #     if c == ca
    #         @show s,a,sp
    #         return true
    #     end
    #     if c.t < ca.t
    #         break
    #     end
    # end
    # return false
end
CRCBS.check_termination_criteria(env::LowLevelEnv,cost,path,s) = false


""" Returns an invalid StateConflict """
# invalid_state_conflict() = StateConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
invalid_state_conflict() = Conflict{PathNode{State,Action},PathNode{State,Action}}(conflict_type=STATE_CONFLICT)

"""
    Detect a `StateConflict` between two CBS path nodes.
"""
function CRCBS.detect_state_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if n1.sp.vtx == n2.sp.vtx
        return true
    end
    return false
end

""" Returns an invalid ActionConflict """
# invalid_action_conflict() = ActionConflict(-1,-1,invalid_state(0),invalid_state(0),-1)
# invalid_action_conflict() = ActionConflict{State,State}()
invalid_action_conflict() = Conflict{PathNode{State,Action},PathNode{State,Action}}(conflict_type=ACTION_CONFLICT)

"""
    Detect an `ActionConflict` between two CBS path nodes.
"""
function CRCBS.detect_action_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if (n1.a.e.src == n2.a.e.dst) && (n1.a.e.dst == n2.a.e.src)
        return true
    end
    return false
end

"""
    Construct an empty `ConstraintTreeNode` from a `MAPF` instance
"""
function initialize_root_node(mapf::MultiMAPF)
    ConstraintTreeNode(
        solution = LowLevelSolution{State,Action}([CBSPath() for a in 1:num_agents(mapf)]),
        constraints = Dict{Int,ConstraintTable}(
            i=>ConstraintTable(a=i) for i in 1:length(mapf.starts)
            ),
        id = 1)
end

"""
    Initialize a new `ConstraintTreeNode` with the same `solution` and
    `constraints` as the parent node
"""
function initialize_child_search_node(parent_node::ConstraintTreeNode)
    ConstraintTreeNode(
        solution = copy(parent_node.solution),
        constraints = copy(parent_node.constraints),
        conflict_table = copy(parent_node.conflict_table),
        parent = parent_node.id
    )
end

"""
    The Conflict-Based Search algorithm for multi-agent path finding - Sharon et
    al 2012

    https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
"""
struct CBS_Solver <: AbstractMAPFSolver end

"""
    Returns a low level solution for a MAPF with constraints
"""
function low_level_search!(
    solver::CBS_Solver,
    mapf::MultiMAPF,
    node::ConstraintTreeNode,
    idxs=collect(1:num_agents(mapf)),
    path_finder=A_star)
    # Only compute a path for the indices specified by idxs
    for i in idxs
        # TODO allow passing custom heuristic
        # dists = [gdistances(mapf.graph, mapf.goals[i].vtx)]
        env = LowLevelEnv(
            graph = mapf.graph,
            constraints = get_constraints(node,i),
            goal_sequence = mapf.goals[i],
            agent_idx = i
            )
        h = s-> heuristic(env,s)
        # Solve!
        path = path_finder(env, mapf.starts[i], h)

        node.solution[i] = path
    end
    node.cost = get_cost(node.solution)
    # TODO check if solution is valid
    return true
end

"""
    Run Conflict-Based Search on an instance of MultiMAPF
"""
function CRCBS.solve!(solver::CBS_Solver, mapf::MultiMAPF, path_finder=A_star)
    # priority queue that stores nodes in order of their cost
    priority_queue = PriorityQueue{ConstraintTreeNode,Int}()
    # node_list = Vector{ConstraintTreeNode}()

    root_node = initialize_root_node(mapf)
    low_level_search!(solver,mapf,root_node)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if is_valid(root_node.solution,mapf)
        # @show root_node
        enqueue!(priority_queue, root_node => root_node.cost)
        # push!(node_list,root_node)
    end

    # k = 0
    while length(priority_queue) > 0
        # @show k += 1
        node = dequeue!(priority_queue)
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        # @show conflict.node1.sp, conflict.agent1_id
        if is_valid(conflict)
            constraints = generate_constraints_from_conflict(conflict)
        else
            print("Optimal Solution Found! Cost = ",node.cost,"\n")
            # for (i,p) in enumerate(node.solution)
            #     @show i=>[n.sp.vtx for n in p.path_nodes]
            # end
            return node.solution, node.cost
        end

        # generate new nodes from constraints
        for constraint in constraints
            new_node = initialize_child_search_node(node)
            # new_node.id = length(node_list) + 1
            if add_constraint!(new_node,constraint)
                low_level_search!(solver,mapf,new_node,[get_agent_id(constraint)])
                # for (i,p) in enumerate(new_node.solution)
                #     @show i=>[n.sp.vtx for n in p.path_nodes]
                # end
                detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)]) # update conflicts related to this agent
                if is_valid(new_node.solution, mapf)
                    # @show new_node.constraints
                    enqueue!(priority_queue, new_node => new_node.cost)
                    # push!(node_list, new_node)
                end
            end
        end
        # if k > 5
        #     break
        # end
    end
    # @show length(node_list)
    print("No Solution Found. Returning default solution")
    return LowLevelSolution{State,Action}(), typemax(Int)
end

end # end module MultiStageCBS
