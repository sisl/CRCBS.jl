export
    LowLevelSearchHeuristic,
    NullHeuristic,
    PerfectHeuristic,
    # DeadlineHeuristic,
    MultiStagePerfectHeuristic,
    HardConflictTable,
        set_path!,
    SoftConflictTable,
    CompositeHeuristic,
        construct_composite_heuristic

abstract type LowLevelSearchHeuristic{C} <: AbstractCostModel{C} end
################################################################################
############################## CompositeHeuristic ##############################
################################################################################
struct CompositeHeuristic{M<:Tuple,T<:Tuple} <: LowLevelSearchHeuristic{T}
    cost_models::M
end
function construct_composite_heuristic(args...)
    models = Tuple(args)
    for m in models
        @assert typeof(m) <: LowLevelSearchHeuristic
    end
    cost_types = map(m->get_cost_type(m),models)
    CompositeHeuristic{typeof(models),Tuple{cost_types...}}(models)
end
# get_cost_type(h::H) where {T,M,H<:CompositeHeuristic{T,M}} = T
function get_heuristic_cost(model::H,args...) where {T,M,H<:CompositeHeuristic{M,T}}
    T(map(h->get_heuristic_cost(h,args...), model.cost_models))
end
function get_heuristic_cost(env::E,model::H,args...) where {E<:AbstractLowLevelEnv,T,M,H<:CompositeHeuristic{M,T}}
    T(map(m->get_heuristic_cost(env,m,args...), model.cost_models))
end

################################################################################
################################ NullHeuristic #################################
################################################################################
"""
    `NullHeuristic`
"""
struct NullHeuristic <: LowLevelSearchHeuristic{Float64} end
get_heuristic_cost(h::NullHeuristic,args...) = 0.0
get_heuristic_cost(env::E,h::NullHeuristic,args...) where {E<:AbstractLowLevelEnv} = get_heuristic_cost(h,args...)

################################################################################
############################### PerfectHeuristic ###############################
################################################################################
"""
    `PerfectHeuristic`

    The Perfect Heuristic stores the exact distance between any vertex in a
    graph to all goal vertices specified during construction. The distances are
    stored in `dists`, a dictionary which maps a goal vertex `v` to a vector of
    distances from any other vertex in `1:nv(G)` to `v`.

    An example of how to access the distance from vertex `v1` to goal `v2`:
    `get_heuristic_cost(h,v1,v2) = h.dists[v2][v1]`
"""
@with_kw struct PerfectHeuristic <: LowLevelSearchHeuristic{Float64}
    dists::Dict{Int,Vector{Float64}} = Dict{Int,Vector{Float64}}()
end
get_heuristic_cost(h::PerfectHeuristic,goal_vtx::Int,vtx::Int) = h.dists[goal_vtx][vtx]
function PerfectHeuristic(graph,starts::Vector{Int},goals::Vector{Int})
    dists = Dict(v => gdistances(graph,v) for v in goals)
    PerfectHeuristic(dists)
end

"""
    `MultiStagePerfectHeuristic`

    Stores multiple lookup tables corresponding to different stages of a Path-
    Finding search. Each stage has a different goal. The heuristic value at a
    particular stage must reflect not just the distance to the next goal but the
    length of the path through all remaining goals.
"""
@with_kw struct MultiStagePerfectHeuristic <: LowLevelSearchHeuristic{Float64}
    dists::Dict{Int,Vector{Vector{Float64}}} = Dict{Int,Vector{Vector{Float64}}}()
end
get_heuristic_cost(h::MultiStagePerfectHeuristic,agent_idx::Int,stage::Int,vtx::Int) = h.dists[agent_idx][stage][vtx]
function construct_multi_stage_distance_array(G,goals)
    if length(goals) > 0
        vtxs = copy(goals)
        dists = Vector{Vector{Float64}}()
        d = 0
        g = vtxs[end]
        while length(vtxs) > 0
            v = pop!(vtxs)
            d = gdistances(G,g)[v] + d
            push!(dists, gdistances(G,v).+ d)
            g = v
        end
        return reverse(dists)
    end
    return Vector{Vector{Float64}}()
end
function MultiStagePerfectHeuristic(graph,goals::Vector{Vector{Int}})
    dists = Dict(idx => construct_multi_stage_distance_array(graph,g) for (idx,g) in enumerate(goals))
    MultiStagePerfectHeuristic(dists)
end

################################################################################
############################### HardConflictTable ##############################
################################################################################
"""
    `HardConflictTable`

    Stores a lookup table of planned paths for all agents, to be used as a tie-
    breaking heuristic for planning a new path through the graph.
    When agent `i` queries the table, `table.paths[i]` (the existing path plan
    for agent `i`) must be subtracted so that the agent does not try to avoid
    conflicts with itself.
"""
@with_kw struct HardConflictTable{V<:AbstractVector,M<:AbstractMatrix} <: LowLevelSearchHeuristic{Float64}
    paths   ::Vector{V} = Vector{SparseVector{Int,Int}}()
    CAT     ::M         = SparseMatrixCSC(zeros(2,2)) # global table
end
get_time_horizon(h::H) where {H<:HardConflictTable} = size(h.CAT,2)
get_planned_vtx(h::H,agent_idx::Int,t::Int) where {H<:HardConflictTable} = h.paths[agent_idx][t]
function reset_path!(h::H,path_idx::Int) where {V,M,H<:HardConflictTable{V,M}}
    # first remove the old path from the lookup table
    for (t,vtx) in enumerate(h.paths[path_idx])
        if vtx > 0
            h.CAT[vtx,t] = h.CAT[vtx,t] - 1
        end
    end
    # initialize an empty new path
    h.paths[path_idx] = V(zeros(Int,get_time_horizon(h)))
    return h
end
function set_path!(h::H,path_idx::Int,path::Vector{Int},start_time::Int=1) where {V,M,H<:HardConflictTable{V,M}}
    # println("Updating Heuristic model with path ", path, " for agent ",path_idx)
    reset_path!(h,path_idx)
    # now add new path vtxs to new path and lookup table
    for (i,vtx) in enumerate(path)
        t = start_time+i-1
        h.paths[path_idx][t] = vtx
        h.CAT[vtx,t] = h.CAT[vtx,t] + 1
    end
    h
end
set_path!(h::H,args...) where {H<:LowLevelSearchHeuristic} = nothing
function set_path!(h::H,args...) where {H<:CompositeHeuristic}
    for m in h.cost_models
        set_path!(m,args...)
    end
end
function get_heuristic_cost(h::HardConflictTable,agent_idx::Int,vtx::Int,t::Int)
    h_cost = h.CAT[vtx,t]
    if get_planned_vtx(h,agent_idx,t) == vtx # conflict with own trajectory
        return h_cost - 1
    end
    return h_cost
end
"""
    `construct_empty_lookup_table(G,T::Int)`

    Returns an empty lookup table.
"""
construct_empty_lookup_table(V::Int,T::Int) = SparseMatrixCSC(zeros(V,T))
construct_empty_lookup_table(graph::G,T::Int) where {G<:AbstractGraph} = construct_empty_lookup_table(nv(graph),T)
function HardConflictTable(graph::G,T::Int,num_agents::Int) where {G<:AbstractGraph}
    HardConflictTable(
        paths = map(i->SparseVector(zeros(Int,T)),1:num_agents),
        CAT = construct_empty_lookup_table(graph,T)
        )
end

################################################################################
############################### SoftConflictTable ##############################
################################################################################
"""
    `SoftConflictTable`

    The `SoftConflictTable` is a tie-breaker heuristic in A* search  within the
    CBS paradigm.
"""
@with_kw struct SoftConflictTable{M<:AbstractMatrix} <: LowLevelSearchHeuristic{Float64}
    CAT::M = SparseMatrixCSC(zeros(2,2)) # global table
end
get_time_horizon(h::SoftConflictTable) = size(h.CAT,2)
get_heuristic_cost(h::SoftConflictTable,vtx::Int,t::Int) = h.CAT[vtx,t]
"""
    `get_fat_path(G,D,start_vtx,goal_vtx)`

    returns a fat path through `G` from `start_vtx` to `goal_vtx`. Each set
    of vertices in the fat path contains all vertices with distance d1 from
    start_vtx and distance d2 to goal_vtx, where d1+d2 == the length of the
    shortest path(s) from `start_vtx` to `goal_vtx`

    G is a graph, D is the distance matrix
"""
function get_fat_path(G,D,start_vtx::Int,goal_vtx::Int)
    fat_path = Vector{Set{Int}}([Set{Int}(start_vtx)])
    for i in 1:D[start_vtx,goal_vtx]
        next_set = Set{Int}()
        for src_vtx in fat_path[end]
            for dst_vtx in outneighbors(G,src_vtx)
                if D[dst_vtx,goal_vtx] <= D[src_vtx,goal_vtx] - 1
                    push!(next_set, dst_vtx)
                end
            end
        end
        push!(fat_path, next_set)
    end
    fat_path
end
"""
    `add_fat_path_to_table(CAT,fat_path)`
"""
function add_fat_path_to_table!(CAT,t0,fat_path)
    for t in 1:length(fat_path)
        idxs = collect(fat_path[t])
        if t+t0 > 0
            CAT[idxs,t+t0] .+= 1.0/length(idxs)
        end
    end
end
"""
    `populate_soft_lookup_table!(CAT,start_times,start_vtxs,goal_vtxs)`
"""
function populate_soft_lookup_table!(CAT,G,D,start_times,start_vtxs,goal_vtxs)
    path_list = [t=>get_fat_path(G,D,s,g) for (s,t,g) in zip(start_vtxs,start_times,goal_vtxs)]
    for (t0,fat_path) in path_list
        add_fat_path_to_table!(CAT,t0,fat_path)
    end
    CAT
end
"""
    `construct_empty_lookup_table(graph,T::Int)`

    Returns a soft lookup table to encode possible paths for each agent through
    `graph`. The argument `T` defines the time horizon of the lookup table.
"""
function SoftConflictTable(graph,T::Int)
    SoftConflictTable(construct_empty_lookup_table(graph,T))
end
"""
    `construct_and_populate_soft_lookup_table!`
"""
function SoftConflictTable(graph,start_times::Vector{Int},start_vtxs::Vector{Int},goal_vtxs::Vector{Int};
        T = Int(round(maximum(start_times) + nv(graph))))
    CAT = construct_empty_lookup_table(graph,T)
    D = get_dist_matrix(graph)
    populate_soft_lookup_table!(CAT,graph,D,start_times,start_vtxs,goal_vtxs)
    SoftConflictTable(CAT)
end
