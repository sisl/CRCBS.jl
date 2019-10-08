export
    LowLevelSearchHeuristic,
    NullHeuristic,
    PerfectHeuristic,
    DefaultPerfectHeuristic,
    # DeadlineHeuristic,
    MultiStagePerfectHeuristic,
    ConflictTableHeuristic,
    HardConflictHeuristic,
    SoftConflictHeuristic,
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

struct DefaultPerfectHeuristic <: LowLevelSearchHeuristic{Float64}
    h::PerfectHeuristic
end
get_heuristic_cost(h::DefaultPerfectHeuristic,goal_vtx::Int,vtx::Int) = haskey(h.h.dists,goal_vtx) ? get_heuristic_cost(h.h,goal_vtx,vtx) : 0.0

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
############################# ConflictTableHeuristic ###########################
################################################################################
struct ConflictTableHeuristic{T<:Union{HardConflictTable,SoftConflictTable}} <: LowLevelSearchHeuristic{Float64}
    table::T
end
get_heuristic_cost(h::H,args...) where {H<:ConflictTableHeuristic} = get_conflict_value(h.table, args...)

HardConflictHeuristic(args...) = ConflictTableHeuristic(HardConflictTable(args...))
SoftConflictHeuristic(args...) = ConflictTableHeuristic(SoftConflictTable(args...))

get_time_horizon(h::H) where {H<:ConflictTableHeuristic} = get_time_horizon(h.table)
get_planned_vtx(h::H,args...) where {T<:HardConflictTable,H<:ConflictTableHeuristic}  = get_planned_vtx(h.table,args...)
reset_path!(h::H,args...) where {T<:HardConflictTable,H<:ConflictTableHeuristic{T}}   = reset_path!(h.table,args...)
set_path!(h::H,args...) where {T<:HardConflictTable,H<:ConflictTableHeuristic{T}}     = set_path!(h.table,args...)
set_path!(h::H,args...) where {H<:LowLevelSearchHeuristic} = nothing
function set_path!(h::H,args...) where {H<:CompositeHeuristic}
    for m in h.cost_models
        set_path!(m,args...)
    end
end
