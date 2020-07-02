export SolverException

"""
    SolverException

Custom exception type for tracking solver timeouts, etc.
"""
struct SolverException <: Exception
    msg::String
end

export SolverLogger

"""
    SolverLogger

A logger type for keeping track of thing like runtime, iterations, optimality
gap (including upper and lower bound), etc.
"""
@with_kw mutable struct SolverLogger{C}
    iterations      ::Int           = 0
    iteration_limit ::Int           = 100
    max_iterations  ::Int           = 0
    start_time      ::Float64       = time()
    runtime_limit   ::Float64       = 100.0
    deadline        ::Float64       = Inf
    lower_bound     ::C             = typemin(C)
    best_cost       ::C             = typemax(C)
    verbosity       ::Int           = 0
    DEBUG           ::Bool          = false
end
cost_type(logger::SolverLogger{C}) where {C} = C
cost_type(solver) = get_cost_type(get_logger(solver))
get_cost_type(logger::SolverLogger{C}) where {C} = C
get_cost_type(solver) = get_cost_type(get_logger(solver))

export get_logger

get_logger(solver) = solver.logger
get_logger(logger::SolverLogger) = logger

export
    iterations,
    max_iterations,
    iteration_limit,
    start_time,
    runtime_limit,
    deadline,
    lower_bound,
    best_cost,
    verbosity,
    debug

iterations(logger)        = get_logger(logger).iterations
iteration_limit(logger)   = get_logger(logger).iteration_limit
max_iterations(logger)    = get_logger(logger).max_iterations
start_time(logger)        = get_logger(logger).start_time
runtime_limit(logger)     = get_logger(logger).runtime_limit
deadline(logger)          = get_logger(logger).deadline
# JuMP.lower_bound(logger)  = get_logger(logger).lower_bound
lower_bound(logger)       = get_logger(logger).lower_bound
best_cost(logger)         = get_logger(logger).best_cost
verbosity(logger)         = get_logger(logger).verbosity
debug(logger)             = get_logger(logger).DEBUG

export
	set_iterations!,
	set_iteration_limit!,
	set_max_iterations!,
	set_start_time!,
	set_runtime_limit!,
	set_deadline!,
	set_lower_bound!,
	set_best_cost!,
	set_verbosity!,
	set_debug!

function set_iterations!(solver,val)
	get_logger(solver).iterations = val
end
function set_iteration_limit!(solver,val)
	get_logger(solver).iteration_limit = val
end
function set_max_iterations!(solver,val)
	get_logger(solver).max_iterations = val
end
function set_start_time!(solver,val)
	get_logger(solver).start_time = val
end
function set_runtime_limit!(solver,val)
	get_logger(solver).runtime_limit = val
end
function set_deadline!(solver,val)
	get_logger(solver).deadline = val
end
function set_lower_bound!(solver,val)
	get_logger(solver).lower_bound = val
end
function set_lower_bound!(logger::SolverLogger{C},val::C) where {C}
    logger.lower_bound = val
end
function set_lower_bound!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
    logger.lower_bound = NTuple{N,T}((T(val),zeros(T,N-1)...))
end
function set_lower_bound!(logger::SolverLogger{T},val::R) where {T<:Tuple,R<:Real}
    v = cost_type(logger)([val,zeros(length(lower_bound(logger))-1)...])
    set_lower_bound!(logger,v)
end
function set_best_cost!(solver,val)
	get_logger(solver).best_cost = val
end
function set_best_cost!(logger::SolverLogger{C},val::C) where {C}
    logger.best_cost = val
end
function set_best_cost!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
    logger.best_cost = NTuple{N,T}((T(val),zeros(T,N-1)...))
end
function set_best_cost!(logger::SolverLogger{T},val::R) where {T<:Tuple,R<:Real}
    v = cost_type(logger)([val,zeros(length(lower_bound(logger))-1)...])
    set_best_cost!(logger,v)
end
function set_verbosity!(solver,val)
	get_logger(solver).verbosity = val
end
function set_debug!(solver,val)
	get_logger(solver).DEBUG = val
end

export
    optimality_gap,
    increment_iteration_count!,
    reset_solver!,
    hard_reset_solver!

optimality_gap(logger) = best_cost(logger) .- lower_bound(logger)
function check_time(logger)
    t = time()
    if t >= deadline(logger) || t - start_time(logger) >= runtime_limit(logger)
        return true
    end
    return false
end
function enforce_time_limit(logger)
    if check_time(logger)
        throw(SolverException("Solver time limit exceeded!"))
    end
end
function check_iterations(logger)
    iterations(logger) > iteration_limit(logger)
end
function enforce_iteration_limit(logger)
    if check_iterations(logger)
        throw(SolverException("Solver iterations exceeded!"))
    end
end

function increment_iteration_count!(logger::SolverLogger)
    logger.iterations += 1
    set_max_iterations!(logger,max(iterations(logger),max_iterations(logger)))
end

"""
    reset_solver!(solver)

Resets iteration counts and start times.
"""
function reset_solver!(logger::SolverLogger)
    set_iterations!(logger, 0)
    set_best_cost!(logger,typemax(cost_type(logger)))
    set_lower_bound!(logger,typemin(cost_type(logger)))
    set_start_time!(logger,time())
    logger
end

"""
    hard_reset_solver!(solver)

To be called when no information (other than iteration and time limits) needs to
be stored.
"""
function hard_reset_solver!(logger::SolverLogger)
    reset_solver!(logger)
    set_max_iterations!(logger,0)
end

get_infeasible_cost(logger::SolverLogger{C}) where {C} = typemax(C)
get_infeasible_cost(solver) = get_infeasible_cost(get_logger(solver))

increment_iteration_count!(solver)  = increment_iteration_count!(get_logger(solver))
set_lower_bound!(solver,val)        = set_lower_bound!(get_logger(solver),val)
set_best_cost!(solver,val)          = set_best_cost!(get_logger(solver),val)
reset_solver!(solver)               = reset_solver!(get_logger(solver))
hard_reset_solver!(solver)          = hard_reset_solver!(get_logger(solver))

export
    log_info

"""
    log_info

A helper function for printing at various verbosity levels
"""
function log_info(limit::Int,verbosity::Int,msg...)
    if verbosity > limit
        println("[ logger ]: ",msg...)
    end
end
log_info(limit::Int,solver,msg...) = log_info(limit,verbosity(solver),msg...)
