let
    SolverLogger{Int}()
    SolverLogger{Float64}()
    logger = SolverLogger{NTuple{3,Float64}}()

    verbosity(logger)
    debug(logger)

	# CRCBS.@log_info(-1,SolverLogger{Int}(),"This message should be printed")
	@log_info(-1,0,"This message should be printed")
	@log_info(-1,verbosity(logger),"This message should be printed")
	@log_info(-1,logger,"This message should be printed")
	limit = -1
	@log_info(limit,logger,"This message should be printed")
	@log_info(limit,logger,"This ","message"," should be printed")
	msg = "This message should be printed"
	@log_info(limit,logger,msg)
	@log_info(limit,logger)
	x = 1
    @log_info(-1,logger,"$x",string(limit)," from s = ",string(limit),"for env ",string(cost_type(logger)), " with env.goal = ")

	# macro checks(a,b,x...)
	# 	ex = Expr(:call,:println,"[ logger ]: ")
	# 	# show(x)
	# 	# show(esc(x).head)
	# 	# show(esc(x).args)
	#     # for xx in esc(x).args[1]
	#     for xx in x
	# 		push!(ex.args,xx)
	#     end
	# 	# ex
	# 	:($(esc(a)) > $(esc(b)) ? $ex : nothing)
	# 	# nothing
	# end
	# a = 1
	# b = 2
	# c = 3
	# d = 4
	# e = 5
	# @checks(a,b,c,d,e)
	# @checks(b,a,c,d,e)
	# @checks(a,b,c,d)
	# @checks(a,b,c)
	# @checks(a,b)


	set_time_out_status!(logger,true)
	@test time_out_status(logger)
	set_iteration_max_out_status!(logger,true)
	@test iteration_max_out_status(logger)
	reset_solver!(logger)
	@test !time_out_status(logger)
	@test !iteration_max_out_status(logger)

    iters = iterations(logger)
    increment_iteration_count!(logger)
    @test iterations(logger) == iters + 1

    lim = 100
    set_iteration_limit!(logger,lim)
    @test iteration_limit(logger) == lim

    CRCBS.set_max_iterations!(logger,lim)
    @test max_iterations(logger) == lim

    t = 10.0
    CRCBS.set_start_time!(logger,t)
    @test start_time(logger) == t

    set_runtime_limit!(logger,t)
    @test runtime_limit(logger) == t

    set_deadline!(logger,t)
    @test deadline(logger) == t

    set_lower_bound!(logger,(1.0,0.0,0.0))
	@test lower_bound(logger) == (1.0,0.0,0.0)
    set_lower_bound!(logger,2.0)
	@test lower_bound(logger) == (2.0,0.0,0.0)
    set_lower_bound!(logger,3)
	@test lower_bound(logger) == (3.0,0.0,0.0)

    set_best_cost!(logger,(2.0,0.0,0.0))
	@test best_cost(logger) == (2.0,0.0,0.0)
    set_best_cost!(logger,3.0)
	@test best_cost(logger) == (3.0,0.0,0.0)
    set_best_cost!(logger,4)
	@test best_cost(logger) == (4.0,0.0,0.0)

    reset_solver!(logger)
    hard_reset_solver!(logger)
end
let
    logger = SolverLogger{Tuple{Int,Float64,Float64}}()
    set_lower_bound!(logger,(1,0.0,0.0))
	@test lower_bound(logger) == (1,0.0,0.0)
    set_lower_bound!(logger,2.0)
	@test lower_bound(logger) == (2,0.0,0.0)
    set_lower_bound!(logger,3)
	@test lower_bound(logger) == (3,0.0,0.0)

    set_best_cost!(logger,(2,0.0,0.0))
	@test best_cost(logger) == (2,0.0,0.0)
    set_best_cost!(logger,3.0)
	@test best_cost(logger) == (3,0.0,0.0)
    set_best_cost!(logger,4)
	@test best_cost(logger) == (4,0.0,0.0)
end
