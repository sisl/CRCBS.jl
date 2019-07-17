export
    Experiment_parameters,
    load_experiment_parameters,
    load_graph,
    run_problem

struct Experiment_parameters
    name::String
    lambda::Float64
    epsilon::Float64
    t_delay::Float64
    num_particles::Int64
    vs::Vector{Int}
    es::Vector{Tuple{Int,Int}}
    starts::Vector{Int}
    goals::Vector{Int}
    xs::Vector{Int}
    ys::Vector{Int}
end




# ----------------------- DATA STORAGE FUNCTIONS ----------------------------- #

function mapf_to_exp_parameters(name,num_particles,mapf)
    vs = [v for v in vertices(mapf.graph)]
    es = [(e.src,e.dst) for e in edges(mapf.graph)]
    return Experiment_parameters(name,mapf.lambda,mapf.epsilon,mapf.t_delay,num_particles,vs,es,mapf.starts,mapf.goals)
end

function solution_to_txt(solution, id)
    io = open(string("../experiments/solutions/CRCBS_",string(id),".txt"), "w")
    for path in solution
        for e in path
            print(io, string(string(e.src), " ", string(e.dst), "; "))
        end
        println(io, "")
    end
    close(io)
    return
end

function load_experiment_parameters(file)
    """Creates an experiment set with all the experiment sets"""
    experiment_vector = Vector{}()


    exp_set = load(string("../experiments/experiment_parameters/",file,".jld"))
    #println(exp_set)
    parameters = exp_set["parms"]

    #parameters = Experiment_parameters(experiment_vector)
    return parameters
end

function solution_to_txt(solution, time, astartime, fnctime, num_iterations,id)
    io = open(string("../experiments/solutions/CBS_",string(id),".txt"), "w")
    println(io,time)
    println(io,astartime)
    println(io,fnctime)
    println(io,num_iterations)
    for path in solution
        for e in path
            print(io, string(string(e.src), " ", string(e.dst), "; "))
        end
        println(io, "")
    end
    close(io)
    return
end

function run_problem(name)
    exp_parms = load_experiment_parameters(name)
    G = MetaGraph()
    for (k,v) in enumerate(exp_parms.vs)
        add_vertex!(G,Dict(:x=>exp_parms.xs[k],:y=>exp_parms.ys[k]))
    end
    for e in exp_parms.es
        add_edge!(G,e[1],e[2])
        set_prop!(G, Edge(e[1],e[2]), :weight, 1.0)
    end
    mapf = MAPF(G,exp_parms.starts,exp_parms.goals)
    a = @timed(CBS(mapf))
    llsolution = a[1][1]
    cost = a[1][2]
    astartime = a[1][3]
    fnctime = a[1][4]
    num_iterations = a[1][5]
    computation_time = a[2]
    solution_to_txt(llsolution,computation_time, astartime, fnctime,num_iterations, name)
    return (llsolution,cost,computation_time)
end
# ---------------------------------------------------------------------------- #
