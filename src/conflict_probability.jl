using Distributions

function grid_collision_probability_node(n1,t1,n2,t2,nn,lambda)

    function h(x)
        y = x[1]
        t = x[2]
        density = (1-cdf(Gamma(nn,lambda),abs(t1-t2-y))) * pdf(Gamma(n2,lambda), t) * pdf(Gamma(n1,lambda),t-y)
        return density
    end

    dt = 0.01

    a = [-100.0;-100.0]
    b = [100.0,100.0]

    gridestimate = @timed(estimate_integral(a,b,dt,h))
    Cgrid=gridestimate[1]
    dtgrid = gridestimate[2]

    res1 = @timed(hcubature(h,a,b,maxevals=10^8))
    C,err = res1[1]
    dt = res1[2] #time spent performing integration

    return C, Cgrid, dt,dtgrid
end

function estimate_integral(a,b,dt,h)
    xs = range(a[1],stop=b[1],step=dt)
    ys = range(a[2],stop=b[2],step=dt)
    dS1dS2 = dt^2

    int=0

    for x in xs
        for y in ys
            int += h([x,y])
        end
    end

    int *= dS1dS2

    return int
end
