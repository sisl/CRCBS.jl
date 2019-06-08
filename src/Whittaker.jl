using Distributions
using HCubature
using SpecialFunctions
using LinearAlgebra

export
Hypergeometric1F1,
conflict_probability_node_w,
WhittakerM,
WhittakerM_cont,
WhittakerW

function Hypergeometric1F1(a::Real,b::Real,t,N=50)

## Created by Fernando Damian Nieuwveldt
## Date : 26 October 2009
## email : fdnieuwveldt@gmail.com
## Solving the Hypergeometric 1F1 function using Laplace inversion and Talbot contours
##
## Example :
## octave:1> Hypergeometric1F1(5,2,100-1000i,50)
## ans = 7.0029e+50 + 8.9738e+50i
##
## References :
## A. ErdÃƒÂ©lyi, W. Magnus, F. Oberhettinger, and F. Tricomi. Higher
## transcendental functions. Vol. I. Robert E. Krieger Publishing Co.
## Inc., Melbourne, Fla., 1981, pg 273
##
## L.N.Trefethen, J.A.C.Weideman, and T.Schmelzer. Talbot quadratures
## and rational approximations. BIT. Numerical Mathematics,
## 46(3):653 670, 2006.
    #println("a= ",a)
    #println("b= ",b)
    #println("t= ",t)

    # Initiate the stepsize
    h = 2*pi/N

    # Optimal constants for Talbot contour
    c1 = 0.5017
    c2 = 0.6407
    c3 = 0.6122
    c4 = 0.2645im

    # Evaluating the Laplace inversion at each point theta which is based on the trapezoidal rule
    k       = collect(1:N)
    theta   = -pi .+ (k .+ 0.5)*h
    z       = N/t*(c1*theta ./ Float64[tan(c2*thetai) for thetai in theta] .- c3 .+ c4*theta)
    dz      = N/t*(-c1*c2*theta ./ Float64[sin(c2*thetai) for thetai in theta].^2 .+ c1./ Float64[tan(c2*thetai) for thetai in theta] .+ c4)

    F       = z .^(-b).*(1 .+ 1.0 ./z).^(a-b)
    evalint = sum([exp(zi*t) for zi in z] .* F .* dz)

    # Eventually you need to raise a complex number to a negative power which is not handled in julia?
    invval = try (t)^(1-b)
    catch
        (conj(t)/(norm(t))^2)^(b-1)
    end

    val     = gamma(b)*invval*exp(t)*h/(2im*pi).*evalint

    return val
end

function WhittakerM(l,m,z)
    if 2*m+1 < 0
        if round(m)==m
            return WhittakerM(l,m+0.0001,z)
        end
        return real(z^(-m+1/2)*exp(-z/2)*Hypergeometric1F1(-m-l+1/2,2*(-m)+1,z))
    else
        return real(z^(m+1/2)*exp(-z/2)*Hypergeometric1F1(m-l+1/2,2*m+1,z))
    end
end

function WhittakerM_cont(l,m,z)
    if round(m) == m
        return  WhittakerM(l,m+0.0001,z)
    end
    return real(z^(-m+1/2)*exp(-z/2)*Hypergeometric1F1(-m-l+1/2,2*(-m)+1,z))
end


function WhittakerW(k,mu,z)

    l = mu - k + 0.5
    m = 1 + 2*mu + 0.00001

    return real(z^(-mu+1/2)*exp(-z/2)*(gamma(1-m)*Hypergeometric1F1(l,m,z)/gamma(l+1-m) + gamma(m-1)*Hypergeometric1F1(l+1-m,2-m,z)/gamma(l+0.00001)))

    if 0.5-m-l < 0
        return gamma(2*m)*WhittakerM(l,-m,z)/gamma(1/2+m-l) + gamma(-2*m)*WhittakerM(l,m,z)/gamma(1/2-m-l)
    else
        return gamma(-2*m)*WhittakerM(l,m,z)/gamma(1/2-m-l) + gamma(2*m)*WhittakerM(l,-m,z)/gamma(1/2+m-l)
    end
end

function conflict_probability_node_w(n1,t1,n2,t2,nn,lambda)
    """Integrate over only one variable instead of two"""

    # constants
    alpha = 0.5*(n1-n2)
    beta = 0.5*(1-n1-n2)
    c1 = 1.0/(gamma(n1)*(2.0*lambda)^((n1+n2)/2.0))
    c2 = 1.0/(gamma(n2)*(2.0*lambda)^((n1+n2)/2.0))
    #c1 = (gamma(n1)*(2.0*lambda)^((n1+n2)/2.0))
    #c2 = (gamma(n2)*(2.0*lambda)^((n1+n2)/2.0))
    phi = 0.5*(n1+n2)-1


    function hplus(y)
        density = (1-cdf(Gamma(nn,lambda),abs(t1-t2-y))) * c1 * (y^phi) * WhittakerM(alpha,beta,2*y/lambda)
        return density
    end
    function hminus(y)
        density = (1-cdf(Gamma(nn,lambda),abs(t1-t2-y))) * c2 * ((-y)^phi) * WhittakerM(-alpha,beta,-2*y/lambda)
        return density
    end

    a = -30.0
    b = 0.01
    c = 30.0

    res1 = @timed(hquadrature(hminus,a,-b,maxevals=10^6))
    C1,err1 = res1[1]
    dt1 = res1[2] #time spent performing integration

    res2 = @timed(hquadrature(hplus,b,c,maxevals=10^6))
    C2,err2 = res2[1]
    dt2 = res2[2] #time spent performing integration

    C = C1 + C2
    err = err1 + err2
    dt = dt1 + dt2

    return C, err, dt
end
