using Distributions

export Hypergeometric1F1

function Hypergeometric1F1(a::Real,b::Real,t,N)

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

    # Initiate the stepsize
    h = 2*pi/N;

    # Optimal constants for Talbot contour
    c1 = 0.5017;
    c2 = 0.6407;
    c3 = 0.6122;
    c4 = 0.2645i;

    # Evaluating the Laplace inversion at each point theta which is based on the trapezoidal rule
    k       = 1:N;
    theta   = -pi + (k + 0.5)*h;
    z       = N/t*(c1*theta./tan(c2*theta) - c3 + c4*theta);
    dz      = N/t*(-c1*c2*theta./sin(c2*theta).^2 + c1./tan(c2*theta)+c4);

    F       = z.^(-b).*(1 + 1./z).^(a-b);
    evalint = sum(exp(z*t).*F.*dz);

    val     = gamma(b)*t^(1-b)*exp(t)*h/(2i*pi)*evalint;
    return val
