syms x

D = (sqrt((x*sqrt(2) + 22.75)*(x*sqrt(2) + 10.5)*(x*sqrt(2) + 12.25)*22.75) + ...
    x^2/2)^2 - 1/4*x^2*(x+10.5)^2

solve(D)