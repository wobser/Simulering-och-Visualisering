function [xdd] = xdd(x1,x2,m)
%XDD Summary of this function goes here
%   Detailed explanation goes here
G = 6.67384e-11;
xdd = ((G * m) / norm(x2-x1)^3)*(x2-x1);
end

