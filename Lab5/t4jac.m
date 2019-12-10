function [j] = t4jac(q)
%T4JAC Summary of this function goes here
%   Detailed explanation goes here
l1=1;
l2=1;
j = [-l1*sin(q(1))-l2*sin(q(1)+q(2)) -l2*sin(q(1)+q(2));
    l1*cos(q(1))+l2*cos(q(1)+q(2)) l2*cos(q(1)+q(2))];
end

