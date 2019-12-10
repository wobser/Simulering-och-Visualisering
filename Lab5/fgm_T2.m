function [p2,pe,R01,R2e] = fgm_T2(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
l1 = 1;
l2 = 1;
%[pe] = [l1*cos(q(1)) + l2*cos(q(1)+q(2)) , l1*sin(q(1)) + l2*sin(q(1)+q(2))];
R01 = rzang(q(1));
p1 = [0 0]';
T01 = [R01 p1;
       0 0 1];
R12 = rzang(q(2));
p2 = [l1 0]';
T12 = [R12 p2;
       0 0 1];
R2e = rzang(0);
p2e = [l2 0]';
T2e = [R2e p2e;
       0 0 1];
T0e = T01*T12*T2e;
T2 = T01*T12;
p2 = T2(1:2,3)';
pe = [T0e(1,3) T0e(2,3)]';
end

