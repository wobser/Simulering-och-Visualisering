function [pe] = fgm_2(q)
%FGM_2 Summary of this function goes here
%   Detailed explanation goes here

%q1 = poly3(0,pi/4,0,0,[0:0.1:1])
%q2 = poly3(0,pi/2,0,0,[0:0.1:1])
%q = [pi/6 pi/2]
l1 = 2;
l2 = 2;
pe = zeros(2,1);
pe(1) = l1*cos(q(1)) + l2*cos(q(1)+q(2));
pe(2) = l1*sin(q(1)) + l2*sin(q(1)+q(2));


end

