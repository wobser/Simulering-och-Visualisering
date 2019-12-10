function [q] = igm2dof(pe,l,up_down)
%IGM2DOF Summary of this function goes here
%   Detailed explanation goes here


c2 = (pe(1)^2+pe(2)^2 -2) / 2;

if up_down == 1
    s2 = -sqrt(1-c2^2);
else
    s2 = sqrt(1-c2^2);
end

alfa = atan2(pe(2),pe(1));

beta = atan2(s2,(1+c2));


q = [alfa - beta , atan2(s2,c2)];


end

