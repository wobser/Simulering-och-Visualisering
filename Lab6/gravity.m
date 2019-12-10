function [out] = gravity(time, input) 
    
% input:  1-3   -> position particle 1
%         4-6   -> position particle 2
%         7-9   -> velocity particle 1
%         10-12 -> velocity particle 2
% 
% output: 1-3   -> velocity particle 1
%         4-6   -> velocity particle 2
%         7-9   -> acceleration particle 1
%         10-12 -> acceleration particle 2
    
% In order to get access to the global variable masses you
% will need to add the following lines to the other .m file
% containg the call to ode45.
%
% global masses;
% mass1 = 10^13;
% mass2 = 10^7;
% masses = [mass1, mass2];
%
% An example of how to call this function using ode45:
% [times, out] = ode45('gravity',[0 : 0.02 : 10],[0 0 0 20 0 0 0 0 0 0 2 0], odeset('RelTol',1e-9));
%


    global masses;
    m1 = masses(1);
    m2 = masses(2);
    p1 = [input(1), input(2), input(3)];
    p2 = [input(4), input(5), input(6)];

    
    d = norm(p1-p2);
    G = 6.67384e-11;

    a1 = G*m2/(d^3)*(p2-p1);
    
    a2 = G*m1/(d^3)*(p1-p2);
    
    out = [input(7); input(8); input(9); 
           input(10); input(11); input(12); 
           a1(1); a1(2); a1(3); 
           a2(1); a2(2); a2(3)];
end
