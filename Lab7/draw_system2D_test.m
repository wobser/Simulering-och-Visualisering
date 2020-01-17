clear all
close all

S = System2D;

S.q(1) = 0.0;
S.q(2) = 0.0;
S.dq(1) = 0.0;
S.dq(2) = 0.0;
delta_t = 0.01; % Time increment

%S.q(1) = pi/2.;
%S.q(2) = pi/2.;

figure
Gravity = [0 0 -9.82]';

ue = zeros(6*S.n, 1);
for i = 1:S.n
    ue(6*i-5:6*i-3) = Gravity*S.L(i).m;
end
M = calc_M2D(S)
for t = [0:delta_t:10]
    
    % Update the positions of the CoM in the world frame
    S = calc_pos2D(S);
     
    Jc = calc_Jc2D(S);
    un = zeros(S.n*6,1)
    H = Jc' * M * Jc;
    S = forward_recursion3D(S);
    Draw_System2D(S);
    axis([-3 3 -3 3])
    

end

