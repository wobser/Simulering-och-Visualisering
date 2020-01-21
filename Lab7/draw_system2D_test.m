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
Gravity = [0 -9.82 0]';

ue = zeros(6*S.n, 1);
for i = 1:S.n
    ue(6*i-5:6*i-3) = Gravity*S.L(i).m;
end
M = calc_M2D(S);
for t = [0:delta_t:30]
    
    % Update the positions of the CoM in the world frame
    S = calc_pos2D(S);
     
    Jc = calc_Jc2D(S);
    
    % un - Compute the gyroscopic effect (no torque is generated)
    % We need more than one joint and also where the other joint has
    % a different rotational axis.
    un = zeros(S.n*6,1);
    
    H = Jc' * M * Jc;
    
    S = forward_recursion2D(S);
    xsi_dot0 = calc_link_accelerations2D(S);
    
    uc = M*xsi_dot0 + un;
    c = Jc'*uc;
    te = Jc'*ue;
    tau = -S.dq * 0.;
    S.dqq = pinv(H)*(te + tau - c);
    S.dq = S.dq + S.dqq * delta_t;
    S.q = S.q + S.dq * delta_t;
    %S.q(1)
    cla;
    Draw_System2D(S);

    axis([-5 5 -5 5])
    
    xlabel('\bf X axis');
    ylabel('\bf Y axis');
    pause(delta_t);

end

