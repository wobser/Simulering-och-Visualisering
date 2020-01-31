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

%forces that will work on the joints later
ue = zeros(6*S.n, 1);
for i = 1:S.n
    ue(6*i-5:6*i-3) = Gravity*S.L(i).m;
end
M = calc_M2D(S);
for t = [0:delta_t:30]
    
    % Update the positions of the CoM in the world frame
    S = calc_pos2D(S);
    
    %Calculates the jacobian
    Jc = calc_Jc2D(S);
    
    % un - Compute the gyroscopic effect (no torque is generated)
    % We need more than one joint and also where the other joint has
    % a different rotational axis.
    un = zeros(S.n*6,1);
    
    
    H = Jc' * M * Jc; %The inertia(tröghet) matrix 
    
    S = forward_recursion2D(S);
    xsi_dot0 = calc_link_accelerations2D(S); %to denote the Cartesian accelerations of the links computed with zero joint accelerations 
    
    %The sum of all velocity dependent inertial forces acting on the CoM of
    %the links
    uc = M*xsi_dot0 + un;
    
    
    c = Jc'*uc;
    
    %The generalized forces/torques acting on the generalized coordinates
    %as a result of ueS.q
    te = Jc'*ue; %a static relation i.e. needs to be recalculated
    
    tau = -S.dq * 0.; % friction in joints kinda
    
    S.dqq = pinv(H)*(te + tau - c); % Calculate the joint acceleration!!!! <-- 
    
    S.dq = S.dq + S.dqq * delta_t; % Joints velocities calculation
    
    S.q = S.q + S.dq * delta_t; % Calculate the new joint positions for next iteration
    cla;
    Draw_System2D(S);
    axis([-5 5 -5 5])
    xlabel('\bf X axis');
    ylabel('\bf Y axis');
    pause(delta_t);
    
    
end

