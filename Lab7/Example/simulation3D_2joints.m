%clear all
%close all

% Define a system
S = System3D_2joints;

% Define the start state of the system
S.q(1) = 0.0;
S.q(2) = 0.0;
S.dq(1) = 0.0;
S.dq(2) = 0.0;

delta_t = 0.01; % Time increment

figure

Gravity = [0 0 -9.82]';

ue = zeros(6*S.n, 1);
for i = 1:S.n
    ue(6*i-5:6*i-3) = Gravity*S.L(i).m;
end
  
    
for t = [0:delta_t:10]
    
   
    S = calc_pos3D(S); % Update all poses etc. based on the current state of the system (always do this first).
    
    
    M = calc_M3D(S); % This could typically be placed outside the loop as well, however, incase of more complex rotations the inertia matrix needs to be updated.
    
    Jc = calc_Jc3D(S);
    un = zeros(S.n*6,1); % We'll only do all rotation along one axis... this will anyway be zero.
    
    % Compute the inertia matrix of the system
    H = Jc' * M * Jc;
    
    % Compute link accelerations
    S = forward_recursion3D(S);
    xsi_dot0 = calc_link_accelerations3D(S);
    
    uc = M*xsi_dot0 + un;
    c = Jc'*uc;
    te = Jc'*ue;
    
    % Use tau to add some friction?
    tau = -S.dq * 0.;
    
    S.dqq = pinv(H)*(te + tau - c);
    
    % We got the acceleration - integrate it to get the new dq and q(!)
    S.dq = S.dq + S.dqq * delta_t;
    S.q = S.q + S.dq * delta_t;
    
    S.q(1)
    
    Draw_System3D(S);
    
    axis([-2 2 -2 2 -2 2])
    view([45 45]);

    xlabel('\bf X axis');
    ylabel('\bf Y axis');
    zlabel('\bf Z axis');

    pause(delta_t);
end

