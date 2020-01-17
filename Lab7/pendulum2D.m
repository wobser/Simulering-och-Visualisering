% Simulation of a pendulum using Newton-Euler formulation in 2D
% Input:
% q - joint angle
% dq - joint velocity
% ue - external forces (here we only use gravity)
% tau - external momentum (here we could add some momentum to simulate friction)
close all;
clear all;

tau = 0;

% Parameters
m = 1; % Mass of the link
I33 = 1; % Inertia for rotation around the z axis (note we're only doing the simulation in 2D(!)

% Forces and torques that act upon the links CoM. Here we only using
% gravitation (y axis) and no torque.
ue = [0; -9.82; 0];

% Form the mass matrix M
M = [m 0 0; 0 m 0; 0 0 I33]

% The Jc and un needs to be computed in the simluation loop as they will
% change depending on q and qdot
% Simulate for 10 seconds
q = 0; % Our initial angle
dq = 0; % Our initial velocity
dqq = 0; % The acceleration in the joint (this we will compute in the simulation)


% Link parameters
link_to_CoM = [0.5] % The CoM of the link is at 0.5 meter relative to the link origin
CoM_to_next_link = [0.1]; % The position of the next link (given from the CoM position) [this will create a link length of 0.6]

delta_t = 0.01; % Time increment

figure


for t = [0:delta_t:10]
    Jc = calc_Jc(q, link_to_CoM);
    un = calc_un();
    % Compute the inertia matrix of the system
    H = Jc' * M * Jc;
    
    % Compute link accelerations
    CoM_position = calc_CoM_pos(q, link_to_CoM);
    xsi_dot0 = calc_link_accelerations(CoM_position, dq);
    
    uc = M*xsi_dot0 + un;
    c = Jc'*uc;
    te = Jc'*ue;
    
    % Use tau to add some friction
    %tau = -dq * 0.1;
    
    dqq = pinv(H)*(te + tau - c);
    
    % We got the acceleration - integrate it to get the new dq and q(!)
    dq = dq + dqq * delta_t;
    q = q + dq * delta_t
    
    % Draw the system
    draw_system(q, link_to_CoM, CoM_to_next_link);
    pause(delta_t);
end

% Computes the jacobian for the CoM of the pendulum link
function Jc = calc_Jc(q, link_to_CoM)
    Jc = [-link_to_CoM * sin(q); % The longer link the faster any rotational changes will affect the pendulum.
          link_to_CoM * cos(q);
          1]; % The rotational velocity of the CoM link will be the same as the rotational velocity of the joint.
end

% Compute the position (x,y) of the CoM
function p = calc_CoM_pos(q, link_to_CoM)
    p = [link_to_CoM * cos(q);
         link_to_CoM * sin(q)]
end

% Compute the gyroscopic effect (no torque is generated)
% We need more than one joint and also where the other joint has
% a different rotational axis.
function un = calc_un()
    un = 0;
end

% Draw the system
function draw_system(q, link_to_CoM, CoM_to_next_link)
    hold off
    p2 = [link_to_CoM * cos(q);
          link_to_CoM * sin(q)];
    pe = p2 + [CoM_to_next_link * cos(q);
               CoM_to_next_link * sin(q)];
    
    plot([0 p2(1) pe(1)],[0 p2(2) pe(2)], '-');
    hold on
    plot(p2(1), p2(2), 'gx'); % CoM
    plot(pe(1), pe(2), 'ro'); % end of link

    axis equal;
    axis ([-1 1 -1 1]);
end


function xsi_dot0 = calc_link_accelerations(CoM_position, dq)
    % Here we only compute a single link acceleration for a revolute joint
    % This will be the link accelerations at the CoM
    % Note that ddq = 0 (the acceleration q is assumed to be 0)

    k = [0 0 1]'; % We're doing all rotations in the z axis.
    
    xsi_dot0 = [0 0 0]'; % Initialize the vector
    
    % CoM_position contains the x,y coordinate of the CoM (and the origin of the link is (0,0)
    % Following the equations (slide 9 in the multi_body_dynamics.pdf)   
    % Here we use 3D vectors containing x,y,z, roll, pitch, yaw velocities / accelerations
    w = k * dq;
    dw = [0 0 0]';
    
    d = [CoM_position; 0];
    v = cross(k, d);
    dv = cross(w, cross(k,d))*dq;

    % Fill in xsi_dot0 
    xsi_dot0(1:2) = dv(1:2);
    xsi_dot0(3) = dw(3);
end
