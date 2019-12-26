%
% example (Lab_4, see figure for Task 1)
%
% equations of motion of a 2 DoF planar manipulator with revolute joints
%
% l1  - length of link 1
% l2  - length of link 2
% lc1 - distance from joint 1 to the CoM of link 1
% lc2 - distance from joint 2 to the CoM of link 2
% m1  - mass of link 1
% m2  - mass of link 2
% I1  - inertia matrix of link 1 
% I2  - inertia matrix of link 2 
% q1  - angle of joint 1
% q2  - angle of joint 2
% g   - gravity
%
% the equations of motion have the following form
%
% H(q)*ddq + C(q,dq) = tau
% 
% the term C(q,dq) includes the gravity related terms G(q) as well (see below)
%

% definition of symbolic variables (and constants)
syms m1 m2 lc1 l1 lc2 l2 q1 q2 dq1 dq2 g
syms I1 I2   % inertia around the Z axis going through the CoM expressed in the local frame


% the manipulator inertia matrix
H = [lc1^2*m1+I1+m2*l1^2+2*m2*l1*lc2*cos(q2)+m2*lc2^2+I2,   m2*l1*lc2*cos(q2)+m2*lc2^2+I2;
                           m2*l1*lc2*cos(q2)+m2*lc2^2+I2,                     m2*lc2^2+I2];

% the nonlinear term 
C = [cos(q1+q2)*lc2*m2*g+cos(q1)*lc1*m1*g+cos(q1)*l1*m2*g-2*l1*m2*dq1*dq2*lc2*sin(q2)-l1*m2*dq2^2*lc2*sin(q2);
                                                                  cos(q1+q2)*lc2*m2*g+lc2*m2*dq1^2*l1*sin(q2)];


% the part of C due to gravity (we did not separate it in the lectures)
G = [cos(q1+q2)*lc2*m2*g+cos(q1)*l1*m2*g+cos(q1)*lc1*m1*g;
                                      cos(q1+q2)*lc2*m2*g];

% Jacobian of the CoM of link 1 w.r.t. the joint motion
Jcm1 = [ -sin(q1)*lc1,  0;
          cos(q1)*lc1,  0;
                    0,  0;
                    0,  0;
                    0,  0;
                    1,  0];

% Jacobian of the CoM of link 2 w.r.t. the joint motion
Jcm2 = [ -sin(q1)*l1-sin(q1+q2)*lc2,    -sin(q1+q2)*lc2;
          cos(q1)*l1+cos(q1+q2)*lc2,     cos(q1+q2)*lc2;
                                  0,                  0;
                                  0,                  0;
                                  0,                  0;
                                  1,                  1];

% Jacobian of the end-point of link 2 w.r.t. the joint motion
Je = [ -sin(q1)*l1-sin(q1+q2)*l2,       -sin(q1+q2)*l2;
        cos(q1)*l1+cos(q1+q2)*l2,        cos(q1+q2)*l2;
                               0,                    0;
                               0,                    0;
                               0,                    0;
                               1,                    1];


% position of CoM of links
CoM = [ cos(q1)*lc1, cos(q1)*l1+cos(q1+q2)*lc2;
        sin(q1)*lc1, sin(q1)*l1+sin(q1+q2)*lc2;
                  0,                         0];

% rotation matrices from the link-fixed frames to the world frame   
Rot = [ cos(q1),    -sin(q1),      0,    cos(q1+q2), -sin(q1+q2),     0;
        sin(q1),     cos(q1),      0,    sin(q1+q2),  cos(q1+q2),     0;
              0,           0,      1,             0,           0,     1];


% position of joints
pJ = [ 0, cos(q1)*l1;
       0, sin(q1)*l1;
       0,          0];


% position of end-point of link 2
pEE = [ cos(q1)*l1+cos(q1+q2)*l2;
        sin(q1)*l1+sin(q1+q2)*l2;
                               0];


% The mass matrices of each link
% Note that, Iw = rz(rand)*I*rz(rand)' -> Iw(3,3) = I(3,3)
% and the entries I(1,1) and I(2,2) are not important in the planar case
% thats why they are set equal to 1.
M1 = [ m1,  0,  0,  0,  0,  0;
       0,  m1,  0,  0,  0,  0;
       0,   0, m1,  0,  0,  0;
       0,   0,  0,  1,  0,  0;
       0,   0,  0,  0,  1,  0;
       0,   0,  0,  0,  0, I1];

M2 = [ m2,  0,  0,  0,  0,  0;
        0, m2,  0,  0,  0,  0;
        0,  0, m2,  0,  0,  0;
        0,  0,  0,  1,  0,  0;
        0,  0,  0,  0,  1,  0;
        0,  0,  0,  0,  0, I2];


% Verify the computation of the manipulator inertia matrix 
H1 = simplify(transpose(Jcm1)*M1*Jcm1 + transpose(Jcm2)*M2*Jcm2);
H - H1


% evaluate numerically 
g = 0; % g = -9.81
m1 = 5;
m2 = 10;
I1 = 3;
I2 = 4;
lc1 = 0.5;
l1 = 1;
lc2 = 0.5;
l2 = 1;
q1 = pi/4;
q2 = pi/3;
dq1 = 0.4;
dq2 = 0.5;

H_s = subs(H);
C_s = subs(C);
Jcm1_s = subs(Jcm1);
Jcm2_s = subs(Jcm2);
Je_s = subs(Je);

%%% EOF
