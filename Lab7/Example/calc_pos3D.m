function S = calc_pos2D(S)
% calc_pos2D
%
% Updates S.L(1:S.n).R, S.L(1:S.n).p and S.L(1:S.n).transform_to_next_link
%
% Syntax:
% -------
% S=calc_pos2D(S);
%
% Input:
% ------
% S    - system structure
%
% Output:
% -------
% S
%
% SV members changed:
% --------------------
% S.L(1:S.n).R
% S.L(1:S.n).p
%


for iL = 1:S.n % update all links positions (CoM) and rotation matrices

   
 % For the first link we only have to use joint 1 to compute the rotation matrix
 if (iL == 1)
    T_prev_link = eye(4);
 end

 T_joint_rotation = transform_3d_func([0 0 0]', S.q(iL), S.J(iL).axis);
 T_joint_to_CoM = transform_3d_func(S.L(iL).CoM, 0, S.J(iL).axis);
 T_CoM = T_prev_link * T_joint_rotation * T_joint_to_CoM;
 
 T_joint_to_next_link = transform_3d_func(S.L(iL).Length, 0, S.J(iL).axis);
 
 % Additional rotations are encoded into Roll,Pitch and Yaw angles. 
 % Add this final rotation matrix in the end.
 R = rx(S.L(iL).RPY(1))*ry(S.L(iL).RPY(2))*rz(S.L(iL).RPY(3));
 T_rotation = eye(4);
 T_rotation(1:3, 1:3) = R;
  
 T_next_link = T_prev_link * T_joint_rotation * T_joint_to_next_link * T_rotation;
    
 S.L(iL).p = T_CoM(1:3,4);  % Extract the position
 S.L(iL).R = T_CoM(1:3, 1:3); % Extract the rotation matrix
 
 
  % Find the k value - that is the rotational vector in the world frame
  % Which axis did we rotate around?
  if (S.J(iL).axis == 'x') 
     % This corresponds to the *first* column in the rotation matrix (hint - check what happens if you take R*[1 0 0]') 
     S.L(iL).k = S.L(iL).R(:,1);
  elseif (S.J(iL).axis == 'y')
     % This corresponds to the *second* column in the rotation matrix (hint - check what happens if you take R*[0 1 0]') 
     S.L(iL).k = S.L(iL).R(:,2);
  else
     % This corresponds to the *third* column in the rotation matrix (hint - check what happens if you take R*[0 0 1]')
     S.L(iL).k = S.L(iL).R(:,3);
  end
  

 
 S.L(iL).transform_to_next_link = T_next_link;
 
 
 T_prev_link = T_next_link;
  
end
%%%EOF
