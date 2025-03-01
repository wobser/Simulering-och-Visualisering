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
    T_prev_link = eye(3);
 end

 T_joint_rotation = transform_2d_func([0 0 ]', S.q(iL));
 T_joint_to_CoM = transform_2d_func(S.L(iL).CoM, 0);
 %T_prev_link 
 %T_joint_rotation 
 %T_joint_to_CoM
 T_CoM = T_prev_link * T_joint_rotation * T_joint_to_CoM;
 %T_CoM
 T_joint_to_next_link = transform_2d_func(S.L(iL).Length, 0); 
 T_next_link = T_prev_link * T_joint_rotation * T_joint_to_next_link;
    
 S.L(iL).p = T_CoM(1:2,3);  % Extract the position CoM
 S.L(iL).R = T_CoM(1:2, 1:2); % Extract the rotation matrix
 S.L(iL).transform_to_next_link = T_next_link;
 
 
 T_prev_link = T_next_link;
  
end
%%%EOF
