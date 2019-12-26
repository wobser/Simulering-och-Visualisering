function p = calc_joint_pos2D(S)
%
% Calculates the position of all joints
%
% Note: This function requires updated positions SV = calc_pos_CoM2D(SP,SV);
%
% Syntax:
% -------
% p=calc_joint_pos2D(S);
%
% Input:
% ------
% S    - system structure
%
% Output:
% -------
% p  [3xS.n]   - positions of the joints
%

p = zeros(3,S.n);

% First joint always at 0,0
p(:,1) = [0 0 0]';

for i = 1:S.n-1

    
    % Joint 1 is connected with link 0 (the world) and link 1 (the first
    % moving link.
    p(:,i+1) = S.L(i).transform_to_next_link(1:3, 4);
  
end

%%%EOF