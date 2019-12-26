function Jc = calc_Jc3D(S)
%
% calc_Jc
%
% Calculates an augmented Jacobian matrix of the CoM of each link 
%
% Note: This function requires updated positions S = calc_pos3D(S);
%
% Syntax:
% -------
% Jc = calc_Jc3D(S);
%
% Input:
% ------
% S    - system
%
% Output:
% -------
% Jc [(6*SP.n)x(SP.n)]  - Jacobian matrix.
%

Jc = zeros(6*S.n,S.n); % initialization of the output
pJ = calc_joint_pos3D(S);  % positions of joints

for j = 1:S.n % iterate over all links
  link_com = S.L(j).p;
  k = S.L(j).k;
  
  for i = 1:j % iterate over the joints that will have an influence on the link      
  
    joint_pos = pJ(:,i);
    if S.J(j).type == 'R' % Revolute joint
      Jc((j)*6-5:(j)*6-3,i) = cross( k , ( link_com - joint_pos ) );
      Jc((j)*6-2:(j)*6,i) = k;
    else % Prismatic joint
       % TODO
    end
  end
end

%%%EOF