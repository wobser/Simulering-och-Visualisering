function Jc = calc_Jc2D(S)

% Calculates an augmented Jacobian matrix of the CoM of each link 
%


Jc = zeros(6*S.n,S.n); % initialization of the output
pJ = calc_joint_pos2D(S);  % positions of joints

for j = 1:S.n % iterate over all links
  link_com = [S.L(j).p];
  link_com
  k = S.L(j).k;
  k
  
  for i = 1:j % iterate over the joints that will have an influence on the link      
  
    joint_pos = [pJ(:,i); 0];
    joint_pos
    %link_to_CoM = link_com - joint_pos;
    if S.J(j).type == 'R' % Revolute joint
      Jc((j)*6-5:(j)*6-3,i) = cross( k , ( link_com - joint_pos ) );
      Jc((j)*6-2:(j)*6,i) = k;
      %Jc((j)*6-5:(j)*6-3,i) = [-link_to_CoM' * sin(S.q); % The longer link the faster any rotational changes will affect the pendulum.
          %link_to_CoM' * cos(S.q);
          %1];
    else % Prismatic joint
       % TODO
    end
  end
end