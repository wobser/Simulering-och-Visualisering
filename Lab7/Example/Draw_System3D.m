function Draw_System3D(S)
% Draw_System3D
%
% Plots the manipulator system
%
% Syntax:
% -------
% Draw_System3D(S);
%
% Input:
% ------
% S          - structure containing the system
%
% Notes:
%
%  The 'R' joints are plotted like circles ('o')
%  The 'P' joints are plotted like diamonds ('d')
%

pJ = calc_joint_pos3D(S);

cla
hold on
line_w = 2;

for iL = 1:S.n % iterate over all joints / links
  
  % Plots the position of joint iJ
  if S.J(iL).type == 'R'
    J(iL) = plot3(pJ(1,iL),pJ(2,iL),pJ(3,iL),'ro','MarkerSize',10);
  else
    J(iL) = plot(pJ(1,iL),pJ(2,iL),pJ(3,iL),'rd','MarkerSize',7.5);
  end
  
  % Plots a vector from joint iL to the CoM of link iL
  plot3([pJ(1,iL) S.L(iL).p(1)], [pJ(2,iL) S.L(iL).p(2)],[pJ(3,iL) S.L(iL).p(3)], 'b','LineWidth',line_w);

  % Plots a vector from link iL CoM to the end of link iL.
  end_of_link = S.L(iL).transform_to_next_link(1:3,4);
  plot3([S.L(iL).p(1) end_of_link(1)], [S.L(iL).p(2) end_of_link(2)], [S.L(iL).p(3) end_of_link(3)],'g','LineWidth',line_w);

  
end

% Plot a star at every CoM for all links
for iL = 1:S.n % iterate over all links
    plot3(S.L(iL).p(1), S.L(iL).p(2), S.L(iL).p(3), 'rx');
end
%%%EOF