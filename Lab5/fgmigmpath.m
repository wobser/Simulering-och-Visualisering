
%path_EE;
clear;clc;cla

hold on
axis([-1 1 -1 1]);grid on

[x,y] = ginput; % press RETURN to stop
z = zeros(length(x),1);
P = [x';y';z'];

pos = trajC(P(1,:),P(2,:),P(3,:),1,0.01);
a = plot(pos(1,:),pos(2,:),'b');

set(a, 'Visible','On');
for i=1:length(pos)
    
    q = igm2dof(pos(1:2,i),1,1);
    [p2,pe,R01,R0e]=fgm_T2(q); % evaluate the FGM
    
    
    % ----------------------------------
    % put your display function here ...
    % set the plot axes ...
    %plot(p2(1),p2(2),'bo')
    %plot([0 p2(1)],[0 p2(2)],'b')
    
    peb = plot(pe(1),pe(2),'mo');
    b = plot([0 p2(1) pe(1)],[0 p2(2) pe(2)],'-m');
    set(b,'Visible','On');
    set(peb,'Visible','On');
    axis([-1 1 -1 1]);
    
    drawnow
    
    set(b,'Visible','Off');
    set(peb,'Visible','Off');
    % ----------------------------------
    
end
