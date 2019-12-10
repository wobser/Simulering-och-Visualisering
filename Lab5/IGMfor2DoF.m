
q1 = poly3(0,pi/4,0,0,0:0.1:1);
q2 = poly3(0,2*pi,0,0,0:0.1:1);

n = size(q1);
for i=1:n
    q = [q1(i);q2(i)];
    [p2,pe,R01,R0e]=fgm_T2(q); % evaluate the FGM
    cla;hold on;
    % ----------------------------------
    % put your display function here ...
    % set the plot axes ...

    plot(p2(1),p2(2),'bo')
    plot([0 p2(1)],[0 p2(2)],'b')
    
    plot(pe(1),pe(2),'mo')
    plot([p2(1) pe(1)],[p2(2) pe(2)],'m')
    axis([-1.0 3.0 -1.0 3.0], 'square');
    % ----------------------------------
    drawnow
end

