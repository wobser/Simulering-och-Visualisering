clear;clc;cla

hold on
axis([-1 1 -1 1]);grid on

[x,y] = ginput; % press RETURN to stop
z = zeros(length(x),1);
Pdes = [x';y';z'];

%pos = trajC(P(1,:),P(2,:),P(3,:),1,0.01);
a = plot(Pdes(1,:),Pdes(2,:),'bo');
set(a, 'Visible','On');
e = 0.005;
alpha = 0.1;
beta = 01;
q = igm2dof([-1 -1],1,1);
c = [0 0];
while true
    [p2,pe,R01,R0e]=fgm_T2(q); %solve FGM Pe(i) = f(q(i))
    
    c = [Pdes(1) Pdes(2)]' - pe;%compute c = Pdes - Pe(i)
    qdot = inv(t4jac(igm2dof(pe,1,1)))* alpha * c;
    %solve:
        %J(q(i))*qd = alphac
        %where alpha > 0 is a scaling factor
    q = q + beta * qdot;
    %update joint angles
        %q(i+1) = q(i) + beta*qdot
        %where beta > 0 is a scaling factor
    %plottidotti \/
    peb = plot(pe(1),pe(2),'mo');
    b = plot([0 p2(1) pe(1)],[0 p2(2) pe(2)],'-m');
    set(b,'Visible','On');
    set(peb,'Visible','On');
    axis([-1 1 -1 1]);
    %plottidotti /\
    
    drawnow
    pause(0.001)
    set(b,'Visible','Off');
    set(peb,'Visible','Off');
    
    if c < 0.001
        break
    end
    
end