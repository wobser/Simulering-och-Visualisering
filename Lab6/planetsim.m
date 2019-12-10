

global masses;
mass1 = 10^13;
mass2 = 10^7;
masses = [mass1, mass2];

[times, out] = ode45('gravity',[0 : 0.02 : 10],[0 0 0 20 0 0 0 0 0 0 2 0], odeset('RelTol',1e-9));


%plot(times,out(:,1),'-o',times,out(:,2),'-o')
for i=1:length(times)
    
    x1 = out(:,1);
    y1 = out(:,2);
    b = plot(x1(i),y1(i),'om');
    set(b,'Visible','On');
    hold on
    x2 = out(:,4);
    y2 = out(:,5);
    c = plot(x2(i),y2(i),'om');
    hold on
    
    set(c,'Visible','On');
    drawnow
    axis([-5 25 -10 10]);
    set(b,'Visible','Off');
    set(c,'Visible','Off');
end