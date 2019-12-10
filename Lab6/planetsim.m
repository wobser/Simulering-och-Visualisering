

global masses;
mass1 = 10^13;
mass2 = 10^7;
masses = [mass1, mass2];

[times, out] = ode45('gravity',[0 : 0.02 : 10],[0 0 0 20 0 0 0 0 0 0 2 0], odeset('RelTol',1e-9));


%plot(times,out(:,1),'-o',times,out(:,2),'-o')
for i=1:length(times)
    
    plot([out(i,1) out(i,2)],'om');
    hold on
    axis([-3 3 -3 3]);
end