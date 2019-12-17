

global masses;
mass1 = 10^13;
mass2 = 10^7;
masses = [mass1, mass2];

[times, out] = ode45('gravity',[0 : 0.02 : 10],[0 0 0 20 0 0 0 0 0 0 2 0], odeset('RelTol',1e-9));


%plot(times,out(:,1),'-o',times,out(:,2),'-o')
for i=1:length(times)
    %angular momentum[ p = m*v (mass times velocity)
    %                  v = hastighet för lilla massan minus hastigheter för
    %                  stora massan
    %                  r = vector expressed in a frame {A}
    % angmom = cross product of r and p, angmom(l) = r x p
    %If a particle has a position r (in frame {A}) and has momentum p, then
    %the angular momentum of this particle about the origin of {A} is
    %? = r × p
    
    
    b = plot(out(i,1),out(i,2),'om');
    set(b,'Visible','On');
    hold on
    c = plot(out(i,4),out(i,5),'om');
    hold on
    set(c,'Visible','On');
    drawnow
    axis([-5 25 -10 10]);
    set(b,'Visible','Off');
    set(c,'Visible','Off');
end