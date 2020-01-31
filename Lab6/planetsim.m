

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
    % angular momentum = cross product of r and p, angmom(l) = r x p
    %If a particle has a position r (in frame {A}) and has momentum p, then
    %the angular momentum of this particle about the origin of {A} is
    %? = r × p
    v1 = out(i,[7 8 9]);
    v2 = out(i, [10 11 12]);
    vel = v2 - v1; %Velocity
    p = mass2*vel;
    l = cross(out(i,[4 5 6]),vel);
    b = plot(out(i,1),out(i,2),'om');       %plotline
    set(b,'Visible','On');                  %plotline
    hold on
    c = plot(out(i,4),out(i,5),'om');       %plotline
    hold on
    set(c,'Visible','On');                  %plotline
    drawnow
    axis([-5 25 -10 10]);                   %plotline
    set(b,'Visible','Off');                 %plotline
    set(c,'Visible','Off');                 %plotline

end