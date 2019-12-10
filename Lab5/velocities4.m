clear all
close all

% Here we use the standard to link arm in 2D.

% Instead of creating some joint angles, velocities.
% We compute a set of x and y coordinates.
[x xdot xdotdot] = poly3(0.3, 1.2, 0.3, 0.6, 0:0.03:10);
[y ydot tdotdot] = poly3(0.6, 0.7, 0.0, 0.0, 0:0.03:10);
%[y ydot ydotdot] = poly3(1, 1, 0, 0, 0:0.03:10);
 
% Some constants...
l1 = 1;
l2 = 0.5;
 
% Loop over the values
for (i = 1:size(x))
   
    
    % Given x and y solve the IGM
    c2 = (x(i)*x(i) + y(i)*y(i) - l1*l1 - l2*l2) / (2*l1*l2);
    if (abs(c2) > 1)
       disp('Out of reach');
       continue
    end
    s2 = +sqrt(1-c2*c2);
    
    q1 = atan2(y(i),x(i)) - atan2(l2*s2, l1 + l2*c2);
    q2 = atan2(s2, c2);
    
    p2 = [l1*cos(q1);
          l1*sin(q1)];
    
    pe = [l1*cos(q1) + l2*cos(q1 + q2);
          l1*sin(q1) + l2*sin(q1 + q2)];
    
    % Compute the Jacobian
    Jpe = [ (-l1*sin(q1) - l2*sin(q1+q2))  -l2*sin(q1+q2);
             (l1*cos(q1) + l2*cos(q1+q2))   l2*cos(q1+q2)];
    
            
    % Compute the Jacobian for p2
    Jp2 = [-l1*sin(q1);
            l1*cos(q1)];
        
    % Speed of q1, q2
    qdot = pinv(Jpe)*[xdot(i) ydot(i)]'
    
            
    % Plot
    clf; 
    % The joint poses / pe.
    plot(pe(1), pe(2), 'ro');
    hold on
    plot(p2(1), p2(2), 'bo');
    
   % The speed of Ap -> plot this as a line where the start of the line is
   % in Ap end is indicating the speed.
   
   % Add it to a plot vector for x and y
   x_plot = [pe(1) pe(1)+xdot(i)];
   y_plot = [pe(2) pe(2)+ydot(i)];
   
   plot(x_plot, y_plot, 'r');
   
   %x_plot = [p2(1) p2(1)+p2dot(1)];
   %y_plot = [p2(2) p2(2)+p2dot(2)];
   
   %plot(x_plot, y_plot, 'b');
   
   % Plot lines between the points 
   x_plot = [0 p2(1) pe(1)];
   y_plot = [0 p2(2) pe(2)];
   
   plot(x_plot, y_plot, 'g')

   plot(x,y,'k')
   
   axis([-0.1 2.0 -0.1 2.0], 'square')
   pause(0.02);
   
    
end

