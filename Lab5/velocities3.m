clear all
close all

% Here we use the standard to link arm in 2D.

% Create some input joint angles, velocities and accelerations.
[q1 q1dot q1dotdot] = poly3(0, 1, 0, 0, 0:0.03:10);
[q2 q2dot q2dotdot] = poly3(0, 1, 0.7, 0, 0:0.03:10);
 
% Some constants...
l1 = 1;
l2 = 0.5;
 
% Loop over the values
for (i = 1:size(q1))
   
    
    p2(1) = l1*cos(q1(i));
    p2(2) = l1*sin(q1(i));
    
    pe(1) = l1*cos(q1(i)) + l2*cos(q1(i) + q2(i));
    pe(2) = l1*sin(q1(i)) + l2*sin(q1(i) + q2(i));
    
    % Compute the Jacobian
    Jpe = [ (-l1*sin(q1(i)) - l2*sin(q1(i)+q2(i)))  -l2*sin(q1(i)+q2(i));
             (l1*cos(q1(i)) + l2*cos(q1(i)+q2(i)))   l2*cos(q1(i)+q2(i))];
    
            
    % Compute the Jacobian for p2
    Jp2 = [-l1*sin(q1(i));
            l1*cos(q1(i))];
        
    % Speed of pe
    pedot = Jpe * [q1dot(i) q2dot(i)]';
        
    % Speed of p2
    p2dot = Jp2*q1dot(i);
    
    
        
   % Plot
   clf; 
   % The joint poses / pe.
   plot(pe(1), pe(2), 'ro');
   hold on
   plot(p2(1), p2(2), 'bo');
   
   % The speed of Ap -> plot this as a line where the start of the line is
   % in Ap end is indicating the speed.
   
   % Add it to a plot vector for x and y
   x_plot = [pe(1) pe(1)+pedot(1)];
   y_plot = [pe(2) pe(2)+pedot(2)];
   
   plot(x_plot, y_plot, 'r');
   
   x_plot = [p2(1) p2(1)+p2dot(1)];
   y_plot = [p2(2) p2(2)+p2dot(2)];
   
   plot(x_plot, y_plot, 'b');
   
   % Plot lines between the points 
   x_plot = [0 p2(1) pe(1)];
   y_plot = [0 p2(2) pe(2)];
   
   plot(x_plot, y_plot, 'g')
   
   axis([-0.1 2.0 -0.1 2.0], 'square')
   
   disp([q1dot(i) q2dot(i)]);
   pause(0.1);
   
    
end

