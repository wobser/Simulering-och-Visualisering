clear all
close all

% Ap(t) = Ar(t) + ABR(t)*Bp(t)
% Apdot(t) = Ardot(t) + ABR(t) * Bpdot(t) + ABRdot(t) * Bp(t)


% Everything is computed rotated / translated in the xy plane.

Ar = [1 0 0]'; % Simply a constant
Bp = [0.5 0 0]'; % A constant
Ardot = [0 0 0]';
Bpdot = [0 0 0]';

Aw = [0 0 0.3]'; % Rotate around the z-axis (0.1 rad / s)

framerate = 10; % Numer of plots per second.

figure 

   

for  i = 1:100

    th = (Aw(3)  / framerate)*i;
  
    ABR = [cos(th) -sin(th) 0;
           sin(th) cos(th) 0;
           0 0 1];

   % Compute the derrivative of R
   Awtilde = skew(Aw);
   ABRdot = Awtilde * ABR;
     
       
   Ap = Ar + ABR*Bp;
   Apdot = Ardot + ABR* Bpdot + ABRdot * Bp;
   
   
   % Plot
   clf;
   % The joint poses / pe.
   plot(Ap(1), Ap(2), 'ro');
   hold on
   plot(Ar(1), Ar(2), 'bo');
   
   % The speed of Ap -> plot this as a line where the start of the line is
   % in Ap end is indicating the speed.
   
   % Add it to a plot vector for x and y
   x_plot = [Ap(1) Ap(1)+Apdot(1)];
   y_plot = [Ap(2) Ap(2)+Apdot(2)];
   
   plot(x_plot, y_plot, 'r');
   
   
   axis([-0.1 2.0 -0.1 2.0], 'square')
   pause(0.1);
   
   
end

