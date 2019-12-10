%
%
%
close all
clear;clc

I1 = 1;
I2 = 2;
I3 = 3;

tf = 50;
dt = 0.02;

%w0 = [0.000,1,0.000];
%w0 = [0.05, 1, 0.05];
w0 = [0.0001,1,0.0001];


R{1} = eye(3);

[T,W] = ode45(@Euler_EM,[0:dt:tf],w0,[],I1,I2,I3);
W = W';

n = size(W,2);

I = [I1 0 0;
     0 I2 0;
     0 0 I3];

dw = (W(:,1) - w0')/dt;
 
for i=1:n
   norm_t(i) = norm(I*dw+cross(W(:,i),I*W(:,i)));
   normIdw(:,i) = norm(I*dw);
   w = R{i}*W(:,i);
   normIW(:,i) = norm(I*W(:,i));
   normWxIW(:,i) = norm(cross(W(:,i),I*W(:,i)));
   
   R{i+1} = aa2R(norm(w)*dt,w/norm(w))*R{i};
   Wi(:,i) = w;
   normW(:,i) = norm(W(:,i));
   normWi(:,i) = norm(Wi(:,i));
   if (i < n)
    dw = (W(:,i+1)-W(:,i))/dt;
   end
end

% Plot w in the body-fixed frame
figure(1);hold on;
plot(T,W(1,:),'b')
plot(T,W(2,:),'g')
plot(T,W(3,:),'r')
plot(T,normW,'k')
plot(T,normIW,'c')
plot(T,normWxIW,'g.')
plot(T,norm_t,'y')
plot(T,normIdw,'r.')
title('body-fixed frame')

% Plot w in the world frame
figure(2);hold on;
plot(T,Wi(1,:),'b')
plot(T,Wi(2,:),'g')
plot(T,Wi(3,:),'r')
plot(T,normWi,'k')
title('world frame')

figure(3);rotate_3D_box(R);

grid on
    
%%%EOF
