function rotate_3D_box(R)
%
% draws the motion of a box specified by
%
% R - a Matlab "cell array" of rotation matrices
%
% For example
% R{1} = eye(3);
% R{2} = rz(pi/9);
% R{3} = rz(pi/8);
% ...
%

Vert = [0 0 0;
        1 0 0;
        1 1 0;
        0 1 0;
        0 0 1;
        1 0 1;
        1 1 1;
        0 1 1]-0.5;

Faces = [1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8;
         1 2 3 4;
         5 6 7 8];

Vert(:,3) = Vert(:,3)*3;  % increase the size along the z axis

ptch.Vertices = Vert;
ptch.Faces = Faces;
patch(ptch,'FaceColor','r');

axis([-1 1 -1 1 -1 1]*4)
axis vis3d
grid on

for i=1:length(R)
    p1 = (R{i}*Vert')';
    
    cla
    ptch.Vertices = p1;
    patch(ptch,'FaceColor','r');
    drawnow
    
end

%%%EOF