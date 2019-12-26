function T = transform_3d_func(t, th, axis)
% -------------------------------
% Compute a transformation matrix
% -------------------------------
%
% Input:
% ------
% t - 2d vector containing the translation
% th - scalar containing the rotation in radians
% the rotation axis used (z - rotation around the z-axis
%
% Output:
% -------
% T - transformation matrix in 2d



% Form a transformation matrix as:
% | r11 r12 r13 t(1) |
% | r21 r22 r23 t(2) |
% | r31 r32 r33 t(3) |
% | 0   0   1   1    |

% Create an empty matrix of size 3x3.
T = zeros(4,4);

% Add translation values
T(1,4) = t(1);
T(2,4) = t(2);
T(3,4) = t(3);

% Bottom right 1.
T(4,4) = 1;

% The rotation matrix, this will depend on the rotation axis.
if (axis == 'x')
    R = rx(th);
elseif (axis == 'y')
    R = ry(th);
else
    R = rz(th);
end 

% Copy the rotation matrix into T.
T(1:3,1:3) = R;
