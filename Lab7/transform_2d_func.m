function T = transform_2d_func(t, th)
% -------------------------------
% Compute a transformation matrix
% -------------------------------
%
% Input:
% ------
% t - 2d vector containing the translation
% th - scalar containing the rotation in radians
%
% Output:
% -------
% T - transformation matrix in 2d



% Form a transformation matrix as:
% | r11 r12 t(1) |
% | r21 r22 t(2) |
% | 0   0   1    |

% Create an empty matrix of size 3x3.
T = zeros(3,3);

% Add translation values
T(1,3) = t(1);
T(2,3) = t(2);

% Bottom right 1.
T(3,3) = 1;

% The rotation matrix.
R = [cos(th) -sin(th);
     sin(th) cos(th)];
 
% Copy the rotation matrix into T.
T(1:2,1:2) = R;
