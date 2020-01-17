function M = calc_M2D(S)
%
% calc_M3D
%
% Calculate the large Mass/Inertia matrix M
% 
% Syntax:
% -------
% M = calc_M3D(S);
%
% Input:
% ------
% S    - system
%
% Output:
% -------
%
% M  [(6*SP.n)x(6*SP.n)] - Augmented Mass/Inertia matrix.
%

% memory allocation
M  = zeros(6*(S.n),6*(S.n));

for iL = 1:S.n
  ind = 6*iL-5:6*iL;  
  S.L;
  M(ind,ind) = [ eye(3)*S.L(iL).m zeros(3,3);
                zeros(3,3) S.L(iL).R*S.L(iL).I*S.L(iL).R' ];
end