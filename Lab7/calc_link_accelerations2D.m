function xsi_dot0 = calc_link_accelerations2D(S)
%
% calc_link_accelerations3D
%
% Returnts the xsi_dot0 vector
% Assume that S = forward_recursions(S) has been called

xsi_dot0 = zeros(S.n,1); % Initialize the vector
   
for i=1:S.n
    % Fill in xsi_dot0 
    xsi_dot0(6*i-5:6*i-3,1) = S.L(i).dv;
    %A = "S.L(i).dv:";disp(A)
    %S.L(i).dv
    xsi_dot0(6*i-2:6*i,1) = S.L(i).dw;
   %A = "S.L(i).dw:";disp(A)
    %S.L(i).dw
end
