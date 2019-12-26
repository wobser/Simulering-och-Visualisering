function S = forward_recursion3D(S)
%
% forward_recorsion3D
%
% Updates S.L(1:S.n).w, S.L(1:S.n).v, S.L(1:S.n).dw and S.L(1:S.n).dv
%
% Note: This function requires updated positions S = calc_pos3D(S);
%
% It is assumed that:
% S.ddq = zeros(SP.n,1);
% S.L(1).dv = zeros(3,1);
% S.L(1).dw = zeros(3,1);
%
% Syntax:
% -------
% S=forward_recursion3D(S);
%


  w_prev = [0 0 0]'; % No rotational speed before the first joint (we'll asume that first joint is connected to something very sturdy)
  v_prev = [0 0 0]'; % No linear speed before the first joint
  dw_prev = [0 0 0]'; % No rotational accelerations either before the first joint
  dv_prev = [0 0 0]'; % No linear accelerations either before the first joint
  r_prev = [0 0 0]'; % Previous center of mass position (for the first iteration the w_prev will anyway be zero.
  
  
for iL = 1:S.n % iterate over all links

  k = S.L(iL).k; % joint axis of rotation/translation expressed in the "world" frame
  d = S.L(iL).R*S.L(iL).CoM; % vector from joint iL to CoM of link iL expressed in the "world"
                             % frame  (in case of a revolute joint)
  
  % Compute this once
  kXd = cross(k,d);
  
  if S.J(iL).type == 'R' % Revolute joint      
        
    S.L(iL).w = w_prev + k*S.dq(iL);
    S.L(iL).v = v_prev + cross(w_prev, (S.L(iL).p-r_prev)) + kXd*S.dq(iL);
    
    S.L(iL).dw = dw_prev + cross(w_prev,k)*S.dq(iL);
    S.L(iL).dv = dv_prev + cross(dw_prev, (S.L(iL).p-r_prev)) + cross(w_prev, S.L(iL).v-v_prev) + cross(S.L(iL).w, kXd)*S.dq(iL);
    
  else % Prismatic joint 
    % TODO
  end
  
  % Update all the 'revious' values.
  w_prev = S.L(iL).w;
  v_prev = S.L(iL).v;
  dw_prev = S.L(iL).dw;
  dv_prev = S.L(iL).dv;
  r_prev = S.L(iL).p;
  
end
