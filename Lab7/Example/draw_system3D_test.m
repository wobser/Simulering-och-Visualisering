clear all
close all

S = System3D;

S.q(1) = 0.;
S.q(2) = 0.;

figure


qq=zeros(2,1); stop=0; inc=0.04;
while ~stop
    clf
    % Update the positions of the CoM in the world frame
    S = calc_pos3D(S);
    
    Draw_System3D(S);
    axis([-3 3 -3 3 -3 3])
    view([45 45]);

    xlabel('\bf X axis');
    ylabel('\bf Y axis');
    zlabel('\bf Z axis');
    
    waitforbuttonpress;
    
    if strcmp(get(gcf,'currentcharacter'),'a');
        S.q(1) = S.q(1) + inc
    elseif strcmp(get(gcf,'currentcharacter'),'z');
        S.q(1) = S.q(1) - inc;
    elseif strcmp(get(gcf,'currentcharacter'),'s');
        S.q(2) = S.q(2) + inc;
    elseif strcmp(get(gcf,'currentcharacter'),'x');
        S.q(2) = S.q(2) - inc;
    elseif strcmp(get(gcf,'currentcharacter'),'q');
        stop=1; disp('quit');
    end

end
