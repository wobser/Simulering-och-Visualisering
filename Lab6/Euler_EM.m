function wd = Euler_EM(t,w,I1,I2,I3)

wd = zeros(3,1);
wd(1) = ((I2 - I3)/I1)*w(2)*w(3);
wd(2) = ((I3 - I1)/I2)*w(1)*w(3);
wd(3) = ((I1 - I2)/I3)*w(1)*w(2);

%%%EOF
