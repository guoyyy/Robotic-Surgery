
%This Tests the inverse and forward euler angles.

alpha = pi/3;
beta = 0; %pi/4;
gamma = 2.5*pi/6;

Rot = Rot_Z(alpha)*Rot_Y(beta)*Rot_Z(gamma);

[alphaS betaS gammaS] = InvRotZYZ(Rot);