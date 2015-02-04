%This function is to calculate needle grasp kinematics.
%The descriptions of gripper frame (G) and needle frame (N) are introduced in the
%figure of needle grasp.
%Unit: Angle(rad) & Translation(mm)

%Here, the shape of needle is a 1/2 circle.
%If the shape of needle is changed, g_GN_0 has to be defined again.

%Inputs:
%R_needle: the radius of needle
%alpha0: translation distance along X axis of gripper frame
%alpha1: rotation angle along Z axis of gripper frame
%alpha2: rotation angle along Y axis of gripper frame
%alpha3: rotation angle along X axis of gripper frame

%Output:
%Gmatrix: 4X4 matrix (GN)
%% This is the original function.
% 
% function Gmatrix = NeedleGraspKinematics(R_needle,alpha0,alpha1,alpha2,alpha3)
% 
% v0 = [-1 0 0]';
% w1 = [0 0 1]';
% w2 = [0 1 0]';
% w3 = [-1 0 0]';
% 
% q1 = [0 0 0]';
% q3 = [0 0 -R_needle]';
% 
% xi0 = [v0' 0 0 0]';
% xi1 = [-cross(w1,q1); w1];
% xi2 = [-cross(w2,q1); w2];
% xi3 = [-cross(w3,q3); w3];
% 
% exp_xi0 = screwmotion(xi0,alpha0);
% exp_xi1 = screwmotion(xi1,alpha1);
% exp_xi2 = screwmotion(xi2,alpha2);
% exp_xi3 = screwmotion(xi3,alpha3);
% 
% g_GN_0 = [Rot_y(90,'d')*Rot_x(-90,'d') [0 0 0]';[0 0 0 1]];
% Gmatrix = exp_xi0*exp_xi1*exp_xi2*exp_xi3*g_GN_0;



%% This is the second Version.
function [Gmatrix,Jac]= NeedleGraspKinematics(R_needle,alpha0,alpha1,alpha2,alpha3)

v0 = [-1 0 0]';
w1 = [0 0 1]';
w2 = [0 1 0]';
w3 = [-1 0 0]';

q1 = [0 0 0]';
q3 = [0 0 -R_needle]';

xi0 = [v0' 0 0 0]';
xi1 = [-cross(w1,q1); w1];
xi2 = [-cross(w2,q1); w2];
xi3 = [-cross(w3,q3); w3];

exp_xi0 = screwmotion(xi0,alpha0);
exp_xi1 = screwmotion(xi1,alpha1);
exp_xi2 = screwmotion(xi2,alpha2);
exp_xi3 = screwmotion(xi3,alpha3);

%the term g_GN_0 is defined by the following:
%the Needle is fundamentally rotated from the gripper frame.


g_GN_0 = [Rot_y(90,'d')*Rot_x(-90,'d') [0 0 0]';[0 0 0 1]];
Gmatrix = exp_xi0*exp_xi1*exp_xi2*exp_xi3*g_GN_0;

Jac = zeros(6,4);

Jac(:,1) = xi0;

Jac(:,2) = Adjoint(exp_xi0)*xi1;

Jac(:,3) = Adjoint(exp_xi0*exp_xi1)*xi2;

Jac(:,4) = Adjoint(exp_xi0*exp_xi1*exp_xi2)*xi3;


