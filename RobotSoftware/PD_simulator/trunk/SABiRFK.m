function [NeedleTip,Rz,Front,Back]=SABiRFK(angles)
% SABiR Forward Kinematics
% Given angles in radians, returns end-effector and needle positions in mm.
% Usage: Needle_Tip_Position=SABiRFK([225,315,0,225,315].*pi./180)
%        [Needle_Tip_Position,R,Front,Back]=SABiRFK([225,315,0,225,315]*pi/180)
% Ozkan Bebek
% Modified syringe mechanism to match pb to pr on Nov/30/2009

global ln
global T_BackFront_mount
global yoffset
global xoffset
global zoffset
t1=angles(1);   %front
t2=angles(2);   %front
t3=angles(3);   %pitch
t4=angles(4);   %back
t5=angles(5);   %back

%% Forward Kinematics Paramenters
% front stage
lf1=100;       %mm link length
lf2=100;       %mm link length
lfe=20.037 ;   %mm  end effector distance
zfgimbal=2.05; %mm offset

% back stage
lb1=100;       %mm link length
lb2=100;       %mm link length
lbe=65.532;    %mm  distance between end last joint of back stage and needle axis (New Syringe Mechanism (December 4,2009))
zbgimbal=2.455; %mm offset

%% Front and back stage end points
xb=lb1*cos(t4)+lb2*cos(t5)+lbe*cos(t5-pi/4);
yb=(lb1*sin(t4)+lb2*sin(t5)+lbe*sin(t5-pi/4))*cos(-t3)-zbgimbal*sin(-t3);
zb=(lb1*sin(t4)+lb2*sin(t5)+lbe*sin(t5-pi/4))*sin(-t3)+zbgimbal*cos(-t3);
Back=[xb,yb,zb];

pf_front=[lf1*cos(t1)+lf2*cos(t2)+lfe*cos(t2-pi/4); lf1*sin(t1)+lf2*sin(t2)+lfe*sin(t2-pi/4);zfgimbal;1];
pf_back=T_BackFront_mount*pf_front;
xf=pf_back(1); yf=pf_back(2); zf=pf_back(3);

Front=[xf,yf,zf];
%% tip position of needle 
k=Front-Back;
t=ln/sqrt(sum(k.^2));
% ignore offsets if don't exist
if isempty(xoffset) && isempty(yoffset) && isempty(zoffset) 
    NeedleTip=t*k+Back;
else
    NeedleTip=t*k+Back+[xoffset yoffset zoffset];    
end
%% Needle tip orientation
% direction of needle
pfb=pf_back(1:3)-Back';
pfb=pfb./norm(pfb);
Rz=pfb;
