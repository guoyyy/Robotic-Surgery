function [NeedleTip,Rz,Front,Back,Jf,Jb]=SABiRFK_v2(angles)
%[NeedleTip,Front,Back]=SABiRFK([t1,t2,t3,t4,t5]) 
% SABiR Forward Kinematics
% Given angles in radians, returns end-effector and needle positions in mm.
% Usage: Needle_Tip_Position=SABiRFK([225,315,0,225,315].*pi./180)
%        [Needle_Tip_Position,R,Front,Back]=SABiRFK([225,315,0,225,315]*pi/180)
%
% Modified from Myun Joong Hwangs file by Ozkan Bebek

% Modified syringe mechanism to match pb to pr on Nov/30/2009
% Include angular offset (alphax, alphay) between syringe and needle on Dec/7/2009



%%
global ln1% length of syringe mechanism
global ln2% length of needle
global alphax
global alphay
global yoffset
global xoffset
global zoffset
global T_BackFront_mount

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
lbe=65.532;    %mm  distance between end last joint of back stage and needle axis
zbgimbal=2.455; %mm offset

%% Front and back stage end points
xb=lb1*cos(t4)+lb2*cos(t5)+lbe*cos(t5-pi/4);
yb=(lb1*sin(t4)+lb2*sin(t5)+lbe*sin(t5-pi/4))*cos(-t3)-zbgimbal*sin(-t3);
zb=(lb1*sin(t4)+lb2*sin(t5)+lbe*sin(t5-pi/4))*sin(-t3)+zbgimbal*cos(-t3);
Back=[xb,yb,zb];

xb1=lb1*cos(t4)+lb2*cos(t5);
yb1=(lb1*sin(t4)+lb2*sin(t5))*cos(-t3)-zbgimbal*sin(-t3);
zb1=(lb1*sin(t4)+lb2*sin(t5))*sin(-t3)+zbgimbal*cos(-t3);
pb1=[xb1 yb1 zb1];

pf_front=[lf1*cos(t1)+lf2*cos(t2)+lfe*cos(t2-pi/4);
          lf1*sin(t1)+lf2*sin(t2)+lfe*sin(t2-pi/4);
          zfgimbal;
          1];
pf_back=T_BackFront_mount*pf_front;
Front=[pf_back(1),pf_back(2),pf_back(3)]; %=[xf,yf,zf]

%% Angular offset
% alphax=1*pi/180;
% alphay=-1*pi/180;

R_BbBs=ypr2R(0,alphay,alphax,'rad');

%% find R_BB1, R_B1Bb, R_BbBs (joint angles of passive joints)

beta=acos(dot([0;0;ln1],R_BbBs*[0;0;1])/ln1);
k=Front-Back;
PfPb=sqrt(sum(k.^2));
PsPf=ln1*cos(pi-beta)+sqrt(ln1*ln1*cos(pi-beta)*cos(pi-beta)-ln1*ln1+PfPb*PfPb);

Pb1Pf=sqrt(sum((Front-pb1).^2));

Bb_Pf=R_BbBs*[0;0;PsPf]+[0;0;ln1];
x=Bb_Pf(1);y=Bb_Pf(2);z=Bb_Pf(3);
lb1f=Pb1Pf;

% gammax : joint angle of 1st passive joint(pitch of syringe)(corrected 1/4/2011)

if y==0,
    gammax=acos((x*x+y*y+z*z+lbe*lbe-lb1f*lb1f)/2/z/lbe);
else
    gammax=atan2((-z*(z*lbe^2+y^2*z-z*lb1f^2+x^2*z+z^3+(-x^4*y^2-y^6-2*y^4*z^2-y^2*z^4-2*y^4*x^2+2*y^4*lb1f^2-y^2*lbe^4+2*y^4*lbe^2-y^2*lb1f^4+2*z^2*lbe^2*y^2+2*y^2*z^2*lb1f^2-2*y^2*z^2*x^2+2*y^2*x^2*lb1f^2-2*y^2*lbe^2*x^2+2*y^2*lbe^2*lb1f^2)^(1/2))/(z^2+y^2)+lbe^2+x^2+y^2+z^2-lb1f^2)/lbe/y,(z*lbe^2+y^2*z-z*lb1f^2+x^2*z+z^3+(-x^4*y^2-y^6-2*y^4*z^2-y^2*z^4-2*y^4*x^2+2*y^4*lb1f^2-y^2*lbe^4+2*y^4*lbe^2-y^2*lb1f^4+2*z^2*lbe^2*y^2+2*y^2*z^2*lb1f^2-2*y^2*z^2*x^2+2*y^2*x^2*lb1f^2-2*y^2*lbe^2*x^2+2*y^2*lbe^2*lb1f^2)^(1/2))/(z^2+y^2)/lbe);
end

R_BB1=[-sin(t5-pi/4), -cos(t5-pi/4), 0; cos(t5-pi/4)*cos(-t3), -sin(t5-pi/4)*cos(-t3), -sin(-t3); cos(t5-pi/4)*sin(-t3), -sin(t5-pi/4)*sin(-t3), cos(-t3)];
R_B2Bb=Rx(gammax-pi/2);

a=R_B2Bb*Bb_Pf;
b=R_BB1'*(Front'-Back');
c=inv([a(1) a(3);a(3) -a(1)])*[b(1);b(3)];
gammay=atan2(c(2),c(1)); % gammay : joint angle of 2nd passive joint(yaw of syringe) (corrected 1/4/2011)
R_B1B2=Ry(gammay);

R_BBb=R_BB1*R_B1B2*R_B2Bb;
R_BBs=R_BBb*R_BbBs;


%% tip position of needle 
ps=R_BBb*[0;0;ln1]+Back';
pn=R_BBs*[0;0;ln2]+ps;

if isempty(xoffset) && isempty(yoffset) && isempty(zoffset) 
    NeedleTip=[pn(1),pn(2),pn(3)];
else
    NeedleTip=[pn(1),pn(2),pn(3)]+[xoffset yoffset zoffset];    
end

Rz=R_BBs(:,3);


%% Jacobian calculations

%calculate B_Bb_R, the rotation from the world frame to the back gimbal
%frame: B_Bb_R = B_Be_R * Be_B2_R * B2_Bb_R = B_Be_R * Rx(gammax-pi/2) * Ry(gammay)
%where gammax and gammay are the angles of the back gimbal

pf = Front';
pb = Back';
pfpb = norm(pf-pb);

R_BBe = [-sin(t5)              -cos(t5-pi/4)           0;
          cos(t5-pi/4)*cos(t3)  sin(t5-pi/4)*cos(t3)    sin(t3);
          cos(t5-pi/4)*cos(t3)  sin(t5-pi/4)*cos(t3)    cos(t3)];
      
R_BbBs = eye(3); %assume a perfect mounting of the needle, beta=180

pspf = ln1*cos(beta) + sqrt(ln2^2*cos(beta)^2-(ln1^2-pfpb^2));

% k=1; %just any integer, i choose 1
% Bb_xf=Bb_Pf(1);Bb_yf=Bb_Pf(2);Bb_zf=Bb_Pf(3);
% A = Bb_yf;
% B = Bb_zf;
% Bb_pf = Bb_Bs_R * [0 0 pspf]' + [0 0 ln1];




Rotx = [1 0 0;
      0 cos(gammax-pi/2) -sin(gammax-pi/2);
      0 sin(gammax-pi/2) cos(gammax-pi/2)];
R_B2Bb = Rotx;
  
Roty = [cos(gammay) 0  sin(gammay);
        0 1 0;
       -sin(gammay) 0 cos(gammay)];
R_BeB2 = Roty;

R_BBb = R_BBe * R_BeB2 * R_B2Bb;
R = R_BBb;
P = pb;
% g_BBb = [R P;0 0 0 1];
Phat = [0 -P(3) P(2);P(3) 0 -P(1);-P(2) P(1) 0];
% Adj_g_BBb = [R' -R*Phat;zeros(3) R'];

%calculate back-stage jacobian
RxTheta3 = [1 0 0;
            0 cos(t3) -sin(t3);
            0 sin(t3) cos(t3)];
RzTheta4 = [cos(t4) -sin(t4) 0;
            sin(t4) cos(t4) 0
            0 0 1];
RzTheta5 = [cos(t5) -sin(t5) 0;
            sin(t5) cos(t5) 0
            0 0 1];

W = -(cross(RxTheta3*[0 0 1]' , RxTheta3 * RzTheta4 * [lb1 0 0]'));

Jb = [-R'*Phat*[1 0 0]', -R'*W, R'*W - R'*Phat*RxTheta3*[0 0 1]'];
% Jb = [Jb;zeros(size(Jb))];

%calculate front-stage jacobian
R = R_BBs;
P = pf;
% g_BBb = [R P;0 0 0 1];
Phat = [0 -P(3) P(2);P(3) 0 -P(1);-P(2) P(1) 0];

RxTheta0 = eye(3);
% RyTheta2 = [cos(t1) 0 sin(t1);
%             0 1 0;
%             -sin(t1) 0 cos(t1)];
RzTheta1 = [cos(t1) -sin(t1) 0;
            sin(t1) cos(t1) 0
            0 0 1];

W = -(cross(RxTheta0*[0 0 1]' , RxTheta0 * RzTheta1 * [lf1 0 0]'));

Jf = [-R'*Phat*[1 0 0]', -R'*W, R'*W - R'*Phat*RxTheta0*[0 0 1]'];
% Jf = [Jf;zeros(size(Jf))];
% Jf = zeros(size(Jb));

% P = pf;
% % g_BBb = [R P;0 0 0 1];
% Phat = [0 -P(3) P(2);P(3) 0 -P(1);-P(2) P(1) 0];
% 
%