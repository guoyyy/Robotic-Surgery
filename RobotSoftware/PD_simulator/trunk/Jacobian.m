function [Jf,Jb]=Jacobian(t1,t2,t3,t4,t5)
%[NeedleTip,Front,Back]=SABiRFK([t1,t2,t3,t4,t5]) 
% SABiR Forward Kinematics
% Given angles in radians, returns end-effector and needle positions in mm.
% Usage: Needle_Tip_Position=SABiRFK([225,315,0,225,315].*pi./180)
%        [Needle_Tip_Position,R,Front,Back]=SABiRFK([225,315,0,225,315]*pi/180)
%
% Modified from Myun Joong Hwangs file by Ozkan Bebek

% Modified syringe mechanism to match pb to pr on Nov/30/2009
% Include angular offset (alphax, alphay) between syringe and needle on Dec/7/2009

% Modified from SABiRFK calculate Jacobian for force/torque calculations

% correct for the biases

% t1bias= 225*pi/180;                      % Home position for axis 1
% t2bias= 315*pi/180;                      % Home position for axis 2
% t3bias= 0;                               % Home position for axis 3
% t4bias= 225*pi/180;                      % Home position for axis 4
% t5bias= 315*pi/180;                      % Home position for axis 5
% 
% t1 = t1 + t1bias;
% t2 = t2 + t2bias;
% t3 = t3 + t3bias;
% t4 = t4 + t4bias;
% t5 = t5 + t5bias;


alphax=0*pi/180;
alphay=0*pi/180;

R_BbBs=ypr2R(0,alphay,alphax,'rad');

yoffset=0;
xoffset=0;
zoffset=0;

ln= 68.6308+65.19+161.86+18.00+14.91;
ln1 = 123.142;
ln2=ln-ln1;
T_BackFront_mount = [ %July 22, 2010
    0.9996514523262   0.0239862420680  -0.0110287829636   -2.6398505898723
    -0.0238096717995   0.9995905085098   0.0158718249104  -45.1866546135221
    0.0114049722052  -0.0156037011200   0.9998132081146  197.5288079400060
    0                 0                 0                  1];


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
% R_BbBs=ypr2R(0,alphay,alphax,'rad');



%% find R_BB1, R_B1Bb, R_BbBs (joint angles of passive joints)


beta=acos(([0;0;ln1]' * R_BbBs*[0;0;1])/ln1);
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


% %% tip position of needle 
% ps=R_BBb*[0;0;ln1]+Back';
% pn=R_BBs*[0;0;ln2]+ps;
% 
% if isempty(xoffset) && isempty(yoffset) && isempty(zoffset) 
%     NeedleTip=[pn(1),pn(2),pn(3)];
% else
%     NeedleTip=[pn(1),pn(2),pn(3)]+[xoffset yoffset zoffset];    
% end
% 
% Rz=R_BBs(:,3);
% 

%% Jacobian calculations

%calculate B_Bb_R, the rotation from the world frame to the back gimbal
%frame: B_Bb_R = B_Be_R * Be_B2_R * B2_Bb_R = B_Be_R * Rx(gammax-pi/2) * Ry(gammay)
%where gammax and gammay are the angles of the back gimbal

pf = Front';
pbe = Back';
pfpbe = norm(pf-pbe);

R_BBe = [-sin(t5)              -cos(t5-pi/4)           0;
          cos(t5-pi/4)*cos(t3)  sin(t5-pi/4)*cos(t3)    sin(t3);
          cos(t5-pi/4)*cos(t3)  sin(t5-pi/4)*cos(t3)    cos(t3)];
      
% R_BbBs = eye(3); %assume a perfect mounting of the needle

% pspf = ln1*cos(beta) + sqrt(ln2^2*cos(beta)^2-(ln1^2-pfpbe^2));

% k=1; %just any integer, i choose 1
% Bb_xf=Bb_Pf(1);Bb_yf=Bb_Pf(2);Bb_zf=Bb_Pf(3);
% A = Bb_yf;
% B = Bb_zf;
% Bb_pf = Bb_Bs_R * [0 0 pspf]' + [0 0 ln1];




Rotx = [1 0 0;
      0 cos(gammax-pi/2) -sin(gammax-pi/2);
      0 sin(gammax-pi/2) cos(gammax-pi/2)];
R_B2Bb = Rotx;
  
Roty = [1 0 0;
      0 cos(gammay) -sin(gammay);
      0 sin(gammay) cos(gammay)];
R_BeB2 = Roty;

R_BBb = R_BBe * R_BeB2 * R_B2Bb;
R = R_BBb;
P = pbe;
% g_BBb = [R P;0 0 0 1];
Phat = [0 -P(3) P(2);P(3) 0 -P(1);-P(2) P(1) 0];
% Adj_g_BBb = [R' -R*Phat;zeros(3) R'];

%calculate back-stage jacobian
RxTheta1 = [1 0 0;
            0 cos(t3) -sin(t3);
            0 sin(t3) cos(t3)];
RyTheta2 = [cos(t4) 0 sin(t4);
            0 1 0;
            -sin(t4) 0 cos(t4)];
RzTheta3 = [cos(t5) -sin(t5) 0;
            sin(t5) cos(t5) 0
            0 0 1];

W = zeros(3,1);
u = RxTheta1*[0 0 1]';
v = RxTheta1 * (RzTheta3 * [lb1 0 0]');
W(1) = u(2)*v(3) - u(3)*v(2);
W(2) = u(3)*v(1) - u(1)*v(3);
W(3) = u(1)*v(2) - u(2)*v(1);
W = -W;
% W = -(cross(RxTheta1*[0 0 1]' , RxTheta1 * (RzTheta3 * [lb1 0 0]')));

Jb = [-R'*(Phat*[1 0 0]'), -R'*W, R'*W - R'*Phat*RxTheta1*[0 0 1]'];
% Jb = [Jb;zeros(size(Jb))];

%calculate front-stage jacobian
RxTheta1 = eye(3);
RyTheta2 = [cos(t1) 0 sin(t1);
            0 1 0;
            -sin(t1) 0 cos(t1)];
RzTheta3 = [cos(t2) -sin(t2) 0;
            sin(t2) cos(t2) 0
            0 0 1];

W = zeros(3,1);
u = RxTheta1*[0 0 1]';
v = RxTheta1 * (RzTheta3 * [lf1 0 0]');
W(1) = u(2)*v(3) - u(3)*v(2);
W(2) = u(3)*v(1) - u(1)*v(3);
W(3) = u(1)*v(2) - u(2)*v(1);
W = -W;
% W = -(cross(RxTheta1*[0 0 1]' , RxTheta1 * (RzTheta3 * [lf1 0 0]')));

Jf = [-R'*(Phat*[1 0 0]'), -R'*W, R'*W - R'*Phat*RxTheta1*[0 0 1]'];
% Jf = [Jf;zeros(size(Jf))];
