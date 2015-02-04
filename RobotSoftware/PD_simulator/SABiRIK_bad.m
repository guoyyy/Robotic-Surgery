function [t1,t2,t3,t4,t5,instrumentVariable]=SABiRIK(NeedleTipPosition,NeedleTipDirection,xoffset,yoffset,zoffset,ln,T_BackFront_mount)
% SABiR Inverse Kinematics

% Modified syringe mechanism to match pb to pr on Nov/30/2009
branch_0=0;branch_1=0;branch_2=0;branch_3=0;
%%           
lf=100;
lfe=20.037;
zfgimbal=2.05;

lb=100;
zbgimbal=2.455;
lbe=65.532;    %mm  distance between end last joint of back stage and needle axis

% global yoffset
% global xoffset
% global zoffset
% global ln
% global T_BackFront_mount



% fix input sizes

if length(NeedleTipPosition) ~= 3
    error('SABiRIK: Invalid size of needle position vector')
end
if length(NeedleTipDirection) ~= 3
    error('SABiRIK: Invalid size of needle direction vector')
end

NeedleTipPosition = reshape(NeedleTipPosition,3,1);
NeedleTipDirection = reshape(NeedleTipDirection,3,1);


if isempty(xoffset) && isempty(yoffset) && isempty(zoffset) 
    NeedleTipPosition=NeedleTipPosition;branch_0=1;
else
    NeedleTipPosition=NeedleTipPosition-[xoffset yoffset zoffset]';branch_1=1;
end

xn=NeedleTipPosition(1);
yn=NeedleTipPosition(2);
zn=NeedleTipPosition(3);
P_BN=[xn;yn;zn];
Rz=NeedleTipDirection;

%% find  pb w.r.t {B}

pb=[0 0 Rz(1);0 0 Rz(2);0 0 Rz(3)]*[0, 0, -ln]'+P_BN;
xb=pb(1); yb=pb(2); zb=pb(3);


%% find pf w.r.t {F}
r11=T_BackFront_mount(1,1);
r12=T_BackFront_mount(1,2);
r13=T_BackFront_mount(1,3);
r21=T_BackFront_mount(2,1);
r22=T_BackFront_mount(2,2);
r23=T_BackFront_mount(2,3);
r31=T_BackFront_mount(3,1);
r32=T_BackFront_mount(3,2);
r33=T_BackFront_mount(3,3);
ax=T_BackFront_mount(1,4);
ay=T_BackFront_mount(2,4);
az=T_BackFront_mount(3,4);

xf=(r12*yb*zn-r12*yb*az-r12*yn*zb+r12*ay*zb+r12*yn*az-r12*ay*zn+xb*az*r22-xb*r32*ay+xn*zb*r22+xn*r32*ay-xb*zn*r22+xb*r32*yn+ax*zn*r22-ax*zb*r22+ax*r32*yb-ax*r32*yn-xn*r32*yb-xn*az*r22+r13*zfgimbal*zn*r22-r13*zfgimbal*zb*r22+r13*zfgimbal*r32*yb-r13*zfgimbal*r32*yn+r12*yn*r33*zfgimbal-r12*yb*r33*zfgimbal-xb*r32*r23*zfgimbal+xb*r33*zfgimbal*r22+xn*r32*r23*zfgimbal-xn*r33*zfgimbal*r22+r12*r23*zfgimbal*zb-r12*r23*zfgimbal*zn)/(r32*r21*xb+r31*xn*r22-r31*xb*r22-zb*r21*r12+r31*r12*yb-r32*r21*xn-r31*r12*yn-r11*zn*r22+r11*zb*r22+zn*r21*r12-r11*r32*yb+r11*r32*yn);
yf=-(-r21*xn*r33*zfgimbal+yb*r31*r13*zfgimbal+r23*zfgimbal*r31*xn-r21*r13*zfgimbal*zb-r23*zfgimbal*r31*xb+r21*xb*r33*zfgimbal-yn*r31*r13*zfgimbal+r21*r13*zfgimbal*zn+r11*yb*zn-r21*xb*zn+r21*ax*zn-r21*ax*zb-ay*r31*xb-yb*r31*xn-r11*yb*az+ay*r31*xn+r21*xn*zb-r11*yn*zb-r21*xn*az+r11*ay*zb+r11*yn*az-r11*ay*zn+yn*r31*xb-yn*r31*ax+r21*xb*az+yb*r31*ax+r11*r23*zfgimbal*zb-r11*r23*zfgimbal*zn+r11*yn*r33*zfgimbal-r11*yb*r33*zfgimbal)/(r32*r21*xb+r31*xn*r22-r31*xb*r22-zb*r21*r12+r31*r12*yb-r32*r21*xn-r31*r12*yn-r11*zn*r22+r11*zb*r22+zn*r21*r12-r11*r32*yb+r11*r32*yn);
% k=(r31*r12*yb+az*r21*r12+r11*r32*r23*zfgimbal-r11*r33*zfgimbal*r22-r32*r21*r13*zfgimbal-r11*az*r22-r31*r12*ay+r33*zfgimbal*r21*r12+r32*r21*xb-r31*xb*r22-zb*r21*r12+r11*r32*ay-r32*r21*ax+r31*ax*r22+r11*zb*r22+r31*r13*zfgimbal*r22-r11*r32*yb-r31*r12*r23*zfgimbal)/(r32*r21*xb+r31*xn*r22-r31*xb*r22-zb*r21*r12+r31*r12*yb-r32*r21*xn-r31*r12*yn-r11*zn*r22+r11*zb*r22+zn*r21*r12-r11*r32*yb+r11*r32*yn);

%% pf w.r.t {B}
pf_back=T_BackFront_mount(1:3,1:3)*[xf;yf;zfgimbal]+T_BackFront_mount(1:3,4);

%% for back stage
% t3=-atan2(-zb,-yb);
% yee =  yb/cos(t3);
yee=-sqrt(yb*yb+zb*zb-zbgimbal*zbgimbal);
c3=(yb*yee+zb*zbgimbal)/(yee*yee+zbgimbal*zbgimbal);
s3=(-yb*zbgimbal+zb*yee)/(yee*yee+zbgimbal*zbgimbal);
t3=-atan2(s3,c3);
 
xee = xb;
kb = lbe/sqrt(2); 
mb = lb+kb;

ab = 2*xee*mb-2*yee*kb;  % sin(alpha)
bb = 2*xee*kb+2*yee*mb; % cos(alpha)
cb = (xee*xee + yee*yee +2*mb*kb)/sqrt(ab*ab + bb*bb);  % sin(alpha+t4)=c

% 
% if cb^2 > 1
%     disp('SABIRIK: unreachable configuration: cb^2>1')
%     t1='error1';
%     t2=0;t3=0;t4=0;t5=0;
%     return;
% end


t5 = atan2(cb,-sqrt(1 - cb*cb)) - atan2(ab,bb); %alpha=atan2(a,b)
if t5<0,
    t5=t5+2*pi;branch_2=1;
end

n1b =  xee*cos(t5)+yee*sin(t5)-lb-kb;
n2b = -xee*sin(t5)+yee*cos(t5)+kb;

t4 = atan2(n2b,n1b)+t5;

%% for front stage
kf = lfe/sqrt(2); 
mf = lf+kf;

af =  2*xf*mf-2*yf*kf;  % sin(alpha)
bf =  2*xf*kf+2*yf*mf; % cos(alpha)
cf = (xf*xf+yf*yf+2*mf*kf)/sqrt(af*af+bf*bf);  % sin(alpha+t2)=c

% if cf^2 > 1
%     disp('SABIRIK: unreachable configuration: cb^2>1')
%     t1='error2';
%     t2=0;t3=0;t4=0;t5=0;
%     return;
% end

t2 = atan2(cf,-sqrt(1-cf*cf)) - atan2(af,bf); %t2 %alpha=atan2(a,b)
if t2<0,
    t2=t2+2*pi;branch_3=1;
end

n1f =  xf*cos(t2)+yf*sin(t2)-lf-kf;
n2f = -xf*sin(t2)+yf*cos(t2)+kf;

t1 = atan2(n2f,n1f)+t2;
instrumentVariable=[xb yb zb xf yf yee xee c3 s3 ab bb n1b n2b af bf n1f n2f branch_0 branch_1 branch_2 branch_3];%17 instrumented variables




