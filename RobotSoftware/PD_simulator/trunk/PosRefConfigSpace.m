% function [time,t1,t2,t3,t4,t5]=PosRefConfigSpace(x0,xdis,maxa,R0,R1,moti,ts)



%debug vars to run the function as a script


% function [t,x,Rz]=PosRef3D(x0,xdis,maxa,R0,R1,moti,ts)
x0 =[-10.7505 -206.2838  330.8692];
xdis =[0.7505   -3.7162    7.1308];
maxa =100;
R0 =[-0.0327 0.0020 0.9995]';
R1 =[0 -0.3186 0.9929]';
moti = 0;
ts = 1.0000e-03;

%first, get starting and end points in configuration space
[t01,t02,t03,t04,t05]=SABiRIK(x0,R0,xoffset,yoffset,zoffset,ln,T_BackFront_mount);

x1=xdis + x0;

[t11,t12,t13,t14,t15]=SABiRIK(x1,R1,xoffset,yoffset,zoffset,ln,T_BackFront_mount);

%check that the starting and end points are valid

t0valid = CheckJointAngles([t01,t02,t03,t04,t05]);
t1valid = CheckJointAngles([t11,t12,t13,t14,t15]);
if ~t0valid || ~t1valid
    error('POsRefConfigSpace: invalid configuration of start or end point');
end


%% first generate position acel,vel,pos ref to follow a rectangle.
% clear all
% ts=1e-3;            % Sampling time
% maxa=1;           % max acceleration
% moti=2;             % motion time, add this much time before begining 
% xdis=[-10,0,0];     % distance to be traveled
% x0=[-100,-100,300];   % starting position
MaxDis=max(abs(xdis));
if MaxDis~=0
    % Calcuate Necessary Reference Constants for Homing using ReferenceCalc Function
    [freq,phase,xgain]=ReferenceCalc(MaxDis,maxa,moti);

    % 3 accelerations
%     tprd1=(moti+0*pi/freq):ts:(moti+1*pi/freq-ts)
%     tprd2=(moti+1*pi/freq):ts:(moti+2*pi/freq-ts)
%     tprd3=(moti+2*pi/freq):ts:(moti+3*pi/freq)

    tprd1=(moti+0*pi/freq):ts:(moti+1*pi/freq-ts);
    tprd2=(tprd1(end)+ts):ts:(moti+2*pi/freq-ts);
    tprd3=(tprd2(end)+ts):ts:(moti+3*pi/freq);
    
    
    t=[tprd1,tprd2,tprd3];   

    a1=(xgain*MaxDis)*sin(phase + freq*tprd1);
    a2=0*tprd2;
    a3=-(xgain*MaxDis)*sin(phase + freq*tprd3);

    vint=cumtrapz(t,cumtrapz(t,[a1,a2,a3]));

    x=[xdis(1)*vint/vint(end);...
       xdis(2)*vint/vint(end);...
       xdis(3)*vint/vint(end)]+...
       [x0(1).*ones(1,length(vint));...
        x0(2).*ones(1,length(vint));...
        x0(3).*ones(1,length(vint))];
    
%     
%     Rz=[R0(1).*ones(1,length(t));...
%         R0(2).*ones(1,length(t));...
%         R0(3).*ones(1,length(t))];
    Rz=zeros(3,length(vint));
    a=acos(dot(R0,R1))/length(vint);
    if norm(cross(R0,R1))~=0
        v = cross(R0,R1)/norm(cross(R0,R1));
        for iii=1:1:length(vint)
            hatmatrix = [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
            RotM = v*v' + cos(a*iii)*(eye(3) - v*v') + sin(a*iii)*hatmatrix;%hat(v);
            Rz(:,iii)=RotM*R0;
        end 
    else
        for iii=1:1:length(vint)
            Rz(:,iii)=R0;
        end 
    end
  
else
    disp('Use other function')
end
 
% figure(1)
% for ppp=1:3
%     subplot(3,1,ppp)
%     plot(t,x(ppp,:))
% %     hold on
% end

plot3(x(1,:),x(2,:),x(3,:))
grid on