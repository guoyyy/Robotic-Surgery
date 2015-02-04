% function [t,x,Rz]=PosRef3D(x0,xdis,maxa,maxwdot,R0,R1,moti,ts)
%% first generate position acel,vel,pos ref to follow a rectangle.
% clear all
% ts=1e-3;            % Sampling time
% maxa=100;           % max acceleration
% maxwdot=0.1;        % max angular acceleration
% moti=0;             % motion time, add this much time before begining 
% xdis=[-10,0,0];     % distance to be traveled
% x0=[-100,-100,300];   % starting position


x0 =[-10.7505 -206.2838  330.8692];


xdis =[0.7505   -3.7162    7.1308];


maxa = 100;


maxwdot =1;


R0 =[

   -0.0327
    0.0020
    0.9995];


R1 =[ 0
    0.1186
    0.9929];


moti = 0;


ts =   1.0000e-03;







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
    %a=acos(dot(R0,R1))/length(vint);
    a=atan2(norm(cross(R0,R1)),dot(R0,R1))/length(vint);  % amount of rotation
    if norm(cross(R0,R1))~=0

        %determine rotation axis
        v = cross(R0,R1)/norm(cross(R0,R1));
        hatmatrix = [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
        
        % Let's do a smooth angle interpolation
        % Calcuate Necessary Reference Constants for Homing using ReferenceCalc Function
        [freq,phase,xgain]=ReferenceCalc(a,maxwdot,moti);

        % 3 accelerations
        tprd1=(moti+0*pi/freq):ts:(moti+1*pi/freq-ts);
        tprd2=(tprd1(end)+ts):ts:(moti+2*pi/freq-ts);
        tprd3=(tprd2(end)+ts):ts:(moti+3*pi/freq);
    
        t=[tprd1,tprd2,tprd3];   

        wdot1=(xgain*a)*sin(phase + freq*tprd1);
        wdot2=0*tprd2;
        wdot3=-(xgain*a)*sin(phase + freq*tprd3);

        wint=cumtrapz(t,cumtrapz(t,[wdot1,wdot2,wdot3]));

        angle=0+a*wint/wint(end);
        
        for iii=1:1:(length(wint)-1)
            RotM = eye(3) + sin(angle(iii))*hatmatrix + hatmatrix*hatmatrix*(1-cos(angle(iii)));
            Rz(:,iii)=RotM*R0;
        end 
        Rz(:,length(wint))=R1;
        cenk=1;
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
