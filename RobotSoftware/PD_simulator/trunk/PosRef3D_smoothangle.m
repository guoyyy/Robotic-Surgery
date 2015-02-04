function [t,x,Rz]=PosRef3D_smoothangle(x0,xdis,maxa,maxwdot,R0,R1,moti,ts)
global PosRef3DInstrumentedVariables
%% first generate position acel,vel,pos ref to follow a rectangle.
% clear all
% ts=1e-3;            % Sampling time
% maxa=100;           % max acceleration
% maxwdot=0.1;        % max angular acceleration
% moti=0;             % motion time, add this much time before begining 
% xdis=[-10,0,0];     % distance to be traveled
% x0=[-100,-100,300];   % starting position
branch_0=0;branch_1=0;branch_2=0;branch_3=0;branch_4=0;
%linear displacements in the 3 directions
MaxDis=max(abs(xdis));
%angular rotation
MaxAngle=atan2(norm(cross(R0,R1)),dot(R0,R1));  % amount of rotation

% Calcuate Necessary Reference Constants for Homing using ReferenceCalc Function
[disp_freq,disp_phase,disp_xgain]=ReferenceCalc(MaxDis,maxa,moti);

% Calcuate Necessary Reference Constants for Homing using ReferenceCalc Function
[angle_freq,angle_phase,angle_xgain]=ReferenceCalc(MaxAngle,maxwdot,moti);


% we pick the slower of the position and angle and adjust the magnitude and
% phase of the other to match, so as to have the two motion complete
% simulatenously
freq=min(disp_freq,angle_freq);
disp_ratio=(1/freq) / (1/disp_freq);
angle_ratio=(1/freq) / (1/angle_freq);
disp_phase=-moti*freq;
angle_phase=-moti*freq;




%time vectors
tprd1=(moti+0*pi/freq):ts:(moti+1*pi/freq-ts);
tprd2=(tprd1(end)+ts):ts:(moti+2*pi/freq-ts);
tprd3=(tprd2(end)+ts):ts:(moti+3*pi/freq);

t=[tprd1,tprd2,tprd3];   

time_length=length(t);

%let's fill the variables with zeros
Rz=repmat(R1,1,time_length);
x=repmat(x0,1,time_length);

if MaxDis~=0
    % accelerations for the three phases (accelerate, const vel, decelerate)
    acc1=(disp_xgain*MaxDis)*sin(disp_phase + freq*tprd1);
    acc2=0*tprd2;
    acc3=-(disp_xgain*MaxDis)*sin(disp_phase + freq*tprd3);

    vint=cumtrapz(t,cumtrapz(t,[acc1,acc2,acc3])) / disp_ratio;

    x=[ xdis(1)*vint/vint(end);...
        xdis(2)*vint/vint(end);...
        xdis(3)*vint/vint(end)      ]+...
      [ x0(1).*ones(1,length(vint));...
        x0(2).*ones(1,length(vint));...
        x0(3).*ones(1,length(vint)) ];
    branch_0=1;
else
    % don't need to do anything
    x = [];
    t= [];
    Rz = [];branch_1=1;
end

if MaxAngle~=0

    %determine rotation axis
    w = cross(R0,R1)/norm(cross(R0,R1));
    hatmatrix = [0,-w(3),w(2);w(3),0,-w(1);-w(2),w(1),0];
        
    % angular accelerations
    wdot1=(angle_xgain*MaxAngle)*sin(angle_phase + freq*tprd1);
    wdot2=0*tprd2;
    wdot3=-(angle_xgain*MaxAngle)*sin(angle_phase + freq*tprd3);

    wint=cumtrapz(t,cumtrapz(t,[wdot1,wdot2,wdot3])) / angle_ratio;

    angle=0+MaxAngle*wint/wint(end);
        
    for iii=1:1:(length(wint)-1)
        RotM = eye(3) + sin(angle(iii))*hatmatrix + hatmatrix*hatmatrix*(1-cos(angle(iii)));
        Rz(:,iii)=RotM*R0;
    end 
    Rz(:,length(wint))=R1;
    branch_2=1;
elseif MaxDis ~= 0
    %if the xvector is not zero-length, create a vector of equal size and
    %fill it with the constant angle
    Rz = repmat(R0,1,length(x));branch_3=1;
end
%%collect instrumented variables
datalength=length(Rz);
instrumentedVariable=[MaxAngle disp_freq disp_xgain angle_freq angle_xgain freq disp_ratio disp_phase angle_phase datalength branch_0 branch_1 branch_2 branch_3];
[instrumentM instrumentN]=size(PosRef3DInstrumentedVariables);
if instrumentM>=4
    PosRef3DInstrumentedVariables=[];%branch_4=1;
end
PosRef3DInstrumentedVariables=[PosRef3DInstrumentedVariables; instrumentedVariable];
% figure(1)
% for ppp=1:3
%     subplot(3,1,ppp)
%     plot(t,x(ppp,:))
% %     hold on
% end
