function [time,SABiRx,Rneedle,TotalTime]=API_moveNeedle(phome,rhome,ptarget,rtarget,maxahome,maxwdot,pausetime,ts)
warning off
rhome=rhome';
rtarget=rtarget';

%% if we must use globals, they should all be set up in one place
%setupGlobals
%setupGoalsAndPara();
%% debug level - for now: 0 off; 1 on
%maxwdot = 1;
ControlReadyTime = 0;

%% Generate Reference Signal
%maxa=100.00;%For experiments (c2.X use maxa=1 and maxahome=100)
%maxahome=speed;

%Home position to insertion starting position in one step
%[time1,x1,R1]=PosRef3D(phome,phome2-phome,maxahome,rhome,rhome,ControlReadyTime,ts);
%[time2,x2,TotalTime,R2]=PosRef3D(phome2,np0-phome2,maxahome,rhome,rz,time1(end)+ts,ts);
[time1,x1,R1]=PosRef3D_smoothangle(phome,ptarget-phome,maxahome,maxwdot,rhome,rtarget,ControlReadyTime,ts);
if pausetime <=0
    time2=[];
    x2=[];
    R2=[];
else
    pt = pausetime * 1/ts;
    time2 = linspace(time1(end),time1(end)+pausetime,pt);
    x2 = repmat(x1(:,end),1,pt);
    R2 = repmat(R1(:,end),1,pt);
    %nexttime = time2(end)+ts;
end
time=[time1 time2];
SABiRx=[x1 x2];
Rneedle=[R1 R2];
TrackingTime=length(time)*ts;          % Also equal to = time5(end)-ControlReadyTime+ts
ParkMotionTime=TrackingTime+ControlReadyTime;
TotalTime=ParkMotionTime;
% time1(end);
fprintf('total trajectory time  is %.2fs\n', TotalTime);