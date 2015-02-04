function [time1,SABiRx,Rneedle,TotalTime]=API_relativeMove(pcurrent,rcurrent,ptarget,rz,speed,time)
warning off
rcurrent=rcurrent';
rz=rz';
ts=0.001;
%% if we must use globals, they should all be set up in one place
%setupGlobals
%setupGoalsAndPara();
%% debug level - for now: 0 off; 1 on
maxwdot = 1;
ControlReadyTime = 0;

%% Generate Reference Signal
%maxa=100.00;%For experiments (c2.X use maxa=1 and maxahome=100)
if ~isempty(time)
    nexttime = time(end)+ts;
else
    nexttime = 0;
end
%% Generate Reference Signal
maxa=speed;
%Home position to insertion starting position in one step
%[time1,x1,R1]=PosRef3D(phome,phome2-phome,maxahome,rhome,rhome,ControlReadyTime,ts);
%[time2,x2,TotalTime,R2]=PosRef3D(phome2,np0-phome2,maxahome,rhome,rz,time1(end)+ts,ts);
[time1,SABiRx,Rneedle]=PosRef3D_smoothangle(pcurrent,ptarget-pcurrent,maxa/4,maxwdot,rcurrent,rz,nexttime,ts);
TrackingTime=length(time1)*ts;          % Also equal to = time5(end)-ControlReadyTime+ts
ParkMotionTime=TrackingTime+ControlReadyTime;
TotalTime=ParkMotionTime;
time1(end)
fprintf('total trajectory time  is %.2fs\n', TotalTime);

