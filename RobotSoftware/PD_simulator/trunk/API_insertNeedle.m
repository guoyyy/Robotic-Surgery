function [time,SABiRx,Rneedle,TotalTime]=API_insertNeedle(pcurrent,rz,insertdist,time,maxa,maxwdot,pausetime,ts)
warning off %#ok<WNOFF>
rz=rz';

%% if we must use globals, they should all be set up in one place
%setupGlobals
%setupGoalsAndPara();
%% debug level - for now: 0 off; 1 on
%maxwdot = 1;
ControlReadyTime = 0;
if ~isempty(time)
    nexttime = time(end)+ts;
else
    nexttime = 0;
end
%% Generate Reference Signal
%maxa=speed;
%[time1,SABiRx,Rneedle]=PosRef3D(phome,pend-phome,maxahome,rhome,rtarget,ControlReadyTime,ts);
pend=pcurrent+insertdist*rz';
[time1,x1,R1]=PosRef3D_smoothangle(pcurrent,pend-pcurrent,maxa,maxwdot,rz,rz,nexttime,ts);

if pausetime <=0
    time2=[];
    x2=[];
    R2=[];
else
    pt = pausetime * 1/ts;
    time2 = linspace(time1(end),time1(end)+pausetime,pt);
    x2 = repmat(x1(:,end),1,pt);
    R2 = repmat(R1(:,end),1,pt);
   % nexttime = time4(end)+ts;
end
time=[time1 time2];
SABiRx=[x1 x2];
Rneedle=[R1 R2];

TrackingTime=length(time)*ts;          % Also equal to = time5(end)-ControlReadyTime+ts
ParkMotionTime=TrackingTime+ControlReadyTime;
TotalTime=ParkMotionTime;

fprintf('total trajectory time  is %.2fs\n', TotalTime);