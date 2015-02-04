function [time,SABiRx,Rneedle,TotalTime]=API_moveHome(ptarget,rtarget,phome,rhome,time,maxahome,maxwdot,ts)
warning off
rhome=rhome';
rtarget=rtarget';

%% if we must use globals, they should all be set up in one place
%setupGlobals
%setupGoalsAndPara();
%% debug level - for now: 0 off; 1 on

ControlReadyTime = 0;

%% Generate Reference Signal
%maxa=100.00;%For experiments (c2.X use maxa=1 and maxahome=100)
%maxahome=speed;
%maxwdot = 1;

%The turnaroundtime will be calculated at java client
% if length(time) > 0
%     nexttime = time(end)+ts;
%     turnaroundtime = time3(1);
% else
%     turnaroundtime = nexttime;
% end
if ~isempty(time)
    nexttime = time(end)+ts;
else
    nexttime=0;
end
%Home position to insertion starting position in one step
%[time1,x1,R1]=PosRef3D(phome,phome2-phome,maxahome,rhome,rhome,ControlReadyTime,ts);
%[time2,x2,TotalTime,R2]=PosRef3D(phome2,np0-phome2,maxahome,rhome,rz,time1(end)+ts,ts);
%[time1,SABiRx,Rneedle]=PosRef3D(phome,ptarget-phome,maxahome,rhome,rtarget,ControlReadyTime,ts);
[time1,x1,R1]=PosRef3D_smoothangle(ptarget,phome-ptarget,maxahome,maxwdot,rtarget,rhome,nexttime,ts);

    pausetime=1.0;
    pt = pausetime * 1/ts;
    time2 = linspace(time1(end),time1(end)+pausetime,pt);
    x2 = repmat(x1(:,end),1,pt);
    R2 = repmat(R1(:,end),1,pt);
    %nexttime = time2(end)+ts;

time=[time1 time2];
SABiRx=[x1 x2];
Rneedle=[R1 R2];

TrackingTime=length(time)*ts;          % Also equal to = time5(end)-ControlReadyTime+ts
ParkMotionTime=TrackingTime+ControlReadyTime;
TotalTime=ParkMotionTime;

fprintf('total trajectory time  is %.2fs\n', TotalTime);