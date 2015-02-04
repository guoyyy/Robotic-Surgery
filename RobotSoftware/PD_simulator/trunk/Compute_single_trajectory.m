function  [time,TotalTime,SABiRx,Rneedle] = Compute_single_trajectory(phome,rhome,pausetime,ptarget,rz,insertdist,ts,maxAcc)

ControlReadyTime = 0;

%% Generate Reference Signal
maxahome=maxAcc(1);
maxa=maxAcc(2);
maxwdot=maxAcc(3);

pend=ptarget + insertdist*rz'; %endpoint
% phome2=+phome-1*rhome'; %Homing path

%Home position to insertion starting position in two steps
%[time1,x1,R1]=PosRef3D(phome,phome2-phome,maxahome,rhome,rhome,ControlReadyTime,ts);
%[time2,x2,R2]=PosRef3D(phome2,pend-phome2,maxahome,rhome,rz,time1(end)+ts,ts);


%go from phome along the vector rhome, going a distance pend-phome and
%ending at a vector rz
[time1,x1,R1]=PosRef3D_smoothangle(phome,ptarget-phome,maxahome,maxwdot,rhome,rz,ControlReadyTime,ts);

%pause needle
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

%Insert needle
if length(time1) > 0
    nexttime = time2(end)+ts;
else
    nexttime = 0;
end
[time3,x3,R3]=PosRef3D_smoothangle(ptarget,pend-ptarget,maxa,maxwdot,rz,rz,nexttime,ts);
nexttime = time3(end)+ts;

%pause needle
if pausetime <=0
    time4=[];
    x4=[];
    R4=[];
else
    pt = pausetime * 1/ts;
    time4 = linspace(time3(end),time3(end)+pausetime,pt);
    x4 = repmat(x3(:,end),1,pt);
    R4 = repmat(R3(:,end),1,pt);
    nexttime = time4(end)+ts;
end

%retract needle

[time5,x5,R5]=PosRef3D_smoothangle(pend,ptarget-pend,maxa,maxwdot,rz,rz,nexttime,ts);

% plot3(x3(2,:),x3(3,:),x3(1,:),'r');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%Alternate Hold                                                   %
% [time4,x4,R4]=PosHold3D(pend,rz,time3(end)+ts,10,ts);               %
% [time5,x5,R5]=PosHold3D(pend,rz,time4(end)+ts,10,ts);               %
% [time6,x6,R6]=PosHold3D(pend,rz,time5(end)+ts,10,ts);               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Move back to home position in one steps
if length(time5) > 0
    nexttime = time5(end)+ts;
    turnaroundtime = time5(1);
else
    turnaroundtime = nexttime;
end
[time6,x6,R6]=PosRef3D_smoothangle(ptarget,phome-ptarget,maxahome,maxwdot,rz,rhome,nexttime,ts);


time = [time1,time2,time3,time4,time5,time6];
SABiRx=[x1,x2,x3,x4,x5,x6];
Rneedle=[R1,R2,R3,R4,R5,R6];
% time = [time1,time2];
% SABiRx=[x1,x2];
% Rneedle=[R1,R2];



% %%
turnaroundts = find(time==turnaroundtime);

TrackingTime=length(time)*ts;          % Also equal to = time5(end)-ControlReadyTime+ts
ParkMotionTime=TrackingTime+ControlReadyTime;                % Time that tracking is finished and the system will go to park = 10 + ControlReadyTime
% %%                                                             % HeartPosition(end,1): Since the Control ready time is added to the data  in Heart data load                                                      
%  
% Reference1park=((3/2*pi-5/4*pi)-00.00*pi/180);   % Position reference (rad) [22.5 deg]
% Reference2park=((7/4*pi-3/2*pi)-14.76*pi/180);   % Position reference (rad) [00.0 deg]
% Reference4park=((3/2*pi-5/4*pi)+05.02*pi/180);   %(5/4*pi-BackStageHomeAngles(1));[-2.25 deg]
% Reference5park=((7/4*pi-3/2*pi)-20.03*pi/180);   %(BackStageHomeAngles(2)-7/4*pi); [-15.65 deg]
% 
% % Calcuate Necessary Reference Constants for Parking using ReferenceCalc Function
% [ref_sine_freq1p,ref_sine_phase1p,x_ref_gain1p]=ReferenceCalc(Reference1park,RefMaxAcceleration1,ParkMotionTime);
% [ref_sine_freq2p,ref_sine_phase2p,x_ref_gain2p]=ReferenceCalc(Reference2park,RefMaxAcceleration2,ParkMotionTime);
% [ref_sine_freq4p,ref_sine_phase4p,x_ref_gain4p]=ReferenceCalc(Reference4park,RefMaxAcceleration4,ParkMotionTime);
% [ref_sine_freq5p,ref_sine_phase5p,x_ref_gain5p]=ReferenceCalc(Reference5park,RefMaxAcceleration5,ParkMotionTime);
                                                             
%%
TotalTime=ParkMotionTime;    % Total time: Simultion stop time
TotalTime=floor((TotalTime/ts))*ts;                          % Need to floor it to make it stop at the sampling time


%     clf
%     hold on
%     plot3(phome(:,2),phome(:,3),phome(:,1),'ro');
%     plot3(ptarget(:,2),ptarget(:,3),ptarget(:,1),'rx');
%     plot3(pend(:,2),pend(:,3),pend(:,1),'ro');
%     plot3(pend(:,2),pend(:,3),pend(:,1),'rx');
%     plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'r');

maxlen = max([length(time) length(SABiRx) length(Rneedle)]);
if length(time) ~= maxlen || length(SABiRx) ~= maxlen || length(Rneedle) ~= maxlen
    error('Compute_single_trajectory: reference array lengths are not identical');
end

