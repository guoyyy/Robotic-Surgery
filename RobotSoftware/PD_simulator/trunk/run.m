%%%%
% this is the top level script. Run this to start the simulator.
%%%%

clear variables
close all
% figure

%% set up constants in one place
%setpathtest();
setupConstants

%% encoder failure variables

%these are the variables to set in order to 

%decide which encoder is going to fail
%0 = no failure, n=(1-5)= encoder n fails
numEncoderToFail = 0;

%decide when it fails
%encoderFailureTime is a number between 0 and 1 that will later be
%multiplied with the total number of timesteps to get the timestep at which
%the encoder fails
encoderFailureTime = .5;

%% robot home position constants
phome =[-10.7505,-206.2838,330.8692]; %needle position at the robot's home position
rhome = [-0.0327;0.0020;0.9995];

%% debug level - for now: 0 off; 1 on
debuglevel=1;

%% set up trajectory

%to produce a sweep error, uncomment one of the following sets of ready
%points and target points.


% %sweep error #1
% ptarget = [-10 -241 341]; %the ready point
% pendpoint = ptarget + [0 .1 20];
% 
% %sweep error #2
% ptarget = [-12 -175 345]; %the ready point
% pendpoint = ptarget + [0 0 10];
% 
% %sweep error #3
% ptarget = [-22 -190 350]; %the ready point
% pendpoint = ptarget + [3 0 9];
% 
% %sweep error #4
% ptarget = [1 -190 345]; %the ready point
% pendpoint = ptarget + [0 0 5];
% 
% %sweep error #4
% ptarget = [1 -190 345]; %the ready point
% pendpoint = ptarget + [0 0 5];
% 
% %sweep error #5 - note that this one will probably be difficult to detect
% ptarget = [-8 -210 342]; %the ready point
% pendpoint = ptarget + [0 0 5];


% a correct trajectory -COMMENT THIS OUT TO PRODUCE ERRORS
pReadypoint = [-1.06547E1 -2.10849E2 3.382E2]; %the ready point
%pReadypoint= [-9.4233,-215.5671,338.8466];
pTargetpoint = pReadypoint +[9.63535E-2 5.21822E-2 8.47595E-1]*6.93472E0;
% 
% % a straight trajectory
% pReadypoint = [phome(1) phome(2) 338];
% pTargetpoint = pReadypoint + [0 0 6];



%% set up the rest of the trajectory variables
rInsertion = pTargetpoint-pReadypoint;
rInsertion = rInsertion';
% rtarget = [0 0 1]';
%rInsertion=[0.204473,-0.0571636,0.977202]';
rInsertion = rInsertion/norm(rInsertion);

% insertdist = 3;
insertdist = norm(pReadypoint-pTargetpoint);
%insertdist=9.05838;
pausetime = 1.0; %pause 1 second after reaching the target

%% set up simulation
[time1,TotalTime,SABiRx,Rneedle] = Compute_single_trajectory(phome,rhome,pausetime,pReadypoint,rInsertion,insertdist,ts,maxAcc);
%naxis=3;
%dtheta = 0.075;
%[t1start,t2start,t3start,t4start,t5start] = SABiRIK(phome,rhome,xoffset,yoffset,zoffset,ln,T_BackFront_mount);

%[time1,TotalTime,SABiRx,Rneedle] = Compute_single_axis_test_trajectory(t1start,t2start,t3start,t4start,t5start,naxis,dtheta,pausetime,ts);

%compute failure time
tsEncoderFailure = TotalTime * encoderFailureTime;

% limittime=1395;
% limittime=limittime+100;
% SABiRx = SABiRx(:,1:limittime);
% Rneedle = Rneedle(:,1:limittime);
% time1=time1(1:limittime);
%
% TotalTime=limittime*ts;
% TotalTime=floor((TotalTime/ts))*ts;
fprintf('total trajectory time  is %.2fs\n', TotalTime);


%% prepare simulink model
sys = 'SABiR_simulator'; 
open_system(sys);

%load a configuration set
load('cset1');
% Set a non-default stop time
%set_param (cset1, 'StopTime', '1')
set_param(cset1, 'StopTime', sprintf('%f',TotalTime));
set_param(cset1, 'SFSimEnableDebug','on');
% load a config reference
load('cref1')
% Resolve the config ref to the set
cref1.WsVarName = 'cset1';
% Attach the config ref to an untitled model
attachConfigSet(sys, cref1, true)
% Set the active configuration set to the reference
setActiveConfigSet(sys,'Reference')

% load the bus object for the simulation
load('busobj')

%% run simulation

%% Calculate angles from postion and direction
theta1=zeros(1,length(SABiRx));
theta2=zeros(1,length(SABiRx));
theta3=zeros(1,length(SABiRx));
theta4=zeros(1,length(SABiRx));
theta5=zeros(1,length(SABiRx));

%% get the planned path

global cbmax
cbmax=0;
for jj=1:length(SABiRx)
    
    [theta1(jj),theta2(jj),theta3(jj),theta4(jj),theta5(jj), instrumentvariable]=SABiRIK(SABiRx(:,jj),Rneedle(:,jj),xoffset,yoffset,zoffset,ln,T_BackFront_mount);
    
    if mod(jj,round(15700/(ts*1e3)))==1 || jj==length(SABiRx)
        mystring=[num2str(jj/length(SABiRx)*100,'%2.0f'),'%'];
        disp(mystring)%disp(num2str(i/length(t1)*100,'%2.0f'))
    end
end

%% error-check the planned path

% for jj=1:length(SABiRx)
%     if ~CheckJointAngles( [theta1(jj),theta2(jj),theta3(jj),theta4(jj),theta5(jj)] );
%         error('Invalid reference path - joint angles out of bounds');
%     end
% end

%% as a second check, feed the joint angles through the forward kinematics and see if the results match

%this seems like an awfully high tolerance...
tolerance = 1e-1;

% for jj=1:length(SABiRx)
%     [SABiRx_check(jj,:),R_check(:,jj),Front_check(jj,:),Back_check(jj,:)] = SABiRFK([theta1(jj),theta2(jj),theta3(jj),theta4(jj),theta5(jj)]);
%     if norm(SABiRx_check(jj,:) - SABiRx(:,jj)') > tolerance
%         error('Path validation error - IK does not match FK');
%         %         warning('Path validation error - IK does not match FK');
%     end
% end


%% draw a diagram if in debug mode

if debuglevel > 0
    row=1;
    for i=1:2
        for j=1:2
            for k=1:2
                block1Corners(row,:) = [block1MinMax(1,i), block1MinMax(2,j), block1MinMax(3,k)];
                block2Corners(row,:) = [block2MinMax(1,i), block2MinMax(2,j), block2MinMax(3,k)];
                row = row+1;
            end
        end
    end
    
    hold on
    plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
    plot3(pReadypoint(:,2),pReadypoint(:,3),pReadypoint(:,1),'bx');
    plot3(pTargetpoint(:,2),pTargetpoint(:,3),pTargetpoint(:,1),'ro');
    plot3(pTargetpoint(:,2),pTargetpoint(:,3),pTargetpoint(:,1),'rx');
    
    line(block1Corners([1 2],2),block1Corners([1 2],3),block1Corners([1 2],1),'Color','b');
    line(block1Corners([1 3],2),block1Corners([1 3],3),block1Corners([1 3],1),'Color','b');
    line(block1Corners([1 5],2),block1Corners([1 5],3),block1Corners([1 5],1),'Color','b');
    line(block1Corners([2 4],2),block1Corners([2 4],3),block1Corners([2 4],1),'Color','b');
    line(block1Corners([2 6],2),block1Corners([2 6],3),block1Corners([2 6],1),'Color','b');
    line(block1Corners([3 4],2),block1Corners([3 4],3),block1Corners([3 4],1),'Color','b');
    line(block1Corners([3 7],2),block1Corners([3 7],3),block1Corners([3 7],1),'Color','b');
    line(block1Corners([4 8],2),block1Corners([4 8],3),block1Corners([4 8],1),'Color','b');
    line(block1Corners([5 6],2),block1Corners([5 6],3),block1Corners([5 6],1),'Color','b');
    line(block1Corners([5 7],2),block1Corners([5 7],3),block1Corners([5 7],1),'Color','b');
    line(block1Corners([6 8],2),block1Corners([6 8],3),block1Corners([6 8],1),'Color','b');
    line(block1Corners([7 8],2),block1Corners([7 8],3),block1Corners([7 8],1),'Color','b');
    
    line(block2Corners([1 2],2),block2Corners([1 2],3),block2Corners([1 2],1),'Color','r');
    line(block2Corners([1 3],2),block2Corners([1 3],3),block2Corners([1 3],1),'Color','r');
    line(block2Corners([1 5],2),block2Corners([1 5],3),block2Corners([1 5],1),'Color','r');
    line(block2Corners([2 4],2),block2Corners([2 4],3),block2Corners([2 4],1),'Color','r');
    line(block2Corners([2 6],2),block2Corners([2 6],3),block2Corners([2 6],1),'Color','r');
    line(block2Corners([3 4],2),block2Corners([3 4],3),block2Corners([3 4],1),'Color','r');
    line(block2Corners([3 7],2),block2Corners([3 7],3),block2Corners([3 7],1),'Color','r');
    line(block2Corners([4 8],2),block2Corners([4 8],3),block2Corners([4 8],1),'Color','r');
    line(block2Corners([5 6],2),block2Corners([5 6],3),block2Corners([5 6],1),'Color','r');
    line(block2Corners([5 7],2),block2Corners([5 7],3),block2Corners([5 7],1),'Color','r');
    line(block2Corners([6 8],2),block2Corners([6 8],3),block2Corners([6 8],1),'Color','r');
    line(block2Corners([7 8],2),block2Corners([7 8],3),block2Corners([7 8],1),'Color','r');
    
  
    plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'r');
    %     legend('phome','ptarget','endpoint','SABiRx');
  
    title('reference needle path');
end

%% run sim

%% setup workspace variables for the simulation

Ref_unbiased =[ time1',...
    unwrap(theta1)',...
    unwrap(theta2)',...
    unwrap(theta3)',...
    unwrap(theta4)',...
    unwrap(theta5)'];

Ref_t =[ time1',...
    unwrap(theta1)'-t1bias,...
    unwrap(theta2)'-t2bias,...
    unwrap(theta3)'-t3bias,...
    unwrap(theta4)'-t4bias,...
    unwrap(theta5)'-t5bias];

%% do the simulation
tic;
sim(sys);
t=toc;
fprintf('simulated trajectory in %.2f s\n',t);

%% generate data
time = axis1.time;
[ref1 actual1 percvd1 torq1] = axis1.signals.values;
[ref2 actual2 percvd2 torq2] = axis2.signals.values;
[ref3 actual3 percvd3 torq3] = axis3.signals.values;
[ref4 actual4 percvd4 torq4] = axis4.signals.values;
[ref5 actual5 percvd5 torq5] = axis5.signals.values;

[needleDepth1 needleDepth2 needleForce1_needleframe needleForce2_needleframe needleTorque1_needleframe needleTorque2_needleframe] = axis6.signals.values;

totalNeedleDepth = needleDepth1;
totalNeedleForce_needleframe = needleForce1_needleframe + needleForce2_needleframe;

totalNeedleTorque_needleframe = needleTorque1_needleframe + needleTorque2_needleframe;

%debug variable
[geltorque1 geltorque2 geltorque3 geltorque4 geltorque5] = axis7.signals.values;

err1 = ref1-actual1;
err2 = ref2-actual2;
err3 = ref3-actual3;
err4 = ref4-actual4;
err5 = ref5-actual5;

pcvd_err1 = ref1-percvd1;
pcvd_err2 = ref2-percvd2;
pcvd_err3 = ref3-percvd3;
pcvd_err4 = ref4-percvd4;
pcvd_err5 = ref5-percvd5;

ref1 = ref1 + t1bias;
ref2 = ref2 + t2bias;
ref3 = ref3 + t3bias;
ref4 = ref4 + t4bias;
ref5 = ref5 + t5bias;

actual1 = actual1 + t1bias;
actual2 = actual2 + t2bias;
actual3 = actual3 + t3bias;
actual4 = actual4 + t4bias;
actual5 = actual5 + t5bias;

percvd1 = percvd1 + t1bias;
percvd2 = percvd2 + t2bias;
percvd3 = percvd3 + t3bias;
percvd4 = percvd4 + t4bias;
percvd5 = percvd5 + t5bias;

%needle data
for i=1:length(time)
    [ref_Needle_Tip_Position(i,:),ref_R(:,i),ref_Front(i,:),ref_Back(i,:)]=...
        SABiRFK([ref1(i),ref2(i),ref3(i),ref4(i),ref5(i)]);
end
for i=1:length(time)
    [actual_Needle_Tip_Position(i,:),actual_R(:,i),actual_Front(i,:),actual_Back(i,:)]=...
        SABiRFK([actual1(i),actual2(i),actual3(i),actual4(i),actual5(i)]);
end

if debuglevel > 0
    figure
    hold on
    grid on
    %draw needle path
    plot3(ref_Needle_Tip_Position(:,2),ref_Needle_Tip_Position(:,3),ref_Needle_Tip_Position(:,1),'r');
    plot3(ref_Needle_Tip_Position(:,2),ref_Needle_Tip_Position(:,3),ref_Needle_Tip_Position(:,1),'r.');
    plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'b');
    plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'bo');
    plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'bx');
    plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
    plot3(pReadypoint(:,2),pReadypoint(:,3),pReadypoint(:,1),'bx');
    plot3(pTargetpoint(:,2),pTargetpoint(:,3),pTargetpoint(:,1),'ro');
    plot3(pTargetpoint(:,2),pTargetpoint(:,3),pTargetpoint(:,1),'rx');
%     %     plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'b');
%     legend('reference','actual', 'actual','home','target','endpoint');%, 'SABiRx')
    
    
    line(block1Corners([1 2],2),block1Corners([1 2],3),block1Corners([1 2],1),'Color','b');
    line(block1Corners([1 3],2),block1Corners([1 3],3),block1Corners([1 3],1),'Color','b');
    line(block1Corners([1 5],2),block1Corners([1 5],3),block1Corners([1 5],1),'Color','b');
    line(block1Corners([2 4],2),block1Corners([2 4],3),block1Corners([2 4],1),'Color','b');
    line(block1Corners([2 6],2),block1Corners([2 6],3),block1Corners([2 6],1),'Color','b');
    line(block1Corners([3 4],2),block1Corners([3 4],3),block1Corners([3 4],1),'Color','b');
    line(block1Corners([3 7],2),block1Corners([3 7],3),block1Corners([3 7],1),'Color','b');
    line(block1Corners([4 8],2),block1Corners([4 8],3),block1Corners([4 8],1),'Color','b');
    line(block1Corners([5 6],2),block1Corners([5 6],3),block1Corners([5 6],1),'Color','b');
    line(block1Corners([5 7],2),block1Corners([5 7],3),block1Corners([5 7],1),'Color','b');
    line(block1Corners([6 8],2),block1Corners([6 8],3),block1Corners([6 8],1),'Color','b');
    line(block1Corners([7 8],2),block1Corners([7 8],3),block1Corners([7 8],1),'Color','b');
    
    line(block2Corners([1 2],2),block2Corners([1 2],3),block2Corners([1 2],1),'Color','r');
    line(block2Corners([1 3],2),block2Corners([1 3],3),block2Corners([1 3],1),'Color','r');
    line(block2Corners([1 5],2),block2Corners([1 5],3),block2Corners([1 5],1),'Color','r');
    line(block2Corners([2 4],2),block2Corners([2 4],3),block2Corners([2 4],1),'Color','r');
    line(block2Corners([2 6],2),block2Corners([2 6],3),block2Corners([2 6],1),'Color','r');
    line(block2Corners([3 4],2),block2Corners([3 4],3),block2Corners([3 4],1),'Color','r');
    line(block2Corners([3 7],2),block2Corners([3 7],3),block2Corners([3 7],1),'Color','r');
    line(block2Corners([4 8],2),block2Corners([4 8],3),block2Corners([4 8],1),'Color','r');
    line(block2Corners([5 6],2),block2Corners([5 6],3),block2Corners([5 6],1),'Color','r');
    line(block2Corners([5 7],2),block2Corners([5 7],3),block2Corners([5 7],1),'Color','r');
    line(block2Corners([6 8],2),block2Corners([6 8],3),block2Corners([6 8],1),'Color','r');
    line(block2Corners([7 8],2),block2Corners([7 8],3),block2Corners([7 8],1),'Color','r');
    
    title('actual needle path')
end


if debuglevel > 0
    figure
    hold on
    grid on
    %draw needle path
    plotn=100; %plot every n points
    plot3(actual_Needle_Tip_Position(1:plotn:end,2),actual_Needle_Tip_Position(1:plotn:end,3),actual_Needle_Tip_Position(1:plotn:end,1),'b');
        plot3(actual_Needle_Tip_Position(1:plotn:end,2),actual_Needle_Tip_Position(1:plotn:end,3),actual_Needle_Tip_Position(1:plotn:end,1),'b.');
    plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
    
    %plot some lines showing the needle's direction
    
    for i=1:plotn:length(actual_Needle_Tip_Position)
        p0 = actual_Needle_Tip_Position(i,:);
        r = actual_R(:,i);
        l=0.5; %(half)length of the line showing the needle direction
        p1 = p0-l*r';
        p2 = p0+l*r';
        line([p1(2) p2(2)],[p1(3) p2(3)], [p1(1) p2(1)],'Color','g');
        plot3(p1(2),p1(3),p1(1),'go');
    end
    
    plot3(pReadypoint(:,2),pReadypoint(:,3),pReadypoint(:,1),'bx');
    plot3(pTargetpoint(:,2),pTargetpoint(:,3),pTargetpoint(:,1),'ro');
    plot3(pTargetpoint(:,2),pTargetpoint(:,3),pTargetpoint(:,1),'rx');
    %     plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'b');
    
    
    line(block1Corners([1 2],2),block1Corners([1 2],3),block1Corners([1 2],1),'Color','b');
    line(block1Corners([1 3],2),block1Corners([1 3],3),block1Corners([1 3],1),'Color','b');
    line(block1Corners([1 5],2),block1Corners([1 5],3),block1Corners([1 5],1),'Color','b');
    line(block1Corners([2 4],2),block1Corners([2 4],3),block1Corners([2 4],1),'Color','b');
    line(block1Corners([2 6],2),block1Corners([2 6],3),block1Corners([2 6],1),'Color','b');
    line(block1Corners([3 4],2),block1Corners([3 4],3),block1Corners([3 4],1),'Color','b');
    line(block1Corners([3 7],2),block1Corners([3 7],3),block1Corners([3 7],1),'Color','b');
    line(block1Corners([4 8],2),block1Corners([4 8],3),block1Corners([4 8],1),'Color','b');
    line(block1Corners([5 6],2),block1Corners([5 6],3),block1Corners([5 6],1),'Color','b');
    line(block1Corners([5 7],2),block1Corners([5 7],3),block1Corners([5 7],1),'Color','b');
    line(block1Corners([6 8],2),block1Corners([6 8],3),block1Corners([6 8],1),'Color','b');
    line(block1Corners([7 8],2),block1Corners([7 8],3),block1Corners([7 8],1),'Color','b');
    
    line(block2Corners([1 2],2),block2Corners([1 2],3),block2Corners([1 2],1),'Color','r');
    line(block2Corners([1 3],2),block2Corners([1 3],3),block2Corners([1 3],1),'Color','r');
    line(block2Corners([1 5],2),block2Corners([1 5],3),block2Corners([1 5],1),'Color','r');
    line(block2Corners([2 4],2),block2Corners([2 4],3),block2Corners([2 4],1),'Color','r');
    line(block2Corners([2 6],2),block2Corners([2 6],3),block2Corners([2 6],1),'Color','r');
    line(block2Corners([3 4],2),block2Corners([3 4],3),block2Corners([3 4],1),'Color','r');
    line(block2Corners([3 7],2),block2Corners([3 7],3),block2Corners([3 7],1),'Color','r');
    line(block2Corners([4 8],2),block2Corners([4 8],3),block2Corners([4 8],1),'Color','r');
    line(block2Corners([5 6],2),block2Corners([5 6],3),block2Corners([5 6],1),'Color','r');
    line(block2Corners([5 7],2),block2Corners([5 7],3),block2Corners([5 7],1),'Color','r');
    line(block2Corners([6 8],2),block2Corners([6 8],3),block2Corners([6 8],1),'Color','r');
    line(block2Corners([7 8],2),block2Corners([7 8],3),block2Corners([7 8],1),'Color','r');
    
    title('actual needle path with direction')
end


%% error-check needle trajectory
tolerance = 1.0;
maxerr=0;
for jj=1:length(time)
    %if any point is too far away from where it's supposed to be, give an
    %error
    err = norm(ref_Needle_Tip_Position(jj,:) - actual_Needle_Tip_Position(jj,:));
    if err > tolerance
        warning('Actual path deviates > 1mm from reference path');
        break;
    end
    if err > maxerr
        maxerr=err;
    end
end

%plot debug plots

% figure
% plot([theta1' theta2' theta3' theta4' theta5'],'color','r')
% grid on
% title('reference and real joint angles')
% hold on
% plot([actual1 actual2 actual3 actual4 actual5],'color','b')
% grid on


%plot joint angles over time

PhysLimits =[
    3.421034655796548   4.712388980384690
    4.712388980384690   5.981899088886965
    -0.261799387799149   0.296076370022756
    3.473848497195918   4.712388980384690
    4.712388980384690   5.937385325132857];



lowerlimit = PhysLimits(:,1);
upperlimit = PhysLimits(:,2);


% plot([1,9000],[upperlimit,upperlimit;lowerlimit,lowerlimit], 'color', 'k')
% 
% clf
% plot(totalNeedleForce_needleframe(:,3))

figure
plot(needleDepth1)
title('needle depth');

figure
plot(needleForce1_needleframe)
title('needle force');

for i=1:length(actual_Needle_Tip_Position)-1
    needleVelocity(i,:) = (actual_Needle_Tip_Position(i+1,:) - actual_Needle_Tip_Position(i,:))/ts;
end
figure
plot(needleVelocity)
title('needle velocity');

for i=1:length(actual_Needle_Tip_Position)
    needleError(i) = norm(actual_Needle_Tip_Position(i,:) - ref_Needle_Tip_Position(i,:));
end
figure
plot(needleError)
title('needle error');

%% prepare output data

d = .01/ts2;

outdata = [time(1:d:end) actual_Needle_Tip_Position(1:d:end,:) actual_R(:,1:d:end)' actual_Front(1:d:end,:) actual_Back(1:d:end,:)...
    ref_Needle_Tip_Position(1:d:end,:) ref_R(:,1:d:end)' ref_Front(1:d:end,:) ref_Back(1:d:end,:)...
    actual1(1:d:end,:) actual2(1:d:end,:) actual3(1:d:end,:) actual4(1:d:end,:) actual5(1:d:end,:)...
    percvd1(1:d:end,:) percvd2(1:d:end,:) percvd3(1:d:end,:) percvd4(1:d:end,:) percvd5(1:d:end,:)...
    torq1(1:d:end,:) torq2(1:d:end,:) torq3(1:d:end,:) torq4(1:d:end,:) torq5(1:d:end,:)...
    err1(1:d:end,:) err2(1:d:end,:) err3(1:d:end,:) err4(1:d:end,:) err5(1:d:end,:)...
    needleDepth1(1:d:end,:) needleDepth2(1:d:end,:) totalNeedleDepth(1:d:end,:)...
    needleForce1_needleframe(1:d:end,:) needleForce2_needleframe(1:d:end,:) totalNeedleForce_needleframe(1:d:end,:)...
   needleTorque1_needleframe(1:d:end,:) needleTorque2_needleframe(1:d:end,:) totalNeedleTorque_needleframe(1:d:end,:)...
    ];
%% instrumented variable
%instrument variables from simulink controller
instrument_CalcTDir_variable=instrument_CalcTDir_T.signals.values;
[m n p]=size(instrument_SABRFK.signals.values);
instrument_SABRFK_variable=reshape(instrument_SABRFK.signals.values,n,p)';
[m n p]=size(instrument_getJointVelocities.signals.values);
instrument_getJointVelocities_variable=reshape(instrument_getJointVelocities.signals.values,n,p)';
[m n p]=size(instrument_torquesToForces.signals.values);
instrument_torquesToForces_variable=reshape(instrument_torquesToForces.signals.values,n,p)';
k=1;
instrument_Cal_set=zeros(length(actual_Needle_Tip_Position),size(instrument_CalcTDir_variable,2));
instrument_SABRFK_set=zeros(length(actual_Needle_Tip_Position),size(instrument_SABRFK_variable,2));
instrument_geJointVelocities_set=zeros(length(actual_Needle_Tip_Position),size(instrument_getJointVelocities_variable,2));
instrument_torquesToForces_set=zeros(length(actual_Needle_Tip_Position),size(instrument_torquesToForces_variable,2));
for i=1:length(instrument_CalcTDir_variable)
    if(k+19>length(instrument_CalcTDir_variable))
        m=length(actual_Needle_Tip_Position)-k+1;
        instrument_Cal_set(k:end,:)=repmat(instrument_CalcTDir_variable(i,:),m,1);
        instrument_SABRFK_set(k:end,:)=repmat(instrument_SABRFK_variable(i,:),m,1);
        instrument_geJointVelocities_set(k:end,:)=repmat(instrument_getJointVelocities_variable(i,:),m,1);
        
    else
        instrument_Cal_set(k:k+19,:)=repmat(instrument_CalcTDir_variable(i,:),20,1);
        instrument_SABRFK_set(k:k+19,:)=repmat(instrument_SABRFK_variable(i,:),20,1);
        instrument_geJointVelocities_set(k:k+19,:)=repmat(instrument_getJointVelocities_variable(i,:),20,1);
        
    end
    k=k+20;
end

if(length(instrument_torquesToForces_set)>length(instrument_torquesToForces_variable))
    instrument_torquesToForces_set(1:length(instrument_torquesToForces_variable),:)=instrument_torquesToForces_variable;
    instrument_torquesToForces_set(length(instrument_torquesToForces_variable)+1:end,:)=...
    repmat(instrument_torquesToForces_variable(end,:),length(instrument_torquesToForces_set)-length(instrument_torquesToForces_variable),1);
else
    instrument_torquesToForces_set=instrument_torquesToForces_variable(1:length(instrument_torquesToForces_set),:);
end

%instrument variables from funtion PosRef3D
instrument_PosRef3D_action1=repmat(PosRef3DInstrumentedVariables(1,1:end-1),20*PosRef3DInstrumentedVariables(1,end),1);
instrument_PosRef3D_action2=repmat(PosRef3DInstrumentedVariables(2,1:end-1),20*PosRef3DInstrumentedVariables(2,end),1);
instrument_PosRef3D_action3=repmat(PosRef3DInstrumentedVariables(3,1:end-1),20*PosRef3DInstrumentedVariables(3,end),1);
remainingSlots=length(actual_Needle_Tip_Position)-(20*PosRef3DInstrumentedVariables(1,end)+20*PosRef3DInstrumentedVariables(2,end)+20*PosRef3DInstrumentedVariables(3,end));
instrument_PosRef3D_action4=repmat(PosRef3DInstrumentedVariables(4,1:end-1),remainingSlots,1);
instrument_PosRef3D_set=[instrument_PosRef3D_action1;instrument_PosRef3D_action2;instrument_PosRef3D_action3;instrument_PosRef3D_action4];

%store all the instrumented variables in outdata2 
outdata2=[instrument_Cal_set instrument_SABRFK_set instrument_geJointVelocities_set instrument_torquesToForces_set];
t=toc;
fprintf('finished in %.2f s total\n',t);