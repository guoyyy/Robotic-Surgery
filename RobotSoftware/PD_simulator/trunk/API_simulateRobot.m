function [outdata,errortype]=API_simulateRobot(ptarget,rtarget,time1,TotalTime,SABiRx,Rneedle,turnaroundtime,numEncoderToFail,encoderFailureTime,ts)
%% set up global parameters for simulator
rtarget=rtarget';
SABiRx1=cell2mat(SABiRx');
%assignin('base', 'SABiRx1', SABiRx1);
SABiRx=SABiRx1';
assignin('base', 'SABiRx', SABiRx);
Rneedle1=cell2mat(Rneedle');
%assignin('base', 'Rneedle', Rneedle1);
Rneedle=Rneedle1';
assignin('base', 'Rneedle', Rneedle);
errortype='';
outdata=[];
assignin('base','time1',time1);
assignin('base','numEncoderToFail',numEncoderToFail);

%API_setupGlobalAndPara;
global yoffset;
global xoffset;
global zoffset;
global ln;
global T_BackFront_mount;
global block1Corners;
global block2Corners;
global PosRef3DInstrumentedVariables
%% encoder failure
%compute failure time
tsEncoderFailure = TotalTime * encoderFailureTime;
assignin('base','tsEncoderFailure',tsEncoderFailure);
%% set up gel blocks and gel parameters
% skinthickness1 = .5;
% A1 = [.003 .002 .0023];%parameters for gel type 1
% B1 = [.012 .008 .0092];%parameters for skin type 1
% 
% skinthickness2 = .1;
% A2 = [.06 .004 .0046]; %parameters for gel type 2
% B2 = [.18 .008 .0096]; %parameters for skin type 2
% 
% %make the blocks soft
% softnessfactor = 1e-5;
% A1 = A1 * softnessfactor;
% B1 = B1 * softnessfactor;
% A2 = A2 * softnessfactor;
% B2 = B2 * softnessfactor;

%% prepare simulink model
sys = 'SABiR_simulator';
open_system(sys);

% Create a new configuration set
% cset1 = Simulink.ConfigSet;
%load a configuration set
load('cset1');
% Set a non-default stop time
%set_param (cset1, 'StopTime', '1')
set_param(cset1, 'StopTime', sprintf('%f',TotalTime));
set_param(cset1, 'SFSimEnableDebug','on');
assignin('base', 'cset1', cset1);
% Create a new config reference
%cref1 = Simulink.ConfigSetRef;
% load a config reference
load('cref1')
% Resolve the config ref to the set
cref1.WsVarName = 'cset1';
% Attach the config ref to an untitled model
attachConfigSet(sys, cref1, true)
% Set the active configuration set to the reference
setActiveConfigSet(sys,'Reference')
% Replace config set in the base workspace
%cset1 = Simulink.ConfigSet;
% Call to refresh is commented out!
%cref1.refresh;
% Set a different stop time
%set_param (cset1, 'StopTime', '1')
load('busobj')
assignin('base','Material_Parameters',Material_Parameters);
%% run simulation

%% Variables

%global fs
%global phome
%global rhome
%global Ref_t
t1bias= 225*pi/180;                      % Home position for axis 1
t2bias= 315*pi/180;                      % Home position for axis 2
t3bias= 0;                               % Home position for axis 3
t4bias= 225*pi/180;                      % Home position for axis 4
t5bias= 315*pi/180;                      % Home position for axis 5


%% Calculate angles from postion and direction
theta1=zeros(1,length(SABiRx));
theta2=zeros(1,length(SABiRx));
theta3=zeros(1,length(SABiRx));
theta4=zeros(1,length(SABiRx));
theta5=zeros(1,length(SABiRx));
instrumentedSABiRIK=[];
global cbmax
cbmax=0;
for jj=1:length(SABiRx)
    
    [t1,t2,t3,t4,t5,instrumentedSABiRIKvariable]=SABiRIK(SABiRx(:,jj),Rneedle(:,jj),xoffset,yoffset,zoffset,ln,T_BackFront_mount);
    if isempty (instrumentedSABiRIK)
    instrumentedSABiRIK=zeros(length(SABiRx),length(instrumentedSABiRIKvariable));
    end
    if strcmp('error1', t1)||strcmp('error2', t1)
    errortype=t1;
    disp('trajectory is not valid, simulation will stop and return');
    return
    end
    theta1(jj)=t1;theta2(jj)=t2;theta3(jj)=t3;theta4(jj)=t4;theta5(jj)=t5;
    instrumentedSABiRIK(jj,:)=instrumentedSABiRIKvariable;
    if mod(jj,round(15700/(ts*1e3)))==1 || jj==length(SABiRx)
        mystring=[num2str(jj/length(SABiRx)*100,'%2.0f'),'%'];
        disp(mystring)%disp(num2str(i/length(t1)*100,'%2.0f'))
    end
end

%% ray-cast to find the intersection points of the needle vector and the gel blocks

% [block1entry, block1exit] = getBlockIntersections(ptarget,rtarget,block1Corners);
% [block2entry, block2exit] = getBlockIntersections(ptarget,rtarget,block2Corners);
%% setup workspace variables for the simulation

Ref_unbiased =[ time1',...
    unwrap(theta1)',...
    unwrap(theta2)',...
    unwrap(theta3)',...
    unwrap(theta4)',...
    unwrap(theta5)'];
assignin('base','Ref_unbiased',Ref_unbiased);

Ref_t =[ time1',...
    unwrap(theta1)'-t1bias,...
    unwrap(theta2)'-t2bias,...
    unwrap(theta3)'-t3bias,...
    unwrap(theta4)'-t4bias,...
    unwrap(theta5)'-t5bias];
assignin('base','Ref_t',Ref_t);


% block1entry_ext = [time1' repmat(block1entry,length(time1),1)];
% block1exit_ext = [time1' repmat(block1exit,length(time1),1)];
% A1_ext = [time1' repmat(A1,length(time1),1)];
% B1_ext = [time1' repmat(B1,length(time1),1)];
% skinthickness1_ext = [time1' repmat(skinthickness1,length(time1),1)];
% 
% assignin('base','block1entry_ext',block1entry_ext);
% assignin('base','block1exit_ext',block1exit_ext);
% assignin('base','A1_ext',A1_ext);
% assignin('base','B1_ext',B1_ext);
% assignin('base','skinthickness1_ext',skinthickness1_ext);
% % block2entry_ext = [time1' repmat(block2entry_offset,length(time1),1)];
% % block2exit_ext = [time1' repmat(block2exit_offset,length(time1),1)];
% block2entry_ext = [time1' repmat(block2entry,length(time1),1)];
% block2exit_ext = [time1' repmat(block2exit,length(time1),1)];
% A2_ext = [time1' repmat(A2,length(time1),1)];
% B2_ext = [time1' repmat(B2,length(time1),1)];
% skinthickness2_ext = [time1' repmat(skinthickness2,length(time1),1)];
% 
% assignin('base','block2entry_ext',block2entry_ext);
% assignin('base','block2exit_ext',block2exit_ext);
% assignin('base','A2_ext',A2_ext);
% assignin('base','B2_ext',B2_ext);
% assignin('base','skinthickness2_ext',skinthickness2_ext);
% 
%  ptarget_ext = [time1' repmat(ptarget,length(time1),1)];
% % % phome_ext = [time1' repmat(phome,length(time1),1)];
%  rtarget_ext = [time1' repmat(rtarget',length(time1),1)];
% % turnaroundtime_ext = [time1' repmat(turnaroundtime,length(time1),1)];
% % 
% % block1Corners_ext = [time1' repmat(block1Corners(:)',length(time1),1)];
% % block2Corners_ext = [time1' repmat(block2Corners(:)',length(time1),1)];
% % 
%  assignin('base','ptarget_ext',ptarget_ext);
%  assignin('base','rtarget_ext',rtarget_ext);
% assignin('base','turnaroundtime_ext',turnaroundtime_ext);
% assignin('base','block1Corners_ext',block1Corners_ext);
% assignin('base','block2Corners_ext',block2Corners_ext);
%%
% block1MinMax = [-20 0; -240 -180; 340 360];
% block2MinMax = [-12 -7; -220 -200; 345 350]; 
% block1MaterialParameters = [2 1 2 1] * 1e-7;
% block2MaterialParameters = 2*block1MaterialParameters;
% block1MinMax_ext = [time1' repmat(block1MinMax(:)',length(time1),1)];
% block1MaterialParameters_ext = [time1' repmat(block1MaterialParameters(:)',length(time1),1)];
% assignin('base','block1MinMax_ext',block1MinMax_ext);
% assignin('base','block1MaterialParameters_ext',block1MaterialParameters_ext);
% block2MinMax_ext = [time1' repmat(block2MinMax(:)',length(time1),1)];
% block2MaterialParameters_ext = [time1' repmat(block2MaterialParameters(:)',length(time1),1)];
% assignin('base','block2MinMax_ext',block2MinMax_ext);
% assignin('base','block2MaterialParameters_ext',block2MaterialParameters_ext);
% 
% numEncoderToFail_ext = [time1' repmat(numEncoderToFail,length(time1),1)];
% tsEncoderFailure_ext = [time1' repmat(tsEncoderFailure,length(time1),1)];
% assignin('base','numEncoderToFail_ext',numEncoderToFail_ext);
% assignin('base','tsEncoderFailure_ext',tsEncoderFailure_ext);
%% run sim
tic;
sim(sys);
t=toc;
fprintf('simulated trajectory in %.2f s\n',t);
%% generate data

%run this after simulation to produce a .mat file containing all variables
global needleDepth1
global needleForce1_needleframe
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

% pos1_delta=[0;actual1(2:end)-actual1(1:end-1)];
% pos2_delta=[0;actual2(2:end)-actual2(1:end-1)];
% pos3_delta=[0;actual3(2:end)-actual3(1:end-1)];
% pos4_delta=[0;actual4(2:end)-actual4(1:end-1)];
% pos5_delta=[0;actual5(2:end)-actual5(1:end-1)];

pos1_delta=[0;percvd1(2:end)-percvd1(1:end-1)];
pos2_delta=[0;percvd2(2:end)-percvd2(1:end-1)];
pos3_delta=[0;percvd3(2:end)-percvd3(1:end-1)];
pos4_delta=[0;percvd4(2:end)-percvd4(1:end-1)];
pos5_delta=[0;percvd5(2:end)-percvd5(1:end-1)];
%error between actual speed and reference speed
speed_err1=[0;pcvd_err1(2:end)-pcvd_err1(1:end-1)];
speed_err2=[0;pcvd_err2(2:end)-pcvd_err2(1:end-1)];
speed_err3=[0;pcvd_err3(2:end)-pcvd_err3(1:end-1)];
speed_err4=[0;pcvd_err4(2:end)-pcvd_err4(1:end-1)];
speed_err5=[0;pcvd_err5(2:end)-pcvd_err5(1:end-1)];

global ref_Needle_Tip_Position
global actual_Needle_Tip_Position
global actual_R
for i=1:length(time)
    [ref_Needle_Tip_Position(i,:),ref_R(:,i),ref_Front(i,:),ref_Back(i,:)]=...
        SABiRFK([ref1(i),ref2(i),ref3(i),ref4(i),ref5(i)]);
end
for i=1:length(time)
    [actual_Needle_Tip_Position(i,:),actual_R(:,i),actual_Front(i,:),actual_Back(i,:)]=...
        SABiRFK([actual1(i),actual2(i),actual3(i),actual4(i),actual5(i)]);
end
%get depth of needle with gel shape.
estimated_depth_gel=zeros(length(time),1);
for i=1:length(time)
    [measured_Needle_Tip_Position(i,:),measured_R(:,i),measured_Front(i,:),measured_Back(i,:)]=...
        SABiRFK([percvd1(i),percvd2(i),percvd3(i),percvd4(i),percvd5(i)]);
    estimated_depth_gel(i)=API_getNeedleDepth(measured_Needle_Tip_Position(i,:),measured_R(:,i)');
end
% tolerance = 1.0;
% maxerr=0;
% for jj=1:length(time)
%     %if any point is too far away from where it's supposed to be, give an
%     %error
%     err = norm(ref_Needle_Tip_Position(jj,:) - actual_Needle_Tip_Position(jj,:));
%     if err > tolerance
%         warning('Actual path deviates > 1mm from reference path');
%         break;
%     end
%     if err > maxerr
%         maxerr=err;
%     end
% end

%plot joint angles over time

PhysLimits =[
    3.421034655796548   4.712388980384690
    4.712388980384690   5.981899088886965
    -0.261799387799149   0.296076370022756
    3.473848497195918   4.712388980384690
    4.712388980384690   5.937385325132857];



lowerlimit = PhysLimits(:,1);
upperlimit = PhysLimits(:,2);
assignin('base','lowerlimit',lowerlimit);
assignin('base','upperlimit',upperlimit);


needleVelocity = [zeros(1,3);(measured_Needle_Tip_Position(2:end,:) - measured_Needle_Tip_Position(1:end-1,:))/ts];
[needleFx_est needleFy_est needleFz_est needleTx_est needleTy_est] = axis8.signals.values;
% d = .01/ts2;
needleFx =needleFx_est(1,:);
needleFy =needleFy_est(1,:);
needleFz =needleFz_est(1,:);
needleTx =needleTx_est(1,:);
needleTy =needleTy_est(1,:);
if length(needleFx)<length(time)
    res_data=zeros(1,length(time)-length(needleFx));
    needleFx=[needleFx res_data];
    needleFy=[needleFy res_data];
    needleFz=[needleFz res_data];
    needleTx=[needleTx res_data];
    needleTy=[needleTy res_data];
    
elseif length(needleFx)>length(time)
    needleFx=needleFx(:,1:length(time));
    needleFy=needleFy(:,1:length(time));
    needleFz=needleFz(:,1:length(time));
    needleTx=needleTx(:,1:length(time));
    needleTy=needleTy(:,1:length(time));
end
abs_needleFz=abs(needleFz);
index=find(abs_needleFz>1E-8);
startTime=index(1);
enterPoint=measured_Needle_Tip_Position(startTime,:);
depth1=zeros(1,startTime-1);
depth2=zeros(1,length(time)-length(depth1));
for i=startTime:length(time);
     if (measured_Needle_Tip_Position(i,:)-enterPoint)*measured_R(:,startTime)>0
         depth2(i-startTime+1)=norm(measured_Needle_Tip_Position(i,:)-enterPoint);
     else
         depth2(i-startTime+1)=0;
     end
end
estimated_depth_torque=[depth1 depth2]';

outdata1 = [
    time ...
    measured_Needle_Tip_Position,measured_R',...
    ref_Needle_Tip_Position ref_R' needleVelocity...
    percvd1 percvd2 percvd3 percvd4 percvd5...
    torq1 torq2 torq3 torq4 torq5...  
    pcvd_err1 pcvd_err2 pcvd_err3 pcvd_err4 pcvd_err5...
    pos1_delta pos2_delta pos3_delta pos4_delta pos5_delta...
    speed_err1 speed_err2 speed_err3 speed_err4 speed_err5...
    needleFx' needleFy' needleFz' needleTx' needleTy'...
    estimated_depth_torque estimated_depth_gel...
    ];
assignin('base', 'outdata1', outdata1);

%% instrumented variable
%instrument variables from simulink controller
instrument_CalcTDir_variable=instrument_CalcTDir_T.signals.values;
[m n p]=size(instrument_SABRFK.signals.values);
instrument_SABRFK_variable=reshape(instrument_SABRFK.signals.values,n,p)';
[m n p]=size(instrument_getJointVelocities.signals.values);
instrument_getJointVelocities_variable=reshape(instrument_getJointVelocities.signals.values,n,p)';
[m n p]=size(instrument_torquesToForces.signals.values);
instrument_torquesToForces_variable=reshape(instrument_torquesToForces.signals.values,n,p)';

hardwareDataLength=length(actual_Needle_Tip_Position);
instrument_Cal_set=zeros(hardwareDataLength,size(instrument_CalcTDir_variable,2));
instrument_SABRFK_set=zeros(hardwareDataLength,size(instrument_SABRFK_variable,2));
instrument_geJointVelocities_set=zeros(hardwareDataLength,size(instrument_getJointVelocities_variable,2));
instrument_torquesToForces_set=zeros(hardwareDataLength,size(instrument_torquesToForces_variable,2));
instrument_SABiRIK_set=zeros(hardwareDataLength,size(instrumentedSABiRIK,2));
k=1;
for i=1:length(instrument_CalcTDir_variable)
    if i<length(instrumentedSABiRIK)
        instrument_SABiRIK_set(k:k+19,:)=repmat(instrumentedSABiRIK(i,:),20,1);
    elseif i==length(instrumentedSABiRIK)
        m1=length(actual_Needle_Tip_Position)-k+1;
        instrument_SABiRIK_set(k:end,:)=repmat(instrumentedSABiRIK(i,:),m1,1);       
    end
    if(k+19>length(instrument_CalcTDir_variable))
        m2=length(actual_Needle_Tip_Position)-k+1;
        instrument_Cal_set(k:end,:)=repmat(instrument_CalcTDir_variable(i,:),m2,1);
        instrument_SABRFK_set(k:end,:)=repmat(instrument_SABRFK_variable(i,:),m2,1);
        instrument_geJointVelocities_set(k:end,:)=repmat(instrument_getJointVelocities_variable(i,:),m2,1);        
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
outdata2=[instrument_Cal_set instrument_SABRFK_set instrument_geJointVelocities_set instrument_torquesToForces_set instrument_PosRef3D_set instrument_SABiRIK_set];
outdata=[outdata1 outdata2];
t=toc;
fprintf('finished in %.2f s total\n',t);

outdata=outdata';
t=toc;
fprintf('finished in %.2f s total\n',t);