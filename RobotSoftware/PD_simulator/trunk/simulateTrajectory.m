function outdata = simulateTrajectory(ptarget, rtarget, insertdist)%, block1Corners, block1Parameters, block2Corners, block2Parameters)

%% set up constants in one place
setupConstants


%% debug level - for now: 0 off; 1 on
debuglevel=1;

%% set up trajectory
ptarget = [-10 -210 338];
% ptarget =  phome + [0 0 4.52045];
% ptarget =  phome + [0 0 3];
rtarget = [0 0 1]';
% rtarget = rhome;
% rtarget = rtarget/norm(rtarget);
insertdist = 23;

pend = ptarget + insertdist*rtarget';


%% set up simulation
[time1,turnaroundtime,turnaroundts,TotalTime,SABiRx,Rneedle] = setupSimulation(ptarget,rtarget,insertdist,ts);
fprintf('total trajectory time  is %.2fs\n', TotalTime);

%% prepare simulink model
sys = 'sabir12simPD_4';
open_system(sys);

% Create a new configuration set
cset1 = Simulink.ConfigSet;
% Set a non-default stop time
%set_param (cset1, 'StopTime', '1')
set_param(cset1, 'StopTime', sprintf('%f',TotalTime));
set_param(cset1, 'SFSimEnableDebug','on');
% Create a new config reference
cref1 = Simulink.ConfigSetRef;
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


%% run simulation

%% Calculate angles from postion and direction
theta1=zeros(1,length(SABiRx));
theta2=zeros(1,length(SABiRx));
theta3=zeros(1,length(SABiRx));
theta4=zeros(1,length(SABiRx));
theta5=zeros(1,length(SABiRx));

%% get the planned path

for jj=1:length(SABiRx)
    [theta1(jj),theta2(jj),theta3(jj),theta4(jj),theta5(jj)]=SABiRIK(SABiRx(:,jj),Rneedle(:,jj),xoffset,yoffset,zoffset,ln,T_BackFront_mount);
    if mod(jj,round(15700/(ts*1e3)))==1 || jj==length(SABiRx)
        mystring=[num2str(jj/length(SABiRx)*100,'%2.0f'),'%'];
        disp(mystring)%disp(num2str(i/length(t1)*100,'%2.0f'))
    end
end

%% ray-cast to find the intersection points of the needle vector and the gel blocks

[block1entry, block1exit] = getBlockIntersections(ptarget,rtarget,block1Corners);
[block2entry, block2exit] = getBlockIntersections(ptarget,rtarget,block2Corners);

%% draw a diagram if in debug mode

if debuglevel > 0
    hold on
    plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
    plot3(ptarget(:,2),ptarget(:,3),ptarget(:,1),'bx');
    plot3(pend(:,2),pend(:,3),pend(:,1),'ro');
    plot3(pend(:,2),pend(:,3),pend(:,1),'rx');
    
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
    
    plot3(block1entry(2),block1entry(3),block1entry(1),'bo');
    plot3(block1exit(2),block1exit(3),block1exit(1),'bo');
    plot3(block2entry(2),block2entry(3),block2entry(1),'ro');
    plot3(block2exit(2),block2exit(3),block2exit(1),'ro');
    
    plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'r');
    %     legend('phome','ptarget','endpoint','SABiRx');
end

%% run sim

load('SABiR_TF.mat')

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

% block1entry_ext = [time1' repmat(block1entry_offset,length(time1),1)];
% block1exit_ext = [time1' repmat(block1exit_offset,length(time1),1)];
block1entry_ext = [time1' repmat(block1entry,length(time1),1)];
block1exit_ext = [time1' repmat(block1exit,length(time1),1)];
A1_ext = [time1' repmat(A1,length(time1),1)];
B1_ext = [time1' repmat(B1,length(time1),1)];
skinthickness1_ext = [time1' repmat(skinthickness1,length(time1),1)];

% block2entry_ext = [time1' repmat(block2entry_offset,length(time1),1)];
% block2exit_ext = [time1' repmat(block2exit_offset,length(time1),1)];
block2entry_ext = [time1' repmat(block2entry,length(time1),1)];
block2exit_ext = [time1' repmat(block2exit,length(time1),1)];
A2_ext = [time1' repmat(A2,length(time1),1)];
B2_ext = [time1' repmat(B2,length(time1),1)];
skinthickness2_ext = [time1' repmat(skinthickness2,length(time1),1)];

ptarget_ext = [time1' repmat(ptarget,length(time1),1)];
% phome_ext = [time1' repmat(phome,length(time1),1)];
rtarget_ext = [time1' repmat(rtarget',length(time1),1)];
turnaroundtime_ext = [time1' repmat(turnaroundtime,length(time1),1)];

block1Corners_ext = [time1' repmat(block1Corners(:)',length(time1),1)];
block2Corners_ext = [time1' repmat(block2Corners(:)',length(time1),1)];

%% do the simulation

sim(sys);

%% generate data

time = axis1.time;
[pos1 ref1 err1 torq1] = axis1.signals.values;
[pos2 ref2 err2 torq2] = axis2.signals.values;
[pos3 ref3 err3 torq3] = axis3.signals.values;
[pos4 ref4 err4 torq4] = axis4.signals.values;
[pos5 ref5 err5 torq5] = axis5.signals.values;
[needleDepth1 needleDepth2 needleForce1 needleForce2] = axis6.signals.values;
totalNeedleDepth = needleDepth1 + needleDepth2;
totalNeedleForce = needleForce1 + needleForce2;

ref1 = ref1 + t1bias;
ref2 = ref2 + t2bias;
ref3 = ref3 + t3bias;
ref4 = ref4 + t4bias;
ref5 = ref5 + t5bias;

pos1 = pos1 + t1bias;
pos2 = pos2 + t2bias;
pos3 = pos3 + t3bias;
pos4 = pos4 + t4bias;
pos5 = pos5 + t5bias;

%needle data
for i=1:length(time)
    [ref_Needle_Tip_Position(i,:),ref_R(:,i),ref_Front(i,:),ref_Back(i,:)]=...
        SABiRFK([ref1(i),ref2(i),ref3(i),ref4(i),ref5(i)]);
end
for i=1:length(time)
    [actual_Needle_Tip_Position(i,:),actual_R(:,i),actual_Front(i,:),actual_Back(i,:)]=...
        SABiRFK([pos1(i),pos2(i),pos3(i),pos4(i),pos5(i)]);
end

if debuglevel > 0
    figure
    hold on
    grid on
    %draw needle path
    plot3(ref_Needle_Tip_Position(:,2),ref_Needle_Tip_Position(:,3),ref_Needle_Tip_Position(:,1),'r');
    plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'b');
    plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'b.');
    plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
    plot3(ptarget(:,2),ptarget(:,3),ptarget(:,1),'bx');
    plot3(pend(:,2),pend(:,3),pend(:,1),'ro');
    plot3(pend(:,2),pend(:,3),pend(:,1),'rx');
    %     plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'b');
    legend('reference','actual', 'actual','home','target','endpoint');%, 'SABiRx')
    
    
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
    
end


outdata = [time actual_Needle_Tip_Position actual_R' actual_Front actual_Back...
    ref_Needle_Tip_Position ref_R' ref_Front ref_Back...
    pos1 pos2 pos3 pos4 pos5...
    torq1 torq2 torq3 torq4 torq5...
    err1 err2 err3 err4 err5...
    needleDepth1 needleDepth2 totalNeedleDepth...
    needleForce1 needleForce2 totalNeedleForce...
    ];

if debuglevel > 0
    rmsError = sqrt(err1.^2+err2.^2+err3.^2+err4.^2+err5.^2);
    figure
    hold on
    plot(rmsError(1:end,:),'b')
    plot(totalNeedleForce(1:end,:)*1e5,'r')
    plot(totalNeedleDepth(1:end,:)/20,'g')
    legend('error','needle force','depth/20')
end
