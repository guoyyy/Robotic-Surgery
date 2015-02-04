
%% Calculate angles from postion and direction
theta1=zeros(1,length(SABiRx));
theta2=zeros(1,length(SABiRx));
theta3=zeros(1,length(SABiRx));
theta4=zeros(1,length(SABiRx));
theta5=zeros(1,length(SABiRx));

for jj=1:length(SABiRx)
    [theta1(jj),theta2(jj),theta3(jj),theta4(jj),theta5(jj)]=SABiRIK(SABiRx(:,jj),Rneedle(:,jj));
    if mod(jj,round(15700/(ts*1e3)))==1 || jj==length(SABiRx)
        mystring=[num2str(jj/length(SABiRx)*100,'%2.0f'),'%'];
        disp(mystring)%disp(num2str(i/length(t1)*100,'%2.0f'))
    end
end

if debuglevel > 0
    hold on
    plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
    plot3(ptarget(:,2),ptarget(:,3),ptarget(:,1),'bx');
    plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'r');
    legend('phome','ptarget','SABiRx');
end


%% run sim
%% Simulator PID controller globals


load('SABiR_TF.mat')


Ref_t =[ time1',...
        unwrap(theta1)'-t1bias,...
        unwrap(theta2)'-t2bias,...
        unwrap(theta3)'-t3bias,...
        unwrap(theta4)'-t4bias,...
        unwrap(theta5)'-t5bias];
    
%% do the simulation    
sim(sys);



%% generate data

time = axis1.time;
[pos1 ref1 err1 torq1] = axis1.signals.values;
[pos2 ref2 err2 torq2] = axis2.signals.values;
[pos3 ref3 err3 torq3] = axis3.signals.values;
[pos4 ref4 err4 torq4] = axis4.signals.values;
[pos5 ref5 err5 torq5] = axis5.signals.values;


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
    clf
    hold on
    grid on
    %draw needle path
    plot3(ref_Needle_Tip_Position(:,2),ref_Needle_Tip_Position(:,3),ref_Needle_Tip_Position(:,1),'r');
    plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1));
    plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
    plot3(ptarget(:,2),ptarget(:,3),ptarget(:,1),'bx');
    % plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'b');
    legend('reference','actual','home','target','SABiRx')
    
    %draw gel block
    % for i=1:length(blockCorners)
    %     plot3(blockCorners(i,1),blockCorners(i,2),blockCorners(i,3),'bo');
    % end
    
    line(blockCorners([1 2],2,1),blockCorners([1 2],3,1),blockCorners([1 2],1,1),'Color','b');
    line(blockCorners([1 3],2,1),blockCorners([1 3],3,1),blockCorners([1 3],1,1),'Color','b');
    line(blockCorners([1 5],2,1),blockCorners([1 5],3,1),blockCorners([1 5],1,1),'Color','b');
    line(blockCorners([2 4],2,1),blockCorners([2 4],3,1),blockCorners([2 4],1,1),'Color','b');
    line(blockCorners([2 6],2,1),blockCorners([2 6],3,1),blockCorners([2 6],1,1),'Color','b');
    line(blockCorners([3 4],2,1),blockCorners([3 4],3,1),blockCorners([3 4],1,1),'Color','b');
    line(blockCorners([3 7],2,1),blockCorners([3 7],3,1),blockCorners([3 7],1,1),'Color','b');
    line(blockCorners([4 8],2,1),blockCorners([4 8],3,1),blockCorners([4 8],1,1),'Color','b');
    line(blockCorners([5 6],2,1),blockCorners([5 6],3,1),blockCorners([5 6],1,1),'Color','b');
    line(blockCorners([5 7],2,1),blockCorners([5 7],3,1),blockCorners([5 7],1,1),'Color','b');
    line(blockCorners([6 8],2,1),blockCorners([6 8],3,1),blockCorners([6 8],1,1),'Color','b');
    line(blockCorners([7 8],2,1),blockCorners([7 8],3,1),blockCorners([7 8],1,1),'Color','b');
    
    line(blockCorners([1 2],2,2),blockCorners([1 2],3,2),blockCorners([1 2],1,2),'Color','r');
    line(blockCorners([1 3],2,2),blockCorners([1 3],3,2),blockCorners([1 3],1,2),'Color','r');
    line(blockCorners([1 5],2,2),blockCorners([1 5],3,2),blockCorners([1 5],1,2),'Color','r');
    line(blockCorners([2 4],2,2),blockCorners([2 4],3,2),blockCorners([2 4],1,2),'Color','r');
    line(blockCorners([2 6],2,2),blockCorners([2 6],3,2),blockCorners([2 6],1,2),'Color','r');
    line(blockCorners([3 4],2,2),blockCorners([3 4],3,2),blockCorners([3 4],1,2),'Color','r');
    line(blockCorners([3 7],2,2),blockCorners([3 7],3,2),blockCorners([3 7],1,2),'Color','r');
    line(blockCorners([4 8],2,2),blockCorners([4 8],3,2),blockCorners([4 8],1,2),'Color','r');
    line(blockCorners([5 6],2,2),blockCorners([5 6],3,2),blockCorners([5 6],1,2),'Color','r');
    line(blockCorners([5 7],2,2),blockCorners([5 7],3,2),blockCorners([5 7],1,2),'Color','r');
    line(blockCorners([6 8],2,2),blockCorners([6 8],3,2),blockCorners([6 8],1,2),'Color','r');
    line(blockCorners([7 8],2,2),blockCorners([7 8],3,2),blockCorners([7 8],1,2),'Color','r');
    
    %make some arrows to indicate the vector as the needle moves through the
    %critical points
    
    len = 10;
    wid = .1;
    % clf;
    p1s = phome;
    p1e = p1s +rhome'*10;
    p2s = ptarget;
    p2e = p2s + rtarget'*10;
    arrow('Start',[p1s(2) p1s(3) p1s(1)],'Stop',[p1e(2) p1e(3) p1e(1)],'Length',len,'Width',wid);
    arrow('Start',[p2s(2) p2s(3) p2s(1)],'Stop',[p2e(2) p2e(3) p2e(1)],'Length',len,'Width',wid);
end

%% calculate the force on the needletip

% needleDepth1 = zeros(size(time));
% needleDepth2 = zeros(size(time));
% needleForce1 = zeros(size(time));
% needleForce2 = zeros(size(time));

%for speed, this should be only calculated once, and the depth estimated
%using the one calculation for all timesteps
% for i=1:length(time)
%     needleDepth1(i) = distanceInserted(actual_Needle_Tip_Position(i,:),actual_R(:,i),blockCorners(:,:,1));
%     needleDepth2(i) = distanceInserted(actual_Needle_Tip_Position(i,:),actual_R(:,i),blockCorners(:,:,2));
% end

%i need to use only the portion of the trajectory that is straight, ***NOT THE ENTIRE TRAJECTORY***
needleDepth1 = getDepth_straightneedle(actual_Needle_Tip_Position(1,:),rtarget,blockCorners(:,:,1),actual_Needle_Tip_Position);
needleDepth2 = getDepth_straightneedle(actual_Needle_Tip_Position(1,:),rtarget,blockCorners(:,:,2),actual_Needle_Tip_Position);

totalNeedleDepth = needleDepth1;
needleDepth1 = needleDepth1-needleDepth2; %adjust the needle depth to account for the fact that block 2 is inside block 1


if debuglevel > 0
    figure
    hold on
    plot(needleDepth1,'color','b'), title('needle depth');
    plot(needleDepth2,'color','r'),
    plot(totalNeedleDepth,'color','g'), legend('needle depth in outer block (block 1)','needle depth in inner block (block 2)','total needle depth');
end



retractionTime = floor(length(time)/2); %close enough

needleForce1 = zeros(size(time));
needleForce2 = zeros(size(time));

for i=1:retractionTime
    needleForce1(i) = needleInsertionForce(needleDepth1(i),gelParams1insert);
    needleForce2(i) = needleInsertionForce(needleDepth2(i),gelParams2insert);
end

for i=retractionTime:length(time)
    needleForce1(i) = needleInsertionForce(needleDepth1(i),gelParams1retract);
    needleForce2(i) = needleInsertionForce(needleDepth2(i),gelParams2retract);
end

totalNeedleForce = needleForce1+needleForce2;

if debuglevel > 0
    figure
    hold on
    plot(needleForce1,'color','b')
    plot(needleForce2,'color','r')
    plot(totalNeedleForce,'color','g') %note that the force profile i get from this doesn't make any sense yet
    % legend('needle force due to outer block (block 1)','needle force due to inner block (block 2)','total needle force');
    legend('force1','force2','total force')
end

geltorque1 = zeros(size(torq1));
geltorque2 = zeros(size(torq2));
geltorque3 = zeros(size(torq3));
geltorque4 = zeros(size(torq4));
geltorque5 = zeros(size(torq5));
for i=1:length(time)
    %get the jacobian for the current configuration
    [Jb,Jf] = Jacobian([pos1(i),pos2(i),pos3(i),pos4(i),pos5(i)]);
    Jb = Jb(1:3,:);
    Jf = Jf(1:3,:);
    
    %get the current force vector
    F = totalNeedleForce(i) * actual_R(:,i);
    
    %get the torques by T = J'*F
    T = Jf'*F;
    geltorque1(i) = T(1);
    geltorque2(i) = T(2);
    T = Jb'*F;
    geltorque3(i) = T(1);
    geltorque4(i) = T(2);
    geltorque5(i) = T(3);
end

torq1 = torq1+geltorque1;
torq2 = torq2+geltorque2;
torq3 = torq3+geltorque3;
torq4 = torq4+geltorque4;
torq5 = torq5+geltorque5;

outdata = [time actual_Needle_Tip_Position actual_R' actual_Front actual_Back...
    ref_Needle_Tip_Position ref_R' ref_Front ref_Back...
    pos1 pos2 pos3 pos4 pos5...
    torq1 torq2 torq3 torq4 torq5...
    err1 err2 err3 err4 err5...
    needleDepth1 needleDepth2 totalNeedleDepth...
    needleForce1 needleForce2 totalNeedleForce...
    ];

%     save(sprintf('gel_experiment_outdata%d.mat',targetnum), 'outdata', 'ptarget','rtarget','insertdist');


% totaltime = toc(tstart);
% fprintf('total elapsed time: %.2fs\n',totaltime);