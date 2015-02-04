%This is Russell's Test script that runs a needle rotation.

close all
clear
clc

%This plots the needle as it moves through space.
load('needlepts.mat');

N = length(needlept0);



%% Loop Data
        structNeedleGeometry = struct('straightL',123.1420,'kinkAngle', 0, 'radius',0,'arc',0);
        structTissueData =struct('center',[0; -200; 400],'widthX',[40],'heightZ',[120],'depthY',[50]);
        

%% Allocate the Needle force vectors
wrenchModeled_0 = zeros(6,N);
wrenchModeled_N = zeros(6,N);
wrenchFriction = zeros(6,N);
wrenchNorm = zeros(6,N);
wrenchCut = zeros(6,N);


%% Set up the force parameters
structMaterialParams.mus = 0*0.16;
structMaterialParams.muk = 0.08;
structMaterialParams.K   =  reshape(eye(3),9,1); %reshape(diag([.0087; .0087; .0087;]),9,1);
structMaterialParams.alpha = 0.25; %1.0622;
structMaterialParams.limit = 0.5; %0.5;

axisLims =  [-30 30; -230 -170; 200 450];


%needleFig = figure('position',[500,500,1000,500]);
time = linspace(1,10,N);
needleFig = figure;
z0 = [0;0;1];
for i = 1:N
    %find the rotation matrix:
    zTemp = actual_R(:,i);
    pTemp = needlept0(i,:)';
    
    omega = cross(z0,zTemp);
    theta = asin(norm(omega));
    
    Qout = [cos(theta/2) sin(theta/2)*omega'/norm(omega)'];
    
    
    NeedleTransDes.rot  = Qout;
    NeedleTransDes.trans  = pTemp;
    if( i == 1)
        [TissueStateOld  NeedlePts2Old]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,NeedleTransDes,50);
        wrenchNormOld = zeros(6,1);
    else
        TissueStateOld = TissueState;
        NeedlePts2Old  = NeedlePts2;
        wrenchNormOld  =  wrenchNorm(:,i-1);
    end
    
    
    [TissueState  NeedlePts2]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,NeedleTransDes,50);
    
    [wrenchModeled_0(:,i),wrenchFriction(:,i),wrenchNorm(:,i),wrenchCut(:,i)] = Needle_Cumulative_Forces_Step(TissueStateOld,NeedlePts2Old,TissueState,NeedlePts2,structMaterialParams,wrenchNormOld);
    
    g_0N = struct2G(NeedleTransDes);
    
    wrenchModeled_N(:,i) = fncWrenchTransform(wrenchModeled_0(:,i),Ginv(g_0N));
    
    
    if(mod(i,20) == 0)
    NeedlePlot(needleFig,structTissueData,NeedlePts2Old,TissueStateOld,NeedlePts2,TissueState,axisLims);
    pause(.1);
    
    end
    
    
    
    %Plot Needle.
    
    
end




%Plot the needle forces.
forcesHand = figure;


plot(time,wrenchModeled_N(1,:),time,wrenchModeled_N(2,:),time,wrenchModeled_N(3,:));
legend('x','y','z');
ylabel('force (N)');
xlabel('time (s)');


torquesHand = figure;
plot(time,wrenchModeled_N(4,:),time,wrenchModeled_N(5,:),time,wrenchModeled_N(6,:));
legend('x','y','z');
ylabel('torque (N-mm)');
xlabel('time (s)');




