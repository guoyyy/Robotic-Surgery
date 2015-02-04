%This is Russell's Test script that runs a needle rotation.

close all
clear
clc

%This plots the needle as it moves through space.


%% Loop Data
%steps:
N = 100;

%Tissue Parameters:





%% Define the Needle Geometry and motion

n = 5;

switch n
    
    case 1
        
        %Needle Type 1. (Ideal Curved Needle)
        
        structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);
        
        NeedleTransDes = struct('rot',[1 0 0 0]','trans', [0 0 0]');
        Q = @(theta) [cos(-theta/2) sin(-theta/2)*[0 0 1]]';
        pos = @(theta) -10*[cos(-theta+pi/2); sin(-theta+pi/2); 0];
        axisLims =  [-20 20; -20 20; -10 10];
        theta = linspace(pi/2,1.5*pi,N);
    case 2
        %Needle Type 2. Ideal Straight Needle
        structNeedleGeometry = struct('straightL',10,'kinkAngle', 0, 'radius',0,'arc',0);
        
        NeedleTransDes = struct('rot',[1 0 0 0]','trans', [0 0 -20]');
        omega = randn(1,3);
        omegap = omega/norm(omega);
        %Q = @(theta) [cos(theta/10) sin(theta/10)*omegap]';
        Q = @(theta)  [1 0 0 0];
        pos = @(theta) [0.0; 0.0; 1.5*theta]+[0; 0; -20];
        axisLims =  [-20 20; -20 20; -20 20];
        theta = linspace(0,10,N);
        
    case 3
        %Needle Type 3. non ideal curved Needle
        structNeedleGeometry = struct('straightL',4,'kinkAngle', pi/7, 'radius',10,'arc',pi);
        
        NeedleTransDes = struct('rot',[1 0 0 0]','trans', [0 0 0]');
        Q = @(theta) [cos(-theta/2) sin(-theta/2)*[0 0 1]]';
        pos = @(theta) -10*[cos(-theta+pi/2); sin(-theta+pi/2); 0];
        axisLims =  [-20 20; -20 20; -10 10];
        theta = linspace(pi/2,1.5*pi,N);
    case 4
        %Needle Type 4.  Ideal Straight Needle at an angle!!!
        structNeedleGeometry = struct('straightL',10,'kinkAngle', 0, 'radius',0,'arc',0);
        
        omegaG = [0 1 0];
        thetaG = 0.4;
        NeedleTransDes = struct('rot',[cos(thetaG) sin(thetaG)*omegaG]','trans', [0 0 -20]');
        omega = randn(1,3);
        omegap = omega/norm(omega);
        %Q = @(theta) [cos(theta/10) sin(theta/10)*omegap]';
        Q = @(theta)  [cos(thetaG) sin(thetaG)*omegaG]';
        zHat = Q2R(Q(0))*[0;0;1];
        pos = @(theta) theta*1.5*[zHat]-zHat*20;
        axisLims =  [-20 20; -20 20; -20 20];
        theta = linspace(0,10,N);
    case 5
        %Needle Type 5. straight needle, thin tissue.
        
        structNeedleGeometry = struct('straightL',10,'kinkAngle', 0, 'radius',0,'arc',0);
        
        omegaG = [0 1 0];
        thetaG = 0;
        NeedleTransDes = struct('rot',[cos(thetaG) sin(thetaG)*omegaG]','trans', [0 0 -15]');
        omega = randn(1,3);
        omegap = omega/norm(omega);
        %Q = @(theta) [cos(theta/10) sin(theta/10)*omegap]';
        Q = @(theta)  [cos(thetaG) sin(thetaG)*omegaG]';
        zHat = Q2R(Q(0))*[0;0;1];
        tCut = 4.5;
        pos = @(theta) theta*1.75*[0 0 1]'-15*[0 0 1]'+(theta>tCut)*((theta-tCut)*.25*[1.0;0;0]-(theta-tCut)*1.75*[0 0 1]');
        axisLims =  [-20 20; -20 20; -20 20];
        theta = linspace(0,10,N);
    case 6
        %Needle Type 6. Ideal straight needle, thin tissue.
        
        structNeedleGeometry = struct('straightL',10,'kinkAngle', 0, 'radius',0,'arc',0);
        
        omegaG = [0 1 0];
        thetaG = 0;
        NeedleTransDes = struct('rot',[cos(thetaG) sin(thetaG)*omegaG]','trans', [0 0 -15]');
        omega = randn(1,3);
        omegap = omega/norm(omega);
        %Q = @(theta) [cos(theta/10) sin(theta/10)*omegap]';
        Q = @(theta)  [cos(thetaG) sin(thetaG)*omegaG]';
        zHat = Q2R(Q(0))*[0;0;1];
        tCut = 4.5;
        pos = @(theta) theta*1.75*[0 0 1]'-15*[0 0 1]';
        axisLims =  [-20 20; -20 20; -20 20];
        theta = linspace(0,10,N);
        
        
        
end





%% Tissue Block.

switch n
    
    
    case 2
        structTissueData =struct('center',[0; 0; 0],'widthX',[10],'heightZ',[10],'depthY',[10]);
    case 4
        structTissueData =struct('center',[0; 0; 0],'widthX',[10],'heightZ',[10],'depthY',[10]);
    case 5
        
        structTissueData =struct('center',[0; 0; 0],'widthX',[10],'heightZ',[1],'depthY',[10]);
    case 6
        
        structTissueData =struct('center',[0; 0; 0],'widthX',[10],'heightZ',[1],'depthY',[10]);
        
        
    otherwise
        structTissueData =struct('center',[0; -10; 0],'widthX',[20],'heightZ',[10],'depthY',[10]);
        
        
end

%% Allocate the Needle force vectors
wrenchModeled_0 = zeros(6,N);
wrenchModeled_N = zeros(6,N);
wrenchFriction = zeros(6,N);
wrenchNorm = zeros(6,N);
wrenchCut = zeros(6,N);


%% Set up the force parameters
structMaterialParams.mus = 0*0.16;
structMaterialParams.muk = 0.00;
structMaterialParams.K   =  reshape(eye(3),9,1); %reshape(diag([.0087; .0087; .0087;]),9,1);
structMaterialParams.alpha = 0; %1.0622;
structMaterialParams.limit = 0; %0.5;




%needleFig = figure('position',[500,500,1000,500]);
needleFig = figure;



%% Begin Loop
time = linspace(0,10,N);
for i = 1:N
    NeedleTransDes.rot  = Q(theta(i));
    NeedleTransDes.trans  = pos(theta(i));
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
    
    NeedlePlot(needleFig,structTissueData,NeedlePts2Old,TissueStateOld,NeedlePts2,TissueState,axisLims);
    
    
    
    
    
    %Plot Needle.
    
    pause(.1);
    
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




