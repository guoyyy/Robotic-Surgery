function [outdata]=API_ChangeDirection(ptarget,rtarget,pcurrent,rcurrent,ts)
warning off
rtarget=rtarget';
rcurrent=rcurrent';

%% if we must use globals, they should all be set up in one place
%setupGlobals
setupGoalsAndPara();
%% debug level - for now: 0 off; 1 on
debuglevel=0;

%% set up gel blocks
%make corners of gel block 1
blockCorners(:,:,1) = [
    0 -240 360;
    0 -180 360;
    0 -240 340;
    0 -180 340;
    -20 -240 360;
    -20 -180 360;
    -20 -240 340;
    -20 -180 340;
    ];

%make corners of gel block 2
blockCorners(:,:,2) = [
    -7 -220 350;
    -7 -200 350;
    -7 -220 345;
    -7 -200 345;
    -12 -220 350;
    -12 -200 350;
    -12 -220 345;
    -12 -200 345;
    ];


%% uniformly sample the workspace
% n=1;                                                   
% %%%%%%% increase this to sample more densely                                               
% %%%%%%% note: 15^5= 759,375 samples. at 5 MB per trajectory,
% %%%%%%% n=15 will require 3,796,875 MB = 3,707 GB of storage
% jointranges = [linspace(3.14,4.7,n);
%                linspace(4.7,6.28,n);
%                linspace(-0.2,0.8,n);
%                linspace(3.14,4.7,n);
%                linspace(4.7,6.28,n)];
% 
% ntargets = n^5;
% 
% counter = [1;1;1;1;1];
% 
%    
%     %get the end point from the kinematics
%     [pend,rend,front,back] = SABiRFK([angle1,angle2,angle3,angle4,angle5]);
    %set up trajectory 
%     ptarget = phome;
%     rtarget = rend;
   % insertdist = norm(phome-pend);
    
    %begin simulation
    
    %% set up simulation
 
ControlReadyTime = 0;

%% Generate Reference Signal


maxa=100.00;
%[time1,SABiRx,Rneedle]=PosRef3D(phome,ptarget-phome,maxahome,rhome,rtarget,ControlReadyTime,ts);

[time1,SABiRx,Rneedle]=PosRef3D(pcurrent,ptarget-pcurrent,maxa/4,rcurrent,rtarget,0,ts);
TrackingTime=length(time1)*ts;          % Also equal to = time5(end)-ControlReadyTime+ts
ParkMotionTime=TrackingTime+ControlReadyTime;
TotalTime=ParkMotionTime;

fprintf('total trajectory time  is %.2fs\n', TotalTime);
    
    
    %% prepare simulink model
    sys = 'sabir12simPD';
    open_system(sys);
    
    % Create a new configuration set
    cset1 = Simulink.ConfigSet;
    % Set a non-default stop time
    %set_param (cset1, 'StopTime', '1')
    set_param(cset1, 'StopTime', sprintf('%f',TotalTime));
    assignin('base', 'cset1', cset1);
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
   
    outdata = runSimulation(ptarget,rcurrent,insertdist,blockCorners,debuglevel,time1,TotalTime,SABiRx,Rneedle,sys);
    %outdata = Modelinitial_gel_block_rigid_needle(ptarget,rtarget,insertdist,debuglevel);
    assignin('base', 'outdata', outdata);
    outdata=outdata';
    %% save data
%     save(sprintf('gel_experiment_outdata%d.mat',targetnum), 'outdata', 'ptarget','rtarget','insertdist');
