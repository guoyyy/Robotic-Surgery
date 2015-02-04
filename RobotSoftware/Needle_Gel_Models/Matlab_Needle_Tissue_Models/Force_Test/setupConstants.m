addpath ../Needle_Geometry
addpath ../../Matlab_Robot_Models/Common

global ln
global T_BackFront_mount
global yoffset
global xoffset
global zoffset

%% robot geometry constants
alphax=0*pi/180;
alphay=0*pi/180;

% global R_BbBs;
R_BbBs=ypr2R(0,alphay,alphax,'rad');
% global yoffset
% global xoffset
% global zoffset
yoffset=0;
xoffset=0;
zoffset=0;
% global ln
ln= 68.6308+65.19+161.86+18.00+14.91; % (26 July 2010)
% global ln1
% global ln2
ln1 = 123.142;
% ln2 = 173.228;
ln2=ln-ln1;
% global T_BackFront_mount
T_BackFront_mount = [ %July 22, 2010
    0.9996514523262   0.0239862420680  -0.0110287829636   -2.6398505898723
    -0.0238096717995   0.9995905085098   0.0158718249104  -45.1866546135221
    0.0114049722052  -0.0156037011200   0.9998132081146  197.5288079400060
    0                 0                 0                  1];

t1bias= 225*pi/180;                      % Home position for axis 1
t2bias= 315*pi/180;                      % Home position for axis 2
t3bias= 0;                               % Home position for axis 3
t4bias= 225*pi/180;                      % Home position for axis 4
t5bias= 315*pi/180;                      % Home position for axis 5


%% simulation constants
% global ts
ts=0.001;                                % Control sampling time [s]
% global fs
fs=1/ts; % Control frequency [Hz]
% global SaturationUpperLimit
SaturationUpperLimit= [2;2;2;10;10;0.2]; % Saturation limits for the AO
% global SaturationLowerLimit
SaturationLowerLimit=-[2;2;2;10;10;0.2]; % Saturation limits for the AO


%% Gear ratio and torque constants % Tm=GR*Ta
% Calibration values 29 July 2009
% global GR_Pitch
GR_Pitch= 0.118858697469487;
% global GR_Front
GR_Front=[0.161365565877988 0.161440644285910];
% global GR_Back
GR_Back= [0.165936000451198 0.165750373491147];

% global GearRatio
GearRatio= [GR_Front(1);... % Motor 1
    GR_Front(2);... % Motor 2
    GR_Pitch(1);... % Motor 3
    GR_Back(1);... % Motor 4
    GR_Back(2);... % Motor 5
    (1/16)*(15/60)]; % Motor 6 (Syringe) + 12-24 screw

% global TorqueConstantMotorRE25
TorqueConstantMotorRE25= 0.0235 ;         % Nm/A (From Maxon Datasheets)
% global TorqueConstantMotorRE16
TorqueConstantMotorRE16= 0.0160 ;         % Nm/A (From Maxon Datasheets)
% global TorqueConstantMotorRE30
TorqueConstantMotorRE30= 0.0139 ;         % Nm/A (From Maxon Datasheets)
% global TorqueConstantMotorRE10
TorqueConstantMotorRE10= 0.00267 ;        % Nm/A (From Maxon Datasheets)

% global TorqueConstant 
 TorqueConstant= [TorqueConstantMotorRE25;...
    TorqueConstantMotorRE25;...
    TorqueConstantMotorRE30;...
    TorqueConstantMotorRE25;...
    TorqueConstantMotorRE25;...
    TorqueConstantMotorRE10]./GearRatio;  % Nm/A (Motor Currents to Axes Torques)

%%Simulator PID controller constants

PID1=[0.75;0;0.02];   
% PID1=[03.0;0;0.0075];
PID2=[0.75;0;0.02];
% PID2=[03.0;0;0.0075];
PID3=[5.0;0;0.01];    
% PID3=[25.0;0;0.017];
PID4=[0.75;0;0.02]; 
% PID4=[05.0;0;0.0201825];
PID5=[0.75;0;0.02];
% PID5=[05.0;0;0.0201825];

% % %these are the PID constants from Ozkan's real robot controller code
% % PID1=[03.0;0;0.0075];    %just prop control 25 also works higher freqs
% % PID2=PID1;
% % PID3=[25.0;0;0.017];     % used during square ref
% % PID4=[05.0;0;0.0201825];
% % PID5=PID4;
% % PID6=[00.1;0;0.0005];    % 

%% TF num's and  den's for simulink simulation
load('SABiR_red2_TF.mat')
% [num1,den1]=tfdata(SABiR(1),'v');
% [num2,den2]=tfdata(SABiR(2),'v');
% [num3,den3]=tfdata(SABiR(3),'v');
% [num4,den4]=tfdata(SABiR(4),'v');
% [num5,den5]=tfdata(SABiR(5),'v');
% 
[num1,den1]=tfdata(SABiR_red2(1),'v');
[num2,den2]=tfdata(SABiR_red2(2),'v');
[num3,den3]=tfdata(SABiR_red2(3),'v');
[num4,den4]=tfdata(SABiR_red2(4),'v');
[num5,den5]=tfdata(SABiR_red2(5),'v');




% global Ref_t
% global ptarget



% %% set up gel parameters
% skinthickness1 = .5;
% A1 = [.003 .002 .0023];%parameters for gel type 1
% B1 = [.012 .008 .0092];%parameters for skin type 1
% 
% 
% 
% %make corners of gel block 1
% 
% block1Corners = [0 -240 360;
%     0 -180 360; 
%     0 -240 340; 
%     0 -180 340; 
%     -20 -240 360; 
%     -20 -180 360; 
%     -20 -240 340; 
%     -20 -180 340];
% % 
% 
% 
% 
% skinthickness2 = .1;
% A2 = [.06 .004 .0046]; %parameters for gel type 2
% B2 = [.18 .008 .0096]; %parameters for skin type 2
% 
% %make corners of gel block 2
% block2Corners = [-7 -220 350;
%     -7 -200 350;
%     -7 -220 345;
%     -7 -200 345;
%     -12 -220 350;
%     -12 -200 350;
%     -12 -220 345;
%     -12 -200 345];
% 
% 
% %make the blocks soft
% softnessfactor = 1e-5;
% A1 = A1 * softnessfactor;
% B1 = B1 * softnessfactor;
% A2 = A2 * softnessfactor;
% B2 = B2 * softnessfactor;

%% block parameters for new block/needle model

block1MinMax = [-20 0; -240 -180; 340 360];
block2MinMax = [-12 -7; -220 -200; 345 350]; 

block1MaterialParameters = [2 1 2 1] * 1e-6;
block2MaterialParameters = 2*block1MaterialParameters;
