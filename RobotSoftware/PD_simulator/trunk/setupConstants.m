close all

%path setup

%uncomment this to use the version from the git repository
% path(path,'~/robosurgery-2/Matlab_Needle_Tissue_Models/Needle_Geometry');
% path(path,'~/robosurgery-2/Matlab_Needle_Tissue_Models/Needle_Forces');
% path(path,'~/robosurgery-2/Matlab_Robot_Models/Common');
% path(path,'~/robosurgery-2/Matlab_Needle_Tissue_Models/Common');

%this is the version from the SVN repository
path(path,' ../../Needle_Gel_Models/Matlab_Needle_Tissue_Models/Needle_Geometry');
path(path,' ../../Needle_Gel_Models/Matlab_Needle_Tissue_Models/Needle_Forces');
path(path,' ../../Needle_Gel_Models/Matlab_Robot_Models/Common');
path(path,' ../../Needle_Gel_Models/Matlab_Needle_Tissue_Models/Common');

global ln
global T_BackFront_mount
global yoffset
global xoffset
global zoffset
global PosRef3DInstrumentedVariables

%% block parameters for new block/needle model

block1MinMax = [-20 0; -240 -180; 340 360];
block2MinMax = [-12 -7; -220 -200; 345 350]; 

materialParameters1.mus   = 1e-4 * 0.08;
materialParameters1.muk   = 1e-4 * 0.08;
materialParameters1.K     = 1e-3 * reshape(diag([.0087; .0087; .012;])/2,9,1);
materialParameters1.alpha = 1e-4 * 1.0622;
materialParameters1.limit = 1e-4 * 0.5;

% Material_Parameters = Simulink.Bus.createObject('sabir12simPD_8', ['materialStructBlk1']);

%be aware that if the second block is inside the first block, the forces
%produced by it are additive. In other words, to make the second block
%twice as stiff as the first block, make its stiffness equal to the first,
%and to make it less stiff, make its stiffness negative.
materialParameters2 = materialParameters1;  

materialParameters1x=[materialParameters1.mus;materialParameters1.muk;materialParameters1.K;materialParameters1.alpha;materialParameters1.limit];
materialParameters2x=[materialParameters2.mus;materialParameters2.muk;materialParameters2.K;materialParameters2.alpha;materialParameters2.limit];

%cut-off frequency of the filter at the environment force output
f_envfilt=30; %Hz

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
ts =0.010;  % Control sampling time for outer (Cartesian) control loop [s] 
ts2=0.0005; % Sampling time for the inner (joint space) control loop [s]
% global fs
fs =1/ts; % Control frequency [Hz]
fs2=1/ts2; % Control frequency for the inner control loop [Hz]
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

%% Maximum acceleration for trajectory generation
maxAcc(1)=25; % mm/sec^2
maxAcc(2)=25; % mm/sec^2
maxAcc(3)=10; % radians /sec^2

%% Simulator PID controller constants

%original constants
PID1=[03.0;0;0.0075];
PID2=[03.0;0;0.0075];
PID3=[25.0;0;0.017];
PID4=[05.0;0;0.0201825];
PID5=[05.0;0;0.0201825];

% hand-tuned constants, DO NOT WORK
% PID1=[0.25;0;0.025];
% PID2=[0.25;0;0.025];
% PID3=[1.0;0;0.1];
% PID4=[0.75;0;0.05];
% PID5=[0.75;0;0.05];

% % %these are the PID constants from Ozkan's real robot controller code
% % PID1=[03.0;0;0.0075];    %just prop control 25 also works higher freqs
% % PID2=PID1;
% % PID3=[25.0;0;0.017];     % used during square ref
% % PID4=[05.0;0;0.0201825];
% % PID5=PID4;
% % PID6=[00.1;0;0.0005];    % 

%% TF num's and  den's for simulink simulation
%load('SABiR_red2_TF.mat')  
load('SABiR_TF.mat')


%note that if you want to go from the non-reduced to reduced transfer
%functions and vice versa, you must change the dimensionality of the 
%estimators in the Simulink model


%SABiR = SABiR_red2; %comment this out when using the full transfer functions


[num1,den1]=tfdata(SABiR(1),'v');
[num2,den2]=tfdata(SABiR(2),'v');
[num3,den3]=tfdata(SABiR(3),'v');
[num4,den4]=tfdata(SABiR(4),'v');
[num5,den5]=tfdata(SABiR(5),'v');

%% set up Cartesian space controller
% Kpos=[1e-6;1e-6;10e-6];
% Kd_pos=[0.1;0.1;0.1];
%Kpos=[0.05; 0.1; 0.05];
%Kpos=[10e-3; 10e-3; 5e-4]; %worked with 1/s plant and 1kHz control and JT
Kpos=[12.5; 12.5; 12.5]; %worked with 1/s plant and 1kHz control and JT
Kd_pos=[0.0;0.0;0.0];
% Kdir=[1e-3;1e-3];
% Kd_dir=[0.100;0.100];
%Kdir=[25;17];
%Kdir=[100;50]; %worked with 1/s plant and 1kHz control and JT
Kdir=[12.5; 12.5]; %worked with 1/s plant and 1kHz control and JT
Kd_dir=[0;0];

%% set up pole placement controller
% num1_r = num1; %the reduced tf is the real tf, in this case. it won't always be that way.
% den1_r = den1;
% num2_r = num2;
% den2_r = den2;
% num3_r = num3;
% den3_r = den3;
% num4_r = num4;
% den4_r = den4;
% num5_r = num5;
% den5_r = den5;
% 
% sys1_r = tf(num1_r, den1_r);
% sys2_r = tf(num2_r, den2_r);
% sys3_r = tf(num3_r, den3_r);
% sys4_r = tf(num4_r, den4_r);
% sys5_r = tf(num5_r, den5_r);

sys1d_r = c2d(SABiR(1),ts2,'zoh');      %Conversion of truncated continuous-time model to discrete time
sys2d_r = c2d(SABiR(2),ts2,'zoh');      %Conversion of truncated continuous-time model to discrete time
sys3d_r = c2d(SABiR(3),ts2,'zoh');      %Conversion of truncated continuous-time model to discrete time
sys4d_r = c2d(SABiR(4),ts2,'zoh');      %Conversion of truncated continuous-time model to discrete time
sys5d_r = c2d(SABiR(5),ts2,'zoh');      %Conversion of truncated continuous-time model to discrete time

[act_Phi_axis5, act_Gamma_axis5, act_H_axis5, J5_r] = ssdata(sys5d_r);
[act_Phi_axis4, act_Gamma_axis4, act_H_axis4, J4_r] = ssdata(sys4d_r);
[act_Phi_axis3, act_Gamma_axis3, act_H_axis3, J3_r] = ssdata(sys3d_r);
[act_Phi_axis2, act_Gamma_axis2, act_H_axis2, J2_r] = ssdata(sys2d_r);
[act_Phi_axis1, act_Gamma_axis1, act_H_axis1, J1_r] = ssdata(sys1d_r);


%% estimator parameters
% Observer Poles
Pole4Lpr_Axis1=  floor(exp(-2*pi*291*ts2)*100)/100; % 0.8; %2kHz %0.4; %1kHz  
Pole4Lpr_Axis2 = floor(exp(-2*pi*291*ts2)*100)/100; % 0.8; %2kHz %0.4; %1kHz
Pole4Lpr_Axis3 = floor(exp(-2*pi*291*ts2)*100)/100; % 0.8; %2kHz %0.4; %1kHz
Pole4Lpr_Axis4 = floor(exp(-2*pi*291*ts2)*100)/100; % 0.8; %2kHz %0.4; %1kHz
Pole4Lpr_Axis5 = floor(exp(-2*pi*291*ts2)*100)/100; % 0.8; %2kHz %0.4; %1kHz
% Observer with Disturbance Estimator Poles
Pole4LprD_Axis1=  floor(exp(-2*pi*36*ts2)*100)/100;   
Pole4LprD_Axis2 = floor(exp(-2*pi*36*ts2)*100)/100; 
Pole4LprD_Axis3 = floor(exp(-2*pi*36*ts2)*100)/100; 
Pole4LprD_Axis4 = floor(exp(-2*pi*36*ts2)*100)/100; 
Pole4LprD_Axis5 = floor(exp(-2*pi*36*ts2)*100)/100; 


est_Phi_axis1=[act_Phi_axis1, act_Gamma_axis1; zeros(1,size(act_Phi_axis1,2)), 1];
est_Gamma_axis1=[act_Gamma_axis1; 0];
est_H_axis1=[act_H_axis1, 0];

est_Phi_axis2=[act_Phi_axis2, act_Gamma_axis2; zeros(1,size(act_Phi_axis2,2)), 1];
est_Gamma_axis2=[act_Gamma_axis2; 0];
est_H_axis2=[act_H_axis2, 0];

est_Phi_axis3=[act_Phi_axis3, act_Gamma_axis3; zeros(1,size(act_Phi_axis3,2)), 1];
est_Gamma_axis3=[act_Gamma_axis3; 0];
est_H_axis3=[act_H_axis3, 0];

est_Phi_axis4=[act_Phi_axis4, act_Gamma_axis4; zeros(1,size(act_Phi_axis4,2)), 1];
est_Gamma_axis4=[act_Gamma_axis4; 0];
est_H_axis4=[act_H_axis4, 0];

est_Phi_axis5=[act_Phi_axis5, act_Gamma_axis5; zeros(1,size(act_Phi_axis5,2)), 1];
est_Gamma_axis5=[act_Gamma_axis5; 0];
est_H_axis5=[act_H_axis5, 0];


poles_Lpr1 = ones(length(est_Phi_axis1),1)*Pole4LprD_Axis1;
est_Lp_axis1 = acker(est_Phi_axis1', est_H_axis1', poles_Lpr1)';
est_Lc_axis1 = est_Phi_axis1 \ est_Lp_axis1;
poles_Lpr1 = ones(length(est_Phi_axis1),1)*Pole4Lpr_Axis1;
act_Lc_axis1 = act_Phi_axis1 \ acker(act_Phi_axis1', act_H_axis1', poles_Lpr1(1:length(act_Phi_axis1)) )';

poles_Lpr2 = ones(length(est_Phi_axis2),1)*Pole4LprD_Axis2;
est_Lp_axis2 = acker(est_Phi_axis2', est_H_axis2', poles_Lpr2)';
est_Lc_axis2 = est_Phi_axis2 \ est_Lp_axis2;
poles_Lpr2 = ones(length(est_Phi_axis2),1)*Pole4Lpr_Axis2;
act_Lc_axis2 = act_Phi_axis2 \ acker(act_Phi_axis2', act_H_axis2', poles_Lpr2(1:length(act_Phi_axis2)) )';

poles_Lpr3 = ones(length(est_Phi_axis3),1)*Pole4LprD_Axis3;
est_Lp_axis3 = acker(est_Phi_axis3', est_H_axis3', poles_Lpr3)';
est_Lc_axis3 = est_Phi_axis3 \ est_Lp_axis3;
poles_Lpr3 = ones(length(est_Phi_axis3),1)*Pole4Lpr_Axis3;
act_Lc_axis3 = act_Phi_axis3 \ acker(act_Phi_axis3', act_H_axis3', poles_Lpr3(1:length(act_Phi_axis3)) )';

poles_Lpr4 = ones(length(est_Phi_axis4),1)*Pole4LprD_Axis4;
est_Lp_axis4 = acker(est_Phi_axis4', est_H_axis4', poles_Lpr4)';
est_Lc_axis4 = est_Phi_axis4 \ est_Lp_axis4;
poles_Lpr4 = ones(length(est_Phi_axis4),1)*Pole4Lpr_Axis4;
act_Lc_axis4 = act_Phi_axis4 \ acker(act_Phi_axis4', act_H_axis4', poles_Lpr4(1:length(act_Phi_axis4)) )';

poles_Lpr5 = ones(length(est_Phi_axis5),1)*Pole4LprD_Axis5;
est_Lp_axis5 = acker(est_Phi_axis5', est_H_axis5', poles_Lpr5)';
est_Lc_axis5 = est_Phi_axis5 \ est_Lp_axis5;
poles_Lpr5 = ones(length(est_Phi_axis5),1)*Pole4Lpr_Axis5;
act_Lc_axis5 = act_Phi_axis5 \ acker(act_Phi_axis5', act_H_axis5', poles_Lpr5(1:length(act_Phi_axis5)) )';

%% K and Nbar controller parameters
Pole4Kr_Axis1 = floor(exp(-2*pi*36*ts2)*100)/100; %0.89 %2kHz %0.8 %1kHz
Pole4Kr_Axis2 = floor(exp(-2*pi*36*ts2)*100)/100; %0.89 %2kHz %0.8 %1kHz
Pole4Kr_Axis3 = floor(exp(-2*pi*36*ts2)*100)/100; %0.89 %2kHz %0.8 %1kHz
Pole4Kr_Axis4 = floor(exp(-2*pi*36*ts2)*100)/100; %0.89 %2kHz %0.8 %1kHz
Pole4Kr_Axis5 = floor(exp(-2*pi*36*ts2)*100)/100; %0.89 %2kHz %0.8 %1kHz

ControlMode=0; %0 for position, 1 for velocity

poles_Kc1 = ones(length(act_Phi_axis1),1)*Pole4Kr_Axis1;
zeros_system1_reduced=zero(sys1d_r);
poles_system1_reduced=pole(sys1d_r);
poles_Kc1(1:2)=zeros_system1_reduced(4:5);
poles_Kc1(3:4)=poles_system1_reduced(5:6);
if (ControlMode==1), 
    poles_Kc1(end)=1.0;
end
K_axis1 = acker(act_Phi_axis1, act_Gamma_axis1, poles_Kc1);
n_axis1=size(act_Phi_axis1,1);
if (ControlMode==1), 
    cCLTF=tf(ss(act_Phi_axis1-act_Gamma_axis1*K_axis1,act_Gamma_axis1,act_H_axis1,0,ts2));
    [cnum,cden]=tfdata(minreal(tf([1 -1],[1],ts2)*cCLTF),'v');
    Nbar_axis1 = ts2/(sum(cnum)/sum(cden));
else
    Nxu_axis1=[act_Phi_axis1-eye(n_axis1), act_Gamma_axis1; act_H_axis1, 0] \ [zeros(n_axis1,1); 1];
    Nx_axis1=Nxu_axis1(1:n_axis1,:);
    Nu_axis1=Nxu_axis1(n_axis1+1,:);
    Nbar_axis1 = Nu_axis1 + K_axis1 * Nx_axis1;
end


poles_Kc2 = ones(length(act_Phi_axis2),1)*Pole4Kr_Axis2;
zeros_system2_reduced=zero(sys2d_r);
poles_system2_reduced=pole(sys2d_r);
poles_Kc2(1:2)=zeros_system2_reduced(2:3);%[0.9351+0.2501i; 0.9351-0.2501i];%zeros_system2_reduced(3:4);
poles_Kc2(3:4)=poles_system2_reduced(5:6);
if (ControlMode==1), 
    poles_Kc2(end)=1.0;
end
K_axis2 = acker(act_Phi_axis2, act_Gamma_axis2, poles_Kc2);
n_axis2=size(act_Phi_axis2,1);
if (ControlMode==1), 
    cCLTF=tf(ss(act_Phi_axis2-act_Gamma_axis2*K_axis2,act_Gamma_axis2,act_H_axis2,0,ts2));
    [cnum,cden]=tfdata(minreal(tf([1 -1],[1],ts2)*cCLTF),'v');
    Nbar_axis2 = ts2/(sum(cnum)/sum(cden));
else
    Nxu_axis2=[act_Phi_axis2-eye(n_axis2), act_Gamma_axis2; act_H_axis2, 0] \ [zeros(n_axis2,1); 1];
    Nx_axis2=Nxu_axis2(1:n_axis2,:);
    Nu_axis2=Nxu_axis2(n_axis2+1,:);
    Nbar_axis2 = Nu_axis2 + K_axis2 * Nx_axis2;
end



poles_Kc3 = ones(length(act_Phi_axis3),1)*Pole4Kr_Axis3;
zeros_system3_reduced=zero(sys3d_r);
poles_system3_reduced=pole(sys3d_r);
poles_Kc3(1:2)=zeros_system3_reduced(4:5);
if (ControlMode==1), 
    poles_Kc3(end)=1.0;
end
K_axis3 = acker(act_Phi_axis3, act_Gamma_axis3, poles_Kc3);
n_axis3=size(act_Phi_axis3,1);
if (ControlMode==1), 
    cCLTF=tf(ss(act_Phi_axis3-act_Gamma_axis3*K_axis3,act_Gamma_axis3,act_H_axis3,0,ts2));
    [cnum,cden]=tfdata(minreal(tf([1 -1],[1],ts2)*cCLTF),'v');
    Nbar_axis3 = ts2/(sum(cnum)/sum(cden));
else
    Nxu_axis3=[act_Phi_axis3-eye(n_axis3), act_Gamma_axis3; act_H_axis3, 0] \ [zeros(n_axis3,1); 1];
    Nx_axis3=Nxu_axis3(1:n_axis3,:);
    Nu_axis3=Nxu_axis3(n_axis3+1,:);
    Nbar_axis3 = Nu_axis3 + K_axis3 * Nx_axis3;
end



poles_Kc4 = ones(length(act_Phi_axis4),1)*Pole4Kr_Axis4;
zeros_system4_reduced=zero(sys4d_r);
poles_system4_reduced=pole(sys4d_r);
poles_Kc4(1:2)=zeros_system4_reduced(2:3);
poles_Kc4(3)=poles_system4_reduced(6);
poles_Kc4(4)=zeros_system4_reduced(4);
if (ControlMode==1), 
    poles_Kc4(end)=1.0;
end
K_axis4 = acker(act_Phi_axis4, act_Gamma_axis4, poles_Kc4);
n_axis4=size(act_Phi_axis4,1);
if (ControlMode==1), 
    cCLTF=tf(ss(act_Phi_axis4-act_Gamma_axis4*K_axis4,act_Gamma_axis4,act_H_axis4,0,ts2));
    [cnum,cden]=tfdata(minreal(tf([1 -1],[1],ts2)*cCLTF),'v');
    Nbar_axis4 = ts2/(sum(cnum)/sum(cden));
else
    Nxu_axis4=[act_Phi_axis4-eye(n_axis4), act_Gamma_axis4; act_H_axis4, 0] \ [zeros(n_axis4,1); 1];
    Nx_axis4=Nxu_axis4(1:n_axis1,:);
    Nu_axis4=Nxu_axis4(n_axis1+1,:);
    Nbar_axis4 = Nu_axis4 + K_axis4 * Nx_axis4;
end


poles_Kc5 = ones(length(act_Phi_axis5),1)*Pole4Kr_Axis5;
zeros_system5_reduced=zero(sys5d_r);
poles_system5_reduced=pole(sys5d_r);
poles_Kc5(1:2)=zeros_system5_reduced(2:3);
poles_Kc5(3)=poles_system5_reduced(6);
poles_Kc5(4)=zeros_system5_reduced(4);
if (ControlMode==1), 
    poles_Kc5(end)=1.0;
end
K_axis5 = acker(act_Phi_axis5, act_Gamma_axis5, poles_Kc5);% %test
n_axis5=size(act_Phi_axis5,1);
if (ControlMode==1), 
    cCLTF=tf(ss(act_Phi_axis5-act_Gamma_axis5*K_axis5,act_Gamma_axis5,act_H_axis5,0,ts2));
    [cnum,cden]=tfdata(minreal(tf([1 -1],[1],ts2)*cCLTF),'v');
    Nbar_axis5 = ts2/(sum(cnum)/sum(cden));
else
    Nxu_axis5=[act_Phi_axis5-eye(n_axis5), act_Gamma_axis5; act_H_axis5, 0] \ [zeros(n_axis5,1); 1];
    Nx_axis5=Nxu_axis5(1:n_axis5,:);
    Nu_axis5=Nxu_axis5(n_axis5+1,:);
    Nbar_axis5 = Nu_axis5 + K_axis5 * Nx_axis5;
end




% naxis = 1;
% Kr = eval(strcat('K_axis',num2str(naxis)));
% Phi_r = eval(strcat('act_Phi_axis',num2str(naxis)));
% Gam_r = eval(strcat('act_Gamma_axis',num2str(naxis)));
% H_r = eval(strcat('act_H_axis',num2str(naxis)));
% Nbar = eval(strcat('Nbar_axis',num2str(naxis)));
% 
% poles_PP_system=pole(tf(ss(Phi_r-Gam_r*Kr,[1;0;0;0;0;0],H_r,0,ts2)));
% zeros_PP_system=zero(tf(ss(Phi_r-Gam_r*Kr,[1;0;0;0;0;0],H_r,0,ts2)));
% sysd_r_PP=ss(Phi_r-Gam_r*Kr,Nbar*Gam_r,H_r,0,ts2);
% figure
% pzmap(sysd_r_PP)
% figure
% step(sysd_r_PP)
% figure
% impulse(sysd_r_PP)
% 
% est_Gamma_axisN = eval(strcat('est_Gamma_axis',num2str(naxis)));
% est_Phi_axisN = eval(strcat('est_Phi_axis',num2str(naxis)));
% JN_r = eval(strcat('J',num2str(naxis), '_r'));
% est_H_axisN = eval(strcat('est_H_axis',num2str(naxis)));
% est_Lc_axisN = eval(strcat('est_Lc_axis',num2str(naxis)));
% Nbar_axisN = eval(strcat('Nbar_axis',num2str(naxis)));
% K_axisN = eval(strcat('K_axis',num2str(naxis)));
% numN = eval(strcat('num',num2str(naxis)));
% denN = eval(strcat('den',num2str(naxis)));
% torquesat_upper = SaturationUpperLimit(naxis)./(1./TorqueConstant(naxis));
% torquesat_lower = SaturationLowerLimit(naxis)./(1./TorqueConstant(naxis));
% 
% sys = 'poleplacement_newtest';
% open_system(sys);
% sim(sys);



