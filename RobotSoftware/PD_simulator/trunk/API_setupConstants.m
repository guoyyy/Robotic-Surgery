function API_setupConstants()
path(path,' ../../Needle_Gel_Models/Matlab_Needle_Tissue_Models/Needle_Geometry');
path(path,' ../../Needle_Gel_Models/Matlab_Needle_Tissue_Models/Needle_Forces');
path(path,' ../../Needle_Gel_Models/Matlab_Robot_Models/Common');
path(path,' ../../Needle_Gel_Models/Matlab_Needle_Tissue_Models/Common');

global ln
global T_BackFront_mount
global yoffset
global xoffset
global zoffset

%% block parameters for new block/needle model

block1MinMax = [-20 0; -240 -180; 340 360];
block2MinMax = [-12 -7; -220 -200; 345 350]; 

assignin('base', 'block1MinMax', block1MinMax);
assignin('base', 'block2MinMax', block2MinMax);
%note: I have tested the model with the parameters multiplied by up to
%1000, and it works fine.
materialParameters1.mus   = 1.0 * 0.08;
materialParameters1.muk   = 1.0 * 0.08;
materialParameters1.K     = 1.0 * reshape(diag([.0087; .0087; .012;])/2,9,1);
materialParameters1.alpha = 1.0 * 1.0622;
materialParameters1.limit = 1.0 * 0.5;

assignin('base', 'materialParameters1.mus', materialParameters1.mus);
assignin('base', 'materialParameters1.muk', materialParameters1.muk);
assignin('base', 'materialParameters1.K',materialParameters1.K );
assignin('base', 'materialParameters1.alpha', materialParameters1.alpha);
assignin('base', 'materialParameters1.limit', materialParameters1.limit);
% Material_Parameters = Simulink.Bus.createObject('sabir12simPD_8', ['materialStructBlk1']);

%be aware that if the second block is inside the first block, the forces
%produced by it are additive. In other words, to make the second block
%twice as stiff as the first block, make its stiffness equal to the first,
%and to make it less stiff, make its stiffness negative.
materialParameters2 = materialParameters1;  
assignin('base', 'materialParameters2', materialParameters2);

%% robot geometry constants
alphax=0*pi/180;
alphay=0*pi/180;

% global R_BbBs;
R_BbBs=ypr2R(0,alphay,alphax,'rad');
assignin('base', 'R_BbBs', R_BbBs);
% global yoffset
% global xoffset
% global zoffset
yoffset=0;
xoffset=0;
zoffset=0;
assignin('base','yoffset',yoffset);
assignin('base','xoffset',xoffset);
assignin('base','zoffset',zoffset);
% global ln
ln= 68.6308+65.19+161.86+18.00+14.91; % (26 July 2010)
% global ln1
% global ln2
ln1 = 123.142;
% ln2 = 173.228;
ln2=ln-ln1;
assignin('base','ln',ln);
assignin('base','ln1',ln1);
assignin('base','ln2',ln2);

T_BackFront_mount = [ 
    0.9996514523262   0.0239862420680  -0.0110287829636   -2.6398505898723
    -0.0238096717995   0.9995905085098   0.0158718249104  -45.1866546135221
    0.0114049722052  -0.0156037011200   0.9998132081146  197.5288079400060
    0                 0                 0                  1];
assignin('base','T_BackFront_mount',T_BackFront_mount);

%**
% t1bias= 225*pi/180;                      % Home position for axis 1
% t2bias= 315*pi/180;                      % Home position for axis 2
% t3bias= 0;                               % Home position for axis 3
% t4bias= 225*pi/180;                      % Home position for axis 4
% t5bias= 315*pi/180;                      % Home position for axis 5
% assignin('base','t1bias',t1bias);
% assignin('base','t2bias',t2bias);
% assignin('base','t3bias',t3bias);
% assignin('base','t4bias',t4bias);
% assignin('base','t5bias',t5bias);

%% simulation constants
global ts
ts=0.010;  % Control sampling time [s]
ts2=0.0005;
% global fs
fs=1/ts; % Control frequency [Hz]
fs2=1/ts2;
% global SaturationUpperLimit
SaturationUpperLimit= [2;2;2;10;10;0.2]; % Saturation limits for the AO
% global SaturationLowerLimit
SaturationLowerLimit=-[2;2;2;10;10;0.2]; % Saturation limits for the AO
assignin('base','ts',ts);
assignin('base','fs',fs);
assignin('base','ts2',ts2);
assignin('base','fs2',fs2);
assignin('base','SaturationUpperLimit',SaturationUpperLimit);
assignin('base','SaturationLowerLimit',SaturationLowerLimit);


%% Gear ratio and torque constants % Tm=GR*Ta
% Calibration values 29 July 2009
% global GR_Pitch
GR_Pitch= 0.118858697469487;
% global GR_Front
GR_Front=[0.161365565877988 0.161440644285910];
% global GR_Back
GR_Back= [0.165936000451198 0.165750373491147];
assignin('base','GR_Pitch',GR_Pitch);
assignin('base','GR_Front',GR_Front);
assignin('base','GR_Back',GR_Back);
% global GearRatio
GearRatio= [GR_Front(1);... % Motor 1
    GR_Front(2);... % Motor 2
    GR_Pitch(1);... % Motor 3
    GR_Back(1);... % Motor 4
    GR_Back(2);... % Motor 5
    (1/16)*(15/60)]; % Motor 6 (Syringe) + 12-24 screw
assignin('base','GearRatio',GearRatio);
% global TorqueConstantMotorRE25
TorqueConstantMotorRE25= 0.0235 ;         % Nm/A (From Maxon Datasheets)
% global TorqueConstantMotorRE16
TorqueConstantMotorRE16= 0.0160 ;         % Nm/A (From Maxon Datasheets)
% global TorqueConstantMotorRE30
TorqueConstantMotorRE30= 0.0139 ;         % Nm/A (From Maxon Datasheets)
% global TorqueConstantMotorRE10
TorqueConstantMotorRE10= 0.00267 ;        % Nm/A (From Maxon Datasheets)

assignin('base','TorqueConstantMotorRE25',TorqueConstantMotorRE25);
assignin('base','TorqueConstantMotorRE16',TorqueConstantMotorRE16);
assignin('base','TorqueConstantMotorRE30',TorqueConstantMotorRE30);
assignin('base','TorqueConstantMotorRE10',TorqueConstantMotorRE10);

% global TorqueConstant
TorqueConstant= [TorqueConstantMotorRE25;...
    TorqueConstantMotorRE25;...
    TorqueConstantMotorRE30;...
    TorqueConstantMotorRE25;...
    TorqueConstantMotorRE25;...
    TorqueConstantMotorRE10]./GearRatio;  % Nm/A (Motor Currents to Axes Torques)

assignin('base','TorqueConstant',TorqueConstant);
%%Simulator PID controller constants

PID1=[03.0;0;0.0075];
PID2=[03.0;0;0.0075];
PID3=[25.0;0;0.017];
PID4=[05.0;0;0.0201825];
PID5=[05.0;0;0.0201825];
assignin('base', 'PID1', PID1);
assignin('base', 'PID2', PID2);
assignin('base', 'PID3', PID3);
assignin('base', 'PID4', PID4);
assignin('base', 'PID5', PID5);

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

[num1,den1]=tfdata(SABiR(1),'v');
[num2,den2]=tfdata(SABiR(2),'v');
[num3,den3]=tfdata(SABiR(3),'v');
[num4,den4]=tfdata(SABiR(4),'v');
[num5,den5]=tfdata(SABiR(5),'v');

assignin('base', 'num1', num1);assignin('base', 'den1', den1);
assignin('base', 'num2', num2);assignin('base', 'den2', den2);
assignin('base', 'num3', num3);assignin('base', 'den3', den3);
assignin('base', 'num4', num4);assignin('base', 'den4', den4);
assignin('base', 'num5', num5);assignin('base', 'den5', den5);


% global Ref_t
% global ptarget

%% robot home position globals
global phome
global rhome
phome =[-10.7505,-206.2838,330.8692]; %needle position at the robot's home position
rhome = [-0.0327;0.0020;0.9995];
assignin('base', 'phome', phome);
assignin('base', 'rhome', rhome);

%%
%% set up Cartesian space controller
% Kpos=[1e-6;1e-6;10e-6];
% Kd_pos=[0.1;0.1;0.1];
%Kpos=[0.05; 0.1; 0.05];
%Kpos=[10e-3; 10e-3; 5e-4]; %worked with 1/s plant and 1kHz control and JT
Kpos=[12.5; 12.5; 12.5]; %worked with 1/s plant and 1kHz control and JT
Kd_pos=[0.0;0.0;0.0];
assignin('base', 'Kpos', Kpos);
assignin('base', 'Kd_pos', Kd_pos);
% Kdir=[1e-3;1e-3];
% Kd_dir=[0.100;0.100];
%Kdir=[25;17];
%Kdir=[100;50]; %worked with 1/s plant and 1kHz control and JT
Kdir=[12.5; 12.5]; %worked with 1/s plant and 1kHz control and JT
Kd_dir=[0;0];
assignin('base', 'Kdir', Kdir);
assignin('base', 'Kd_dir',Kd_dir);
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

est_Phi_axis1=[act_Phi_axis1, act_Gamma_axis1; zeros(1,size(act_Phi_axis1,2)), 1];
est_Gamma_axis1=[act_Gamma_axis1; 0];
est_H_axis1=[act_H_axis1, 0];

assignin('base', 'est_Gamma_axis1',est_Gamma_axis1);

est_Phi_axis2=[act_Phi_axis2, act_Gamma_axis2; zeros(1,size(act_Phi_axis2,2)), 1];
est_Gamma_axis2=[act_Gamma_axis2; 0];
est_H_axis2=[act_H_axis2, 0];

assignin('base', 'est_Gamma_axis2',est_Gamma_axis2);

est_Phi_axis3=[act_Phi_axis3, act_Gamma_axis3; zeros(1,size(act_Phi_axis3,2)), 1];
est_Gamma_axis3=[act_Gamma_axis3; 0];
est_H_axis3=[act_H_axis3, 0];

est_Phi_axis4=[act_Phi_axis4, act_Gamma_axis4; zeros(1,size(act_Phi_axis4,2)), 1];
est_Gamma_axis4=[act_Gamma_axis4; 0];
est_H_axis4=[act_H_axis4, 0];

est_Phi_axis5=[act_Phi_axis5, act_Gamma_axis5; zeros(1,size(act_Phi_axis5,2)), 1];
est_Gamma_axis5=[act_Gamma_axis5; 0];
est_H_axis5=[act_H_axis5, 0];


poles_Lpr1 = ones(length(est_Phi_axis1),1)*Pole4Lpr_Axis1;
est_Lp_axis1 = acker(est_Phi_axis1', est_H_axis1', poles_Lpr1)';
est_Lc_axis1 = est_Phi_axis1 \ est_Lp_axis1;
act_Lc_axis1 = act_Phi_axis1 \ acker(act_Phi_axis1', act_H_axis1', poles_Lpr1(1:length(act_Phi_axis1)) )';

poles_Lpr2 = ones(length(est_Phi_axis2),1)*Pole4Lpr_Axis2;
est_Lp_axis2 = acker(est_Phi_axis2', est_H_axis2', poles_Lpr2)';
est_Lc_axis2 = est_Phi_axis2 \ est_Lp_axis2;
act_Lc_axis2 = act_Phi_axis2 \ acker(act_Phi_axis2', act_H_axis2', poles_Lpr2(1:length(act_Phi_axis2)) )';

poles_Lpr3 = ones(length(est_Phi_axis3),1)*Pole4Lpr_Axis3;
est_Lp_axis3 = acker(est_Phi_axis3', est_H_axis3', poles_Lpr3)';
est_Lc_axis3 = est_Phi_axis3 \ est_Lp_axis3;
act_Lc_axis3 = act_Phi_axis3 \ acker(act_Phi_axis3', act_H_axis3', poles_Lpr3(1:length(act_Phi_axis3)) )';

poles_Lpr4 = ones(length(est_Phi_axis4),1)*Pole4Lpr_Axis4;
est_Lp_axis4 = acker(est_Phi_axis4', est_H_axis4', poles_Lpr4)';
est_Lc_axis4 = est_Phi_axis4 \ est_Lp_axis4;
act_Lc_axis4 = act_Phi_axis4 \ acker(act_Phi_axis4', act_H_axis4', poles_Lpr4(1:length(act_Phi_axis4)) )';

poles_Lpr5 = ones(length(est_Phi_axis5),1)*Pole4Lpr_Axis5;
est_Lp_axis5 = acker(est_Phi_axis5', est_H_axis5', poles_Lpr5)';
est_Lc_axis5 = est_Phi_axis5 \ est_Lp_axis5;
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