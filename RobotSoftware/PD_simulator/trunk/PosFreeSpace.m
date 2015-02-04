function [t,x,R]=PosBreath3D(x0,R0,moti,motl,ts)
%% create data to hold pos and rotation.
% clear all
% ts=1e-1;            % Sampling time
% moti=15;             % initial motion time
% motl=28;             % motion time length
% x0=[0 -200  290];   % starting position
% R0 =[0,-0.1,0.95]'/norm([0,-0.1,0.95]);%breathing angle

 

%% Breathing motion
% (3.0*sin(0.45*pi*2+0)+0)+(-0.5*sin(1.11*pi*2+0)+0)
SineAmplitude1=+3.0*4;       % mm 
SineAmplitude2=-0.5*4;       % mm 

SineFreq1=0.45*pi*2 ;               % radian/second (2*pi*f) 
SineFreq2=1.11*pi*2 ;               % radian/second (2*pi*f) 
%% Generate Reference Signal
% NoCyc=12;
NoCyc=round(motl*SineFreq1/2/pi); % Number of cycles (rounded to nearest according to motl)

BreathRef_y=(SineAmplitude1.*(sin(SineFreq1*(0:ts:NoCyc*2*pi/SineFreq1-ts)))'+ ...
             SineAmplitude2.*(sin(SineFreq2*(0:ts:NoCyc*2*pi/SineFreq1-ts)))').*...
             [linspace(0,1,1*1/(SineFreq1/2/pi)/ts),...
              ones(1,length(0:ts:(NoCyc-2)*2*pi/SineFreq1) ),... -1
              linspace(1,0,1*1/(SineFreq1/2/pi)/ts)]'; % this is to smooth first sine
         
BreathRef=[zeros(1,length(BreathRef_y));BreathRef_y';zeros(1,length(BreathRef_y))];



t=(moti+0):ts:(moti+NoCyc*2*pi/SineFreq1-ts);
% x=[x0(1)*ones(length(t),1),x0(2)+BreathRef_y,x0(3)*ones(length(t),1)]';
x=[x0(1)*ones(length(t),1),x0(2)+BreathRef_y*1,x0(3)+BreathRef_y*1]';

R=[R0(1)*ones(length(t),1),R0(2)*ones(length(t),1),R0(3)*ones(length(t),1)]';