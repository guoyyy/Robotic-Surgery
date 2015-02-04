%This function initializes the variables. 
%runs the simulink code,
%Draws out the results.

clear
clc
close all
%System definitions.

dt = 0.0001; %Time step
tf = 10;     %Final time.


P0 = zeros(7,1);
P0(4) = 1;
Vel = randn(6,1);
A0 = eye(4);


sim('NeedleMotion');

subplot 121
plot(timeVect.signals.values,PF.signals.values(:,1),'r',timeVect.signals.values,PF.signals.values(:,2),'g',timeVect.signals.values,PF.signals.values(:,3),'b');

subplot 122

plot(timeVect.signals.values,PF.signals.values(:,4),'r',timeVect.signals.values,PF.signals.values(:,5),'g',timeVect.signals.values,PF.signals.values(:,6),'b'...
    ,timeVect.signals.values,PF.signals.values(:,7),'k');



figure

subplot 121
plot(timeVect.signals.values,PF1.signals.values(:,1),'r',timeVect.signals.values,PF1.signals.values(:,2),'g',timeVect.signals.values,PF1.signals.values(:,3),'b');

subplot 122

plot(timeVect.signals.values,PF1.signals.values(:,4),'r',timeVect.signals.values,PF1.signals.values(:,5),'g',timeVect.signals.values,PF1.signals.values(:,6),'b'...
    ,timeVect.signals.values,PF1.signals.values(:,7),'k');

