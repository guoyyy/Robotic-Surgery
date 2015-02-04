function [ref_freq,ref_phase,ref_gain]=ReferenceCalc(PositionReference,MaxRefAcceleration,MoveCWTime)
%Reference Calculation Function
% Given the maximum reference acceleration, reference distance and the
% time to activate reference motion return the frequency for reference
% acceleration function, phase of the acceleration and the gain. This
% reference generation scheme is used widely in CNC machines and robotic
% tools.
% [f,p,g]=ReferenceCalc(PositionReference,MaxRefAcceleration,MoveCWTime)
% Created by Ozkan Bebek - May 2nd 2005
if PositionReference~=0
n=((((log(MaxRefAcceleration/PositionReference))/(log(2)))+5)/2)-1;  %Slope coefficient of the position reference
freq_mult=2^n;                                
ref_gain=(2^(2*n-5));                       
ref_freq=2*pi*freq_mult/10;
ref_phase=-MoveCWTime*ref_freq;
else
    ref_gain=0;                       
    ref_freq=pi;
    ref_phase=0;
end
