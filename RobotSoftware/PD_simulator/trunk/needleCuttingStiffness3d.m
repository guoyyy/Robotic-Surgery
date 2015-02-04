function [flin,fmom] = needleCuttingStiffness3d(TissueState,NeedlePtList,Matparameters,ts)

%This function is a combination of the cutting force and the stiffness
%force applied to the needle. The stiffness forces are felt the most 


%Parameters is a set of tunable system parameters to try and tweek the force responce of the syste 
% Geometric parameters.
%-------------------------------------
% Geoparameters(1) -The radius of the Needle Mount
% Geoparameters(2) -The radius of motion of the geometric center of the needle.
% Geoparameters(3) -The starting angle of the geometric center of the Needle
% Geoparameters(4) -The angle off set of the starting point of the needle
% Geoparameters(5) -The radius of the needle.
% Geoparameters(6) -The arc angle of the Needle. 
% Geoparameters(7) -The Height of the skin. 
%---------------------------------------
% Material property parameters.
%--------------------------------------- 
% Matparameters(1) -mus of the system.
% Matparameters(2) -K the tissue area spring const.
% Matparameters(3) -xNorm Scale
% Matparameters(4) -Cutting Force
%dummyData%
%----------


%Step 1. Find The needle tip.

N = length(TissueState);


%Step 2. Find where the needle enters the tissue. Use the first point


index = find(TissueState > 0 & TissueState < 1);

if length(index) == 0;
%     flin = [0 0 0]';
%     fmom= [0 0 0]';
%     return;

    indexT = N;
else

    indexT = max(index);
end

%Step 3.  Calculate the tip dist from the grey dist.

dist = 0;

for Ni = indexT:N
    
    dist = dist + norm(NeedlePtList(1:3,Ni+1)-NeedlePtList(1:3,Ni));
    
    
end

%Use this to calculate the polynomial force. The tip direction. This is in
%opposition.

PF = .5*(dist^2);

CF =  Matparameters(4);


TF = min(PF,CF);

flin = TF*(NeedlePtList(1:3,N+1)-NeedlePtList(1:3,N))/norm(NeedlePtList(1:3,N+1)-NeedlePtList(1:3,N));

momentArm = NeedlePtList(1:3,N+1)-NeedlePtList(1:3,1);

fmom = cross(momentArm(1:3,1),flin(1:3,:));

end

    


