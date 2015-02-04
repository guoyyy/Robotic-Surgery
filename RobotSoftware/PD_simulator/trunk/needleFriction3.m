function [flin,fmom] =needleFriction3(TissueState,NeedlePtList,Matparameters,needleDiff)


%This function will compute the friction on segments of the needle defined
%by theta and thetadot.

%Parameters is a set of tunable system parameters to try and tweek the force responce of the syste 
% Geometric parameters.
%-------------------------------------
% Geoparameters(1) -The radius of the Needle Mount
% Geoparameters(2) -The radius of motion of the geometric center of the needle.
% Geoparameters(3) -The starting angle of the geometric center of the Needle
% Geoparameters(4) -The angle off set of the starting point of the needle
% Geoparameters(5) -The radius of the needle.
% Geoparameters(7) -The Height of the skin. 
%---------------------------------------
% Material property parameters.
%--------------------------------------- 
% Matparameters(1) -mus of the system.
% Matparameters(2) -K the tissue area spring const.
% Matparameters(3) -xNorm Scale
% Matparameters(4) -Cutting Force
%dummyData%



mus = Matparameters(1);

%Length of output.
N = length(TissueState);

NTL = find(TissueState == 1);

if(~isempty(NTL));


%Compute the tangent vectors.
Tanv = NeedlePtList(:,NTL+1)-NeedlePtList(:,NTL);

%compute the normals of the vectors;
temp = Tanv.^2;
temp2 = sum(temp,1);
normals = sqrt(temp2);

%Compute the Tangent motion
%Ignored for now


%Vtan = Vcp+thetadot*NeeR;

%Generate the output friction.
%friction = -muk*utan.*(ones(3,1)*Vtan) - utan*mus;
% friction = -(Tanv)*mus./(repmat(normals,4,1));

% diff = NeedlePtList(:,NTL+1)-NeedlePtListOld(:,NTL+1);
diff = needleDiff;


dir = Tanv(1:3,1)'*diff;

%magic number alert
N=5;
friction = -mus*N*sign(dir)*Tanv(:,1); 

%friction = (-(Tanv)*mus./(repmat(normals,4,1)) .* repmat((dot(diff,Tanv) > 0 ),4,1)) + ((Tanv)*mus./(repmat(normals,4,1)) .*  repmat((dot(diff,Tanv) <= 0 ),4,1));

% friction = ((-(Tanv)*mus.*repmat((dot(diff,Tanv) > 0 ),4,1)) + ((Tanv)*mus .*repmat((dot(diff,Tanv) < 0 ),4,1))).*repmat(TissueState(NTL)',4,1);

 
 
    

%generate the vector of moment arms. 
meanV = (NeedlePtList(:,NTL+1)+NeedlePtList(:,NTL))/2;


Sens = NeedlePtList(:,1);



momentArm =  meanV-Sens*ones(1,length(NTL)) ;

% fmom = sum(cross(momentArm(1:3,:),friction(1:3,:)),2);
    fmom = zeros(3,1);
    
flin = sum(friction(1:3,:),2); 

else
    fmom = zeros(3,1);
    flin = zeros(3,1);
    


end

end
    
    











