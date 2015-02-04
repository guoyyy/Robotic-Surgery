%#eml
function  [wrenchModeled,wrenchFriction,wrenchNorm,wrenchCut] = Needle_Cumulative_Forces_Step(TissueStateOld,NeedlePtsOld,TissueState,NeedlePts,structMatParameters,wrenchNormOld)
%# codeGen
%This function is meant to generated the forces and torques of an arbitrary
%needle motion in 3d.
%This is done by stepping

wrenchModeled = zeros(6,1);
wrenchFriction= zeros(6,1);
wrenchNorm = zeros(6,1);
wrenchCut= zeros(6,1);


%Parameters is a set of tunable system parameters to try and tweak the force responce of the system
%
% Geometric parameters.
%-------------------------------------
% TissueStateOld
% NeedlePtsOld
%
% TissueState
% NeedlePts
%
%---------------------------------------
%
% Material property parameters.
% structMaterialParams.mus = 0.16;  Static Friction
% structMaterialParams.muk = 0.0;   Sliding Friction
% structMaterialParams.K   = eye(3)*[.0087; .0087; .012;]; %Spring params
% This must be Positive definate.
% structMaterialParams.alpha = 1.0622; cutting alpha 
% structMaterialParams.limit = 1.6717; cutting limit.
%
%BaseP
%---------------------------------------
%This is the location of the base of the needle.
%Function of Time
%---------------------------------------
%
%BaseT
%---------------------------------------
%This is the tangent (in R3) to the base of the needle
%Function of Time
%---------------------------------------
%N
%BaseN
%---------------------------------------
%This is the Normal (in R3) to the base of the needle
%Points towards needle center
%Function of Time
%---------------------------------------
%
%NeedleCount
%---------------------------------------
%This is the number of elements of the Needle.
%Constant
%---------------------------------------


%thetaMount = thetaMount+pi/2;





%Unused Friction parameters.
% thetadot = .1;
% Vc = [0;0;0];
% muk = 0;
% 
% 
% areaOld = 0;
% 
% vectOld = [0;0;0];
% areas = zeros(1,1);
% %allocate Variables
% fricVect = zeros(3,1);
% fnormVect = zeros(3,1);
% fcutVect = zeros(3,1);
% 
% Fmodeled = zeros(3,1);
% Tmodeled = zeros(3,1);
% frictionL = zeros(3,1);
% frictionT = zeros(3,1);
% TnormVect = zeros(3,1);
% TcutVect  = zeros(3,1);
% fVect = cell(1,1);
% momArm = cell(1,1);
% motorLocation = zeros(4,1);

%structNeedleData.BaseCoords = [basexL(i); baseyL(i)];
%baseCoordsNext = [cos(thetaMount(i+1));sin(thetaMount(i+1))]*(MotR)+dn*[cos(thetaMount(i+1)+kappa-pi/2);sin(thetaMount(i+1)+kappa-pi/2)];
%structNeedleData.BaseAngle = thetaMount(i)-pi/2+kappa;
%These will be calculated to include whend the skin starts.
%and when the skin ends.
%Find the actual start and end points.
%thetabase0 = thetaMount(i)+kappa-pi/2;


%Detect where the needle is in the tissue sample.


% motorLocationOld = NeedlePtsOld(:,1);
% motorLocation       = NeedlePts(:,1);
% NeedleL = length(TissueState);

%Friction Component
[wrenchFriction] = Needle_Friction_Forces(TissueState,NeedlePts,NeedlePtsOld,structMatParameters);

%Normal Error component. 
[dwrench] = Needle_Linear_Normal_Forces(TissueState,NeedlePts,NeedlePtsOld,structMatParameters);


%Measure for depreciation:
Tanv = NeedlePts(:,2:end)-NeedlePts(:,1:(end-1));
temp = Tanv.^2;
temp2 = sum(temp,1);
normals = sqrt(temp2);

TanvOld = NeedlePtsOld(:,2:end)-NeedlePtsOld(:,1:(end-1));
temp = TanvOld.^2;
temp2 = sum(temp,1);
normalsOld = sqrt(temp2);


%Embedded lengths:
TissueStateTemp = TissueState;
TissueStateOldTemp = TissueStateOld;

ind = find(TissueState < 0);
indOld = find(TissueStateOld < 0);

if(~isempty(ind))
TissueStateTemp(ind) = 1+ TissueState(ind);
end

if(~isempty(indOld))
TissueStateOldTemp(indOld) = 1+ TissueStateOld(indOld);
end


NT = normals*TissueStateTemp;
NTOld = normalsOld*TissueStateOldTemp;
wrenchNormB = zeros(6,1);

if(NT < NTOld && NTOld > 0)
    wrenchNormB(:,:) = wrenchNormOld*NT/NTOld;
else
    wrenchNormB(:,:) = wrenchNormOld;
end

%Use the Adjoint to tweak the old wrench
%This is unnecessary if the base frame is invarient.
% gM_NN1 = Ginv(Aold)*Anew;
% Adj_NN1 = Adjoint(gM_NN1);
% wrench_Post = Adj_NN1'*wrenchNormB(:);
%wrenchNorm = dwrench+wrench_Post;

%Invarient Position:
wrenchNorm(:,:)= dwrench(:,:)+wrenchNormB(:,:);



%original force sum term.
%TnormVect(i)   = TnormVect(i-1)+TnormVectT;


[wrenchCut(:,:)] = Needle_Cutting_Forces(TissueState,NeedlePts,NeedlePtsOld,structMatParameters);
    
  

% wrenchFriction = zeros(6,1);
% %wrenchNorm = zeros(6,1);
% wrenchCut = zeros(6,1);

wrenchModeled(:,:) = wrenchCut+wrenchFriction+wrenchNorm;



end

