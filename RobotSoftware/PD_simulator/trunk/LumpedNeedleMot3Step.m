function [Fmodeled,frictionL,fnormVect,fcutVect,Tmodeled, frictionT, TnormVect,TcutVect] = LumpedNeedleMot3Step(structSkinPlane,structNeedleGeometry,structNeedleData,structNeedleDataOld,Matparameters,Geoparameters,fnormVectOld,TnormVectOld,nD)

needleDiff = nD;
%This function is meant to generated the forces and torques of an arbitrary
%needle motion in 3d.



%Parameters is a set of tunable system parameters to try and tweak the force responce of the system
%
% Geometric parameters.
%-------------------------------------
% Geoparameters(1) -The radius of the Needle Mount (ignored)
% Geoparameters(2) -The radius of motion of the geometric center of the
% needle. (ignored)
% Geoparameters(3) -The starting angle of the geometric center of the
% Needle (ignored)
% Geoparameters(4) -The angle off set of the starting point of the needle
% (ignored)
% Geoparameters(5) -The radius of the needle.
% Geoparameters(6) -The arc angle of the needle.
% Geoparameters(7) -The Height of the skin.
%---------------------------------------
%
% Material property parameters.
%---------------------------------------
% Matparameters(1) -mus of the system.
% Matparameters(2) -K the tissue area spring const.
% Matparameters(3) -xNorm Scale
% Matparameters(4) -Cutting Force
%---------------------------------------
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
%
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
thetadot = .1;
Vc = [0;0;0];
muk = 0;


areaOld = 0;

vectOld = [0;0;0];
areas = zeros(1,1);
%allocate Variables
fricVect = zeros(3,1);
fnormVect = zeros(3,1);
fcutVect = zeros(3,1);

Fmodeled = zeros(3,1);
Tmodeled = zeros(3,1);
frictionL = zeros(3,1);
frictionT = zeros(3,1);
TnormVect = zeros(3,1);
TcutVect  = zeros(3,1);
% fVect = cell(1,1);
% momArm = cell(1,1);
motorLocation = zeros(4,1);

%structNeedleData.BaseCoords = [basexL(i); baseyL(i)];
%baseCoordsNext = [cos(thetaMount(i+1));sin(thetaMount(i+1))]*(MotR)+dn*[cos(thetaMount(i+1)+kappa-pi/2);sin(thetaMount(i+1)+kappa-pi/2)];
%structNeedleData.BaseAngle = thetaMount(i)-pi/2+kappa;
%These will be calculated to include whend the skin starts.
%and when the skin ends.
%Find the actual start and end points.
%thetabase0 = thetaMount(i)+kappa-pi/2;


%Detect where the needle is in the tissue sample.

[TissueStateOld NeedlePtListOld] = fnInvNeedlePts3dY(structSkinPlane,structNeedleGeometry,structNeedleDataOld);
[TissueState NeedlePtList] = fnInvNeedlePts3dY(structSkinPlane,structNeedleGeometry,structNeedleData);

TissueState = ones(5,1);
TissueStateOld = ones(5,1);

motorLocationOld = NeedlePtListOld(:,1);
motorLocation       = NeedlePtList(:,1);
NeedleL = length(TissueState);

%Friction Component
[frictionL(:), frictionT(:)] = needleFriction3(TissueState,NeedlePtList,Matparameters,needleDiff);

% frictionL = [0 0 0]';
% frictionT = [0 0 0]';

% structNeedleData.rot
% if abs(structNeedleData.rot(1) - (-.5)) > 1e-6
%     test = 1;
% end
% 
% if norm(frictionL(2)) > .5
%     NeedlePtList;
%     test=1;
% else 
%     NeedlePtList;
%     test=1;
% end

% frictionL = [0 0 0]';
% frictionT = [0 0 0]';


%Normal Error component.
% [dNormL,dNormT] = needleNormal3(TissueStateOld,TissueState,NeedlePtListOld,NeedlePtList,Matparameters);
% 
dNormL = [0 0 0]';
dNormT = [0 0 0]';

fnormVect = dNormL+fnormVectOld;

%corrected torque sum term:
dbaseVect = [motorLocationOld(1:3)]-[motorLocation(1:3)];
%modify the torque line.
dtorque = cross(dbaseVect,fnormVectOld(:));
TnormVect   = TnormVectOld(:)+dtorque(:)+dNormT(:);

%original force sum term.
%TnormVect(i)   = TnormVect(i-1)+TnormVectT;


[fcutVect(:),TcutVect(:)] = needleCuttingStiffness3d(TissueState,NeedlePtList,Matparameters);
% fcutVect = [0 0 0]';
% TcutVect = [0 0 0]';
  

%Note: moment is unused.

Tmodeled(:) =  TnormVect(:)+frictionT(:)+TcutVect(:);
Fmodeled(:) =  fnormVect(:)+frictionL(:)+fcutVect(:);

end

