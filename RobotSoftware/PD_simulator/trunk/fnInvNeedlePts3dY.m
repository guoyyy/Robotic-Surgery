function [TissueState NeedlePtList] = fnInvNeedlePts3dY(structSkinPlane,structNeedleGeometry,structNeedleData)
%TissueState: 1 if needle segment is in tissue
%             0 if not
%             0-1 represents proportion of needle segment in tissue

%NeedlePtList: coordinates in Cartesian space of the N points defining the N+1 needle segments


%Goal: find the locations where the needle actually enters and exits the tissue.
%This problem is performed in 3 space.
%This is the numerical version for 100 needle segments.

% Geometric parameters.
%-------------------------------------
% Geoparameters(1) -The radius of the Needle Mount
% Geoparameters(2) -The radius of motion of the geometric center of the needle.
% Geoparameters(3) -The starting angle of the geometric center of the Needle
% Geoparameters(4) -The angle off set of the starting point of the needle
% Geoparameters(5) -The radius of the needle.
% Geoparameters(6) -The arc angle of the Needle.
% Geoparameters(7) -The Height of the skin.

% motR = Geoparameters(1);
% GeoR = Geoparameters(2);
% GeoA = Geoparameters(3);
% kappa = Geoparameters(4);


%structNeedleData = struct('trans',[0; 0; 0],'rot',[a;b;c;d];);
%structNeedleGeometry = struct('straightL',4.5,'kinkAngle', -15/180*pi, 'radius',11.5,'arc',150*pi/180);
%structSkinPlane = struct('normal',[0; 0; 0],'b',[0]);

%%to define a straight needle, do something like: struct('straightL',ln,'arc',0)

N=5;


Snormal = [structSkinPlane.normal; 0];
Sb      = structSkinPlane.b;

NeedlePtList = fnNeedlePts3d(structNeedleData,structNeedleGeometry,N);

TissueState = -1*ones(N,1);

for i = 1:N
    pts = NeedlePtList(:,i:i+1)'*Snormal;
    
    if(pts(1)> Sb && pts(2) > Sb)
        
        TissueState(i) = 0;
        
    elseif(pts(1) < Sb && pts(2) < Sb)
        TissueState(i) = 1;
        
    else
        A = [pts(1) pts(2); 1 1];
        b = [Sb; 1];
        lambda = A\b;
        TissueState(i) = lambda(1);
    end
    
    
    
    
end
end