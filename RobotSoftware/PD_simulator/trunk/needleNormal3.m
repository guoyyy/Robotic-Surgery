function [dNormL,dNormT] = needleNormal3(TissueState,TissueStateN,NeedlePtList,NeedlePtListN,Matparameters)



%This function will compute the normal on segments of the needle defined
%by theta.


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



%NeedleT = structNeedlePos.BaseAngle;
%NeedleS = structNeedlePos.BaseCoords;



mus = Matparameters(1);
K  = Matparameters(2);
%xRat = Matparameters(3); %This had to be dropped

NTL = find(TissueState == 1);

if(~isempty(NTL))

Tanv = NeedlePtList(1:3,NTL+1)-NeedlePtList(1:3,NTL);
meanPt = (NeedlePtList(1:3,NTL+1)+NeedlePtList(1:3,NTL))/2;

Motv  = (NeedlePtListN(1:3,NTL+1)+NeedlePtListN(1:3,NTL))/2-(NeedlePtList(1:3,NTL+1)+NeedlePtList(1:3,NTL))/2;

areas = cross(Tanv,Motv);



%compute the dot product component.

temp = Tanv.^2;
temp2 = sum(temp,1);
TanvNormals = sqrt(temp2);

inProd = sum(Motv.*Tanv,1);
inProdnorm = inProd./TanvNormals.^2;


nVects = Motv - (ones(3,1)*inProdnorm).*Tanv;
%Now compute the inner product

areas2 = cross(Tanv,nVects);

areas2p = sum(areas2.*areas2,1);
nVectsNorm = sum(nVects.*nVects,1);
nVectsNormed = zeros(3,length(nVectsNorm));
for i=1:length(nVectsNorm)
    if nVectsNorm(i) ~=0 
        nVectsNormed(:,i) = nVects(:,i)/nVectsNorm(i);
    else
        nVectsNormed(:,i) = [0 0 0]';      
    end
    
end

%Use the derivative with the 
Fnorm = -nVectsNormed.*(ones(3,1)*areas2p)*K;

N = length(NTL);


%generate the vector of moment arms. 

Sens = NeedlePtList(1:3,1);

momentArm =  meanPt-Sens*ones(1,N) ;

fmom3 = cross(momentArm,Fnorm);



dNormL = sum(Fnorm,2);
dNormT = sum(fmom3,2);
%No moment calculations yet.

else
    dNormL = zeros(3,1);
    dNormT = zeros(3,1);
    
    
    
    
end

end

