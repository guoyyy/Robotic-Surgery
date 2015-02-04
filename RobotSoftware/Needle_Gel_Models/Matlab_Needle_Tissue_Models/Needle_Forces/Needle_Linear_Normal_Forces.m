function [dWrenchN] = Needle_Linear_Normal_Forces(TissueState,NeedlePtList,NeedlePtListOld,structMatParameters)



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

Ka = zeros(9,1);
Ka(:,:)  = structMatParameters.K;

K = zeros(3,3);
K = reshape(Ka,3,3);
%xRat = Matparameters(3); %This had to be dropped

NTL = find(TissueState ~= 0);

if(~isempty(NTL))
    
    %Update the locations of the Needle Segments.
    
    NTT = find(TissueState == 1);
    
    indexL = length(NTT);
    
    
    %Output Points!
    NTO = find(TissueState > 0 & TissueState < 1);
    %Input Points!
    NTI = find(TissueState < 0 & TissueState > -1);
    %Calculate the new needle points based on the results.
    
    bufferO = length(NTO);
    bufferI = length(NTI);
    
    tissueLow  = zeros(1,indexL+bufferO+bufferI);
    tissueHigh = zeros(1,indexL+bufferO+bufferI);
    
    
    NeedlePtTemp = zeros(3,indexL+bufferO+bufferI+1);
    NeedlePtTempOld = zeros(3,indexL+bufferO+bufferI+1);
    tLambda = zeros(3,indexL+bufferO+bufferI);
    
    
    
    %assign interior Points:
    if(indexL > 0)
        NeedlePtTemp(1:3,(1:indexL+1)+bufferI) = NeedlePtList(1:3,[NTT' NTT(end)+1]);
        NeedlePtTempOld(1:3,(1:indexL+1)+bufferI) = NeedlePtListOld(1:3,[NTT' NTT(end)+1]);
        tissueLow((1:indexL+1)+bufferI) = 0;
        tissueHigh((1:indexL+1)+bufferI) = 1;
    end
    
    %Output Points!
    
    
    if(bufferO > 0)
        lambda = zeros(1);
        lambda(:) = TissueState(NTO);
        
        NeedlePtTemp(1:3,indexL+bufferI+bufferO+1) = NeedlePtList(1:3,NTO(1)+1);
        NeedlePtTempOld(1:3,indexL+bufferI+bufferO+1) = NeedlePtListOld(1:3,NTO(1)+1);
        
        NeedlePtTemp(1:3,indexL+bufferI+bufferO) = NeedlePtList(1:3,NTO(end));
        NeedlePtTempOld(1:3,indexL+bufferI+bufferO) = NeedlePtListOld(1:3,NTO(end));
        
        tissueLow(indexL+bufferI+bufferO) = 0;
        tissueHigh(indexL+bufferI+bufferO) = lambda;
        
        
    end
    
    if(bufferI > 0)
        lambda = zeros(1);
        lambda(:) = abs(TissueState(NTI));
        
        
        NeedlePtTemp(1:3,1) = NeedlePtList(1:3,NTI(1));
        NeedlePtTempOld(1:3,1) = NeedlePtListOld(1:3,NTI(1));
        
        NeedlePtTemp(1:3,2) = NeedlePtList(1:3,NTI(1)+1);
        NeedlePtTempOld(1:3,2) = NeedlePtListOld(1:3,NTI(1)+1);
        
        tissueLow(1) = lambda;
        tissueHigh(1) = 1;
        
    end
    
    %Generate the mean Points: (used for moments)
    segCenters =  zeros(3,indexL+bufferO+bufferI);
    segCenters(1:3,:) = (NeedlePtTemp(1:3,2:end)+NeedlePtTemp(1:3,1:end-1))/2;
    
    
    %Now that the temporary points are discovered, Compute the motion of
    %each point.
    needlePtsMot = zeros(3,indexL+bufferO+bufferI+1);
    needlePtsMot(1:3,:)= NeedlePtTemp(1:3,:)-NeedlePtTempOld(1:3,:);
    
    %From this extrapolate the overall segments velocities using averaging.
    segmentMot = (needlePtsMot(1:3,2:end)+needlePtsMot(1:3,1:end-1))/2;
    
    %Compute the tangents (and normalized tangents) of the segments;
    segTans = zeros(3,indexL+bufferO+bufferI);
    
    segTans(1:3,:) = NeedlePtTemp(1:3,2:end)-NeedlePtTemp(1:3,1:end-1);
    
    segTansN =  zeros(3,indexL+bufferO+bufferI);
    
    %temporary Vars:
    tempTN = segTans.^2;
    temp2TN = sum(tempTN,1);
    segNormals = temp2TN.^0.5;
    
    segTansN(1:3,:) = segTans./repmat(segNormals,3,1);
    
    
    
    %Generate the inner product of the tangent motion with the segments lengths.
    innerProd = zeros(3,indexL+bufferO+bufferI);
    innerProd(1:3,:) = segmentMot.*segTansN;
    
    innerVect = zeros(1,indexL+bufferO+bufferI);
    
    innerVect(1,:) = sum(innerProd,1);
    
    fNormT = zeros(3,indexL+bufferO+bufferI);
    tNormT = zeros(3,indexL+bufferO+bufferI);
    
    
    for j = 1:indexL+bufferO+bufferI
        
        %Find the vector points here.
        nV = segTansN(1:3,j);
        
        
        V1 = needlePtsMot(:,j);
        V2 = needlePtsMot(:,j+1);
        P1 = NeedlePtTemp(:,j);
        P2 = NeedlePtTemp(:,j+1);
        
        
        dl = segNormals(j);
        
        
        lLow = tissueLow(j);
        lHigh = tissueHigh(j);
        
        normalDist2 = (lHigh^2/2-lLow^2/2)*(V2-nV*(V2'*nV));
        normalDist1 = ((lHigh-lHigh^2/2)-(lLow-lLow^2/2))*(V1-nV*(V1'*nV));
        
        torqueDist11 = ((lHigh^3/3-1.5*lHigh^2/2+0.5*lHigh)-(lLow^3/3-1.5*lLow^2/2+0.5*lLow))*VectHat(P1)*(V1-nV*(V1'*nV));
        torqueDist12 = ((lHigh^2/4-lHigh^3/3)-(lLow^2/4-lLow^3/3))*VectHat(P1)*(V2-nV*(V2'*nV));
        
        torqueDist21 = ((-lHigh^3/3+1.5*lHigh^2/2-0.5*lHigh)-(-lLow^3/3+1.5*lLow^2/2-0.5*lLow))*VectHat(P2)*(V1-nV*(V1'*nV));
        torqueDist22 = ((lHigh^3/3-0.5*lHigh^2/2)-(lLow^3/3-0.5*lLow^2/2))*VectHat(P2)*(V2-nV*(V2'*nV));
        
%         lam = sym('lam');
%         
%         fnc3a = (0.5-lam)*(1-lam)*VectHat(P1)*(V1-nV*(V1'*nV))
%         
%         fnc3b = (0.5-lam)*(lam)*VectHat(P1)*(V2-nV*(V2'*nV))
%         
%         fnc3c = (lam-0.5)*(1-lam)*VectHat(P2)*(V1-nV*(V1'*nV))
%         
%         
%         fnc3d = (lam-0.5)*(lam)*VectHat(P2)*(V2-nV*(V2'*nV))
%         
%         testTa{j} = int(fnc3a,lam,lLow,lHigh)*dl;
%         testTb{j} = int(fnc3b,lam,lLow,lHigh)*dl;
%         testTc{j} = int(fnc3c,lam,lLow,lHigh)*dl;
%         testTd{j} = int(fnc3d,lam,lLow,lHigh)*dl;
%         
%         
%         fnc2 = lam*V2+(1-lam)*V1-nV*((lam*V2+(1-lam)*V1)'*nV);
%         fnc1 = VectHat(lam*P2+(1-lam)*P1-P1/2-P2/2);
%         
%         testT{j} = int(fnc1*fnc2,lam,lLow,lHigh)*dl;
        
        fNormT(:,j) = K*(normalDist1+normalDist2)*dl;
        tNormT(:,j) = K*(torqueDist11+torqueDist12+torqueDist21+torqueDist22)*dl;
        
    end
    
    
    %generate the vector of moment arms.
    
    Sens = zeros(3,1);
    
    momentArm =  segCenters-repmat(Sens,1,indexL+bufferO+bufferI);
    
    fmom3 = cross(momentArm,fNormT)+tNormT;
    
    
    
    dNormL = sum(fNormT,2);
    dNormT = sum(fmom3,2);
    %No moment calculations yet.
    
else
    dNormL = zeros(3,1);
    dNormT = zeros(3,1);
    
    
    
    
end

dWrenchN = [dNormL; dNormT];

end
%
%
%
%
%
%
%
%
%
%
%
%
% %% This is Version 1.
% function [dNormL,dNormT] = needleNormal3(TissueState,TissueStateN,NeedlePtList,NeedlePtListN,Matparameters)
%
%
% %This function will compute the normal on segments of the needle defined
% %by theta.
%
%
% % Geometric parameters.
% %-------------------------------------
% % Geoparameters(1) -The radius of the Needle Mount
% % Geoparameters(2) -The radius of motion of the geometric center of the needle.
% % Geoparameters(3) -The starting angle of the geometric center of the Needle
% % Geoparameters(4) -The angle off set of the starting point of the needle
% % Geoparameters(5) -The radius of the needle.
% % Geoparameters(6) -The arc angle of the Needle.
% % Geoparameters(7) -The Height of the skin.
% %---------------------------------------
% % Material property parameters.
% %---------------------------------------
% % Matparameters(1) -mus of the system.
% % Matparameters(2) -K the tissue area spring const.
% % Matparameters(3) -xNorm Scale
% % Matparameters(4) -Cutting Force
% %dummyData%
% %----------
%
%
%
% %NeedleT = structNeedlePos.BaseAngle;
% %NeedleS = structNeedlePos.BaseCoords;
%
%
%
% mus = Matparameters(1);
% K  = Matparameters(2);
% %xRat = Matparameters(3); %This had to be dropped
%
% NTL = find(TissueState == 1);
%
% if(length(NTL) > 0)
%
% Tanv = NeedlePtList(1:3,NTL+1)-NeedlePtList(1:3,NTL);
% meanPt = (NeedlePtList(1:3,NTL+1)+NeedlePtList(1:3,NTL))/2;
%
% Motv  = (NeedlePtListN(1:3,NTL+1)+NeedlePtListN(1:3,NTL))/2-(NeedlePtList(1:3,NTL+1)+NeedlePtList(1:3,NTL))/2;
%
% areas = cross(Tanv,Motv);
%
%
%
% %compute the dot product component.
%
% temp = Tanv.^2;
% temp2 = sum(temp,1);
% TanvNormals = sqrt(temp2);
%
% inProd = sum(Motv.*Tanv,1);
% inProdnorm = inProd./TanvNormals.^2;
%
%
% nVects = Motv - (ones(3,1)*inProdnorm).*Tanv;
% %Now compute the inner product
%
% areas2 = cross(Tanv,nVects);
%
% areas2p = sum(areas2.*areas2,1);
% nVectsNorm = sum(nVects.*nVects,1);
% nVectsNormM = repmat(nVectsNorm,3,1);
% %If the nVects Norms are zeros, then knock them out.
%
% if( nVectsNormM(1) == 0)
%
%     nVectsNormed = nVects*0;
%
%
% else
%
%     nVectsNormed = nVects./nVectsNormM;
%
% end
% %Use the derivative with the
% Fnorm = -nVectsNormed.*(ones(3,1)*areas2p)*K;
%
% N = length(NTL);
%
%
% %generate the vector of moment arms.
%
% Sens = NeedlePtList(1:3,1);
%
% momentArm =  meanPt-Sens*ones(1,N) ;
%
% fmom3 = cross(momentArm,Fnorm);
%
%
%
% dNormL = sum(Fnorm,2);
% dNormT = sum(fmom3,2);
% %No moment calculations yet.
%
% else
%     dNormL = zeros(3,1);
%     dNormT = zeros(3,1);
%
%
%
%
% end
%
% end
%% End V1