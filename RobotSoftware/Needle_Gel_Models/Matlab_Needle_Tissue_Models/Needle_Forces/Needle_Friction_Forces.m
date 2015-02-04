function [wrenchF] = Needle_Friction_Forces(TissueState,NeedlePtList,NeedlePtListOld,structMatParameters)
%#codegen

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
% structMaterialParams.mus = 0.16;  Static Friction
% structMaterialParams.muk = 0.0;   Sliding Friction
% structMaterialParams.K   = eye(3)*[.0087; .0087; .012;]; %Spring params
% This must be Positive definate.
% structMaterialParams.alpha = 1.0622; cutting alpha
% structMaterialParams.limit = 1.6717; cutting limit.
%dummyData%


mus = structMatParameters.mus;
muk = structMatParameters.muk;
%Length of output.
N = length(TissueState);

NTL = find(TissueState ~= 0);

if(~isempty(NTL))
    
    NTT = find(TissueState == 1);
    
    indexL = length(NTT);
    
    
    %Output Points!
    NTO = find(TissueState > 0 & TissueState < 1);
    %Input Points!
    NTI = find(TissueState < 0 & TissueState > -1);
    %Calculate the new needle points based on the results.
    
    bufferO = length(NTO);
    bufferI = length(NTI);
    
    
    
    NeedlePtTemp = zeros(3,indexL+bufferO+bufferI+1);
    NeedlePtTempOld = zeros(3,indexL+bufferO+bufferI+1);
    
    
    
    %assign interior Points:
    if(indexL > 0)
        NeedlePtTemp(1:3,(1:indexL+1)+bufferI) = NeedlePtList(1:3,[NTT' NTT(end)+1]);
        NeedlePtTempOld(1:3,(1:indexL+1)+bufferI) = NeedlePtListOld(1:3,[NTT' NTT(end)+1]);
    end
    
    %Output Points!
    
    
    if(bufferO > 0)
        lambda = zeros(1);
        lambda(:) = TissueState(NTO);
        
        NeedlePtTemp(1:3,indexL+bufferI+bufferO+1) = (lambda)*NeedlePtList(1:3,NTO(1)+1)+(1-lambda)*NeedlePtList(1:3,NTO(1));
        NeedlePtTempOld(1:3,indexL+bufferI+bufferO+1) = (lambda)*NeedlePtListOld(1:3,NTO(1)+1)+(1-lambda)*NeedlePtListOld(1:3,NTO(1));
%         try
%             NeedlePtTemp(1:3,indexL+bufferI+bufferO) = NeedlePtList(1:3,NTO(1));
%             NeedlePtTempOld(1:3,indexL+bufferI+bufferO) = NeedlePtListOld(1:3,NTO(1));
%         catch
%             fprintf('what is the problem here\n');
%         end
    end
    
    if(bufferI > 0)
        lambda = zeros(1);
        lambda(:) = abs(TissueState(NTI));
        
        NeedlePtTemp(1:3,1) = (1-lambda)*NeedlePtList(1:3,NTI(1))+(lambda)*NeedlePtList(1:3,NTI(1)+1);
        NeedlePtTempOld(1:3,1) = (1-lambda)*NeedlePtListOld(1:3,NTI(1))+(lambda)*NeedlePtListOld(1:3,NTI(1)+1);
        
        NeedlePtTemp(1:3,2) = NeedlePtList(1:3,NTI(1)+1);
        NeedlePtTempOld(1:3,2) = NeedlePtListOld(1:3,NTI(1)+1);
        
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
    
    
    
    
    %innerVect(1,:) = diag(innerProd)';
    
    %Remove the summing.
    %Add a slight jitter window on the friction...
    d = 0.0000001;
    friction_s = mus*(-segTans.*repmat((innerVect > d ),3,1) + segTans.*repmat((innerVect < -d ),3,1));
    friction_v = -muk*(segTans).*repmat(innerVect,3,1);
    % catch
    %    test = 1;
    %    error('something went wrong');
    % end
    %
    
    friction = friction_s+friction_v;
    

    
    %This is the needle base, oriented to global 0.
    %instead use the global 0.
    %SensPt = NeedlePtList(:,1);
    Sens = zeros(3,1);
    
    
    momentArm =  segCenters-Sens*ones(1,indexL+bufferO+bufferI) ;
    
    fmom = sum(cross(momentArm(1:3,:),friction(1:3,:)),2);
    flin = sum(friction(1:3,:),2);
    
else
    fmom = zeros(3,1);
    flin = zeros(3,1);
    
    
    
end

wrenchF = [flin; fmom];

end













