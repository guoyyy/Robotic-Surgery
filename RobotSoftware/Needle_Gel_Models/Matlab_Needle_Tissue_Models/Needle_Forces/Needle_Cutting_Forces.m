function [wrenchCut] = Needle_Cutting_Forces(TissueState,NeedlePtList,NeedlePtListOld,structMatParams)












alpha = structMatParams.alpha;
limit = structMatParams.limit;
wrenchCut = zeros(6,1);

%Length of output.
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
        
        NeedlePtTemp(1:3,indexL+bufferI+bufferO) = NeedlePtList(1:3,NTO(1));
        NeedlePtTempOld(1:3,indexL+bufferI+bufferO) = NeedlePtListOld(1:3,NTO(1));
        
    end
    
    if(bufferI > 0)
        lambda = zeros(1);
        lambda(:) = abs(TissueState(NTI));
        
        NeedlePtTemp(1:3,1) = (1-lambda)*NeedlePtList(1:3,NTI(1))+(lambda)*NeedlePtList(1:3,NTI(1)+1);
        NeedlePtTempOld(1:3,1) = (1-lambda)*NeedlePtListOld(1:3,NTI(1))+(lambda)*NeedlePtListOld(1:3,NTI(1)+1);
        
        NeedlePtTemp(1:3,2) = NeedlePtList(1:3,NTI(1)+1);
        NeedlePtTempOld(1:3,2) = NeedlePtListOld(1:3,NTI(1)+1);
        
    end
   
    %Now that the temporary points are discovered, Compute the motion of
    %Make sure that the points are moving forward.
    %each point. 
    needlePtsMot = zeros(3,indexL+bufferO+bufferI+1);
    needlePtsMot(1:3,:)= NeedlePtTemp(1:3,:)-NeedlePtTempOld(1:3,:);
    
    
    segTans = zeros(3,indexL+bufferO+bufferI);
    
    segTans(1:3,:) = NeedlePtTemp(1:3,2:end)-NeedlePtTemp(1:3,1:end-1);
    
    segTansN =  zeros(3,indexL+bufferO+bufferI);
    
    %temporary Vars:
    tempTN = segTans.^2;
    temp2TN = sum(tempTN,1);
    segNormals = temp2TN.^0.5;
    
    segTansN(1:3,:) = segTans./repmat(segNormals,3,1);

    dir = needlePtsMot(1:3,end)'*segTansN(1:3,end);
    
    
    
    if(dir > 0.000001)
       forceEst = sum(segNormals)^2*alpha; 
    else
        forceEst = 0;
    end

if(forceEst > limit)
   
    forceOut = limit;
else
    forceOut = forceEst;
    
end

forceDir = -segTansN(1:3,end)*forceOut;

leverArm = (NeedlePtList(1:3,end));

torqueDir = cross(leverArm,forceDir); 

wrenchCut = [forceDir; torqueDir;];


end