function NeedlePlot(figHand,structTissueData,NeedlePts1,TissueState1,NeedlePts2,TissueState2,axisLims)


figure(figHand)

wX = structTissueData.widthX;
hZ = structTissueData.heightZ;
dY = structTissueData.depthY;
c  = structTissueData.center;
Tissue = repmat(c,1,14)+[-wX/2  wX/2  wX/2 -wX/2 -wX/2  wX/2  wX/2 -wX/2 -wX/2 -wX/2 -wX/2  wX/2  wX/2 -wX/2;...
    dY/2  dY/2 -dY/2 -dY/2  dY/2  dY/2  dY/2  dY/2 -dY/2 -dY/2 -dY/2 -dY/2  dY/2  dY/2;...
    hZ/2  hZ/2  hZ/2  hZ/2  hZ/2  hZ/2 -hZ/2 -hZ/2 -hZ/2  hZ/2 -hZ/2 -hZ/2 -hZ/2 -hZ/2];

%Generate axis ranges using needle Points

if(isempty(axisLims))
   axisLims = ones(3,1)*[-40 40];    
end

lW = 2;


clf
    
    %% Plot the x-y axis
    subplot 121
    %axis([c(1)-wX/2-20 c(1)+wX/2+20 c(2)-dY/2-20 c(2)+dY/2+20]);
    axis([axisLims(1,:) axisLims(2,:)]);
    axis square
    
    
    %Plot the box.
   line(Tissue(1,:),Tissue(2,:),'color','blue');
    
   %Plot the estimated Needle.
    line(NeedlePts1(1,1),NeedlePts1(2,1),'marker','*','color','black','LineStyle','none','LineWidth',lW);
    outside = find(TissueState1 ==0);
    if(~isempty(outside))
        for k = outside'
            line(NeedlePts1(1,k:k+1),NeedlePts1(2,k:k+1),'marker','none','color','green','LineStyle',':','LineWidth',lW);
        end
    end
    
    inside  = find(TissueState1 ==1);
    if(~isempty(inside))
        for k = inside'
            line(NeedlePts1(1,k:k+1),NeedlePts1(2,k:k+1),'marker','none','color','red','LineStyle',':','LineWidth',lW);
        end
    end
    
    
    neither  = find(TissueState1 ~= 0 & TissueState1 ~= 1);
    if(~isempty(neither))
        for k = neither'
            line(NeedlePts1(1,k:k+1),NeedlePts1(2,k:k+1),'marker','none','color','yellow','LineStyle',':','LineWidth',lW);
        end
    end
    
   
   
    
    %Plot the Actual Needle
    line(NeedlePts2(1,1),NeedlePts2(2,1),'marker','o','color','black','LineStyle','none','LineWidth',lW);
    outside = find(TissueState2 ==0);
    if(~isempty(outside))
        for k = outside'
            line(NeedlePts2(1,k:k+1),NeedlePts2(2,k:k+1),'marker','none','color','green','LineWidth',lW);
        end
    end
    
    inside  = find(TissueState2 ==1);
    if(~isempty(inside))
        for k = inside'
            line(NeedlePts2(1,k:k+1),NeedlePts2(2,k:k+1),'marker','none','color','red','LineWidth',lW);
        end
    end
    
    
    neither  = find(TissueState2 ~= 0 & TissueState2 ~= 1);
    if(~isempty(neither))
        for k = neither'
            line(NeedlePts2(1,k:k+1),NeedlePts2(2,k:k+1),'marker','none','color','yellow','LineWidth',lW);
        end
    end
    
    xlabel('X-Axis');
    ylabel('Y-Axis');
    
    
    
    %% Plot y-z
    subplot 122
    axis([axisLims(2,:) axisLims(3,:)]);
    axis square
    
    
    %Plot the box.
   line(Tissue(2,:),Tissue(3,:),'color','blue');
    
    %Plot the estimated Needle.
    line(NeedlePts1(2,1),NeedlePts1(3,1),'marker','*','color','black','LineStyle','none','LineWidth',lW);
    outside = find(TissueState1 ==0);
    if(~isempty(outside))
        for k = outside'
            line(NeedlePts1(2,k:k+1),NeedlePts1(3,k:k+1),'marker','none','color','green','LineStyle',':','LineWidth',lW);
        end
    end
    
    inside  = find(TissueState1 ==1);
    if(~isempty(inside))
        for k = inside'
            line(NeedlePts1(2,k:k+1),NeedlePts1(3,k:k+1),'marker','none','color','red','LineStyle',':','LineWidth',lW);
        end
    end
    
    
    neither  = find(TissueState1 ~= 0 & TissueState1 ~= 1);
    if(~isempty(neither))
        for k = neither'
            line(NeedlePts1(2,k:k+1),NeedlePts1(3,k:k+1),'marker','none','color','yellow','LineStyle',':','LineWidth',lW);
        end
    end
    
    %Plot the Actual Needle
    line(NeedlePts2(2,1),NeedlePts2(3,1),'marker','o','color','black','LineStyle','none','LineWidth',lW);
    outside = find(TissueState2 ==0);
    if(~isempty(outside))
        for k = outside'
            line(NeedlePts2(2,k:k+1),NeedlePts2(3,k:k+1),'marker','none','color','green','LineWidth',lW);
        end
    end
    
    inside  = find(TissueState2 ==1);
    if(~isempty(inside))
        for k = inside'
            line(NeedlePts2(2,k:k+1),NeedlePts2(3,k:k+1),'marker','none','color','red','LineWidth',lW);
        end
    end
    
    
    neither  = find(TissueState2 ~= 0 & TissueState2 ~= 1);
    if(~isempty(neither))
        for k = neither'
            line(NeedlePts2(2,k:k+1),NeedlePts2(3,k:k+1),'marker','none','color','yellow','LineWidth',lW);
        end
    end
    
    xlabel('Y-Axis');
    ylabel('Z-Axis');
