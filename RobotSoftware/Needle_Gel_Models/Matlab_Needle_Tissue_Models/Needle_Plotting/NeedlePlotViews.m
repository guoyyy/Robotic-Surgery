function NeedlePlotViews(figHand,Tissue,NeedlePts1,TissueState1,NeedlePts2,TissueState2)




 
     s1 = subplot(221);
   
    %Generate the needle and skin info Pose.
    [TissueState NeedlePtList] = fnInvNeedlePts3dY(structSkinPlane,structNeedleGeometry,structNeedleData(j));
    L = length(TissueState);
    
    for i = 1:L
    
    
    if(TissueState(i) == 0)
         line(NeedlePtList(1,i:i+1),NeedlePtList(2,i:i+1),NeedlePtList(3,i:i+1),'Marker','none','Color','r','LineWidth',2);
     
    elseif(TissueState(i) < 1)
        
        line(NeedlePtList(1,i:i+1),NeedlePtList(2,i:i+1),NeedlePtList(3,i:i+1),'Marker','none','Color','y','LineWidth',2);
        
    else
        
        line(NeedlePtList(1,i:i+1),NeedlePtList(2,i:i+1),NeedlePtList(3,i:i+1),'Marker','none','Color','g','LineWidth',2);
    end
    end
    hold on
    mesh(X,Y,Z);
    hold on;
    axis(AxisVect);
    axis equal;
    view(3);
    
    
    s2 = subplot(222);
    copyobj(allchild(s1),s2);
    
    axis(s2,AxisVect);
    view(2);
    axis equal;

    
    s3 = subplot(223);
    copyobj(allchild(s1),s3);
    
    axis(s3,AxisVect);
    view([0,0]);
    axis equal;
    
   
    
    s4 = subplot(224);
    copyobj(allchild(s1),s4);
   
    axis(s4,AxisVect);
    view([90,0]);
        axis equal;
    
        
        
        xlabel(s1,'x');
        xlabel(s2,'x');
        xlabel(s3,'x');
        xlabel(s4,'x');
        
        
        ylabel(s1,'y');
        ylabel(s2,'y');
        ylabel(s3,'y');
        ylabel(s4,'y');
        
        
        zlabel(s1,'z');
        zlabel(s2,'z');
        zlabel(s3,'z');
        zlabel(s4,'z');
        