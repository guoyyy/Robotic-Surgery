%complete a file organization
%TODO

%structNeedleData = struct('trans',[0; 0; 0],'rot',[a;b;c;d];);
%structNeedleGeometry = struct('straightL',4.5,'kinkAngle', -15/180*pi, 'radius',11.5,'arc',150*pi/180);
%n is the number of points.

%incorporate the quaternion terms here.


function [NeedlePts] = fnNeedlePts3d(structNeedleData,structNeedleGeometry,N)

R_SN = Qua2RotMat(structNeedleData.rot);
P_SN = structNeedleData.trans;

g_SN = [R_SN P_SN; 0 0 0 1];

NL = structNeedleGeometry.straightL;


%%%%% I have commented out the if statements because this library is only
%%%%% being used for straight needles right now.

% if structNeedleGeometry.arc == 0
    %straight needle case

    theta = linspace(0,-NL,N+1);
    NeedlePtN1 = [theta; zeros(2,N+1);ones(1,N+1)];
    
% else
%     %bent needle case    
%     
%     %Use the quaternion to develope a transform for the needle.
%     %The Needle naturally lies in the x-y plane and is tangent to the -x
%     %direction at first. The center vect in in the +y direction (before kink
%     %angle)
% 
%     
%     %NeedleL = zeros(2,N+1); 
%     
%     
%     Rad = structNeedleGeometry.radius;
%     kink = structNeedleGeometry.kinkAngle;
%     
%     if(NL > 0)
%         theta = linspace(0,structNeedleGeometry.arc,N);
%         
%         NeedlePtN = [Rad*sin(kink); -Rad*(1-cos(kink)); 0; 0]*ones(1,N)+[ -NL*ones(1,N); zeros(3,N)]+[-Rad*sin(theta+kink); Rad*(1-cos(theta+kink)); zeros(1,N); ones(1,N)];
%         
%         NeedlePtN1 = [[0 0 0 1]' NeedlePtN];
%         
%     else
%         theta = linspace(0,structNeedleGeometry.arc,N+1);
%         NeedlePtN1 = [Rad*sin(kink); -Rad*(1-cos(kink)); 0; 0]*ones(1,N+1)+[ -NL*ones(1,N+1); zeros(3,N+1)]+[-Rad*sin(theta+kink); Rad*(1-cos(theta+kink)); zeros(1,N+1); ones(1,N+1)];
%     end
%     
%     
% end
    
NeedlePts = g_SN*NeedlePtN1;

end
