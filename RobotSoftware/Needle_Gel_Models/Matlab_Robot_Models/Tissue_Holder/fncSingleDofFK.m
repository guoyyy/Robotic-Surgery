function [g_SN] = fncSingleDofFK(qState, n_r)
%fncSingleDofFK This function transforms from the sensor frame to the base
%of the needle. 
%--------------------------------------------------------------------------
% Required Inputs:
%--------------------------------------------------------------------------
% qState: a 4 dimension vector of the joint angles.
% n_r   : needle Radius.
%--------------------------------------------------------------------------
% Required Outputs:
%--------------------------------------------------------------------------
% g_SN the transformation from Needle Coordinates to Sensor frame.

%% Generate Joint twists.

% Joint 1 is the linear actuation along the z axis of the force sensor.
% Joint 2 is a rotation about the sensor z axis.
% joint 3 is a rotation about the needle x axis.
% joint 4 is holding the needle at different locations.


%Position vectors:

qV = zeros(3,4);

qV(3,1) = 0;

qV(:,4) = [0 n_r 0]';

%Rotation vectors 

omegas = zeros(3,4);

omegas(3,1) = 1;


omegas(3,2) = 1;

omegas(1,3) = 1;

omegas(3,4) = 1;


%Final generation?

qTwists = zeros(6,4);

qTwists(3,1) = 1;



%Generate overal transform.
for k = 2:4
    
    qTwists(1:3,k) = -cross(omegas(:,k),qV(:,k));
    
    qTwists(4:6,k) = omegas(:,k);
    
    
end


g_SN = screwmotion(qTwists(:,1),qState(1))*...
       screwmotion(qTwists(:,2),qState(2))*...
       screwmotion(qTwists(:,3),qState(3))*...
       screwmotion(qTwists(:,4),qState(4));




end

