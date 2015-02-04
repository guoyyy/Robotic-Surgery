function Euler=ypr2R(Yaw,Pitch,Roll,angle)
% Yaw-Pitch-Roll zyx-Euler angles to rotation matrix
if nargin==3 %return radians
    if length(Yaw)>1
        Euler=zeros(3,3,length(Yaw));
        for jj=1:length(Yaw);
            Euler(:,:,jj)=Rz(Yaw(jj))*Ry(Pitch(jj))*Rx(Roll(jj));
        end            
    else 
        Euler=Rz(Yaw)*Ry(Pitch)*Rx(Roll);
    end
elseif  nargin==4 && strcmp(angle,'deg') 
    if length(Yaw)>1
        Euler=zeros(3,3,length(Yaw));
        for jj=1:length(Yaw);
            Euler(:,:,jj)=Rz(Yaw(jj)*pi/180)*Ry(Pitch(jj)*pi/180)*Rx(Roll(jj)*pi/180);
        end            
    else 
        Euler=Rz(Yaw*pi/180)*Ry(Pitch*pi/180)*Rx(Roll*pi/180);
    end
elseif  nargin==4 && strcmp(angle,'rad') 
    if length(Yaw)>1
        Euler=zeros(3,3,length(Yaw));
        for jj=1:length(Yaw);
            Euler(:,:,jj)=Rz(Yaw(jj))*Ry(Pitch(jj))*Rx(Roll(jj));
        end            
    else 
        Euler=Rz(Yaw)*Ry(Pitch)*Rx(Roll);
    end
end
    
