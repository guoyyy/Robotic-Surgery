function T=DHMatrix(theta,alpha,a,d,dr)
%This function generates a DH matrix for Robot Joints Using the 
%Theta, Alpha, a and d parameters
%The final input dr will set the angle to degrees 
%If dr is left out the default is radians.




if(nargin == 5)
    if(dr == 'd' || dr == 'D')
        theta = theta*pi/180;
        alpha = alpha*pi/180;
    end
end


T=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
   sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
   0 sin(alpha) cos(alpha) d;...
   0 0 0 1];
end