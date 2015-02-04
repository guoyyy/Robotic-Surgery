function T=Rot_y(theta,option)
%Returns a 3x3 rotation matrix for theta radians rotation about y-axis 
if nargin == 1
    T1=[cos(theta)  zeros(size(theta)) sin(theta)  ;
       zeros(size(theta)) ones(size(theta)) zeros(size(theta)) ;
       -sin(theta) zeros(size(theta)) cos(theta)  ];
   
    if size(T1,1)>size(T1,2)
        T=permute(reshape(T1,[size(T1,1)/3,3,size(T1,2)]),[2 3 1]); %theta is a column vector
    else
        T=permute(reshape(T1,[size(T1,1),size(T1,2)/3,3]),[1 3 2]); %theta is a row vector
    end
   
elseif nargin==2 && strcmp(option,'d')
    T=[cos(theta*pi/180)  0 sin(theta*pi/180)  ;
       0 1 0 ;
       -sin(theta*pi/180) 0 cos(theta*pi/180)  ];
else %small angle
    T=[1 0 theta;0 1 0;-theta 0 1 ];
end