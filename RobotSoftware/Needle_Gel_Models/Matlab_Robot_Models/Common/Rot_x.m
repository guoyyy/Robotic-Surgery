function T=Rot_x(psi,option)
%Returns a 3x3 rotation matrix for psi radians rotation about x-axis 
if nargin == 1
    T1=[ones(size(psi))  zeros(size(psi)) zeros(size(psi));  
        zeros(size(psi)) cos(psi)         -sin(psi) ;
        zeros(size(psi)) sin(psi)         cos(psi) ];
    if size(T1,1)>size(T1,2)
        T=permute(reshape(T1,[size(T1,1)/3,3,size(T1,2)]),[2 3 1]); %psi is a column vector
    else
        T=permute(reshape(T1,[size(T1,1),size(T1,2)/3,3]),[1 3 2]); %psi is a row vector
    end
   
elseif nargin==2 && strcmp(option,'d')
    T=[1 0 0;
       0 cos(psi*pi/180) -sin(psi*pi/180);
       0 sin(psi*pi/180) cos(psi*pi/180) ];
else %small angle
    T=[1 0 0;
       0 1 -psi;
       0 psi 1];
end




