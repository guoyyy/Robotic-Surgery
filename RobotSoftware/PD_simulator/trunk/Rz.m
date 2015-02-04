function T=Rz(gamma,option)
%#eml
%Returns a 3x3 rotation matrix for gamma radians rotation about z-axis 
if nargin == 1
    T1=[cos(gamma)        -sin(gamma)         zeros(size(gamma));
        sin(gamma)         cos(gamma)         zeros(size(gamma));
        zeros(size(gamma)) zeros(size(gamma)) ones(size(gamma))];   
   
    if size(T1,1)>size(T1,2)
        T=permute(reshape(T1,[size(T1,1)/3,3,size(T1,2)]),[2 3 1]); %gamma is a column vector
    else
        T=permute(reshape(T1,[size(T1,1),size(T1,2)/3,3]),[1 3 2]); %gamma is a row vector
    end
   
elseif nargin==2 &&  strcmp(option,'d')
    T=[cos(gamma/180*pi) -sin(gamma/180*pi) 0;
       sin(gamma/180*pi)  cos(gamma/180*pi) 0;
       0 0 1];
else
    T=[1 -gamma 0;gamma 1 0 ;0 0 1 ];
end