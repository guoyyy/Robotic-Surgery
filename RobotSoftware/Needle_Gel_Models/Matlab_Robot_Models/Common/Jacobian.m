function J = Jacobian(A)
%Geometric Jacobian: Computes the end effector motion relative to the robot base
%frame given a set of intermediate joints transformations.

%Input
%A is a cell of the different intermediate tranforms
%e.g A{1} is the identity
% A{2} is the transform due to joint 1.
%etc



Nj = length(A)-1;

J = zeros(6,Nj);

for i = 1:Nj
    
    J(1:3,i) = cross(A{i}(1:3,3),A{Nj+1}(1:3,4)-A{i}(1:3,4));
    J(4:6,i) = A{i}(1:3,3);
    
end


end

