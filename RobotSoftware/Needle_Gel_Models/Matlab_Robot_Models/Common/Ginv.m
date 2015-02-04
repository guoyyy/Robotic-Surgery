function [ Gi ] = Ginv(G)
%This function calculates the inverse of a transformation matrix.

Gi = eye(4);

Gi(1:3,1:3) = G(1:3,1:3)';
Gi(1:3,4) = -Gi(1:3,1:3)*G(1:3,4);



end

