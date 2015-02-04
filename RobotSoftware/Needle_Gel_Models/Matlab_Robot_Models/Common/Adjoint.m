function  Adj = Adjoint(G)
%This function calculates the adjoint transformation matrix of the system. 

R = G(1:3,1:3);
P = G(1:3,4);

Phat = VectHat(P);

Adj = [R Phat*R; zeros(3) R];









end
