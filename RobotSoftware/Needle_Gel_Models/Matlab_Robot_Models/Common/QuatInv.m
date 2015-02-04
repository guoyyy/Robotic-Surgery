function Qout = QuatInv(Q1);

%This gives Q1*Q2
Qout = zeros(1,4);


Qout(1) = Q1(1);
Qout(2) = -Q1(2);
Qout(3) = -Q1(3);
Qout(4) = -Q1(4);