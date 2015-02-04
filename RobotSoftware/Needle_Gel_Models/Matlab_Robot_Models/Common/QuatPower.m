function Qout = QuatPower(Q1,t);



Omega = acos(Q1(1));

V = Q1(2:4)/sin(Omega);


Qout = [cos(Omega*t) sin(Omega*t)*V];