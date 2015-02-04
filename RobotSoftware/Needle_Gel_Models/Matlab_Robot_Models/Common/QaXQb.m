function Qout = QaXQb(Qa,Qb);
%#codegen
%This is a Quaternion Product between Qa and Qb in the order Qa*Qb

%This gives Qa*Qb
Qout = zeros(1,4);


Qout(1) = Qa(1)*Qb(1)-Qa(2)*Qb(2)-Qa(3)*Qb(3)-Qa(4)*Qb(4);
Qout(2) = Qa(1)*Qb(2)+Qa(2)*Qb(1)+Qa(3)*Qb(4)-Qa(4)*Qb(3);
Qout(3) = Qa(1)*Qb(3)-Qa(2)*Qb(4)+Qa(3)*Qb(1)+Qa(4)*Qb(2);
Qout(4) = Qa(1)*Qb(4)+Qa(2)*Qb(3)-Qa(3)*Qb(2)+Qa(4)*Qb(1);
