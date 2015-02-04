function G = struct2G(structPos)
%#codegen
%The input is a structure which contains the following members
% trans is a 3x1 translation vector
% rot is a   4x1 quaternion  vector


R =  Q2R(structPos.rot);
Ta = structPos.trans;

[s1,s2] = size(Ta);

if(s2 ==3)
    T= Ta';
else
    T=Ta;
end

G = [R T; zeros(1,3) 1]; 

end
