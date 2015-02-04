function [ struct_t ] = G2struct( G )
%#codegen
%This function converts a 4x4 tranformation matrix to a matlab stucture
%The output is a structure with the following members
% trans: the 3x1 vector of the TCP
% rot  : the 4x1 quaternion of the TCP orientation


struct_t = struct('rot',R2Q(G(1:3,1:3)),'trans',G(1:3,4));

end

