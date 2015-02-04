%This is the Generic Forward Kinematics for a robot defined by DH
%parameters.

function [aCell] = FK_Generic(DHParams,Jnts);
%Input List:
%DHParams is a function of the input Jnts. (This allows for prismatic and
%revolute Joints to be distinguished.
%Jnts is the list of the joint values.

nJ = length(Jnts);



DHParamsOut = DHParams(Jnts);

aCell = cell(nJ,1);

aCell{1} = eye(4);



for i = 2:nJ+1
   aCell{i} = aCell{i-1}*DHMatrix(DHParamsOut(i-1,1),DHParamsOut(i-1,2),DHParamsOut(i-1,3),DHParamsOut(i-1,4));
end





end