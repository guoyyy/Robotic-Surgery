function [ wOut ] = fncWrenchMove(wIn,p0,p1)
%This function moves a force (wIn) from p0 to p1.
%This is done using cross products.
    wOut = zeros(6,1);
    
    wOut(1:3) = wIn(1:3);
    
    vdiff = p1(1:3)-p0(1:3);
    
    wOut(4:6) = VectHat(vdiff)*wIn(1:3)+wIn(4:6);

end

