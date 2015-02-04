function [ w_1 ] = fncWrenchTransform(w_0,g_10)
%This function transforms a force (wIn) from frame 0 to frame 1.
%The transformation is performed using the adjoint.
    
    Adj_01 = Adjoint(Ginv(g_10));
    %Adj_10 = Adjoint(g_10);
    %w_1 = Adj_10'*w_0;
    w_1 = Adj_01'*w_0;
    

end

