function R = Rot_Proj(R0)
%#codegen
    A = R0(1:3,1);
    B = R0(1:3,2);
    C = R0(1:3,3);
    
    A = A/norm(A);
    
    B = B-A*(A'*B);
    B = B/norm(B);
    
    C = C-A*(A'*C)-B*(B'*C);
    
    C = C/norm(C);
    
    R = [A B C];
    

end