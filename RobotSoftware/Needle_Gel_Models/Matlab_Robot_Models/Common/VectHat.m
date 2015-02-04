function Vhat = VectHat(V)
%#codegen
%This function adds the hat to either a vector of Length 3 or 6.
%A 3 vector leads to the cross matrix, (3x3)
%A 6 vector generates a velocity computation (This is 4x4)
n = length(V);

switch(n)
    case 3
        Vhat = Hat3(V);
    case 6
        omega = Hat3(V(4:6));
        v = V(1:3);
        Vhat = [omega v; zeros(1,3) 0];
    otherwise
        Vhat = zeros(6);
        error('The length of the vector must be either 3 or 6');  
end






end


function Vh = Hat3(omega)
%#codegen
    Vh = [ 0      -omega(3) omega(2);
          omega(3)    0     -omega(1);
          -omega(2)   omega(1)  0 ];


end