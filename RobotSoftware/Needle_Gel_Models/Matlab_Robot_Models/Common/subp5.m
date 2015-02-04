%subproblem 5
function theta = subp5(delta,p,q,v)
theta(1) = v'*(q-p) + sqrt((v'*(q-p))^2-(norm(q-p))^2 + delta^2);
theta(2) = v'*(q-p)- sqrt((v'*(q-p))^2-(norm(q-p))^2 + delta^2);
%%% how to select the sign of theta 
%%%if v'*(q-p) = 0, then theta must be pos
