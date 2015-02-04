%calculate exp(xi_hat*theta)
function exp_xi_hat = screwmotion(xi,theta)
v = xi(1:3);
w = xi(4:6);

w_hat = [0 -w(3) w(2);
         w(3) 0 -w(1);
         -w(2) w(1) 0];
exp_w_hat = eye(3) + w_hat*sin(theta) + w_hat^2*(1-cos(theta));

w_norm = sqrt(w(1)^2+w(2)^2+w(3)^2); 
if w_norm ~= 0
exp_xi_hat = [exp_w_hat (eye(3)-exp_w_hat)*cross(w,v)+w*w'*v*theta;
                0 0 0            1];
else
    exp_xi_hat = [eye(3) v*theta;
                  0 0 0       1];
end