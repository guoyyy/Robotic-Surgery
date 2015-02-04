% function [theta1,theta2,t] = subp2(p,q,r,w1,w2)
% u = p-r;
% v = q-r;
% alpha = (((w1'*w2)*(w2'*u))-w1'*v)/((w1'*w2)^2-1);
% beta = (w1'*w2*w1'*v-w2'*u)/((w1'*w2)^2-1);
% s = ((norm(u))^2-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2)))^2;
% if s <0
%     t = 0;
%     theta1 = 0;
%     theta2 = 0;
%   
% elseif s >0
%     gamma_1 = sqrt(s);
%     gamma_2 = -sqrt(s);
%     z_1 = alpha*w1+beta*w2+gamma_1*cross(w1,w2);
%     z_2 = alpha*w1+beta*w2+gamma_2*cross(w1,w2);
%     c_1 = r+z_1;
%     c_2 = r+z_2;
%     
%     theta1 = [-subp1(q,c_1,r,w1),-subp1(q,c_2,r,w1)];
%     theta2 = [subp1(p,c_1,r,w2), subp1(p,c_2,r,w2)];
%     t = 2;
%     
% else
%     gamma = 0
%     z = alpha*w1+beta*w2;
%     c = r+z;
%     theta1 = -subp1(q,c,r,w1);
%     theta2 = subp1(p,c,r,w2);
%     t = 1;
% end


%Old Version Retired by RCJ on 1/24/2012
% function [theta1,theta2,t] = subp2(p,q,r,w1,w2)
% u = p-r;
% v = q-r;
% alpha = (((w1'*w2)*(w2'*u))-w1'*v)/((w1'*w2)^2-1);
% beta = (w1'*w2*w1'*v-w2'*u)/((w1'*w2)^2-1);
% s = ((norm(u))^2-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2)))^2;
% if s <0
%     t = 0;
%     theta1 = 0;
%     theta2 = 0;
%   
% elseif s >0
%     gamma_1 = sqrt(s);
%     gamma_2 = -sqrt(s);
%     z_1 = alpha*w1+beta*w2+gamma_1*cross(w1,w2);
%     z_2 = alpha*w1+beta*w2+gamma_2*cross(w1,w2);
%     c_1 = r+z_1;
%     c_2 = r+z_2;
%     
%     theta1 = [-subp1(q,c_1,r,w1),-subp1(q,c_2,r,w1)];
%     theta2 = [subp1(p,c_1,r,w2), subp1(p,c_2,r,w2)];
%     t = 2;
%     
% else
%     gamma = 0
%     z = alpha*w1+beta*w2;
%     c = r+z;
%     theta1 = -subp1(q,c,r,w1);
%     theta2 = subp1(p,c,r,w2);
%     t = 1;
% end


% 
% 
% function [theta1,theta2,t] = subp2(p,q,q_1,q_2,xi_1,xi_2)
% 
% %Find the intersection r.
% %Line 1 =  
% w1 = xi_1(4:6);
% w2 = xi_2(4:6);
% 
% 
% 
% LvMat = [w1 w2];
% 
% AB=LvMat\(q_1(1:3)-q_2(1:3));
% 
% ra = AB(1)*w1+q_1(1:3);
% rb = AB(2)*w2+q_2(1:3);
% 
% if ra==rb
%     
%     r = ra;
% else
%     error('r is not found');
% end
% 
% %
% u = p(1:3)-r;
% v = q(1:3)-r;
% alpha = (((w1'*w2)*(w2'*u))-w1'*v)/((w1'*w2)^2-1);
% beta = (w1'*w2*w1'*v-w2'*u)/((w1'*w2)^2-1);
% s = ((norm(u))^2-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2)))^2;
% if s <0
%     t = 0;
%     theta1 = 0;
%     theta2 = 0;
%   
% elseif s >0
%     gamma_1 = sqrt(s);
%     gamma_2 = -sqrt(s);
%     z_1 = alpha*w1+beta*w2+gamma_1*cross(w1,w2);
%     z_2 = alpha*w1+beta*w2+gamma_2*cross(w1,w2);
%     c_1 = r+z_1;
%     c_2 = r+z_2;
%     
%     theta1 = [-subp1(q,c_1,r,w1),-subp1(q,c_2,r,w1)];
%     theta2 = [subp1(p,c_1,r,w2), subp1(p,c_2,r,w2)];
%     t = 2;
%     
% else
%     gamma = 0
%     z = alpha*w1+beta*w2;
%     c = r+z;
%     theta1 = -subp1(q,c,r,w1);
%     theta2 = subp1(p,c,r,w2);
%     t = 1;
% end






function [theta1,theta2,t] = subp2(p,q,r,w1,w2)
u = p-r;
v = q-r;
alpha = (((w1'*w2)*(w2'*u))-w1'*v)/((w1'*w2)^2-1);
beta = (w1'*w2*w1'*v-w2'*u)/((w1'*w2)^2-1);
s = ((norm(u))^2-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(norm(cross(w1,w2)))^2;
if s <0
    t = 0;
    theta1 = 0;
    theta2 = 0;
  
elseif s >0
    gamma_1 = sqrt(s);
    gamma_2 = -sqrt(s);
    z_1 = alpha*w1+beta*w2+gamma_1*cross(w1,w2);
    z_2 = alpha*w1+beta*w2+gamma_2*cross(w1,w2);
    c_1 = r+z_1;
    c_2 = r+z_2;
    
    theta1 = [-subp1(q,c_1,r,w1),-subp1(q,c_2,r,w1)];
    theta2 = [subp1(p,c_1,r,w2), subp1(p,c_2,r,w2)];
    t = 2;
    
else
    gamma = 0
    z = alpha*w1+beta*w2;
    c = r+z;
    theta1 = -subp1(q,c,r,w1);
    theta2 = subp1(p,c,r,w2);
    t = 1;
end
