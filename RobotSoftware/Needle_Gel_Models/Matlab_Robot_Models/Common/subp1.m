function theta_s = subp1(p_s,q_s,r_s,w_s)
u_s = p_s(1:3)-r_s(1:3);
v_s = q_s(1:3)-r_s(1:3);
u_c = u_s(1:3)-w_s*w_s'*u_s(1:3);
v_c = v_s(1:3)-w_s*w_s'*v_s(1:3);
if norm(u_c) > 0 || norm(u_c) == 0 ;  %norm(u_c) > 0 || norm(u_c) < 0 
  theta_s = atan2(w_s'*cross(u_c,v_c),u_c'*v_c);
else
    error('The position is unsolvable');
end

