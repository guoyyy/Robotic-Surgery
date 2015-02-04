%calculate quaternion via rotation matrix
%input: Rotation Matrix M 3X3
%output: Quaternion Q = [q1 q2 q3 q4]

function Q = RotMat2Qua(M)

Q = [0 0 0 0];

[C,I] = max([M(1,1),M(2,2),M(3,3)]);
if I == 1
    u = 1;
    v = 2;
    w = 3;
elseif I == 2;
    u = 2;
    v = 3;
    w = 1;
elseif I == 3
    u = 3; 
    v = 1;
    w = 2;
end


r = sqrt(1 + M(u,u)- M(v,v) - M(w,w));
if r == 0
    Q = [1 0 0 0];
else q1 = (M(w,v) - M(v,w))/(2*r);
     qu = r/2;
     qv = (M(u,v) + M(v,u))/(2*r);
     qw = (M(w,u)+M(u,w))/(2*r);
     
     Q(1) = q1;
     Q(u+1) = qu;
     Q(v+1) = qv;
     Q(w+1) = qw;
     Q = Q/norm(Q);%%unit quaternion
end

%%%mathmatics reference link:
%%%http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
%%%session of "From an orthogonal matrix to a quaternion"