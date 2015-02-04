


p1 = [510; -209; 379;];
p2 = [651.3; -304.3; 407.8;];
p3 = [457.2; -365.7; 407.8;];


m21 = (p2+p1)/2;
m32 = (p3+p2)/2;
m13 = (p3+p1)/2;


l21 = (p2-p1);
l32 = (p3-p2);
l13 = (p1-p3);

a = norm(l21);
b = norm(l32);
c = norm(l13);


R = a*b*c/(sqrt(2*a^2*b^2+2*b^2*c^2+2*a^2*c^2-a^4-b^4-c^4));

%Make the plane normal. 


n = cross(l21,l32);
 
n = n/norm(n);

l21n = l21/norm(l21);
l13n = l13/norm(l13);

n21n = cross(n,l21n);
n13n = cross(n,l13n);

A =  [n21n(1:2) n13n(1:2)];
lambdas = A\(m21(1:2)-m13(1:2));



c13 = m13+lambdas(2)*n13n;
c21 = m21-lambdas(1)*n21n;

R132 = norm(p2-c13);
R133 = norm(p3-c13);
R131 = norm(p1-c13);

v1 = p2-c13;
v2 = cross(v1,n);

t = linspace(0,2*pi,1000);

PtList = repmat(c21,1,1000)+[v1]*cos(t)+[v2]*sin(t);

plot3(PtList(1,:),PtList(2,:),PtList(3,:));