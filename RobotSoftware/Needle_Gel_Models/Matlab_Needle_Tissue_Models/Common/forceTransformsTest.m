%The purpose of this script is to generate a force and then test it agains
%differen transforms to verify that they work. 




%Start with a wrench and two points

pa0 = [zeros(3,1);1];


pb0 = [0;1;0;1];

wa_0 = [1; zeros(5,1)]



wb_0 = fncWrenchMove(wa_0,pa0,pb0);


R = Rot_x(pi/4)*Rot_y(pi/3)*Rot_z(pi/6);

p = randn(3,1)*20;

g_10 = [R p; zeros(1,3) 1];



wa_1 = fncWrenchTransform(wa_0,g_10);




wb_1 = fncWrenchMove(wa_1,pa1,pb1);


 






