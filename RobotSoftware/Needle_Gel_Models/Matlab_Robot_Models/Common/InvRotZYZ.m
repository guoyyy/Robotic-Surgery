function [alpha,beta,gamma] = InvRotZYZ(Rot)

indeces = [3 6 7 8];

if(sum(abs(Rot(indeces))) > 0);
    theta2 = atan2(sqrt(Rot(7)^2+Rot(8)^2),Rot(9));
    theta1 = atan2(Rot(8)/sin(theta2),Rot(7)/sin(theta2));
    theta3 = atan2(Rot(6)/sin(theta2),-Rot(3)/sin(theta2));
   
else
    if(Rot(9) > 0)
        theta2 = 0;
    else
        theta2 = pi;
    end
    theta3 = 0;
    
    theta1 = atan2(Rot(2),Rot(1));
    
    
end






% 
% 
% 
% 
% 
% 
% 
% 
% theta2a = acos(Rot(9));
% 
% 
% theta2b = 2*pi-theta2a;
% 
% theta2 = [theta2a theta2b];
% 
% if(abs(Rot(6)) < .0000001)
% theta3a = asin([Rot(6) Rot(6)]./sin(theta2));
% else
%     
%     
% end
% theta3b = 2*pi-theta3a;  
% 
% theta3 =[theta3a; theta3b];
% 
% theta1a = asin([Rot(8) Rot(8)]./sin(theta2));
% theta1b = 2*pi-theta1a;
% 
% theta1 = [theta1a; theta1b];
% 
% thetaSol = cell(1);
% 
% SolC = 0;
% 
% for index1 = 1:4
%     for index2 = 1:2
%         for index3 = 1:4
%             
%             Rtest = Rot_Z(theta1(index1))*Rot_Y(theta2(index2))*Rot_Z(theta3(index3));
%             if(norm(Rtest-Rot) < .01)
%                SolC = SolC+1;
%                thetaSol{SolC} = [theta1(index1) theta2(index2) theta3(index3)];
%                 
%             end
%             
%             
%         end
%     end
% end
% 
% 
% 
% %theta1a= asin([Rot(6)
% 
% alpha = zeros(1,SolC);
% beta = zeros(1,SolC);
% gamma = zeros(1,SolC);
% 
% for indexS = 1:SolC
    
    alpha = theta1;
    beta= theta2;
    gamma= theta3;
    
% end


end