function [isValid errortype]=API_checkRefTrajectory(SABiRx,Rneedle)
SABiRx1=cell2mat(SABiRx');
%assignin('base', 'SABiRx1', SABiRx1);
SABiRx=SABiRx1';
Rneedle1=cell2mat(Rneedle');
%assignin('base', 'Rneedle', Rneedle1);
Rneedle=Rneedle1';
yoffset=0;
xoffset=0;
zoffset=0;
T_BackFront_mount = [ 
    0.9996514523262   0.0239862420680  -0.0110287829636   -2.6398505898723
    -0.0238096717995   0.9995905085098   0.0158718249104  -45.1866546135221
    0.0114049722052  -0.0156037011200   0.9998132081146  197.5288079400060
    0                 0                 0                  1];
ln= 68.6308+65.19+161.86+18.00+14.91;
errortype='null';
isValid=1;
for jj=1:length(SABiRx)
    
    [theta1(jj),theta2(jj),theta3(jj),theta4(jj),theta5(jj)]=SABiRIK(SABiRx(:,jj),Rneedle(:,jj),xoffset,yoffset,zoffset,ln,T_BackFront_mount);
    if ~CheckJointAngles( [theta1(jj),theta2(jj),theta3(jj),theta4(jj),theta5(jj)] );
        disp('Invalid reference path - joint angles out of bounds');
        isValid=0;
        errortype='error3';
        return
    end
end