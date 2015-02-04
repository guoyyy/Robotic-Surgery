function [t0valid errortype stats]=API_checkPoint(P0,R0)
profile on
profile -memory on
%profile -history
yoffset=0;
xoffset=0;
zoffset=0;
T_BackFront_mount = [ 
    0.9996514523262   0.0239862420680  -0.0110287829636   -2.6398505898723
    -0.0238096717995   0.9995905085098   0.0158718249104  -45.1866546135221
    0.0114049722052  -0.0156037011200   0.9998132081146  197.5288079400060
    0                 0                 0                  1];
ln= 68.6308+65.19+161.86+18.00+14.91;
R0=R0';
%setupConstants;
errortype='null';
[t01,t02,t03,t04,t05]=SABiRIK(P0,R0,xoffset,yoffset,zoffset,ln,T_BackFront_mount);
if strcmp('error1', t01)||strcmp('error2', t01)
    t0valid=0;
    errortype=t01;
else
t0valid = CheckJointAngles([t01,t02,t03,t04,t05]);
end
if ~t0valid 
    disp('POsRefConfigSpace: invalid configuration of ready or end point');
    errortype='error3';
end
stats = profile('info');

profile off
profile clear