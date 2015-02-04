function isValid = CheckJointAngles(Angles)
%Check Limits after Inverse Kinematics
PhysLimits =[
   3.421034655796548   4.712388980384690
   4.712388980384690   5.981899088886965
  -0.261799387799149   0.296076370022756
   3.473848497195918   4.712388980384690
   4.712388980384690   5.937385325132857];

% %if using these numbers elsewhere, note the biases if necessary
% t1bias= 225*pi/180;                      % Home position for axis 1
% t2bias= 315*pi/180;                      % Home position for axis 2
% t3bias= 0;                               % Home position for axis 3
% t4bias= 225*pi/180;                      % Home position for axis 4
% t5bias= 315*pi/180;                      % Home position for axis 5



isValid = 1;

    for i=1:5
        if Angles(i)>PhysLimits(i,2) || Angles(i)<PhysLimits(i,1)
            isValid=0;
            return;
        end
    end
