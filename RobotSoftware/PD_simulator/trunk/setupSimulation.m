function [time1,turnaroundtime,turnaroundts,TotalTime,SABiRx,Rneedle] = setupSimulation(ptarget,rtarget,insertdist,ts)


%% Generate trajectories
phome =[-10.7505,-206.2838,330.8692]; %needle position at the robot's home position
rhome = [-0.0327;0.0020;0.9995];      %needle vector at the robot's home position


[time1,turnaroundtime,turnaroundts,TotalTime,SABiRx,Rneedle] = Compute_single_trajectory(phome,rhome,ptarget,rtarget,insertdist,ts);

