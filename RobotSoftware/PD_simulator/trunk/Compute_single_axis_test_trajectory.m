function  [time,TotalTime,SABiRx,Rneedle] = Compute_single_axis_test_trajectory(t1start,t2start,t3start,t4start,t5start,naxis,dtheta,pausetime,ts)
%create a trajectory that moves only one axis. This is done in a rather
%inefficent way, but who cares.

ControlReadyTime = 0;

%% Generate Reference Signal
maxa=100.00;%For experiments (c2.X use maxa=1 and maxahome=100)
maxahome=100;

maxwdot = .01;

t1end = t1start;
t2end = t2start;
t3end = t3start;
t4end = t4start;
t5end = t5start;

switch naxis
    case 1
        t1end = t1start+dtheta;
    case 2
        t2end = t2start+dtheta;
    case 3
        t3end = t3start+dtheta;
    case 4
        t4end = t4start+dtheta;
    case 5
        t5end = t5start+dtheta;
end

insertTime = dtheta*50; %go at the rate of 50 seconds per radian


%go out
npoints = ceil(insertTime/ts);
t1 = linspace(t1start,t1end,npoints);
t2 = linspace(t2start,t2end,npoints);
t3 = linspace(t3start,t3end,npoints);
t4 = linspace(t4start,t4end,npoints);
t5 = linspace(t5start,t5end,npoints);
time_1 = linspace(0,insertTime,npoints);

SABiRx = zeros(3,npoints);
Rneedle = zeros(3,npoints);
for i=1:npoints 
    [SABiRx_1(:,i), Rneedle_1(:,i),front,back] = SABiRFK([t1(i),t2(i),t3(i),t4(i),t5(i)]);
end

t1last = t1(end);
t2last = t2(end);
t3last = t3(end);
t4last = t4(end);
t5last = t5(end);

%pause
npoints = ceil(pausetime/ts);
time_2 = linspace(time_1(end),time_1(end)+pausetime,npoints);
t1 = linspace(t1last,t1last,npoints);
t2 = linspace(t2last,t2last,npoints);
t3 = linspace(t3last,t3last,npoints);
t4 = linspace(t4last,t4last,npoints);
t5 = linspace(t5last,t5last,npoints);
for i=1:npoints 
    [SABiRx_2(:,i), Rneedle_2(:,i),front,back] = SABiRFK([t1(i),t2(i),t3(i),t4(i),t5(i)]);
end

%go back
extractTime = insertTime;
npoints = ceil(extractTime/ts);
t1 = linspace(t1last,t1start,npoints);
t2 = linspace(t2last,t2start,npoints);
t3 = linspace(t3last,t3start,npoints);
t4 = linspace(t4last,t4start,npoints);
t5 = linspace(t5last,t5start,npoints);

time_3 = linspace(time_2(end),extractTime+time_2(end),npoints);

SABiRx_3 = zeros(3,npoints);
Rneedle_3 = zeros(3,npoints);
for i=1:npoints 
    [SABiRx_3(:,i), Rneedle_3(:,i),front,back] = SABiRFK([t1(i),t2(i),t3(i),t4(i),t5(i)]);
end



time = [time_1,time_2,time_3];
SABiRx=[SABiRx_1,SABiRx_2,SABiRx_3];
Rneedle=[Rneedle_1,Rneedle_2, Rneedle_3];

TotalTime = time(end);





maxlen = max([length(time) length(SABiRx) length(Rneedle)]);
if length(time) ~= maxlen || length(SABiRx) ~= maxlen || length(Rneedle) ~= maxlen
    error('Compute_single_trajectory: reference array lengths are not identical');
end

