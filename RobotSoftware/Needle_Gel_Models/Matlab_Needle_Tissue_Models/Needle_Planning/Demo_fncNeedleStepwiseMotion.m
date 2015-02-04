%3d Needle motion Velocity.
%This plots the needle as it moves through an typical best practice suture.


close all
clear
clc




%% Needle Position
p0a{1} = [7;5;0;];
p1a{1} = [sin(0.1);cos(0.1);0];
p2a{1} = [0; 0; 1];
p3a{1} = [0;5;0];
da{1}  = 5;


%% Needle Bite
p0a{2} = p0a{1};
p1a{2} = p1a{1};
p2a{2} = p2a{1};
p3a{2} = p3a{1};
da{2}  = da{1};

depthIns = 1;
%% Needle reorient


p0a{3} = [7;5;0;];
p1a{3} = [0;1;0];
p2a{3} = [0; 0; 1];
p3a{3} = [-10;5;5];
da{3}  = 0.05;

%% Needle Drive
p0a{4} = [7;5;0;];
p1a{4} = [0;1;0];
p2a{4} = [0; 0; 1];
p3a{4} = [-10;5;5];

da{4} = 0.1;


structMaterialParams.mus = 0.08;
structMaterialParams.muk = 0.0;
structMaterialParams.K   = diag([.0087; .0087; .012;])/2;
structMaterialParams.alpha = 1.0622;
structMaterialParams.limit = 0.5;

%[0.16 0.0087 1.6717 1.0622]';
structNeedleData = struct('trans',[0; 0; 0],'rot',[sqrt(2)/2;0;-sqrt(2)/2;0]);
structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);
structTissueData =struct('center',[0; -10; 0],'widthX',40,'heightZ',40,'depthY',30);

moviemode = false;



%% allocate Movie Recording Resources.

%% define simulation variables.
dtime = 0.001;
Num = 20000;
time = 0:.001:(Num-1)*dtime;
%Parameterloops

%% allocate large Memory variables Outside repitition loop.


g_0N = cell(1,Num);

tTime = zeros(1,3);









if(moviemode == true)
    p = VideoWriter.getProfiles();
    scrsz = get(0,'ScreenSize');
    figHand = figure('Position',scrsz);
    fps = 15;
    
    
    
    %! mkdir presentationOutput
end


% Load data
%Stage 1:
%p0 is the entry point (in frame 0)
%p1 is the normal vector. (in frame 0)
%p2 is the vector of the gash
%p3 is the location of the gash
%d  is the distance from the







%% allocate large Memory variables.
wrenchM_0 = zeros(6,Num);
wrenchMFr_0 = zeros(6,Num);
wrenchMN_0 = zeros(6,Num);

%,wrenchMC_0]
wrenchM_S = zeros(6,Num);











wX = structTissueData.widthX;
hZ = structTissueData.heightZ;
dY = structTissueData.depthY;
c  = structTissueData.center;
Tissue = repmat(c,1,12)+[-wX/2  wX/2  wX/2 -wX/2 -wX/2  wX/2  wX/2 -wX/2 -wX/2  wX/2  wX/2 -wX/2;...
    dY/2  dY/2 -dY/2 -dY/2  dY/2  dY/2  dY/2  dY/2 -dY/2 -dY/2  dY/2  dY/2;...
    hZ/2  hZ/2  hZ/2  hZ/2  hZ/2  hZ/2 -hZ/2 -hZ/2 -hZ/2 -hZ/2 -hZ/2 -hZ/2];



MovieStr = 'GeneralizedMotion.avi';


if(moviemode == true)
    MoviePathFin = MovieStr;
    vidObj = VideoWriter(MoviePathFin);
    vidObj.FrameRate = fps;
    figTitStr = 'Ideal_fig.png';
    open(vidObj);
end




%% Generate and Plot the motion plan

n_r = structNeedleGeometry.radius;




%

p0 = p0a{1}; %[7;5;0;]
p1 = p1a{1}; %[sin(0.2);cos(0.2);0];
p2 = p2a{1}; %[0; 0; 1];
p3 = p3a{1}; %[0;5;0];
d =   da{1}; %5;

needleState = 1;

g_0N{1} = [ eye(3) [0; 30; 0]; zeros(1,3) 1];

%Loop initialization
tissue1MomOut =0;
tissue0MomOut =0;
tissue10MomOut =0;





% vidObj = avifile('Needle_Position.avi','fps',5,'compression','None');
scrsz = get(0,'ScreenSize');

%scrsz logic:
% if(scrsz(3) < 1400)
%     NeedleFig = figure('Position',[1 1 scrsz(3) scrsz(4)]);
% else
%     NeedleFig = figure('Position',[1 1 scrsz(3)/2 scrsz(4)/2]);
% end
NeedleA = G2struct(g_0N{1});
[TissueState  NeedlePts]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,NeedleA,100);


Num = 20000;
Kf = eye(6)*10;
exited = 0;
counter = 0;
for j = 2:Num
    
    [Vb_0N, gTarg_0N] = fncNeedleStepwiseMotion(g_0N{j-1},needleState,n_r,p0,p1,p2,p3,d);
    
    gdot_0N = g_0N{j-1}*VectHat(Kf*Vb_0N);
    gTemp_0N = g_0N{j-1}+gdot_0N*dtime;
    %This is used to ensure that g stays an ideal transformation
    gTemp_0N(1:3,1:3) = Rot_Proj(gTemp_0N(1:3,1:3));
    g_0N{j} = gTemp_0N;
    
    
    NeedleA = G2struct(g_0N{j});
    if(needleState == 1)
        Needleb = G2struct(gTarg_0N);
    else
        Needleb = G2struct(g_0N{j});
    end
    
    TissueStateOld = TissueState;
    NeedlePtsOld = NeedlePts;
    [TissueState  NeedlePts]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,NeedleA,100);
    [TissueStateG  NeedlePtsG]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,Needleb,100);
    
    
    %Add force  information.
    [wrenchM_0(:,j),wrenchMFr_0(:,j),wrenchMN_0(:,j),wrenchMC_0] = Needle_Cumulative_Forces_Step(TissueStateOld,NeedlePtsOld,TissueState,NeedlePts,structMaterialParams,wrenchMN_0(:,j-1));
    
    wrenchM_S(:,j) = fncWrenchTransform(wrenchM_0(:,j),Ginv(g_0N{j}));
    %wrenchM_S(:,j) = wrenchM_0(:,j);
    
    if( mod(j,20) == 0 && moviemode == true)
        
        
        NeedlePlot(figHand,Tissue,NeedlePtsG,TissueStateG,NeedlePts,TissueState)
        
        
        % pause(1);
        if(moviemode == true)
            currFrame = getframe(figHand);
            writeVideo(vidObj,currFrame);
            
            
        end
        
        
    end
    
    
    switch needleState
        
        case 1
            if(norm(gTarg_0N-g_0N{j}) < 0.1)
                fprintf('The needle has converged to position 1: pre-pose\n');
                
                needleState = 2;
                tTime(1) = time(j);
                
            end
        case 2
            NeedleTans = NeedlePts(:,2:end)-NeedlePts(:,1:end-1);
            needleLenths=(diag(NeedleTans'*NeedleTans)).^(0.5);
            needleIns  = needleLenths.*TissueState;
            
            
            if(sum(needleIns) > depthIns);
                fprintf('The needle has converged to position 2: Initial Bite\n');
                %Find bite angle This code implicitly assumes that the needle is only in one side.:
                ind = find(TissueState > 0);
                beta = NeedlePts(:,max(ind)+1)-NeedlePts(:,min(ind));
               
                needleState = 3;
                p1 = p1a{3};% [0;1;0];
                p3 = p3a{3}; % [-10;5;5];
                
                %Forward motion during alignment.
                d = da{3}; %  0.01;
                tTime(2) = time(j);
                
            end
            
            
        case 3
            if(norm(Vb_0N-[-n_r;0;0;0;0;-1]*d) < 0.001)
                fprintf('The needle has converged to position 3: reorientation\n');
                needleState = 4;
                d = da{4}; % .1;
                tTime(3) = time(j);
                
            end
            
        case 4
            
            
            
    end
    %Update the regrasp variable if neccesary.
    if(needleState > 2)
        %
        
        tissue1Mom = (1:100)*TissueState;
        tissue0Mom = sum(TissueState);
        tissue10Mom = tissue1Mom./tissue0Mom;
        
         
    end
    
    %wrap up Simulation if the needle is done!!!
    if(sum(TissueState) == 0 && needleState > 2)
        exited = 1;
    else
        exited = 0;
    end
    if(exited == 1)
        counter = counter+1;
    else
        counter = 0;
    end
    if(counter > 100)
        fprintf('The Needle finished suturing the tissue\n');
        tTime(4) = time(j);
        
        indEnd =  j;
        
        wrenchOutM_S = wrenchM_S;
        
        
       
        
        break;
    end
    
    
    
    
end

if(moviemode == true)
    close(vidObj);
    %close(figHand);
end


%% Print out other valuable feedback


%% Final Force Plots

colorStr = {'g--','g';'r--','r';'m--','m';'b--','b';'r--','r'};

Mdiv = 50;

%
forceFig = figure('Position',[scrsz(3)/2+1 scrsz(4)/2-300 scrsz(3)/2 scrsz(4)/1.25]);
%figure


subplot 211
title('Linear Forces in Needle Frame');
hold on

plot(time(1:Mdiv:indEnd),wrenchOutM_S(1,1:Mdiv:indEnd),colorStr{1,2});

plot([tTime(1) tTime(1)],[-5 5],colorStr{1,1},[tTime(2) tTime(2)],[-5 5],colorStr{2,1},[tTime(3) tTime(3)],[-5 5],colorStr{3,1},[tTime(4) tTime(4)],[-5 5],colorStr{4,1});

hold off

xlabel('Time (s)');
ylabel('X-force (N)');
%legend('Des_N','Meas_N','Meas_S','Est S');
legend('Meas_C1','Meas_C2','Meas_C3');


subplot 212

hold on

plot([tTime(1) tTime(1)],[-5 5],colorStr{1,1},[tTime(2) tTime(2)],[-5 5],colorStr{2,1},[tTime(3) tTime(3)],[-5 5],colorStr{3,1},[tTime(4) tTime(4)],[-5 5],colorStr{4,1});
plot(time(1:Mdiv:indEnd),wrenchOutM_S(2,1:Mdiv:indEnd),colorStr{1,2});
hold off

xlabel('Time (s)');
ylabel('Y-force (N)');



torqueFig = figure('Position',[1 scrsz(4)/2-300 scrsz(3)/2 scrsz(4)/1.25]);

title('Torques in Needle Frame');


hold on
plot([tTime(1) tTime(1)],[-5 5],colorStr{1,1},[tTime(2) tTime(2)],[-5 5],colorStr{2,1},[tTime(3) tTime(3)],[-5 5],colorStr{3,1},[tTime(4) tTime(4)],[-5 5],colorStr{4,1});
plot(time(1:Mdiv:indEnd),wrenchOutM_S(6,1:Mdiv:indEnd),colorStr{1,2});
hold off

xlabel('Time (s)');
ylabel('Z-Torque (N-mm)');

%     torqueFilestr = sprintf('%s/NeedleTorques.fig',svfile);
%     saveas(torqueFig,torqueFilestr);





