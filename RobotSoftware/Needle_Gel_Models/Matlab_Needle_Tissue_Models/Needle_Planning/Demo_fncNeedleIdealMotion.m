%3d Needle motion Velocity.
%This plots the needle as it moves through an Ideal Suture Motion.


close all
clear
clc




%% Needle Entrance/Exit Position
I = [7;5;0;];
O = [-10;5;5];

omega = 5;



%Tissue Parameters:
structMaterialParams.mus = 0.08;
structMaterialParams.muk = 0.0;
structMaterialParams.K   = reshape(diag([.0087; .0087; .012;])/2,9,1);
structMaterialParams.alpha = 1.0622;
structMaterialParams.limit = 0.5;


structNeedleData = struct('trans',[0; 0; 0],'rot',[sqrt(2)/2;0;-sqrt(2)/2;0]);
structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);
structTissueData =struct('center',[0; -10; 0],'widthX',40,'heightZ',40,'depthY',30);

moviemode = true;


%Include the Needle Motor transform.


%% allocate Movie Recording Resources.

%% define simulation variables.
dtime = 0.001;
Num = 20000;
time = 0:.001:(Num-1)*dtime;
%Parameterloops

%% allocate large Memory variables.

g_0N = cell(1,Num);

tTime = zeros(1,3);
%% allocate large Memory variables.

wrenchM_0 = zeros(6,Num);
wrenchMFr_0 = zeros(6,Num);
wrenchMN_0 = zeros(6,Num);


wrenchM_S = zeros(6,Num);


if(moviemode == true)
    p = VideoWriter.getProfiles();
    scrsz = get(0,'ScreenSize');
    figHand = figure('Position',scrsz);
    fps = 15;
    
    
    
    %! mkdir presentationOutput
end






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

qState = [0 0 0 0.0];

g_SN = fncSingleDofFK(qState, n_r);

%Compute The initial pose using the depth information:
d1a = norm(I-O)/2;

Center = [0; 1; 0]*sqrt(n_r^2-d1a^2)+I/2+O/2;

Rot = Rot_z(-pi/2); 

g_0N{1} = [ Rot Center+Rot'*[0; n_r; 0]; zeros(1,3) 1];


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
needleState = 0;
exited = 0;
counter = 0;
for j = 2:Num
    
    [Vb_0N ] = [-1*n_r; zeros(4,1); -1]*omega;
    
    gdot_0N = g_0N{j-1}*VectHat(Vb_0N);
    gTemp_0N = g_0N{j-1}+gdot_0N*dtime;
    %This is used to ensure that g stays an ideal transformation
    gTemp_0N(1:3,1:3) = Rot_Proj(gTemp_0N(1:3,1:3));
    g_0N{j} = gTemp_0N;
    
    g_0S = g_0N{j}*Ginv(g_SN);
    
    NeedleA = G2struct(g_0N{j});
    Needleb = G2struct(g_0S);
    
    TissueStateOld = TissueState;
    NeedlePtsOld = NeedlePts;
    [TissueState  NeedlePts]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,NeedleA,100);
    [TissueStateG  NeedlePtsG]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,Needleb,100);
    
    
    %Add force  information.
    [wrenchM_0(:,j),wrenchMFr_0(:,j),wrenchMN_0(:,j),wrenchMC_0] = Needle_Cumulative_Forces_Step(TissueStateOld,NeedlePtsOld,TissueState,NeedlePts,structMaterialParams,wrenchMN_0(:,j-1));
    
    wrenchM_S(:,j) = fncWrenchTransform(wrenchM_0(:,j),Ginv(g_0N{j}));
    %wrenchM_S(:,j) = wrenchM_0(:,j);
    
    if( mod(j,20) == 0 && moviemode == true)
        
        
        NeedlePlot(figHand,structTissueData,NeedlePtsG,TissueStateG,NeedlePts,TissueState)
        
        
        % pause(1);
        if(moviemode == true)
            currFrame = getframe(figHand);
            writeVideo(vidObj,currFrame);
            
            
        end
        
        
    end
    
    if(needleState ==0)
        if(sum(TissueState) > 0)
            needleState = 1;
        end
    end
  
    %wrap up Simulation if the needle is done!!!
    if(sum(TissueState) == 0 && needleState == 1 )
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
        tTime(2) = time(j);
        
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

plot([tTime(1) tTime(1)],[-5 5],colorStr{1,1},[tTime(2) tTime(2)],[-5 5],colorStr{2,1});

hold off

xlabel('Time (s)');
ylabel('X-force (N)');
%legend('Des_N','Meas_N','Meas_S','Est S');
legend('Meas_C1','Meas_C2','Meas_C3');


subplot 212

hold on

plot([tTime(1) tTime(1)],[-5 5],colorStr{1,1},[tTime(2) tTime(2)],[-5 5],colorStr{2,1});
plot(time(1:Mdiv:indEnd),wrenchOutM_S(2,1:Mdiv:indEnd),colorStr{1,2});
hold off

xlabel('Time (s)');
ylabel('Y-force (N)');



torqueFig = figure('Position',[1 scrsz(4)/2-300 scrsz(3)/2 scrsz(4)/1.25]);

title('Torques in Needle Frame');


hold on
plot([tTime(1) tTime(1)],[-5 5],colorStr{1,1},[tTime(2) tTime(2)],[-5 5],colorStr{2,1});
plot(time(1:Mdiv:indEnd),wrenchOutM_S(6,1:Mdiv:indEnd),colorStr{1,2});
hold off

xlabel('Time (s)');
ylabel('Z-Torque (N-mm)');

%     torqueFilestr = sprintf('%s/NeedleTorques.fig',svfile);
%     saveas(torqueFig,torqueFilestr);





