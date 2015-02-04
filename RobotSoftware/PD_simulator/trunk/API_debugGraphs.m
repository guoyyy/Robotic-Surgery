function isSuccess=API_debugGraphs(phome,pReadypoint,pEndpoint,SABiRx,index,ts)
SABiRx1=cell2mat(SABiRx');
assignin('base', 'SABiRx1', SABiRx1);
SABiRx=SABiRx1';
isSuccess=0;
global ref_Needle_Tip_Position
global actual_Needle_Tip_Position
global actual_R
global needleDepth1
global needleForce1_needleframe
if isempty(ref_Needle_Tip_Position)||isempty(actual_Needle_Tip_Position)
    return
end


block1MinMax = [-20 0; -240 -180; 340 360];
block2MinMax = [-12 -7; -220 -200; 345 350];
sabirx_path=figure;
row=1;
for i=1:2
    for j=1:2
        for k=1:2
            block1Corners(row,:) = [block1MinMax(1,i), block1MinMax(2,j), block1MinMax(3,k)];
            block2Corners(row,:) = [block2MinMax(1,i), block2MinMax(2,j), block2MinMax(3,k)];
            row = row+1;
        end
    end
end
hold on
grid on
plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
plot3(pReadypoint(:,2),pReadypoint(:,3),pReadypoint(:,1),'bx');
plot3(pEndpoint(:,2),pEndpoint(:,3),pEndpoint(:,1),'ro');
plot3(pEndpoint(:,2),pEndpoint(:,3),pEndpoint(:,1),'rx');

line(block1Corners([1 2],2),block1Corners([1 2],3),block1Corners([1 2],1),'Color','b');
line(block1Corners([1 3],2),block1Corners([1 3],3),block1Corners([1 3],1),'Color','b');
line(block1Corners([1 5],2),block1Corners([1 5],3),block1Corners([1 5],1),'Color','b');
line(block1Corners([2 4],2),block1Corners([2 4],3),block1Corners([2 4],1),'Color','b');
line(block1Corners([2 6],2),block1Corners([2 6],3),block1Corners([2 6],1),'Color','b');
line(block1Corners([3 4],2),block1Corners([3 4],3),block1Corners([3 4],1),'Color','b');
line(block1Corners([3 7],2),block1Corners([3 7],3),block1Corners([3 7],1),'Color','b');
line(block1Corners([4 8],2),block1Corners([4 8],3),block1Corners([4 8],1),'Color','b');
line(block1Corners([5 6],2),block1Corners([5 6],3),block1Corners([5 6],1),'Color','b');
line(block1Corners([5 7],2),block1Corners([5 7],3),block1Corners([5 7],1),'Color','b');
line(block1Corners([6 8],2),block1Corners([6 8],3),block1Corners([6 8],1),'Color','b');
line(block1Corners([7 8],2),block1Corners([7 8],3),block1Corners([7 8],1),'Color','b');

line(block2Corners([1 2],2),block2Corners([1 2],3),block2Corners([1 2],1),'Color','r');
line(block2Corners([1 3],2),block2Corners([1 3],3),block2Corners([1 3],1),'Color','r');
line(block2Corners([1 5],2),block2Corners([1 5],3),block2Corners([1 5],1),'Color','r');
line(block2Corners([2 4],2),block2Corners([2 4],3),block2Corners([2 4],1),'Color','r');
line(block2Corners([2 6],2),block2Corners([2 6],3),block2Corners([2 6],1),'Color','r');
line(block2Corners([3 4],2),block2Corners([3 4],3),block2Corners([3 4],1),'Color','r');
line(block2Corners([3 7],2),block2Corners([3 7],3),block2Corners([3 7],1),'Color','r');
line(block2Corners([4 8],2),block2Corners([4 8],3),block2Corners([4 8],1),'Color','r');
line(block2Corners([5 6],2),block2Corners([5 6],3),block2Corners([5 6],1),'Color','r');
line(block2Corners([5 7],2),block2Corners([5 7],3),block2Corners([5 7],1),'Color','r');
line(block2Corners([6 8],2),block2Corners([6 8],3),block2Corners([6 8],1),'Color','r');
line(block2Corners([7 8],2),block2Corners([7 8],3),block2Corners([7 8],1),'Color','r');

% plot3(block1entry(2),block1entry(3),block1entry(1),'bo');
% plot3(block1exit(2),block1exit(3),block1exit(1),'bo');
% plot3(block2entry(2),block2entry(3),block2entry(1),'ro');
% plot3(block2exit(2),block2exit(3),block2exit(1),'ro');

plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'r');
%     legend('phome','pReadypoint','endpoint','SABiRx');
title('reference needle path');
%debug graph 2
%save(sprintf('C:\\Documents and Settings\\mercis\\My Documents\\MATLAB\\SABiR_PD_Sim_11-1-10\\SABiR PD Sim\\outdata-11-17\\outdata%d.mat',targetnum), 'outdata', 'pinsert','rinsert','insertdist');



needle_path=figure;
hold on
grid on
%draw needle path
plot3(ref_Needle_Tip_Position(:,2),ref_Needle_Tip_Position(:,3),ref_Needle_Tip_Position(:,1),'r');
plot3(ref_Needle_Tip_Position(:,2),ref_Needle_Tip_Position(:,3),ref_Needle_Tip_Position(:,1),'r.');
plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'b');
plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'bo');
plot3(actual_Needle_Tip_Position(:,2),actual_Needle_Tip_Position(:,3),actual_Needle_Tip_Position(:,1),'bx');
plot3(phome(:,2),phome(:,3),phome(:,1),'bo');
plot3(pReadypoint(:,2),pReadypoint(:,3),pReadypoint(:,1),'bx');
plot3(pEndpoint(:,2),pEndpoint(:,3),pEndpoint(:,1),'ro');
plot3(pEndpoint(:,2),pEndpoint(:,3),pEndpoint(:,1),'rx');
%     plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'b');
%legend('reference','actual', 'actual','home','target','endpoint');%, 'SABiRx')


line(block1Corners([1 2],2),block1Corners([1 2],3),block1Corners([1 2],1),'Color','b');
line(block1Corners([1 3],2),block1Corners([1 3],3),block1Corners([1 3],1),'Color','b');
line(block1Corners([1 5],2),block1Corners([1 5],3),block1Corners([1 5],1),'Color','b');
line(block1Corners([2 4],2),block1Corners([2 4],3),block1Corners([2 4],1),'Color','b');
line(block1Corners([2 6],2),block1Corners([2 6],3),block1Corners([2 6],1),'Color','b');
line(block1Corners([3 4],2),block1Corners([3 4],3),block1Corners([3 4],1),'Color','b');
line(block1Corners([3 7],2),block1Corners([3 7],3),block1Corners([3 7],1),'Color','b');
line(block1Corners([4 8],2),block1Corners([4 8],3),block1Corners([4 8],1),'Color','b');
line(block1Corners([5 6],2),block1Corners([5 6],3),block1Corners([5 6],1),'Color','b');
line(block1Corners([5 7],2),block1Corners([5 7],3),block1Corners([5 7],1),'Color','b');
line(block1Corners([6 8],2),block1Corners([6 8],3),block1Corners([6 8],1),'Color','b');
line(block1Corners([7 8],2),block1Corners([7 8],3),block1Corners([7 8],1),'Color','b');

line(block2Corners([1 2],2),block2Corners([1 2],3),block2Corners([1 2],1),'Color','r');
line(block2Corners([1 3],2),block2Corners([1 3],3),block2Corners([1 3],1),'Color','r');
line(block2Corners([1 5],2),block2Corners([1 5],3),block2Corners([1 5],1),'Color','r');
line(block2Corners([2 4],2),block2Corners([2 4],3),block2Corners([2 4],1),'Color','r');
line(block2Corners([2 6],2),block2Corners([2 6],3),block2Corners([2 6],1),'Color','r');
line(block2Corners([3 4],2),block2Corners([3 4],3),block2Corners([3 4],1),'Color','r');
line(block2Corners([3 7],2),block2Corners([3 7],3),block2Corners([3 7],1),'Color','r');
line(block2Corners([4 8],2),block2Corners([4 8],3),block2Corners([4 8],1),'Color','r');
line(block2Corners([5 6],2),block2Corners([5 6],3),block2Corners([5 6],1),'Color','r');
line(block2Corners([5 7],2),block2Corners([5 7],3),block2Corners([5 7],1),'Color','r');
line(block2Corners([6 8],2),block2Corners([6 8],3),block2Corners([6 8],1),'Color','r');
line(block2Corners([7 8],2),block2Corners([7 8],3),block2Corners([7 8],1),'Color','r');
title('actual needle path')
%%
needle_path_with_direction=figure;
hold on
grid on
%draw needle path
plotn=100; %plot every n points
plot3(actual_Needle_Tip_Position(1:plotn:end,2),actual_Needle_Tip_Position(1:plotn:end,3),actual_Needle_Tip_Position(1:plotn:end,1),'b');
plot3(actual_Needle_Tip_Position(1:plotn:end,2),actual_Needle_Tip_Position(1:plotn:end,3),actual_Needle_Tip_Position(1:plotn:end,1),'b.');
plot3(phome(:,2),phome(:,3),phome(:,1),'bo');

%plot some lines showing the needle's direction

for i=1:plotn:length(actual_Needle_Tip_Position)
    p0 = actual_Needle_Tip_Position(i,:);
    r = actual_R(:,i);
    l=0.5; %(half)length of the line showing the needle direction
    p1 = p0-l*r';
    p2 = p0+l*r';
    line([p1(2) p2(2)],[p1(3) p2(3)], [p1(1) p2(1)],'Color','g');
    plot3(p1(2),p1(3),p1(1),'go');
end

plot3(pReadypoint(:,2),pReadypoint(:,3),pReadypoint(:,1),'bx');
plot3(pEndpoint(:,2),pEndpoint(:,3),pEndpoint(:,1),'ro');
plot3(pEndpoint(:,2),pEndpoint(:,3),pEndpoint(:,1),'rx');
%     plot3(SABiRx(2,:),SABiRx(3,:),SABiRx(1,:),'b');


line(block1Corners([1 2],2),block1Corners([1 2],3),block1Corners([1 2],1),'Color','b');
line(block1Corners([1 3],2),block1Corners([1 3],3),block1Corners([1 3],1),'Color','b');
line(block1Corners([1 5],2),block1Corners([1 5],3),block1Corners([1 5],1),'Color','b');
line(block1Corners([2 4],2),block1Corners([2 4],3),block1Corners([2 4],1),'Color','b');
line(block1Corners([2 6],2),block1Corners([2 6],3),block1Corners([2 6],1),'Color','b');
line(block1Corners([3 4],2),block1Corners([3 4],3),block1Corners([3 4],1),'Color','b');
line(block1Corners([3 7],2),block1Corners([3 7],3),block1Corners([3 7],1),'Color','b');
line(block1Corners([4 8],2),block1Corners([4 8],3),block1Corners([4 8],1),'Color','b');
line(block1Corners([5 6],2),block1Corners([5 6],3),block1Corners([5 6],1),'Color','b');
line(block1Corners([5 7],2),block1Corners([5 7],3),block1Corners([5 7],1),'Color','b');
line(block1Corners([6 8],2),block1Corners([6 8],3),block1Corners([6 8],1),'Color','b');
line(block1Corners([7 8],2),block1Corners([7 8],3),block1Corners([7 8],1),'Color','b');

line(block2Corners([1 2],2),block2Corners([1 2],3),block2Corners([1 2],1),'Color','r');
line(block2Corners([1 3],2),block2Corners([1 3],3),block2Corners([1 3],1),'Color','r');
line(block2Corners([1 5],2),block2Corners([1 5],3),block2Corners([1 5],1),'Color','r');
line(block2Corners([2 4],2),block2Corners([2 4],3),block2Corners([2 4],1),'Color','r');
line(block2Corners([2 6],2),block2Corners([2 6],3),block2Corners([2 6],1),'Color','r');
line(block2Corners([3 4],2),block2Corners([3 4],3),block2Corners([3 4],1),'Color','r');
line(block2Corners([3 7],2),block2Corners([3 7],3),block2Corners([3 7],1),'Color','r');
line(block2Corners([4 8],2),block2Corners([4 8],3),block2Corners([4 8],1),'Color','r');
line(block2Corners([5 6],2),block2Corners([5 6],3),block2Corners([5 6],1),'Color','r');
line(block2Corners([5 7],2),block2Corners([5 7],3),block2Corners([5 7],1),'Color','r');
line(block2Corners([6 8],2),block2Corners([6 8],3),block2Corners([6 8],1),'Color','r');
line(block2Corners([7 8],2),block2Corners([7 8],3),block2Corners([7 8],1),'Color','r');

title('actual needle path with direction')

needle_depth=figure;
plot(needleDepth1)
title('needle depth');

needle_force=figure;
plot(needleForce1_needleframe)
title('needle force');

for i=1:length(actual_Needle_Tip_Position)-1
    needleVelocity(i,:) = (actual_Needle_Tip_Position(i+1,:) - actual_Needle_Tip_Position(i,:))/ts;
end
needle_velocity=figure;
plot(needleVelocity)
title('needle velocity');

for i=1:length(actual_Needle_Tip_Position)
    needleError(i) = norm(actual_Needle_Tip_Position(i,:) - ref_Needle_Tip_Position(i,:));
end
needle_error=figure;
plot(needleError)
title('needle error');

dir=[fileparts(fileparts(pwd)),'/raw_trajectory/figures/'];
fig1_name=[dir,'sabirx_path',num2str(index)];
fig2_name=[dir,'needle_path',num2str(index)];
fig3_name=[dir,'needle_path_with_direction',num2str(index)];
fig4_name=[dir,'needle_depth',num2str(index)];
fig5_name=[dir,'needle_force',num2str(index)];
fig6_name=[dir,'needle_velocity',num2str(index)];
fig7_name=[dir,'needle_error',num2str(index)];
saveas(sabirx_path,fig1_name);
saveas(needle_path,fig2_name);
saveas(needle_path_with_direction,fig3_name);
saveas(needle_depth,fig4_name);
saveas(needle_force,fig5_name);
saveas(needle_velocity,fig6_name);
saveas(needle_error,fig7_name);