%function [] = DWA_3D()
clear;
clc;
%句柄函数，求距离需要使用
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
%%定义无人机初始状态
startPoint = [0 0 0];
endPoint=[100 100 100];

v0=[0 0 0];
% v0=[3 4 3];
%定义初始状态
x = [startPoint v0];
%%定义障碍物
%静态障碍物
sphereInfo.exist = 0;
% sphereInfo = creatSphereObject(sphereInfo);  

%动态障碍物
sphereInfo = creatSphereObjectDynamic(sphereInfo);

%边界限制，防止无人机调整时跑出边界
axisStart = [0 0 0];
axisLWH = [105 105 105];

% global dt;
dt = 0.1;%时间，每条路径由多个值构成，dt为每个点之间的时间间隔
paraT=3;%前向模拟的时间

%%无人机运动学参数
vMax = [5,5,5];%各轴最大速度
acc = [5,5,5];%各轴加速度
VResolution = 0.1;%速度分辨率，用于轨迹推算
model = [vMax acc VResolution];

%评价函数参数[heading,dist,velocity,predictDT]
%航向得分比重，距离得分比重，速度得分比重，向前模拟轨迹的时间
% evalParam = [0.045,0.1,0.1,3.0];
% evalParam = [0.1,0.15,0.1,3.0];%效果较好
evalParam = [0.1,0.2,0.1,3.0];
% evalParam = [0.1,0.1,0.08,3.0];
%模拟区域的范围=边界限制
% area = [0 0 0 105 105 105];

%模拟实验的结果
result.x=[];
result.x=[result.x;x];
k3=0;
colorMatSphere = [0 0 1];%颜色
pellucidity = 0.6; %透明度
%     %画出起点和终点
%     scatter3(startPoint(1),startPoint(1),startPoint(1),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);
%     scatter3(endPoint(1),endPoint(2),endPoint(3),'MarkerEdgeColor','k','MarkerFaceColor','b');
%     text(startPoint(1),startPoint(1),startPoint(1),'起点');
%     text(endPoint(1),endPoint(2),endPoint(3),'终点');
%     %画出所有的障碍物
%     drawSphereObject(sphereInfo,colorMatSphere,pellucidity);
%     grid on;
%     axis equal;
%     axis([0 105 0 105 0 105]);
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
%     hold on;
%主循环
for k2=1:2000
    xt=x;
    [u,trajDB,sphereInfo] = dynamicWindowApproach(x,model,dt,paraT,endPoint,sphereInfo,evalParam);
    %无人机移动到下一时刻的状态量,包括速度和加速度
    x(1)=x(1)+u(1)*dt;
    x(2)=x(2)+u(2)*dt;
    x(3)=x(3)+u(3)*dt;
    x(4)=u(1);
    x(5)=u(2);
    x(6)=u(3);
%     x
    k3=k3+1;
    %记录结果
    result.x=[result.x;x];

    %判断是否到达了终点
    distToGoal = calcuDis(x(:,1:3),endPoint);
    if distToGoal <=3
        disp('==========Arrive Goal!!==========');
        break;
    end

    %%画图显示效果
    hold off;%新图出现，取消原图显示
%     ArrowLength = 0.5;      % 箭头长度
%     figure(1);
%     hold on;
    %无人机运动绘图
    
%     line([result.x(k2,1) result.x(k2+1,1)],[result.x(k2,2) result.x(k2+1,2)],[result.x(k2,3) result.x(k2+1,3)],'Color','k','LineWidth',1);
    plot3([result.x(k2,1) result.x(k2+1,1)],[result.x(k2,2) result.x(k2+1,2)],[result.x(k2,3) result.x(k2+1,3)],'*R');
     hold on;
    %画出所有路径
%     for k6=1:size(result,1)
%         plot3([result.x(k6,1) result.x(k6+1,1)],[result.x(k6,2) result.x(k6+1,2)],[result.x(k6,3) result.x(k6+1,3)],'or');hold on;
%     end
   
    %画出起点和终点
    scatter3(startPoint(1),startPoint(1),startPoint(1),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);
    scatter3(endPoint(1),endPoint(2),endPoint(3),'MarkerEdgeColor','k','MarkerFaceColor','b');
    text(startPoint(1),startPoint(1),startPoint(1),'起点');
    text(endPoint(1),endPoint(2),endPoint(3),'终点');
    %画出所有的障碍物
    drawSphereObject(sphereInfo,colorMatSphere,pellucidity);

%     %画出所有的轨迹,trajDB中存储的是一个一个的x;代码可以运行，但每次运行占用内存较大，不建议展示出来
%     if ~isempty(trajDB) %轨迹非空
%         for it=1:length(trajDB(:,1))/6    %计算所有轨迹数  traj 每6行数据 表示一条轨迹点 
%             ind = 1+(it-1)*6; %第 it 条轨迹对应在traj中的下标 
%             plot3(trajDB(ind,:),trajDB(ind+1,:),trajDB(ind+2,:),'--or');hold on;  
%         end
%     end
    grid on;
    axis equal;
    axis([0 105 0 105 0 105]);
    xlabel('x')
    ylabel('y')
    zlabel('z')
     drawnow limitrate;
    disp(k3);
end
result.x=[result.x;[endPoint 0 0 0]];
   for k6=1:size(result.x,1)-1
        plot3([result.x(k6,1) result.x(k6+1,1)],[result.x(k6,2) result.x(k6+1,2)],[result.x(k6,3) result.x(k6+1,3)],'--g',LineWidth=3);hold on;
    end
%end

