在上一篇博客中，障碍物都是静态障碍物，DWA算法实际上是可以实现动态避障的，这里我们补充其动态避障代码。  
代码的关键主要是动态障碍物的定义。  
creatSphereObjectDynamic.m(创建动态障碍物，静态障碍物设置为速度为0的动态障碍物)

```java
function sphereInfo= creatSphereObjectDynamic(sphereInfo)

sphereInfo.centerX = [70 80 20 40];
sphereInfo.centerY = [70 80 20 40];
sphereInfo.centerZ = [70 80 20 40];
sphereInfo.originalcenterX = [70 80 20 40];
sphereInfo.originalcenterY = [70 80 20 40];
sphereInfo.originalcenterZ = [70 80 20 40];
sphereInfo.radius = [5 8 5 8];
sphereInfo.vX = [0 0 0.1 0];
sphereInfo.vY = [0 0 0.1 0];
sphereInfo.vZ = [0 0 0.1 0];
sphereInfo.limtX = 10;
sphereInfo.limtY = 10;
sphereInfo.limtZ = 10;
sphereInfo.exist = 1;
end
```

dynamicWindowApproach.m（动态窗口的计算发生的变化，因为需要更新动态障障碍物的位置）

```java
function [u,trajDB,sphereInfo] = dynamicWindowApproach(x,model,dt,paraT,endPiont,sphereInfo,evalParam)
%%DWA算法实现
%输入
%输出

%速度采样
%根据当前状态和运动模型计算当前参数允许的范围
%计算动态窗口
vs = calcDynamicWindow(x,model,dt);

%评价函数计算,推算可用的轨迹
[evalDB,trajDB] = Evaluation(vs,model,paraT,x,endPiont,sphereInfo,dt);
%更新障碍物信息
 for n2 = 1:size(sphereInfo.centerX,2)
        %X
        if sphereInfo.centerX(n2) >= sphereInfo.originalcenterX(n2)+sphereInfo.limtX ||...
                sphereInfo.centerX(n2) <= sphereInfo.originalcenterX(n2)-sphereInfo.limtX
        sphereInfo.vX(n2)=-sphereInfo.vX(n2);
        end
         %sphereInfo.centerX(n1)=sphereInfo.centerX(n1)+sphereInfo.vX(n1)*t1;
         %Y
        if sphereInfo.centerY(n2) >= sphereInfo.originalcenterY(n2)+sphereInfo.limtY ||...
                sphereInfo.centerY(n2) <= sphereInfo.originalcenterY(n2)-sphereInfo.limtY
        sphereInfo.vY(n2)=-sphereInfo.vY(n2);
        end
         %sphereInfo.centerY(n1)=sphereInfo.centerY(n1)+sphereInfo.vY(n1)*t1;
         %z
        if sphereInfo.centerZ(n2) >= sphereInfo.originalcenterZ(n2)+sphereInfo.limtZ ||...
                sphereInfo.centerZ(n2) <= sphereInfo.originalcenterZ(n2)-sphereInfo.limtZ
        sphereInfo.vZ(n2)=-sphereInfo.vZ(n2);
        end
    sphereInfo.centerX(n2)=sphereInfo.centerX(n2)+sphereInfo.vX(n2)*paraT;
    sphereInfo.centerY(n2)=sphereInfo.centerY(n2)+sphereInfo.vY(n2)*paraT;
    sphereInfo.centerZ(n2)=sphereInfo.centerZ(n2)+sphereInfo.vZ(n2)*paraT;
 end
% trajDB
% evalDB
if isempty(evalDB)
    disp('no path to goal!!');
    u=[0 0 0];return;
end
%各评价函数归一化处理
if sum(evalDB(:,4)) ~= 0
    sum(evalDB(:,4));
    evalDB(:,4) =evalDB(:,4)/sum(evalDB(:,4));
end
if sum(evalDB(:,5)) ~= 0
    evalDB(:,5) =evalDB(:,5)/sum(evalDB(:,5));
end
if sum(evalDB(:,6)) ~= 0
    evalDB(:,6) =evalDB(:,6)/sum(evalDB(:,6));
end
%最终评价函数计算，从所有的轨迹中找出性能最优的一组
feval=[];
for k1 = 1:length(evalDB(:,1))
    feval = [feval;evalParam(1:3)*evalDB(k1,4:6)'];
end
evalDB=[evalDB feval];%纪录每一组轨迹的得分
[mav,ind] =max(feval);
u=evalDB(ind,1:3)%返回最优的速度值

end
```

calcDynamicWindow.m（速度采样空间）

```java
function [vs] = calcDynamicWindow(x,model,dt)
%速度采样空间计算
%主要考虑速度限制和加速度限制
% 速度限制 (三轴最小速度和三轴最大速度)
vs = [0 0 0 model(:,1:3)];
% %考虑无人机全向运动
% vs = [-model(:,1:3) model(:,1:3)];
%加速度限制
vd =[x(4)-model(4)*dt x(5)-model(5)*dt x(6)-model(6)*dt...
    x(4)+model(4)*dt x(5)+model(5)*dt x(6)+model(6)*dt];
%速度采样空间（未考虑障碍物限制）
vs =[max([vs(1) vd(1)]) max([vs(2) vd(2)]) max([vs(3) vd(3)])...
    min([vs(4) vd(4)]) min([vs(5) vd(5)]) min([vs(6) vd(6)])];


end
```

Evaluation.m（评价函数）

```java
function [evalDB,trajDB] = Evaluation(vs,model,paraT,x,endPiont,sphereInfo,dt)
%评价函数计算
%评价函数参数存储、轨迹点位置状态存储
evalDB = [];
trajDB = [];
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
for vtx =vs(1):model(7):vs(4)
    for vty=vs(2):model(7):vs(5)
        for vtz=vs(3):model(7):vs(6)
            %轨迹推算
            [xTemp,traj] = generateTrajectory(vtx,vty,vtz,paraT,x,dt);
            %各评价函数的计算
            %航向评价指标
            heading = calcHeadingEval(xTemp,endPiont);
            %距离评价指标
%             [dist,flag] = calcDisEval(xTemp,sphereInfo);%只存在静态障碍物时
            [dist,flag] = calcDisEval(xTemp,sphereInfo,dt,paraT);%考虑障碍物移动
            %速度得分指标
            vel = calcuDis(xTemp(:,4:6),[0 0 0]);

            %计算制动距离（速度采样第三部分，保证不发生碰撞）
            stopDist = calcBreakingDist(xTemp,model,dt);

            %当障碍物距离大于制动距离且终点不发生碰撞时

            if dist >stopDist && flag ==0
                evalDB=[evalDB;[vtx vty vtz heading dist vel]];
                trajDB=[trajDB;traj];
            end
        end
    end
end
end

%%轨迹推算
function [xTemp,traj] = generateTrajectory(vtx,vty,vtz,paraT,x,dt)
t1 = 0;
 traj = [];
xTemp = x;
while t1 <= paraT
    %更新时间
    t1 = t1+dt; 
    %更新状态,假设前向模拟的速度值为恒定值
    xTemp = [xTemp(1)+dt*vtx xTemp(2)+dt*vty xTemp(3)+dt*vtz vtx vty vtz];
    %将轨迹和状态保存在traj中
    traj = [traj;xTemp];
end
end
%%航向评价指标
function [heading] = calcHeadingEval(x,endPiont)
%以运动方向向量和当前位置与终点连线的向量的夹角(单位为°)
theta = 180-acosd(dot([endPiont(1)-x(1),endPiont(2)-x(2),endPiont(3)-x(3)], ...
    [x(4),x(5),x(6)])/(norm([endPiont(1)-x(1),endPiont(2)-x(2),endPiont(3)-x(3)]) ...
    *norm([x(4),x(5),x(6)])));
if theta <= 0
    heading = -theta;
else
    heading = theta;
end

% if max(x) ==0
%     heading =0;
% end
if isnan(heading)
    heading =0;
end
end

% % %%障碍物距离指标
% % function [dist,flag] = calcDisEval(x,sphereInfo)
% % calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
% % dist = 100; %没有障碍物时的默认值
% % flag =0;%碰到障碍物则置为1；
% % for n1 = 1:size(sphereInfo.centerX,2)
% %     center =[sphereInfo.centerX(n1),sphereInfo.centerY(n1),sphereInfo.centerZ(n1)];
% %     disttemp = calcuDis(x(:,1:3),center)-sphereInfo.radius(n1);
% %     if(disttemp < 0)
% %         flag =1;
% %         break;
% %     else
% %         %选取所有障碍物中距离最小值
% %         if dist > disttemp
% %             dist =disttemp;
% %         end
% %     end
% % end
% % %%%需要根据实际更改
% % %限定障碍物距离最大值，如果不限定，则其占比会非常大 20效果较好
% % if dist >= 20
% %     dist = 20;
% % end
% % end

%%考虑动态的障碍物距离指标
function [dist,flag] = calcDisEval(x,sphereInfo,dt,paraT)
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
dist = 100; %没有障碍物时的默认值
flag =0;%碰到障碍物则置为1；
for t1 =0:dt:paraT
    for n1 = 1:size(sphereInfo.centerX,2)
        %考虑障碍物移动，当球心位置大于或小于限制值时，运动方向反向
        
        %X
        if sphereInfo.centerX(n1) >= sphereInfo.originalcenterX(n1)+sphereInfo.limtX ||...
                sphereInfo.centerX(n1) <= sphereInfo.originalcenterX(n1)-sphereInfo.limtX
        sphereInfo.vX(n1)=-sphereInfo.vX(n1);
        end
         %sphereInfo.centerX(n1)=sphereInfo.centerX(n1)+sphereInfo.vX(n1)*t1;
         %Y
        if sphereInfo.centerY(n1) >= sphereInfo.originalcenterY(n1)+sphereInfo.limtY ||...
                sphereInfo.centerY(n1) <= sphereInfo.originalcenterY(n1)-sphereInfo.limtY
        sphereInfo.vY(n1)=-sphereInfo.vY(n1);
        end
         %sphereInfo.centerY(n1)=sphereInfo.centerY(n1)+sphereInfo.vY(n1)*t1;
         %z
        if sphereInfo.centerZ(n1) >= sphereInfo.originalcenterZ(n1)+sphereInfo.limtZ ||...
                sphereInfo.centerZ(n1) <= sphereInfo.originalcenterZ(n1)-sphereInfo.limtZ
        sphereInfo.vZ(n1)=-sphereInfo.vZ(n1);
        end
         %sphereInfo.centerZ(n1)=sphereInfo.centerZ(n1)+sphereInfo.vZ(n1)*t1;
        center =[sphereInfo.centerX(n1)+sphereInfo.vX(n1)*t1,...
            sphereInfo.centerY(n1)+sphereInfo.vY(n1)*t1,...
            sphereInfo.centerZ(n1)+sphereInfo.vZ(n1)*t1];
        disttemp = calcuDis(x(:,1:3),center)-sphereInfo.radius(n1);
        if(disttemp < 0)
            flag =1; %发生碰撞设置为1
            break;
        else
        %选取所有障碍物中距离最小值
            if dist > disttemp
            dist =disttemp;
            end
        end
    end  
end

%%%需要根据实际更改
%限定障碍物距离最大值，如果不限定，则其占比会非常大 20效果较好
if dist >= 20
    dist = 20;
end
% sphereInfo.centerX(n1)=sphereInfo.centerX(n1)+sphereInfo.vX(n1)*paraT;
% sphereInfo.centerY(n1)=sphereInfo.centerY(n1)+sphereInfo.vY(n1)*paraT;
% sphereInfo.centerZ(n1)=sphereInfo.centerZ(n1)+sphereInfo.vZ(n1)*paraT;
end

%%计算制动距离
function stopDist = calcBreakingDist(x,model,dt)
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
%将曲线的距离转换为一小段一小段直线
stopDist = 0;
vx=x(4);
vy=x(5);
vz=x(6);
while vx>0 || vy > 0 || vz >0
    stopDist = stopDist+calcuDis([vx*dt vy*dt vz*dt],[0 0 0]);
    if vx>0
        vx=vx-model(4)*dt;
    end
    if vy>0
        vy=vy-model(5)*dt;
    end
    if vz>0
        vz=vz-model(6)*dt;
    end
end
end
```

drawSphereObject.m（画障碍物）

```java
function drawSphereObject(sphereInfo,colorMatSphere,pellucidity)
 
if sphereInfo.exist
    for k1 = 1:size(sphereInfo.centerX,2)
        xCoor = sphereInfo.centerX(k1);
        yCoor = sphereInfo.centerY(k1);
        zCoor = sphereInfo.centerZ(k1);
        radius = sphereInfo.radius(k1);
        
        [x,y,z] = sphere(50);
        mesh(x*radius+xCoor,y*radius+yCoor,z*radius+zCoor,'FaceColor',colorMatSphere,'EdgeColor','none','FaceAlpha',pellucidity);
        
        
    end 
    
end
 
end
```

代码运行结果  
![最下面的障碍物即为动态的](https://i-blog.csdnimg.cn/blog_migrate/93dcb501001260d12f976ba6801d42f0.png)  
代码地址：https://blog.csdn.net/BBeymax?type=download  
代码中都有详细注释，跑不通可以私聊。