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

