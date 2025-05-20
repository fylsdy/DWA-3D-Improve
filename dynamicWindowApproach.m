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



