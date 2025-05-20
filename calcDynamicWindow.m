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

