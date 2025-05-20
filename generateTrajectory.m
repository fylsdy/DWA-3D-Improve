% function [x,traj] = generateTrajectory(vtx,vty,vtz,paraT,x)
% t1 = 0;
% traj = x;
% while t1 <= paraT
%     %更新时间
%     t1 = t1+dt; 
%     %更新状态,假设前向模拟的速度值为恒定值
%     x = [x(1)+dt*vtx x(2)+dt*vty x(3)+dt*vtz vtx vty vtz];
%     %将轨迹和状态保存在traj中
%     traj = [traj x];
% 
% end
% end

