% function [dist,flag] = calcDisEval(x,sphereInfo)
% %障碍物距离评价指标
% dist = 100; %没有障碍物时的默认值
% flag =0;%碰到障碍物则置为1；
% for i = 1:size(sphereInfo.centerX,2)
%     center =[sphereInfo.centerX(i),sphereInfo.centeY(i),sphereInfo.centerZ(i)];
%     disttemp = calcuDis(x(:,1:3),center)-sphereInfo.radius(i);
%     if(disttemp < 0)
%         flag =1;
%         break;
%     else
%         %选取所有障碍物中距离最小值
%         if dist > disttemp
%             dist =disttemp;
%         end
%     end
% end
% %%%需要根据实际更改
% %限定障碍物距离最大值，如果不限定，则其占比会非常大
% if dist >= 24
%     dist = 24;
% end
% end

