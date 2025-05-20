% function [heading] = calcHeadingEval(x,endPiont)
% %以运动方向向量和当前位置与终点连线的向量的夹角(单位为°)
% theta = 180-acosd(dot([endPiont(1)-x(1),endPiont(2)-x(2),endPiont(3)-x(3)], ...
%     [x(4),x(5),x(6)])/(norm([endPiont(1)-x(1),endPiont(2)-x(2),endPiont(3)-x(3)]) ...
%     *norm([x(4),x(5),x(6)])));
% if theta <= 0
%     heading = -theta;
% else
%     heading = theta;
% end
% 
% end

