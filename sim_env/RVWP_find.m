function [solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d)

distPrev = sqrt((robotx-X0)^2 + (roboty-Y0)^2);

pathAngle = atan2(Y1-Y0,X1-X0);
robotAngle = atan2(roboty - Y0, robotx - X0);

distpath = distPrev * cos(robotAngle - pathAngle);
distUpPath = d + distpath;

solx = X0 + distUpPath*cos(pathAngle);
soly = Y0 + distUpPath*sin(pathAngle);
% 
% mPath = (Y1-Y0)/(X1-X0);
% cPath = Y1 - mPath*X1;
% 
% ycheck = mPath*solx + cPath;
% 
% if ycheck ~= soly
%     soly = Y1;
%     solx = X1;
% end
% 
% mPath = (X1-X0)/(Y1-Y0);
% cPath = X1 - mPath*Y1;
% 
% xcheck = mPath*soly + cPath;
% 
% if xcheck ~= solx
%     soly = Y1;
%     solx = X1;
% end
% 

 
% syms x y
% [solx,soly] = vpasolve(y== ((Y1-Y0)/(X1-X0)) * (x-X0) + Y0, (x-robotx)^2 + (y-roboty)^2 == d, [x,y],[X1,Y1]);
% 
% 
% 
% 
% sol_1_dist = sqrt((solx(1)-X1)^2 + (soly(1) - Y1)^2);
% sol_2_dist = sqrt((solx(2)-X1)^2 + (soly(2) - Y1)^2);
% 
% if sol_1_dist < sol_2_dist
%     solx = double(solx(1));
%     soly = double(soly(1));
% else
%     solx = double(solx(2));
%     soly = double(soly(2));
% end
