function [xd,yd] = findMin(dt,xr,yr)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
base = 100;
xd = xr;
yd = yr;
for i = -1:1
    for j = -1:1
        y = yr+i;
        x = xr+j;
        if dt(y,x)<base
            base = dt(y,x);
            yd = y;
            xd = x;
        end
    end
end

