function newmap = expand(map)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
newmap = map;
newmap(2,:) = 1;
newmap(:,2) = 1;
[ny,nx] = size(map);
newmap(:,nx-1) = 1;
newmap(ny-1,:) = 1;
[row,column] = find(map==1);
for b = 1:length(row)
   if row(b) == 1 || row(b) == 2 || column(b) == 1 || column(b) == 2 || row(b) == 41 || row(b) == 40 || column(b) == 52 || column(b) == 50
       continue;
   end
   for i = -1:1
       for j = -1:1
           newmap(row(b)+j,column(b)+i) = 1;
       end
   end
end
end

