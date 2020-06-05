x = map_logical_values;
for i = 1:length(obstacles)
    ox = round(obstacles(i,1)*2);
    oy = round(obstacles(i,2)*2);
    x(oy,ox) = 1;
end
dt = distanceTransform(x,[46,20]);
idisp(dt)