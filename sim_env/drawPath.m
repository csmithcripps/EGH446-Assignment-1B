%% Visualise the Scene

temp_col = ceil(startPos(1)/xmax * mapWidth);
temp_row = ceil((ymax-startPos(2))/ymax * mapHeight);
tempMap = map_logical_values;
tempMap(temp_row,temp_col) = 0.5;
figure();
colormap (flip(gray))
imagesc([0,xmax],[0,ymax],flipud(tempMap))
set(gca,'YDir','normal')
hold on
scatter(wpList(:,1),wpList(:,2),'gs','MarkerEdgeColor',[0.2 0.9 0.2],...
                                     'MarkerFaceColor','g',...
                                     'LineWidth', 2)
rgbArr = ['r' 'g' 'b'];
for i = 1:size(obstacles,1)
    
    scatter(obstacles(i,1),obstacles(i,2),'ro','MarkerEdgeColor','k',...
                                               'MarkerFaceColor',rgbArr(obstacles(i,3)),...
                                               'LineWidth', 1)
end
scatter(startPos(1),startPos(2),'mo','MarkerEdgeColor','m',...
                                     'MarkerFaceColor','m',...
                                     'LineWidth', 2)
                                 

%% Plot Path
plot( path(:,1),path(:,2),'m-','LineWidth',2)
plot( path(:,1),path(:,2),'b.','MarkerSize',10)
drawnow