
%% User-defined Parameters that affect the simulation
draw = 0;
NWaypoints = 5;
xmax = 26;
ymax = 20.5;
startPos = [2 2];
%% Load Data
load('logical_occupancy_map.mat')
load('complexMap.mat')
load('obstacles.mat')
[mapHeight, mapWidth] = size(map_logical_values);
disp('Calculating Path')

%% inflate walls to allow required space
inflatedMap = map_logical_values;
for i = 1:mapHeight
    for j = 1:mapWidth
        M = PathPlanning.window(map_logical_values,j,i);
        inflatedMap(i,j) = sum(M,'all')>0;
    end
end

%% Generate Waypoints
wpList = getWaypoints(NWaypoints,inflatedMap, obstacles, xmax,ymax);

%% Visualise the Scene
if draw
    
    temp_col = ceil(startPos(1)/xmax * mapWidth);
    temp_row = ceil((ymax-startPos(2))/ymax * mapHeight);
    tempMap = map_logical_values;
    tempMap(temp_row,temp_col) = 0.5;
    figure();
    colormap (flip(gray))
    imagesc([0,xmax],[0,ymax],flipud(tempMap))
    set(gca,'YDir','normal')
    hold on
    scatter(wpList(:,1),wpList(:,2),50,'gs','MarkerEdgeColor',[0.2 0.9 0.2],...
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
end

%% Plan Path
tic;
currentxy = startPos;
path = [];

% Use Brute Force Search to select order of waypoints
WPOrder = Astar.WPGraphSearch(wpList, startPos);

% wp = wpList(WPOrder(1),:);
% if draw    
%     plot([startPos(1) wp(1)], [startPos(2) wp(2)],'r--','LineWidth',2)
%     for i = 2:length(WPOrder)
%         wp1 = wpList(WPOrder(i),:);
%         wp2 = wpList(WPOrder(i-1),:);
%         plot([wp1(1) wp2(1)], [wp1(2) wp2(2)],'r--','LineWidth',2)
%     end
% end
%%

for wp = 1:NWaypoints
    
    %% Extract WP to plan to
    nextWp = WPOrder(wp);
    
    % Convert location information to grid space
    goal_col = ceil(wpList(nextWp,1)/xmax * mapWidth);
    goal_row = ceil((ymax-wpList(nextWp,2))/ymax * mapHeight);
    temp_col = ceil(currentxy(1)/xmax * mapWidth);
    temp_row = ceil((ymax-currentxy(2))/ymax * mapHeight);
    
    
    % Create Path usin Astar
    tmp_path = Astar.AstarPath(inflatedMap, temp_row, temp_col, goal_row, goal_col);
    
    % Convert back to cartesian coordinates
    tmp_pathxy = zeros(size(tmp_path));
    tmp_pathxy(:,2) = ymax*(mapHeight-tmp_path(:,1))/mapHeight;
    tmp_pathxy(:,1) = xmax*tmp_path(:,2)/mapWidth;
    
    % Ensure path passes over waypoints
    tmp_pathxy(1,:) = currentxy;
    tmp_pathxy(end,:) = wpList(nextWp,:);
    
    % Add leg of journey to total path
    path = [path;tmp_pathxy];
    currentxy = wpList(nextWp,:);
    
end


% REMOVED: Remove non-essential intermediary points
% path = simplifyPath(path);

timeTaken = toc
%% Plot Path
if draw    
    plot( path(:,1),path(:,2),'m-','LineWidth',2)
    plot( path(:,1),path(:,2),'b.','MarkerSize',10)
    drawnow
end


%% Functions
function points = getWaypoints(numPoints, map, obstacles, xmax,ymax)
% Extract Grid Size
[mapHeight, mapWidth] = size(map);

points = zeros(numPoints,2);

%% Calculate required number of waypoints
for i = 1:numPoints
    occupied = 1;
    
    %% Calculate new WP and check it is in a viable position
    while(occupied)
        randx = rand(1)*xmax;
        randy = rand(1)*ymax;
        
        % Check overlap with obstacles
        obstacleOverlap = sum(sqrt((obstacles(:,1)-randx).^2 + (obstacles(:,2) - randy).^2)<0.5,'all')>0;
        
        % Convert new points to grid space
        cellx0 = max(floor(randx/xmax * mapWidth),1);
        celly0 = max(floor((ymax-randy)/ymax * mapHeight),1);
        
        cellx1 = ceil(randx/xmax * mapWidth);
        celly1 = ceil((ymax-randy)/ymax * mapHeight);
        
        % Check if new point overlaps with walls in map
        wallOverlap = sum(sum(find(PathPlanning.window(map, cellx0,celly0)==1)))>0 ||...
            sum(sum(find(PathPlanning.window(map, cellx1,celly1)==1)))>0;
        
        % Check if new point overlaps with previous waypoints
        wpOverlap = sum(sqrt((points(:,1)-randx).^2 + (points(:,2) - randy).^2)<0.5,'all')>0;
        
        if ~obstacleOverlap && ~wallOverlap && ~wpOverlap
            points(i,:) = [randx randy];
            occupied = 0;
        end
        
    end
end
end


function path = simplifyPath(path)
mprev = inf;
i=2;
while(1)
    if(i>length(path))
        break;
    end
    x1 = path(i-1,1);
    y1 = path(i-1,2);
    x2 = path(i,1);
    y2 = path(i,2);
    mnext = (y2-y1)/(x2-x1);
    if abs(mnext - mprev)<0.001
        path = [path(1:i-2,:); path(i:end,:)];
    else
        i = i+1;
    end
    mprev = mnext;
end
mprev = inf;
i=2;
while(1)
    if(i>length(path))
        break;
    end
    y1 = path(i-1,1);
    x1 = path(i-1,2);
    y2 = path(i,1);
    x2 = path(i,2);
    mnext = (y2-y1)/(x2-x1);
    if abs(mnext - mprev)<0.001
        path = [path(1:i-2,:); path(i:end,:)];
    else
        i = i+1;
    end
    mprev = mnext;
end
end
